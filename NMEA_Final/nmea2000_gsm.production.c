/*
 * NMEA2000 Recepteur Complet
 * GPS + Moteur SELVA Sei Whale + AIS em-trak B921
 * ESP32-S3 + SN65HVD230 | TWAI 250 kbit/s
 *
 * GPIO5 = TX CAN
 * GPIO4 = RX CAN
 *
 * PGNs decodes :
 *  GPS    : 129025, 129026, 127250, 129029, 129539, 129033
 *  Moteur : 127488, 127489, 127493, 127505, 127508, 128259, 128267, 130310
 *  AIS    : 129038, 129039, 129794, 129809, 129810
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "nvs_flash.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ─── Configuration ─────────────────────────────────────────────────────── */
#define CAN_TX_GPIO   GPIO_NUM_5
#define CAN_RX_GPIO   GPIO_NUM_4
#define CAN_BITRATE   250000
#define RX_POOL_DEPTH 64

/* Reseau LAN fourni par le routeur 4G/GSM D-Link DWR-960.
 * L'ESP32 se connecte en Wi-Fi STATION sur ce reseau (au lieu de creer
 * son propre point d'acces). Remplace SSID/mot de passe par ceux du
 * DWR-960 (visibles dans son interface d'admin, page "Wireless/WLAN"). */
/* IMPORTANT : l'ESP32 ne supporte que le Wi-Fi 2.4 GHz. Utilise bien le SSID
 * de la bande 2.4 GHz du DWR-960 (celui SANS "-5G" dans le nom), sinon
 * l'ESP32 ne verra jamais le reseau. */
#define GSM_ROUTER_SSID       "dlink_DWR-960_69C6"
#define GSM_ROUTER_PASSWORD   "zScFh79684"
#define WIFI_RECONNECT_DELAY_MS 2000

static const char *TAG = "NMEA_RX";

/* ─── PGNs ───────────────────────────────────────────────────────────────── */
/* GPS */
#define PGN_POSITION_RAPID    129025U
#define PGN_COG_SOG           129026U
#define PGN_VESSEL_HEADING    127250U
#define PGN_GNSS_POSITION     129029U
#define PGN_GNSS_DOPS         129539U
#define PGN_TIME_DATE         129033U
/* Moteur */
#define PGN_ENGINE_RAPID      127488U
#define PGN_ENGINE_DYNAMIC    127489U
#define PGN_TRANSMISSION      127493U
#define PGN_FLUID_LEVEL       127505U
#define PGN_BATTERY_STATUS    127508U
#define PGN_SPEED_WATER       128259U
#define PGN_WATER_DEPTH       128267U
#define PGN_ENVIRONMENT       130310U
/* AIS */
#define PGN_AIS_CLASS_A       129038U
#define PGN_AIS_CLASS_B       129039U
#define PGN_AIS_STATIC_A      129794U
#define PGN_AIS_STATIC_B1     129809U
#define PGN_AIS_STATIC_B2     129810U

/* ─── Etat GPS ───────────────────────────────────────────────────────────── */
typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, hdg_deg;
    float   hdop, vdop;
    uint8_t num_svs;
    char    utc[16];
    uint8_t fix;
} gps_t;

/* ─── Etat Moteur ────────────────────────────────────────────────────────── */
typedef struct {
    float   rpm;
    float   coolant_temp_c;
    float   oil_pressure_bar;
    float   oil_temp_c;
    float   fuel_rate_lh;
    float   battery_v;
    float   fuel_level_pct;
    float   water_depth_m;
    float   water_temp_c;
    float   speed_water_kn;
    float   hours;
    uint8_t check_engine;
    uint8_t over_temp;
    uint8_t low_oil_pressure;
    uint8_t low_voltage;
} engine_t;

/* ─── Etat AIS ───────────────────────────────────────────────────────────── */
#define MAX_AIS_TARGETS 16
typedef struct {
    uint32_t mmsi;
    double   lat, lon;
    float    cog_deg, sog_kn, hdg_deg;
    char     name[21];
    char     callsign[8];
    uint8_t  nav_status;
    uint8_t  valid;
    uint32_t last_seen_ms;
} ais_target_t;

static gps_t        g_gps    = { .hdop=99.9f, .vdop=99.9f };
static engine_t     g_engine = { .battery_v=0.0f };
static ais_target_t g_ais[MAX_AIS_TARGETS];
static SemaphoreHandle_t g_mutex;
static volatile bool g_time_synced = false; /* horloge systeme synchronisee depuis le GPS (PGN 129033) */

/* ─── AIS helpers ────────────────────────────────────────────────────────── */
static ais_target_t *ais_find_or_create(uint32_t mmsi)
{
    ais_target_t *oldest = &g_ais[0];
    for (int i = 0; i < MAX_AIS_TARGETS; i++) {
        if (g_ais[i].valid && g_ais[i].mmsi == mmsi) return &g_ais[i];
        if (!g_ais[i].valid) { oldest = &g_ais[i]; break; }
        if (g_ais[i].last_seen_ms < oldest->last_seen_ms) oldest = &g_ais[i];
    }
    memset(oldest, 0, sizeof(*oldest));
    oldest->mmsi  = mmsi;
    oldest->valid = 1;
    return oldest;
}

static const char *ais_nav_status(uint8_t s)
{
    switch (s) {
    case 0:  return "Underway";
    case 1:  return "At anchor";
    case 2:  return "Not under cmd";
    case 3:  return "Restricted";
    case 5:  return "Moored";
    case 8:  return "Sailing";
    case 15: return "Unknown";
    default: return "Other";
    }
}

/* ─── Parseur ID CAN 29 bits ─────────────────────────────────────────────── */
static void parse_can_id(uint32_t can_id,
                         uint8_t *prio, uint32_t *pgn, uint8_t *src)
{
    *prio = (can_id >> 26) & 0x07;
    uint32_t pgn_raw = (can_id >> 8) & 0x3FFFF;
    *src  = can_id & 0xFF;
    uint8_t pf = (pgn_raw >> 8) & 0xFF;
    *pgn  = (pf < 0xF0) ? (pgn_raw & 0x1FF00U) : pgn_raw;
}

/* ─── Fast-Packet ────────────────────────────────────────────────────────── */
#define FP_MAX_STREAMS  8
#define FP_MAX_PAYLOAD  223
typedef struct {
    uint8_t  active, src, seq_id, total, received;
    uint32_t pgn;
    uint8_t  buf[FP_MAX_PAYLOAD];
} fp_stream_t;
static fp_stream_t fp_streams[FP_MAX_STREAMS];

static const uint8_t *fp_feed(uint8_t src, uint32_t pgn,
                               const uint8_t *frame, uint8_t *out_len)
{
    uint8_t seq_id    = (frame[0] >> 5) & 0x07;
    uint8_t frame_num =  frame[0]       & 0x1F;
    fp_stream_t *s    = NULL;

    for (int i = 0; i < FP_MAX_STREAMS; i++) {
        if (fp_streams[i].active && fp_streams[i].src == src &&
            fp_streams[i].pgn == pgn && fp_streams[i].seq_id == seq_id) {
            s = &fp_streams[i]; break;
        }
    }
    if (frame_num == 0) {
        for (int i = 0; i < FP_MAX_STREAMS; i++) {
            if (!fp_streams[i].active) { s = &fp_streams[i]; break; }
        }
        if (!s) s = &fp_streams[0];
        memset(s, 0, sizeof(*s));
        s->active=1; s->src=src; s->pgn=pgn; s->seq_id=seq_id;
        s->total=frame[1];
        uint8_t n = s->total<6 ? s->total : 6;
        memcpy(s->buf, &frame[2], n);
        s->received=1;
    } else {
        if (!s) return NULL;
        uint8_t offset = 6+(frame_num-1)*7;
        uint8_t remain = s->total>offset ? s->total-offset : 0;
        uint8_t n = remain<7 ? remain : 7;
        if (offset+n <= FP_MAX_PAYLOAD) memcpy(&s->buf[offset], &frame[1], n);
        s->received++;
    }
    uint8_t nf = (s->total<=6) ? 1 : (1+(s->total-6+6)/7);
    if (s->received >= nf) { *out_len=s->total; s->active=0; return s->buf; }
    return NULL;
}

/* ─── Fast-Packet PGNs list ─────────────────────────────────────────────── */
static bool is_fast_packet(uint32_t pgn)
{
    switch (pgn) {
    case PGN_GNSS_POSITION:
    case PGN_TIME_DATE:
    case PGN_AIS_CLASS_A:
    case PGN_AIS_CLASS_B:
    case PGN_AIS_STATIC_A:
    case PGN_AIS_STATIC_B1:
    case PGN_AIS_STATIC_B2:
        return true;
    default:
        return false;
    }
}

/* ─── Decodeurs GPS ──────────────────────────────────────────────────────── */
static void dec_position_rapid(const uint8_t *d, uint8_t l)
{
    if (l < 8) return;
    int32_t la = (int32_t)(d[0]|((uint32_t)d[1]<<8)|((uint32_t)d[2]<<16)|((uint32_t)d[3]<<24));
    int32_t lo = (int32_t)(d[4]|((uint32_t)d[5]<<8)|((uint32_t)d[6]<<16)|((uint32_t)d[7]<<24));
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.lat=la*1e-7; g_gps.lon=lo*1e-7; g_gps.fix=1;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[GPS] POS lat=%.6f lon=%.6f", la*1e-7, lo*1e-7);
}

static void dec_cog_sog(const uint8_t *d, uint8_t l)
{
    if (l < 6) return;
    uint16_t ci=d[2]|((uint16_t)d[3]<<8), si=d[4]|((uint16_t)d[5]<<8);
    float cog=ci*1e-4f*180.0f/(float)M_PI, sog=si*1e-4f/0.514444f;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.cog_deg=cog; g_gps.sog_kn=sog;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[GPS] COG=%.1f SOG=%.2fkn", cog, sog);
}

static void dec_heading(const uint8_t *d, uint8_t l)
{
    ESP_LOGE(TAG, "!!! dec_heading appelée l=%d", l);
    if (l < 4) return;
    uint16_t hi=d[2]|((uint16_t)d[3]<<8);
    float hdg=hi*1e-4f*180.0f/(float)M_PI;
    ESP_LOGI(TAG, "[GPS] HDG=%.1f raw=0x%04X", hdg, hi);  /* ← ajoute ça */
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.hdg_deg=hdg;
    xSemaphoreGive(g_mutex);
    ESP_LOGD(TAG, "[GPS] HDG=%.1f", hdg);
}

static void dec_dops(const uint8_t *d, uint8_t l)
{
    if (l < 8) return;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.hdop=(int16_t)(d[2]|((uint16_t)d[3]<<8))*0.01f;
    g_gps.vdop=(int16_t)(d[4]|((uint16_t)d[5]<<8))*0.01f;
    xSemaphoreGive(g_mutex);
}

static void dec_gnss_position(const uint8_t *d, uint8_t l)
{
    if (l < 27) return;
    int64_t la=0, lo=0;
    for(int i=0;i<8;i++) la|=((int64_t)d[11+i]<<(8*i));
    for(int i=0;i<8;i++) lo|=((int64_t)d[19+i]<<(8*i));
    if (la&((int64_t)1<<55)) la|=~(((int64_t)1<<56)-1);
    if (lo&((int64_t)1<<55)) lo|=~(((int64_t)1<<56)-1);
    /* NOTE : sur ce GPS (Sportnav SPO25F), le decodage lat/lon de cette PGN
     * (129029, fast-packet) produit des valeurs aberrantes - probablement
     * un decalage d'octets specifique a cette implementation. On ne s'en
     * sert donc PAS pour ecraser g_gps.lat/lon (deja mis a jour de facon
     * fiable par PGN 129025). On garde juste le nombre de satellites. */
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.fix=1;
    g_gps.num_svs=(l>28 && d[28]!=0xFF)?d[28]:g_gps.num_svs;
    xSemaphoreGive(g_mutex);
    ESP_LOGD(TAG, "[GPS] GNSS (129029) recue, svs=%d (lat/lon ignores, voir 129025)", g_gps.num_svs);
}

static void dec_time_date(const uint8_t *d, uint8_t l)
{
    if (l < 7) return;
    /* PGN 129033 : SID(1) + Date jours depuis epoch (2, LE) +
     * Time 0.0001s depuis minuit (4, LE) + Local offset minutes (2, LE) */
    uint16_t days = d[1] | ((uint16_t)d[2] << 8);
    uint32_t t_i  = d[3] | ((uint32_t)d[4]<<8) | ((uint32_t)d[5]<<16) | ((uint32_t)d[6]<<24);
    uint32_t secs_of_day = t_i / 10000;

    time_t epoch = (time_t)days * 86400 + secs_of_day;
    struct tm tm_utc;
    gmtime_r(&epoch, &tm_utc);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    snprintf(g_gps.utc, sizeof(g_gps.utc), "%02d:%02d:%02d",
             tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
    xSemaphoreGive(g_mutex);

    if (!g_time_synced) {
        struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        g_time_synced = true;
        ESP_LOGI(TAG, "[GPS] Horloge systeme synchronisee sur l'heure GPS (UTC)");
    }
    ESP_LOGI(TAG, "[GPS] UTC=%s", g_gps.utc);
}

/* ─── Decodeurs Moteur ───────────────────────────────────────────────────── */

/* PGN 127488 – Engine Rapid Update (RPM, boost, tilt) */
static void dec_engine_rapid(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 8) return;
    uint16_t rpm_i = d[0]|(uint16_t)d[1]<<8;
    float rpm = rpm_i * 0.25f;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.rpm = rpm;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[ENG] RPM=%.0f src=0x%02X", rpm, src);
}

/* PGN 127489 – Engine Dynamic (temp, pression huile, heures, alarmes) */
static void dec_engine_dynamic(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 8) return;
    /* oil_pressure bytes 0-1 (0.1 hPa -> bar) */
    uint16_t op_i = d[0]|(uint16_t)d[1]<<8;
    float oil_pres = (op_i != 0xFFFF) ? op_i * 100.0f / 1e5f : 0.0f;
    /* oil_temp bytes 2-3 (0.01 K -> C) */
    uint16_t ot_i = d[2]|(uint16_t)d[3]<<8;
    float oil_temp = (ot_i != 0xFFFF) ? ot_i * 0.01f - 273.15f : 0.0f;
    /* coolant_temp bytes 4-5 (0.01 K -> C) */
    uint16_t ct_i = d[4]|(uint16_t)d[5]<<8;
    float cool_temp = (ct_i != 0xFFFF) ? ct_i * 0.01f - 273.15f : 0.0f;
    /* status bytes 6-7 (bitmask alarmes) */
    uint16_t status = d[6]|(uint16_t)d[7]<<8;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.oil_pressure_bar = oil_pres;
    g_engine.oil_temp_c       = oil_temp;
    g_engine.coolant_temp_c   = cool_temp;
    g_engine.check_engine     = (status & 0x0001) ? 1 : 0;
    g_engine.over_temp        = (status & 0x0002) ? 1 : 0;
    g_engine.low_oil_pressure = (status & 0x0004) ? 1 : 0;
    g_engine.low_voltage      = (status & 0x0010) ? 1 : 0;
    xSemaphoreGive(g_mutex);

    ESP_LOGI(TAG, "[ENG] OilP=%.2fbar OilT=%.1fC CoolT=%.1fC %s%s%s src=0x%02X",
             oil_pres, oil_temp, cool_temp,
             (status&0x01)?"CHK_ENG ":"",
             (status&0x02)?"OVERTEMP ":"",
             (status&0x04)?"LOW_OIL ":"", src);
}

/* PGN 127505 – Fluid Level (carburant, eau douce, eaux usees) */
static void dec_fluid_level(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 4) return;
    uint8_t  fluid_type = d[0] & 0x0F;
    uint16_t level_i    = d[2]|(uint16_t)d[3]<<8;
    float level_pct = level_i * 0.004f;   /* 0.004 % per bit */

    const char *type_str;
    switch (fluid_type) {
    case 0: type_str = "Fuel";       break;
    case 1: type_str = "FreshWater"; break;
    case 2: type_str = "WasteWater"; break;
    case 3: type_str = "LiveWell";   break;
    case 4: type_str = "Oil";        break;
    default: type_str = "Unknown";   break;
    }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (fluid_type == 0) g_engine.fuel_level_pct = level_pct;
    xSemaphoreGive(g_mutex);

    ESP_LOGI(TAG, "[ENG] Fluid %s=%.1f%% src=0x%02X", type_str, level_pct, src);
}

/* PGN 127508 – Battery Status */
static void dec_battery(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 4) return;
    uint16_t volt_i = d[0]|(uint16_t)d[1]<<8;
    float volt = volt_i * 0.01f;   /* 0.01 V per bit */
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.battery_v = volt;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[ENG] Battery=%.2fV src=0x%02X", volt, src);
}

/* PGN 128259 – Speed Water Referenced */
static void dec_speed_water(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 3) return;
    uint16_t sw_i = d[1]|(uint16_t)d[2]<<8;
    float spd = sw_i * 0.01f / 0.514444f;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.speed_water_kn = spd;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[ENG] SpeedWater=%.2fkn src=0x%02X", spd, src);
}

/* PGN 128267 – Water Depth */
static void dec_water_depth(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 5) return;
    uint32_t dep_i = d[1]|((uint32_t)d[2]<<8)|((uint32_t)d[3]<<16)|((uint32_t)d[4]<<24);
    float depth = dep_i * 0.01f;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.water_depth_m = depth;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[ENG] Depth=%.2fm src=0x%02X", depth, src);
}

/* PGN 130310 – Environmental Parameters (temp eau) */
static void dec_environment(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 4) return;
    uint16_t wt_i = d[2]|(uint16_t)d[3]<<8;
    float wt = (wt_i != 0xFFFF) ? wt_i * 0.01f - 273.15f : 0.0f;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_engine.water_temp_c = wt;
    xSemaphoreGive(g_mutex);
    ESP_LOGI(TAG, "[ENG] WaterTemp=%.1fC src=0x%02X", wt, src);
}

/* ─── Decodeurs AIS ──────────────────────────────────────────────────────── */

/* Extraire N bits depuis payload AIS (little-endian bit order) */
static uint32_t ais_bits(const uint8_t *d, int start, int len)
{
    uint32_t val = 0;
    for (int i = 0; i < len; i++) {
        int bit = start + i;
        if ((d[bit/8] >> (bit%8)) & 1) val |= (1U << i);
    }
    return val;
}

static int32_t ais_bits_s(const uint8_t *d, int start, int len)
{
    uint32_t raw = ais_bits(d, start, len);
    if (len < 32 && (raw & (1U << (len-1)))) raw |= ~((1U << len) - 1);
    return (int32_t)raw;
}

static void ais_str(const uint8_t *d, int start, int len, char *out, int outlen)
{
    int j = 0;
    for (int i = 0; i < len && j < outlen-1; i++) {
        char c = (char)ais_bits(d, start + i*6, 6);
        if (c < 32) c += 64;
        if (c == '@') break;
        out[j++] = c;
    }
    while (j > 0 && out[j-1] == ' ') j--;
    out[j] = '\0';
}

/* PGN 129038 – AIS Class A Position Report */
static void dec_ais_class_a(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 27) return;
    (void)ais_bits(d, 0, 6); /* msg_type non utilise */
    uint32_t mmsi      = ais_bits(d, 8, 30);
    uint8_t  nav_stat  = ais_bits(d, 38, 4);
    int32_t  lon_i     = ais_bits_s(d, 61, 28);
    int32_t  lat_i     = ais_bits_s(d, 89, 27);
    uint16_t sog_i     = ais_bits(d, 50, 10);
    uint16_t cog_i     = ais_bits(d, 116, 12);

    double lat = lat_i / 600000.0;
    double lon = lon_i / 600000.0;
    float  sog = sog_i * 0.1f;
    float  cog = cog_i * 0.1f;

    ais_target_t *t = ais_find_or_create(mmsi);
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    t->lat = lat; t->lon = lon;
    t->sog_kn = sog; t->cog_deg = cog;
    t->nav_status = nav_stat;
    t->last_seen_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(g_mutex);

    ESP_LOGI(TAG, "[AIS-A] MMSI=%"PRIu32" lat=%.5f lon=%.5f SOG=%.1fkn COG=%.1f %s src=0x%02X",
             mmsi, lat, lon, sog, cog, ais_nav_status(nav_stat), src);
}

/* PGN 129039 – AIS Class B Position Report */
static void dec_ais_class_b(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 28) return;
    /* PGN 129039 format NMEA2000 natif */
    uint8_t  msg_id  = d[0] & 0x3F;
    uint8_t  repeat  = (d[0] >> 6) & 0x03;
    uint32_t mmsi    = (uint32_t)d[1] | ((uint32_t)d[2]<<8) |
                       ((uint32_t)d[3]<<16) | ((uint32_t)d[4]<<24);
    /* SOG en 0.01 m/s */
    uint16_t sog_i   = d[5] | ((uint16_t)d[6]<<8);
    float    sog     = sog_i * 0.01f / 0.514444f;
    /* Longitude en 1e-7 deg */
    int32_t  lon_i   = (int32_t)(d[7] | ((uint32_t)d[8]<<8) |
                       ((uint32_t)d[9]<<16) | ((uint32_t)d[10]<<24));
    /* Latitude en 1e-7 deg */
    int32_t  lat_i   = (int32_t)(d[11] | ((uint32_t)d[12]<<8) |
                       ((uint32_t)d[13]<<16) | ((uint32_t)d[14]<<24));
    /* COG en 1e-4 rad */
    uint16_t cog_i   = d[15] | ((uint16_t)d[16]<<8);
    float    cog     = cog_i * 1e-4f * 180.0f / (float)M_PI;
    /* HDG en 1e-4 rad */
    uint16_t hdg_i   = d[17] | ((uint16_t)d[18]<<8);
    float    hdg     = hdg_i * 1e-4f * 180.0f / (float)M_PI;

    double lat = lat_i * 1e-7;
    double lon = lon_i * 1e-7;

    ais_target_t *t = ais_find_or_create(mmsi);
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    t->lat=lat; t->lon=lon;
    t->sog_kn=sog; t->cog_deg=cog; t->hdg_deg=hdg;
    t->last_seen_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(g_mutex);

    ESP_LOGI(TAG, "[AIS-B] MMSI=%"PRIu32" lat=%.5f lon=%.5f SOG=%.1fkn COG=%.1f src=0x%02X",
             mmsi, lat, lon, sog, cog, src);
}

/* PGN 129794 – AIS Class A Static Data */
static void dec_ais_static_a(const uint8_t *d, uint8_t l, uint8_t src)
{
    if (l < 40) return;
    uint32_t mmsi = ais_bits(d, 8, 30);
    char name[21]={0}, call[8]={0};
    ais_str(d, 112, 20, name, sizeof(name));
    ais_str(d, 70,  7, call, sizeof(call));

    ais_target_t *t = ais_find_or_create(mmsi);
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    strncpy(t->name,     name, sizeof(t->name)-1);
    strncpy(t->callsign, call, sizeof(t->callsign)-1);
    t->last_seen_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(g_mutex);

    ESP_LOGI(TAG, "[AIS-A] MMSI=%"PRIu32" Name='%s' Call='%s' src=0x%02X",
             mmsi, name, call, src);
}

/* PGN 129809/129810 – AIS Class B Static Data */
static void dec_ais_static_b(const uint8_t *d, uint8_t l, uint8_t src, uint32_t pgn)
{
    if (l < 20) return;
    uint32_t mmsi = ais_bits(d, 8, 30);
    char name[21]={0}, call[8]={0};

    if (pgn == PGN_AIS_STATIC_B1) {
        ais_str(d, 46, 7, call, sizeof(call));
        ais_target_t *t = ais_find_or_create(mmsi);
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        strncpy(t->callsign, call, sizeof(t->callsign)-1);
        xSemaphoreGive(g_mutex);
        ESP_LOGI(TAG, "[AIS-B1] MMSI=%"PRIu32" Call='%s' src=0x%02X", mmsi, call, src);
    } else {
        ais_str(d, 46, 20, name, sizeof(name));
        ais_target_t *t = ais_find_or_create(mmsi);
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        strncpy(t->name, name, sizeof(t->name)-1);
        xSemaphoreGive(g_mutex);
        ESP_LOGI(TAG, "[AIS-B2] MMSI=%"PRIu32" Name='%s' src=0x%02X", mmsi, name, src);
    }
}

/* ─── JSON builder ───────────────────────────────────────────────────────── */

/* Construit le tableau JSON des cibles AIS actives.
 * ATTENTION : suppose que g_mutex est DEJA pris par l'appelant
 * (g_mutex n'est pas recursif). */
static void ais_array_json_locked(char *buf, size_t len)
{
    int used = snprintf(buf, len, "[");
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    for (int i = 0; i < MAX_AIS_TARGETS; i++) {
        if (!g_ais[i].valid) continue;
        if (now_ms - g_ais[i].last_seen_ms > 30000) continue;
        int n = snprintf(buf + used, (len > (size_t)used + 2) ? len - used - 2 : 0,
            "%s{\"mmsi\":%"PRIu32",\"lat\":%.5f,\"lon\":%.5f,"
            "\"sog\":%.1f,\"cog\":%.1f,\"name\":\"%s\","
            "\"call\":\"%s\",\"status\":\"%s\"}",
            (used > 1) ? "," : "",
            g_ais[i].mmsi, g_ais[i].lat, g_ais[i].lon,
            g_ais[i].sog_kn, g_ais[i].cog_deg,
            g_ais[i].name, g_ais[i].callsign,
            ais_nav_status(g_ais[i].nav_status));
        if (n > 0) used += n;
        if ((size_t)used >= len - 2) break;
    }
    snprintf(buf + used, len - used, "]");
}

static void build_json(char *buf, size_t len)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);

    /* AIS targets JSON */
    char ais_json[1024];
    ais_array_json_locked(ais_json, sizeof(ais_json));

    snprintf(buf, len,
        "{"
        "\"gps\":{"
            "\"fix\":%s,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,"
            "\"sog\":%.2f,\"cog\":%.1f,\"hdg\":%.1f,"
            "\"hdop\":%.2f,\"vdop\":%.2f,\"svs\":%d,\"utc\":\"%s\""
        "},"
        "\"engine\":{"
            "\"rpm\":%.0f,\"coolant\":%.1f,\"oil_pres\":%.2f,"
            "\"oil_temp\":%.1f,\"battery\":%.2f,"
            "\"fuel_pct\":%.1f,\"depth\":%.2f,\"water_temp\":%.1f,"
            "\"speed_water\":%.2f,"
            "\"check_engine\":%d,\"over_temp\":%d,"
            "\"low_oil\":%d,\"low_volt\":%d"
        "},"
        "\"ais\":%s"
        "}",
        g_gps.fix ? "true" : "false",
        g_gps.lat, g_gps.lon, g_gps.alt,
        g_gps.sog_kn, g_gps.cog_deg,
        (g_gps.hdg_deg > 0.0f) ? g_gps.hdg_deg : g_gps.cog_deg,
        g_gps.hdop, g_gps.vdop, g_gps.num_svs, g_gps.utc,
        g_engine.rpm, g_engine.coolant_temp_c, g_engine.oil_pressure_bar,
        g_engine.oil_temp_c, g_engine.battery_v,
        g_engine.fuel_level_pct, g_engine.water_depth_m, g_engine.water_temp_c,
        g_engine.speed_water_kn,
        g_engine.check_engine, g_engine.over_temp,
        g_engine.low_oil_pressure, g_engine.low_voltage,
        ais_json);

    xSemaphoreGive(g_mutex);
}

/* ─── WebSocket ──────────────────────────────────────────────────────────── */
#define MAX_WS_CLIENTS 4
static int g_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t g_ws_mutex;
static httpd_handle_t g_server = NULL;

static void ws_init(void) {
    for (int i=0;i<MAX_WS_CLIENTS;i++) g_ws_fds[i]=-1;
}
static void ws_add(int fd) {
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i=0;i<MAX_WS_CLIENTS;i++) { if(g_ws_fds[i]==-1){g_ws_fds[i]=fd;break;} }
    xSemaphoreGive(g_ws_mutex);
    ESP_LOGI(TAG, "WS client connected fd=%d", fd);
}
static void ws_broadcast(const char *json) {
    if (!g_server) return;
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i=0;i<MAX_WS_CLIENTS;i++) {
        int fd=g_ws_fds[i]; if (fd<0) continue;
        httpd_ws_frame_t frame={.type=HTTPD_WS_TYPE_TEXT,.payload=(uint8_t*)json,.len=strlen(json),.final=true};
        if (httpd_ws_send_frame_async(g_server,fd,&frame)!=ESP_OK) g_ws_fds[i]=-1;
    }
    xSemaphoreGive(g_ws_mutex);
}

/* ─── Dashboard HTML ─────────────────────────────────────────────────────── */
static const char DASHBOARD_HTML[] =
"<!DOCTYPE html><html lang='en'><head>"
"<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>NMEA2000 Dashboard</title><style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{font-family:'Segoe UI',sans-serif;background:#0a1628;color:#e0e8f0;min-height:100vh}"
"header{background:#0d2137;padding:10px 16px;display:flex;align-items:center;gap:10px;border-bottom:1px solid #1e3a5f}"
"header h1{font-size:1rem;color:#7dd3fc;flex:1}"
".dot{width:10px;height:10px;border-radius:50%;background:#ef4444;flex-shrink:0}"
".dot.ok{background:#22c55e;box-shadow:0 0 8px #22c55e}"
".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:12px;padding:14px}"
".card{background:#0a1e35;border:1px solid #1e3a5f;border-radius:8px;padding:12px}"
".card h2{font-size:.68rem;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:10px}"
".card.gps h2{color:#22c55e}"
".card.engine h2{color:#f97316}"
".card.ais h2{color:#3b82f6}"
".row{display:flex;justify-content:space-between;align-items:baseline;padding:3px 0;border-bottom:1px solid #1e3a5f18}"
".row:last-child{border:none}"
".lbl{font-size:.72rem;color:#94a3b8}"
".val{font-size:.9rem;font-weight:600}"
".big{font-size:1.5rem;color:#7dd3fc}"
".ok{color:#22c55e}.no{color:#ef4444}.warn{color:#f97316}"
"canvas{display:block;margin:0 auto}"
"#ais-list{display:flex;flex-direction:column;gap:6px}"
".ais-target{background:#060f1e;border:1px solid #1e3a5f;border-radius:6px;padding:8px;font-size:.75rem}"
".ais-mmsi{font-weight:700;color:#3b82f6;font-size:.85rem}"
".ais-name{color:#7dd3fc;font-size:.9rem;font-weight:600}"
"footer{padding:5px 14px;font-size:.65rem;color:#334155;border-top:1px solid #1e3a5f;text-align:center}"
"</style></head><body>"
"<header><div class='dot' id='dot'></div><h1>NMEA2000 Dashboard</h1><span id='slbl' style='font-size:.75rem;color:#94a3b8'>Connecting...</span></header>"
"<div class='grid'>"
/* GPS */
"<div class='card gps'><h2>GPS Position</h2>"
"<div class='row'><span class='lbl'>Fix</span><span class='val' id='vfix'>-</span></div>"
"<div class='row'><span class='lbl'>Latitude</span><span class='val' id='vlat'>-</span></div>"
"<div class='row'><span class='lbl'>Longitude</span><span class='val' id='vlon'>-</span></div>"
"<div class='row'><span class='lbl'>Altitude</span><span class='val' id='valt'>-</span></div>"
"<div class='row'><span class='lbl'>Satellites</span><span class='val' id='vsvs'>-</span></div>"
"<div class='row'><span class='lbl'>UTC</span><span class='val' id='vutc'>-</span></div>"
"</div>"
/* Navigation */
"<div class='card gps'><h2>Navigation</h2>"
"<div class='row'><span class='lbl'>SOG</span><span class='val big' id='vsog'>-</span></div>"
"<div class='row'><span class='lbl'>COG</span><span class='val' id='vcog'>-</span></div>"
"<div class='row'><span class='lbl'>Heading</span><span class='val' id='vhdg'>-</span></div>"
"<div class='row'><span class='lbl'>HDOP</span><span class='val' id='vhdop'>-</span></div>"
"<div class='row'><span class='lbl'>Speed Water</span><span class='val' id='vspw'>-</span></div>"
"<div class='row'><span class='lbl'>Depth</span><span class='val' id='vdep'>-</span></div>"
"<div class='row'><span class='lbl'>Water Temp</span><span class='val' id='vwt'>-</span></div>"
"</div>"
/* Compass */
"<div class='card gps'><h2>Compass</h2>"
"<canvas id='compass' width='160' height='160'></canvas>"
"</div>"
/* Engine */
"<div class='card engine'><h2>Engine - Sei Whale SELVA</h2>"
"<div class='row'><span class='lbl'>RPM</span><span class='val big' id='vrpm'>-</span></div>"
"<div class='row'><span class='lbl'>Coolant Temp</span><span class='val' id='vcool'>-</span></div>"
"<div class='row'><span class='lbl'>Oil Pressure</span><span class='val' id='voilp'>-</span></div>"
"<div class='row'><span class='lbl'>Oil Temp</span><span class='val' id='voilt'>-</span></div>"
"<div class='row'><span class='lbl'>Battery</span><span class='val' id='vbatt'>-</span></div>"
"<div class='row'><span class='lbl'>Fuel Level</span><span class='val' id='vfuel'>-</span></div>"
"</div>"
/* Engine alerts */
"<div class='card engine'><h2>Engine Alerts</h2>"
"<div class='row'><span class='lbl'>Check Engine</span><span class='val' id='vchk'>-</span></div>"
"<div class='row'><span class='lbl'>Over Temp</span><span class='val' id='vovt'>-</span></div>"
"<div class='row'><span class='lbl'>Low Oil</span><span class='val' id='vloi'>-</span></div>"
"<div class='row'><span class='lbl'>Low Voltage</span><span class='val' id='vlvo'>-</span></div>"
"</div>"
/* AIS */
"<div class='card ais' style='grid-column:1/-1'><h2>AIS Targets</h2>"
"<div id='ais-list'><span style='color:#475569;font-size:.8rem'>No AIS targets</span></div>"
"</div>"
"</div>"
"<footer>ESP32-S3 - NMEA2000 250kbps - GPS + Engine SELVA + AIS em-trak - <span id='ts'>-</span></footer>"
"<script>"
"var cv=document.getElementById('compass'),ctx=cv.getContext('2d');"
"function drawCompass(h){"
"var cx=80,cy=80,r=74;"
"ctx.clearRect(0,0,160,160);"
"ctx.beginPath();ctx.arc(cx,cy,r,0,6.283);ctx.fillStyle='#060f1e';ctx.fill();"
"ctx.strokeStyle='#1e3a5f';ctx.lineWidth=2;ctx.stroke();"
"var cards=['N','E','S','W'],cols=['#ef4444','#94a3b8','#94a3b8','#94a3b8'];"
"cards.forEach(function(c,i){"
"var a=(i*90-90)*Math.PI/180;"
"ctx.fillStyle=cols[i];ctx.font='bold 12px Segoe UI';"
"ctx.textAlign='center';ctx.textBaseline='middle';"
"ctx.fillText(c,cx+(r-16)*Math.cos(a),cy+(r-16)*Math.sin(a));});"
"var a=(h-90)*Math.PI/180;"
"ctx.beginPath();"
"ctx.moveTo(cx+r*0.58*Math.cos(a),cy+r*0.58*Math.sin(a));"
"ctx.lineTo(cx+r*0.18*Math.cos(a+2.7),cy+r*0.18*Math.sin(a+2.7));"
"ctx.lineTo(cx+r*0.18*Math.cos(a-2.7),cy+r*0.18*Math.sin(a-2.7));"
"ctx.closePath();ctx.fillStyle='#22c55e';ctx.fill();"
"ctx.beginPath();ctx.arc(cx,cy,5,0,6.283);ctx.fillStyle='#e0e8f0';ctx.fill();"
"ctx.fillStyle='#e0e8f0';ctx.font='bold 13px Segoe UI';"
"ctx.textAlign='center';ctx.fillText(Math.round(h)+'deg',cx,cy+r-14);}"
"drawCompass(0);"
"function sv(id,v,cls){var e=document.getElementById(id);if(!e)return;"
"e.textContent=(v!==null&&v!==undefined)?v:'-';"
"if(cls)e.className='val '+cls;}"
"function alert_val(id,v){"
"var e=document.getElementById(id);if(!e)return;"
"e.textContent=v?'YES':'OK';"
"e.className='val '+(v?'warn':'ok');}"
"var ws,rt;"
"function conn(){"
"ws=new WebSocket('ws://'+location.hostname+'/ws');"
"ws.onopen=function(){document.getElementById('dot').classList.add('ok');sv('slbl','Connected');clearTimeout(rt);};"
"ws.onclose=function(){document.getElementById('dot').classList.remove('ok');sv('slbl','Reconnecting...');rt=setTimeout(conn,2000);};"
"ws.onmessage=function(e){try{"
"var d=JSON.parse(e.data),g=d.gps,en=d.engine,ais=d.ais;"
/* GPS */
"var fe=document.getElementById('vfix');"
"if(fe){fe.textContent=g.fix?'YES':'NO';fe.className='val '+(g.fix?'ok':'no');}"
"sv('vlat',g.lat?g.lat.toFixed(6)+'deg':null);"
"sv('vlon',g.lon?g.lon.toFixed(6)+'deg':null);"
"sv('valt',g.alt!=null?g.alt.toFixed(1)+' m':null);"
"sv('vsvs',g.svs);"
"sv('vutc',g.utc);"
"sv('vsog',g.sog!=null?g.sog.toFixed(2)+' kn':null);"
"sv('vcog',g.cog!=null?g.cog.toFixed(1)+'deg':null);"
"sv('vhdg',g.hdg!=null?g.hdg.toFixed(1)+'deg':null);"
"sv('vhdop',g.hdop!=null?g.hdop.toFixed(2):null);"
"sv('vspw',en.speed_water!=null&&en.speed_water>0?en.speed_water.toFixed(2)+' kn':null);"
"sv('vdep',en.depth!=null&&en.depth>0?en.depth.toFixed(2)+' m':null);"
"sv('vwt',en.water_temp!=null&&en.water_temp>-100?en.water_temp.toFixed(1)+' C':null);"
"var h=(g.hdg!=null)?g.hdg:(g.cog!=null?g.cog:0);drawCompass(h);"
/* Engine */
"sv('vrpm',en.rpm!=null&&en.rpm>0?Math.round(en.rpm)+' RPM':null);"
"sv('vcool',en.coolant!=null&&en.coolant>-100?en.coolant.toFixed(1)+' C':null);"
"sv('voilp',en.oil_pres!=null&&en.oil_pres>0?en.oil_pres.toFixed(2)+' bar':null);"
"sv('voilt',en.oil_temp!=null&&en.oil_temp>-100?en.oil_temp.toFixed(1)+' C':null);"
"sv('vbatt',en.battery!=null&&en.battery>0?en.battery.toFixed(2)+' V':null);"
"sv('vfuel',en.fuel_pct!=null&&en.fuel_pct>=0?en.fuel_pct.toFixed(1)+'%':null);"
"alert_val('vchk',en.check_engine);"
"alert_val('vovt',en.over_temp);"
"alert_val('vloi',en.low_oil);"
"alert_val('vlvo',en.low_volt);"
/* AIS */
"var al=document.getElementById('ais-list');"
"if(ais&&ais.length>0){"
"al.innerHTML=ais.map(function(t){"
"return '<div class=\\'ais-target\\'>"
"<div class=\\'ais-mmsi\\'>'+t.mmsi+(t.name?' - <span class=\\'ais-name\\'>'+t.name+'</span>':'')+'</div>"
"<div>Pos: '+t.lat.toFixed(4)+'N '+t.lon.toFixed(4)+'E | "
"SOG: '+t.sog.toFixed(1)+'kn COG: '+t.cog.toFixed(0)+'deg | "
"'+t.status+'</div>"
"</div>';"
"}).join('');"
"}else{"
"al.innerHTML='<span style=\\'color:#475569;font-size:.8rem\\'>No AIS targets</span>';}"
"document.getElementById('ts').textContent=new Date().toLocaleTimeString();"
"}catch(e){}};"
"}conn();"
"</script></body></html>";

/* ─── HTTP + WebSocket handlers ──────────────────────────────────────────── */
static esp_err_t handle_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_ws(httpd_req_t *req) {
    if (req->method == HTTP_GET) { ws_add(httpd_req_to_sockfd(req)); return ESP_OK; }
    httpd_ws_frame_t frame={.type=HTTPD_WS_TYPE_TEXT};
    uint8_t buf[32]={0}; frame.payload=buf;
    esp_err_t err=httpd_ws_recv_frame(req,&frame,sizeof(buf)-1);
    if (err!=ESP_OK) { int fd=httpd_req_to_sockfd(req);
        xSemaphoreTake(g_ws_mutex,portMAX_DELAY);
        for(int i=0;i<MAX_WS_CLIENTS;i++) if(g_ws_fds[i]==fd){g_ws_fds[i]=-1;break;}
        xSemaphoreGive(g_ws_mutex); }
    return err;
}

static void http_start(void) {
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.server_port=80; cfg.max_open_sockets=7;
    ESP_ERROR_CHECK(httpd_start(&g_server,&cfg));
    httpd_uri_t root={.uri="/",   .method=HTTP_GET,.handler=handle_root};
    httpd_uri_t wsep={.uri="/ws", .method=HTTP_GET,.handler=handle_ws,.is_websocket=true};
    httpd_register_uri_handler(g_server,&root);
    httpd_register_uri_handler(g_server,&wsep);
    ESP_LOGI(TAG,"HTTP + WebSocket on port 80");
}

/* ─── Wi-Fi STATION (rejoint le LAN du DWR-960) ──────────────────────────── */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *) event_data;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi deconnecte du DWR-960 (raison=%d), nouvelle tentative dans %dms",
                 event->reason, WIFI_RECONNECT_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Connecte au DWR-960 - Dashboard disponible sur http://" IPSTR,
                 IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_sta_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));

    wifi_config_t sta = {
        .sta = {
            .ssid     = GSM_ROUTER_SSID,
            .password = GSM_ROUTER_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Adresse MAC Wi-Fi ESP32 (a saisir dans la reservation DHCP "
                  "du DWR-960): %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "Connexion au LAN du DWR-960 (SSID: %s)...", GSM_ROUTER_SSID);

    /* Attend la connexion (30s max) avant de continuer, pour logguer l'IP.
     * Le serveur HTTP demarre de toute facon meme si ce delai expire :
     * la reconnexion continuera en tache de fond via wifi_event_handler. */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connecte au DWR-960.");
    } else {
        ESP_LOGW(TAG, "Pas encore connecte au DWR-960 apres 30s, "
                      "la reconnexion continue en arriere-plan.");
    }
}

/* ─── FROST-Server (SensorThings API) ────────────────────────────────────── */
/* Publie GPS + moteur chaque seconde vers un serveur FROST-Server via HTTPS,
 * en utilisant l'extension dataArray / CreateObservations (un seul POST
 * regroupant toutes les grandeurs, plus economique sur une liaison WAN
 * cellulaire qu'une requete HTTP par grandeur). */
/* MODE TEST LOCAL : pointe vers ton PC (meme reseau Wi-Fi DWR-960 que
 * l'ESP32). Remplace 10.5.159.131 par l'IP de ton PC ("ipconfig" sous
 * Windows). Pour la production plus tard, remets l'URL HTTPS ci-dessous. */
#define FROST_BASE_URL   "https://nmea2k.obsea.es/FROST-Server/v1.1"
#define FROST_PERIOD_MS  1000

/* Identifiants (@iot.id) des Datastreams crees par bootstrap_frost.py.
 * L'ORDRE DOIT CORRESPONDRE exactement a l'ordre des valeurs dans
 * frost_publish_task() ci-dessous (et a la liste PARAMETERS du script). */
static const int FROST_DATASTREAM_IDS[] = {
    1,  /* Latitude            */
    2,  /* Longitude           */
    3,  /* SpeedOverGround     */
    4,  /* CourseOverGround    */
    5,  /* Heading             */
    6,  /* HDOP                */
    7,  /* EngineRPM           */
    8,  /* CoolantTemperature  */
    9,  /* OilPressure         */
    10, /* OilTemperature      */
    11, /* BatteryVoltage      */
    12, /* FuelLevel           */
    13, /* WaterDepth          */
    14, /* WaterTemperature    */
    15, /* SpeedThroughWater   */
};
#define FROST_NUM_DATASTREAMS (sizeof(FROST_DATASTREAM_IDS)/sizeof(FROST_DATASTREAM_IDS[0]))

/* Datastream distinct pour l'AIS : le nombre de cibles varie et chaque cible
 * a un MMSI different, donc on ne peut pas faire "un Datastream par valeur".
 * On publie a la place un seul Datastream dont le "result" est le tableau
 * JSON complet des cibles actives. Mettre a 0 pour desactiver l'envoi AIS. */
#define FROST_AIS_DATASTREAM_ID 16

static void frost_publish_task(void *arg)
{
    static char json[6144];
    static char iso_time[32];
    static char ais_json[1024];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FROST_PERIOD_MS));

        if (!g_time_synced) continue; /* pas encore d'heure GPS valide, on saute */

        time_t now = time(NULL);
        struct tm tm_utc;
        gmtime_r(&now, &tm_utc);
        strftime(iso_time, sizeof(iso_time), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        double values[FROST_NUM_DATASTREAMS] = {
            g_gps.lat, g_gps.lon, g_gps.sog_kn, g_gps.cog_deg,
            g_gps.hdg_deg, g_gps.hdop,
            g_engine.rpm, g_engine.coolant_temp_c, g_engine.oil_pressure_bar,
            g_engine.oil_temp_c, g_engine.battery_v, g_engine.fuel_level_pct,
            g_engine.water_depth_m, g_engine.water_temp_c, g_engine.speed_water_kn,
        };
        ais_array_json_locked(ais_json, sizeof(ais_json));
        xSemaphoreGive(g_mutex);

        int len = snprintf(json, sizeof(json), "[");
        for (size_t i = 0; i < FROST_NUM_DATASTREAMS; i++) {
            len += snprintf(json + len, sizeof(json) - len,
                "%s{\"Datastream\":{\"@iot.id\":%d},"
                "\"components\":[\"phenomenonTime\",\"result\"],"
                "\"dataArray\":[[\"%s\",%.6f]]}",
                (i > 0) ? "," : "", FROST_DATASTREAM_IDS[i], iso_time, values[i]);
        }
        /* Cibles AIS : le "result" est ici le tableau JSON complet, pas un nombre. */
        if (FROST_AIS_DATASTREAM_ID > 0) {
            len += snprintf(json + len, sizeof(json) - len,
                ",{\"Datastream\":{\"@iot.id\":%d},"
                "\"components\":[\"phenomenonTime\",\"result\"],"
                "\"dataArray\":[[\"%s\",%s]]}",
                FROST_AIS_DATASTREAM_ID, iso_time, ais_json);
        }
        snprintf(json + len, sizeof(json) - len, "]");

        esp_http_client_config_t cfg = {
            .url = FROST_BASE_URL "/CreateObservations",
            .method = HTTP_METHOD_POST,
            .crt_bundle_attach = esp_crt_bundle_attach, /* ignore en HTTP, utile quand on repassera en HTTPS */
            .timeout_ms = 4000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, json, strlen(json));

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            if (status < 200 || status >= 300)
                ESP_LOGW(TAG, "[FROST] POST HTTP %d", status);
        } else {
            ESP_LOGW(TAG, "[FROST] Echec POST: %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
    }
}

/* ─── Synchronisation horaire via NTP (secours si le GPS n'envoie pas ─────
 * PGN 129033, ou si sa reception echoue) ──────────────────────────────── */
static void time_sync_task(void *arg)
{
    /* Attend que le Wi-Fi soit connecte avant de lancer la synchro NTP */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    while (!g_time_synced) {
        ESP_LOGI(TAG, "Synchronisation de l'horloge via NTP...");
        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_netif_sntp_init(&config);

        if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000)) == ESP_OK) {
            g_time_synced = true;
            ESP_LOGI(TAG, "Horloge systeme synchronisee via NTP.");
        } else {
            ESP_LOGW(TAG, "Echec de synchronisation NTP, nouvelle tentative dans 30s...");
            esp_netif_sntp_deinit();
            vTaskDelay(pdMS_TO_TICKS(30000));
        }
    }
    vTaskDelete(NULL);
}

/* ─── Pool RX CAN ────────────────────────────────────────────────────────── */
typedef struct { twai_frame_t frame; uint8_t data[8]; } rx_item_t;
static rx_item_t         rx_pool[RX_POOL_DEPTH];
static SemaphoreHandle_t free_slots, pending_frames;
static int write_idx=0, read_idx=0;

static bool IRAM_ATTR on_rx(twai_node_handle_t h,
                             const twai_rx_done_event_data_t *e, void *ctx) {
    (void)e;(void)ctx;
    BaseType_t w=pdFALSE;
    if (xSemaphoreTakeFromISR(free_slots,&w)!=pdTRUE) return w;
    if (twai_node_receive_from_isr(h,&rx_pool[write_idx].frame)==ESP_OK) {
        write_idx=(write_idx+1)%RX_POOL_DEPTH;
        xSemaphoreGiveFromISR(pending_frames,&w); }
    return w;
}
static bool IRAM_ATTR on_err(twai_node_handle_t h,
                              const twai_error_event_data_t *e, void *ctx) {
    (void)h;(void)e;(void)ctx; return false;
}

/* ─── Broadcast task ─────────────────────────────────────────────────────── */
static void broadcast_task(void *arg) {
    static char json[2048];
    while (1) {
        build_json(json, sizeof(json));
        ws_broadcast(json);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ─── app_main ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== NMEA2000 Full Receiver - GPS + Engine + AIS ===");

    esp_err_t ret = nvs_flash_init();
    if (ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret=nvs_flash_init(); }
    ESP_ERROR_CHECK(ret);

    g_mutex    = xSemaphoreCreateMutex();
    g_ws_mutex = xSemaphoreCreateMutex();
    ws_init();
    memset(g_ais, 0, sizeof(g_ais));

    wifi_sta_init();
    http_start();
    xTaskCreate(broadcast_task, "broadcast", 8192, NULL, 5, NULL);
    xTaskCreate(frost_publish_task, "frost_pub", 12288, NULL, 4, NULL);
    xTaskCreate(time_sync_task, "time_sync", 4096, NULL, 4, NULL);

    free_slots     = xSemaphoreCreateCounting(RX_POOL_DEPTH, RX_POOL_DEPTH);
    pending_frames = xSemaphoreCreateCounting(RX_POOL_DEPTH, 0);
    for (int i=0;i<RX_POOL_DEPTH;i++) {
        rx_pool[i].frame.buffer=rx_pool[i].data;
        rx_pool[i].frame.buffer_len=8; }

    twai_onchip_node_config_t ncfg={
        .io_cfg={.tx=CAN_TX_GPIO,.rx=CAN_RX_GPIO,
                 .quanta_clk_out=GPIO_NUM_NC,.bus_off_indicator=GPIO_NUM_NC},
        .bit_timing={.bitrate=CAN_BITRATE},
        .tx_queue_depth=4,.flags={.enable_listen_only=false}};
    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&ncfg,&node));
    twai_mask_filter_config_t af={.id=0,.mask=0,.is_ext=true,.no_fd=true};
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node,0,&af));
    twai_event_callbacks_t cbs={.on_rx_done=on_rx,.on_error=on_err};
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node,&cbs,NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));

    ESP_LOGI(TAG,"TWAI TX=GPIO%d RX=GPIO%d @ %d bps",CAN_TX_GPIO,CAN_RX_GPIO,CAN_BITRATE);
    ESP_LOGI(TAG,"Dashboard accessible depuis n'importe quel appareil sur le LAN du DWR-960 "
                 "(voir l'IP loguee ci-dessus, ou consultez la liste des clients DHCP "
                 "dans l'interface d'admin du DWR-960).");

    while (1) {
        if (xSemaphoreTake(pending_frames, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        rx_item_t *item = &rx_pool[read_idx];
        read_idx=(read_idx+1)%RX_POOL_DEPTH;

        uint8_t prio, src; uint32_t pgn;
        parse_can_id(item->frame.header.id, &prio, &pgn, &src);
        const uint8_t *raw=item->frame.buffer;
        uint8_t dlc=item->frame.header.dlc;
        xSemaphoreGive(free_slots);

        const uint8_t *payload=NULL; uint8_t plen=0;
        if (is_fast_packet(pgn)) {
            payload=fp_feed(src,pgn,raw,&plen);
            if (!payload) continue;
        } else { payload=raw; plen=dlc; }

        switch (pgn) {
        /* GPS */
        case PGN_POSITION_RAPID:  dec_position_rapid(payload,plen);        break;
        case PGN_COG_SOG:         dec_cog_sog(payload,plen);               break;
        case PGN_VESSEL_HEADING:  dec_heading(payload,plen);               break;
        case PGN_GNSS_DOPS:       dec_dops(payload,plen);                  break;
        case PGN_GNSS_POSITION:   dec_gnss_position(payload,plen);         break;
        case PGN_TIME_DATE:       dec_time_date(payload,plen);             break;
        /* Moteur */
        case PGN_ENGINE_RAPID:    dec_engine_rapid(payload,plen,src);      break;
        case PGN_ENGINE_DYNAMIC:  dec_engine_dynamic(payload,plen,src);    break;
        case PGN_FLUID_LEVEL:     dec_fluid_level(payload,plen,src);       break;
        case PGN_BATTERY_STATUS:  dec_battery(payload,plen,src);           break;
        case PGN_SPEED_WATER:     dec_speed_water(payload,plen,src);       break;
        case PGN_WATER_DEPTH:     dec_water_depth(payload,plen,src);       break;
        case PGN_ENVIRONMENT:     dec_environment(payload,plen,src);       break;
        /* AIS */
        case PGN_AIS_CLASS_A:     dec_ais_class_a(payload,plen,src);      break;
        case PGN_AIS_CLASS_B:     dec_ais_class_b(payload,plen,src);      break;
        case PGN_AIS_STATIC_A:    dec_ais_static_a(payload,plen,src);     break;
        case PGN_AIS_STATIC_B1:
        case PGN_AIS_STATIC_B2:   dec_ais_static_b(payload,plen,src,pgn); break;
        default:
            if (pgn == 127250)
                ESP_LOGE(TAG, "!!! HDG PGN 127250 recu dans default src=0x%02X", src);
            else
                ESP_LOGD(TAG, "RX PGN=%"PRIu32" src=0x%02X", pgn, src);
            ESP_LOGD(TAG,"RX PGN=%"PRIu32" src=0x%02X len=%d",pgn,src,plen);
            break;
        }
    }
}
