/*
 * NMEA2000 Sender Complet - GPS + AIS + Moteur
 * Simule le SPO25F (GPS), em-trak B921 (AIS) et Sei Whale SELVA (moteur)
 *
 * ESP32-S3 + SN65HVD230 | TWAI 250 kbit/s
 * GPIO5 = TX CAN
 * GPIO4 = RX CAN
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ─── Configuration ─────────────────────────────────────────────────────── */
#define CAN_TX_GPIO  GPIO_NUM_5
#define CAN_RX_GPIO  GPIO_NUM_4
#define CAN_BITRATE  250000

/* Adresses source */
#define SRC_GPS      0x15U   /* SPO25F */
#define SRC_AIS      0x23U   /* em-trak B921 */
#define SRC_ENGINE   0x31U   /* DLC-Plus SELVA */

static const char *TAG = "SENDER_FULL";

/* ─── PGNs ───────────────────────────────────────────────────────────────── */
/* GPS */
#define PGN_POSITION_RAPID   129025U
#define PGN_COG_SOG          129026U
#define PGN_VESSEL_HEADING   127250U
#define PGN_GNSS_POSITION    129029U
#define PGN_GNSS_DOPS        129539U
#define PGN_TIME_DATE        129033U
#define PGN_SYSTEM_TIME      126992U
#define PGN_ADDRESS_CLAIM     60928U
/* Moteur */
#define PGN_ENGINE_RAPID     127488U
#define PGN_ENGINE_DYNAMIC   127489U
#define PGN_FLUID_LEVEL      127505U
#define PGN_BATTERY_STATUS   127508U
#define PGN_SPEED_WATER      128259U
#define PGN_WATER_DEPTH      128267U
#define PGN_ENVIRONMENT      130310U
/* AIS */
#define PGN_AIS_CLASS_B      129039U
#define PGN_AIS_STATIC_B1    129809U
#define PGN_AIS_STATIC_B2    129810U

/* ─── CAN ID builder ─────────────────────────────────────────────────────── */
static uint32_t build_can_id(uint8_t prio, uint32_t pgn, uint8_t src)
{
    uint8_t  pf      = (pgn >> 8) & 0xFF;
    uint32_t can_pgn = (pf < 0xF0) ? (pgn & 0x1FF00U) : (pgn & 0x1FFFFU);
    uint32_t id      = ((uint32_t)(prio & 0x07)) << 26;
    id |= (can_pgn & 0x3FFFFU) << 8;
    id |= src;
    return id;
}

static void send_frame(twai_node_handle_t node, uint32_t pgn, uint8_t prio,
                        uint8_t src, const uint8_t *data, uint8_t len)
{
    static uint8_t tx_bufs[4][8];
    static uint8_t slot = 0;
    uint8_t *buf = tx_bufs[slot % 4];
    slot++;

    memset(buf, 0xFF, 8);
    if (len > 8) len = 8;
    memcpy(buf, data, len);

    twai_frame_t frame = {
        .header = { .id=build_can_id(prio,pgn,src),
                    .ide=1,.rtr=0,.fdf=0,.dlc=8 },
        .buffer=buf, .buffer_len=8,
    };
    twai_node_transmit(node, &frame, pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "TX PGN=%-6"PRIu32" src=0x%02X %02X%02X%02X%02X%02X%02X%02X%02X",
             pgn, src, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
}

static void send_fast_packet(twai_node_handle_t node, uint32_t pgn,
                              uint8_t prio, uint8_t src,
                              const uint8_t *payload, uint8_t total, uint8_t seq_id)
{
    uint32_t can_id    = build_can_id(prio, pgn, src);
    uint8_t  frame_num = 0, offset = 0;
    static uint8_t tx_pool[8][8];  /* 8 slots rotatifs */
    static uint8_t pool_idx = 0;

    while (frame_num == 0 || offset < total) {
        uint8_t *tx_buf = tx_pool[pool_idx % 8];
        pool_idx++;
        memset(tx_buf, 0xFF, 8);
        tx_buf[0] = ((seq_id & 0x07) << 5) | (frame_num & 0x1F);
        if (frame_num == 0) {
            tx_buf[1] = total;
            uint8_t n = total < 6 ? total : 6;
            memcpy(&tx_buf[2], &payload[offset], n);
            offset += n;
        } else {
            uint8_t remain = total - offset;
            uint8_t n = remain < 7 ? remain : 7;
            memcpy(&tx_buf[1], &payload[offset], n);
            offset += n;
        }
        twai_frame_t frame = {
            .header={.id=can_id,.ide=1,.rtr=0,.fdf=0,.dlc=8},
            .buffer=tx_buf,.buffer_len=8,
        };
        twai_node_transmit(node, &frame, pdMS_TO_TICKS(50));
        frame_num++;
        vTaskDelay(pdMS_TO_TICKS(5));
        if (offset >= total) break;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * GPS ENCODERS
 * ══════════════════════════════════════════════════════════════════════════*/

typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, hdg_deg;
    float   hdop, pdop, tdop;
    uint8_t num_svs, sid;
    uint32_t tick;
} gps_sim_t;

static void gps_update(gps_sim_t *g)
{
    float t  = g->tick * 0.1f;
    float R  = 500.0f;
    float om = 2.0f * (float)M_PI / 120.0f;
    g->lat     = 41.3809 + (R * cosf(om*t)) / 111320.0;
    g->lon     = 2.1734  + (R * sinf(om*t)) / (111320.0f * cosf(41.38f*(float)M_PI/180.0f));
    g->cog_deg = fmodf(om*t*180.0f/(float)M_PI + 90.0f, 360.0f);
    g->hdg_deg = g->cog_deg;
    g->sid     = (g->sid + 1) & 0xFF;
    g->tick++;
}

static void enc_position_rapid(uint8_t *b, double lat, double lon)
{
    int32_t la=(int32_t)(lat/1e-7), lo=(int32_t)(lon/1e-7);
    b[0]=la&0xFF;b[1]=(la>>8)&0xFF;b[2]=(la>>16)&0xFF;b[3]=(la>>24)&0xFF;
    b[4]=lo&0xFF;b[5]=(lo>>8)&0xFF;b[6]=(lo>>16)&0xFF;b[7]=(lo>>24)&0xFF;
}

static void enc_cog_sog(uint8_t *b, uint8_t sid, float cog, float sog)
{
    uint16_t c=(uint16_t)(cog*(float)M_PI/180.0f/1e-4f);
    uint16_t s=(uint16_t)(sog*0.514444f/1e-4f);
    b[0]=sid;b[1]=0;b[2]=c&0xFF;b[3]=(c>>8)&0xFF;
    b[4]=s&0xFF;b[5]=(s>>8)&0xFF;b[6]=0xFF;b[7]=0xFF;
}

static void enc_heading(uint8_t *b, uint8_t sid, float hdg)
{
    uint16_t h=(uint16_t)(hdg*(float)M_PI/180.0f/1e-4f);
    b[0]=sid;b[1]=1;b[2]=h&0xFF;b[3]=(h>>8)&0xFF;
    b[4]=0xFF;b[5]=0xFF;b[6]=0xFF;b[7]=0x00;
}

static void enc_dops(uint8_t *b, uint8_t sid, float hdop, float pdop, float tdop)
{
    int16_t h=(int16_t)(hdop/0.01f),p=(int16_t)(pdop/0.01f),t=(int16_t)(tdop/0.01f);
    b[0]=sid;b[1]=(3&7)|((3&7)<<3);
    b[2]=h&0xFF;b[3]=(h>>8)&0xFF;b[4]=p&0xFF;b[5]=(p>>8)&0xFF;
    b[6]=t&0xFF;b[7]=(t>>8)&0xFF;
}

static uint8_t enc_gnss_position(uint8_t *b, const gps_sim_t *g,
                                  uint16_t days, uint64_t sx)
{
    int64_t la=(int64_t)(g->lat/1e-7), lo=(int64_t)(g->lon/1e-7);
    int32_t al=(int32_t)(g->alt/1e-6);
    int16_t hd=(int16_t)(g->hdop/0.01f), pd=(int16_t)(g->pdop/0.01f), ge=4800;
    uint8_t i=0;
    b[i++]=g->sid;
    b[i++]=days&0xFF;b[i++]=(days>>8)&0xFF;
    for(int j=0;j<8;j++) b[i++]=(uint8_t)((sx>>(8*j))&0xFF);
    for(int j=0;j<8;j++) b[i++]=(uint8_t)((la>>(8*j))&0xFF);
    for(int j=0;j<8;j++) b[i++]=(uint8_t)((lo>>(8*j))&0xFF);
    for(int j=0;j<4;j++) b[i++]=(uint8_t)((al>>(8*j))&0xFF);
    b[i++]=(1<<4);b[i++]=0;b[i++]=g->num_svs;
    b[i++]=hd&0xFF;b[i++]=(hd>>8)&0xFF;
    b[i++]=pd&0xFF;b[i++]=(pd>>8)&0xFF;
    b[i++]=ge&0xFF;b[i++]=(ge>>8)&0xFF;
    b[i++]=0;b[i++]=0xFF;
    return i;
}

static uint8_t enc_time_date(uint8_t *b, uint16_t days, uint32_t secs)
{
    uint64_t sx=(uint64_t)secs*10000ULL;
    b[0]=days&0xFF;b[1]=(days>>8)&0xFF;
    for(int i=0;i<8;i++) b[i+2]=(uint8_t)((sx>>(8*i))&0xFF);
    return 10;
}

static void enc_system_time(uint8_t *b, uint8_t sid, uint16_t days, uint32_t secs)
{
    uint64_t sx=(uint64_t)secs*10000ULL;
    b[0]=sid;b[1]=0xF0;b[2]=days&0xFF;b[3]=(days>>8)&0xFF;
    b[4]=(uint8_t)(sx&0xFF);b[5]=(uint8_t)((sx>>8)&0xFF);
    b[6]=(uint8_t)((sx>>16)&0xFF);b[7]=(uint8_t)((sx>>24)&0xFF);
}

static void enc_address_claim(uint8_t *b, uint32_t unique, uint16_t manuf)
{
    uint64_t name=0;
    name|=(uint64_t)(unique&0x1FFFFF);
    name|=(uint64_t)(manuf &0x07FF)<<21;
    name|=(uint64_t)(145   &0xFF)  <<40;
    name|=(uint64_t)(60    &0x7F)  <<48;
    name|=(uint64_t)(4     &0x07)  <<60;
    name|=(uint64_t)(1     &0x01)  <<63;
    for(int i=0;i<8;i++) b[i]=(uint8_t)((name>>(8*i))&0xFF);
}

/* ════════════════════════════════════════════════════════════════════════════
 * ENGINE ENCODERS
 * ══════════════════════════════════════════════════════════════════════════*/

typedef struct {
    float   rpm;
    float   coolant_c, oil_pres_bar, oil_temp_c;
    float   battery_v, fuel_pct;
    float   depth_m, water_temp_c, speed_water_kn;
    uint8_t sid, tick;
} engine_sim_t;

static void engine_update(engine_sim_t *e, float t_s)
{
    /* RPM oscille entre 800 et 3500 */
    e->rpm         = 2150.0f + 1350.0f * sinf(t_s * 0.05f);
    e->coolant_c   = 82.0f  + 8.0f   * sinf(t_s * 0.02f);
    e->oil_pres_bar= 3.5f   + 0.5f   * sinf(t_s * 0.03f);
    e->oil_temp_c  = 90.0f  + 5.0f   * sinf(t_s * 0.02f);
    e->battery_v   = 13.8f  + 0.3f   * sinf(t_s * 0.1f);
    e->fuel_pct    = 75.0f  - t_s * 0.001f;  /* consommation lente */
    e->depth_m     = 12.5f  + 2.0f   * sinf(t_s * 0.04f);
    e->water_temp_c= 18.5f  + 1.0f   * sinf(t_s * 0.01f);
    e->speed_water_kn = 6.2f + 0.8f  * sinf(t_s * 0.05f);
    e->sid = (e->sid + 1) & 0xFF;
}

/* PGN 127488 – Engine Rapid (RPM) */
static void enc_engine_rapid(uint8_t *b, uint8_t sid, float rpm)
{
    uint16_t r = (uint16_t)(rpm / 0.25f);
    b[0]=r&0xFF;b[1]=(r>>8)&0xFF;
    b[2]=0xFF;b[3]=0xFF;  /* boost press */
    b[4]=0x00;            /* tilt/trim */
    b[5]=0xFF;b[6]=0xFF;b[7]=sid;
}

/* PGN 127489 – Engine Dynamic */
static void enc_engine_dynamic(uint8_t *b, uint8_t sid,
                                float oil_pres, float oil_temp, float cool_temp,
                                uint8_t check_engine)
{
    /* oil pressure : Pa (0.1 hPa/bit) -> hPa */
    uint16_t op = (uint16_t)(oil_pres * 1e5f / 100.0f);
    /* temperatures : K * 100 */
    uint16_t ot = (uint16_t)((oil_temp  + 273.15f) * 100.0f);
    uint16_t ct = (uint16_t)((cool_temp + 273.15f) * 100.0f);
    uint16_t status = check_engine ? 0x0001 : 0x0000;
    b[0]=op&0xFF;b[1]=(op>>8)&0xFF;
    b[2]=ot&0xFF;b[3]=(ot>>8)&0xFF;
    b[4]=ct&0xFF;b[5]=(ct>>8)&0xFF;
    b[6]=status&0xFF;b[7]=(status>>8)&0xFF;
}

/* PGN 127505 – Fluid Level */
static void enc_fluid_level(uint8_t *b, uint8_t sid, uint8_t fluid_type, float pct)
{
    uint16_t level = (uint16_t)(pct / 0.004f);
    b[0]=(sid&0xF0)|fluid_type;
    b[1]=0xFF;  /* capacity high byte */
    b[2]=level&0xFF;b[3]=(level>>8)&0xFF;
    b[4]=0xFF;b[5]=0xFF;b[6]=0xFF;b[7]=0xFF;
}

/* PGN 127508 – Battery Status */
static void enc_battery(uint8_t *b, uint8_t sid, float volt)
{
    uint16_t v = (uint16_t)(volt / 0.01f);
    b[0]=v&0xFF;b[1]=(v>>8)&0xFF;
    b[2]=0xFF;b[3]=0xFF;  /* current */
    b[4]=0xFF;b[5]=0xFF;  /* temperature */
    b[6]=sid;b[7]=0xFF;
}

/* PGN 128259 – Speed Water */
static void enc_speed_water(uint8_t *b, uint8_t sid, float spd_kn)
{
    uint16_t sw = (uint16_t)(spd_kn * 0.514444f / 0.01f);
    b[0]=sid;b[1]=sw&0xFF;b[2]=(sw>>8)&0xFF;
    b[3]=sw&0xFF;b[4]=(sw>>8)&0xFF;
    b[5]=0xFF;b[6]=0xFF;b[7]=0xFF;
}

/* PGN 128267 – Water Depth */
static void enc_water_depth(uint8_t *b, uint8_t sid, float depth_m)
{
    uint32_t d = (uint32_t)(depth_m / 0.01f);
    b[0]=sid;
    b[1]=d&0xFF;b[2]=(d>>8)&0xFF;b[3]=(d>>16)&0xFF;b[4]=(d>>24)&0xFF;
    b[5]=0x00;b[6]=0xFD;b[7]=0xFF;
}

/* PGN 130310 – Environmental (water temp) */
static void enc_environment(uint8_t *b, uint8_t sid, float water_temp_c)
{
    uint16_t wt = (uint16_t)((water_temp_c + 273.15f) * 100.0f);
    b[0]=sid;b[1]=0x00;
    b[2]=wt&0xFF;b[3]=(wt>>8)&0xFF;
    b[4]=0xFF;b[5]=0xFF;b[6]=0xFF;b[7]=0xFF;
}

/* ════════════════════════════════════════════════════════════════════════════
 * AIS ENCODERS
 * ══════════════════════════════════════════════════════════════════════════*/

/* Navires simulés autour de Barcelone */
typedef struct {
    uint32_t mmsi;
    char     name[21];
    char     callsign[8];
    double   lat, lon;
    float    sog_kn, cog_deg;
    uint8_t  nav_status;
} ais_vessel_t;

static ais_vessel_t ais_vessels[] = {
    { 211234567, "COSTA BRAVA",  "ECBV",  41.3600, 2.1800, 12.5f, 45.0f,  0 },
    { 224123456, "NEPTUNO",      "ENPT",  41.4100, 2.2100,  8.0f, 200.0f, 0 },
    { 247987654, "MEDITERRANE",  "IMDX",  41.3200, 2.1200, 15.2f, 120.0f, 0 },
    { 338765432, "ATLANTIC SUN", "WATS",  41.3900, 2.1500,  0.0f,   0.0f, 1 },
};
#define N_VESSELS (sizeof(ais_vessels)/sizeof(ais_vessels[0]))

static void ais_update(ais_vessel_t *v, float dt_s)
{
    if (v->nav_status == 1) return; /* mouille */
    float speed_ms = v->sog_kn * 0.514444f;
    float rad      = v->cog_deg * (float)M_PI / 180.0f;
    v->lat += (speed_ms * cosf(rad) * dt_s) / 111320.0;
    v->lon += (speed_ms * sinf(rad) * dt_s) / (111320.0f * cosf(v->lat*(float)M_PI/180.0f));
}

/* Ecrire N bits dans buffer AIS (little-endian) */
static void ais_set_bits(uint8_t *d, int start, int len, uint32_t val)
{
    uint32_t mask = (len < 32) ? ((1U << len) - 1) : 0xFFFFFFFF;
    val &= mask;
    for (int i = 0; i < len; i++) {
        int bit = start + i;
        if ((val >> i) & 1) d[bit/8] |=  (1 << (bit%8));
        else                d[bit/8] &= ~(1 << (bit%8));
    }
}

/* Ecrire string AIS (6 bits par caractere) */
static void ais_set_str(uint8_t *d, int start, int max_chars, const char *s)
{
    for (int i = 0; i < max_chars; i++) {
        char c = (i < (int)strlen(s)) ? s[i] : '@';
        if (c >= 'a' && c <= 'z') c -= 32;
        uint8_t val = (c >= 64) ? c - 64 : c;
        ais_set_bits(d, start + i*6, 6, val);
    }
}

/* PGN 129039 – AIS Class B Position (fast-packet ~28 bytes) */
static uint8_t enc_ais_class_b(uint8_t *b, const ais_vessel_t *v)
{
    memset(b, 0, 28);
    /* msg_type = 18 (Class B) */
    ais_set_bits(b, 0,  6,  18);
    /* repeat = 0 */
    ais_set_bits(b, 6,  2,  0);
    /* MMSI */
    ais_set_bits(b, 8,  30, v->mmsi);
    /* SOG (0.1 kn) */
    ais_set_bits(b, 46, 10, (uint32_t)(v->sog_kn * 10.0f));
    /* Lon (1/10000 min) */
    int32_t lon_i = (int32_t)(v->lon * 600000.0);
    ais_set_bits(b, 57, 28, (uint32_t)lon_i);
    /* Lat (1/10000 min) */
    int32_t lat_i = (int32_t)(v->lat * 600000.0);
    ais_set_bits(b, 85, 27, (uint32_t)lat_i);
    /* COG (0.1 deg) */
    ais_set_bits(b, 112, 12, (uint32_t)(v->cog_deg * 10.0f));
    /* nav status */
    ais_set_bits(b, 148, 4, v->nav_status);
    return 28;
}

/* PGN 129809 – AIS Class B Static Part A (callsign, name) */
static uint8_t enc_ais_static_b1(uint8_t *b, const ais_vessel_t *v)
{
    memset(b, 0, 24);
    ais_set_bits(b, 0, 6, 24);       /* msg 24 part A */
    ais_set_bits(b, 6, 2, 0);
    ais_set_bits(b, 8, 30, v->mmsi);
    ais_set_bits(b, 38, 2, 0);       /* part number = 0 */
    ais_set_str(b, 40, 7, v->callsign);
    return 24;
}

/* PGN 129810 – AIS Class B Static Part B (name) */
static uint8_t enc_ais_static_b2(uint8_t *b, const ais_vessel_t *v)
{
    memset(b, 0, 37);
    ais_set_bits(b, 0, 6, 24);       /* msg 24 part B */
    ais_set_bits(b, 6, 2, 0);
    ais_set_bits(b, 8, 30, v->mmsi);
    ais_set_bits(b, 38, 2, 1);       /* part number = 1 */
    ais_set_str(b, 46, 20, v->name);
    return 37;
}

/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ══════════════════════════════════════════════════════════════════════════*/

void app_main(void)
{
    ESP_LOGI(TAG, "=== NMEA2000 Full Sender - GPS + Engine + AIS ===");

    twai_onchip_node_config_t ncfg = {
        .io_cfg = { .tx=CAN_TX_GPIO,.rx=CAN_RX_GPIO,
                    .quanta_clk_out=GPIO_NUM_NC,.bus_off_indicator=GPIO_NUM_NC },
        .bit_timing = { .bitrate=CAN_BITRATE },
        .tx_queue_depth = 32,
        .flags = { .enable_listen_only=false },
    };
    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&ncfg, &node));
    ESP_ERROR_CHECK(twai_node_enable(node));
    ESP_LOGI(TAG, "TWAI TX=GPIO%d RX=GPIO%d @ %d bps", CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);

    /* ── Address Claims au boot ── */
    uint8_t buf[256];
    enc_address_claim(buf, 0x1A2B3C, 0x0099);
    send_frame(node, PGN_ADDRESS_CLAIM, 6, SRC_GPS, buf, 8);
    vTaskDelay(pdMS_TO_TICKS(50));
    enc_address_claim(buf, 0x4D5E6F, 0x0101);
    send_frame(node, PGN_ADDRESS_CLAIM, 6, SRC_AIS, buf, 8);
    vTaskDelay(pdMS_TO_TICKS(50));
    enc_address_claim(buf, 0x7A8B9C, 0x0202);
    send_frame(node, PGN_ADDRESS_CLAIM, 6, SRC_ENGINE, buf, 8);
    vTaskDelay(pdMS_TO_TICKS(250));

    /* ── Simulation state ── */
    gps_sim_t    gps    = { .alt=2.0f,.sog_kn=6.5f,.hdop=1.1f,.pdop=1.5f,.tdop=1.0f,.num_svs=11 };
    engine_sim_t engine = { .fuel_pct=75.0f };
    uint8_t seq = 0;
    uint16_t days = 20000;
    uint32_t base_s = 12 * 3600;

    uint32_t t_rapid=0, t_cog=0,   t_hdg=0;
    uint32_t t_pos=0,   t_dop=0,   t_time=0;
    uint32_t t_eng_r=0, t_eng_d=0, t_fluid=0;
    uint32_t t_batt=0,  t_depth=0, t_env=0;
    uint32_t t_ais_pos=0, t_ais_stat=0;
    uint8_t  ais_stat_idx = 0;

    ESP_LOGI(TAG, "Simulation started — GPS + Engine + %d AIS vessels", (int)N_VESSELS);

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float t_s    = now / 1000.0f;

        gps_update(&gps);
        engine_update(&engine, t_s);

        uint32_t secs      = base_s + gps.tick / 10;
        uint64_t secs_x10k = (uint64_t)secs * 10000ULL;

        /* ── GPS 100 ms ── */
        if (now - t_rapid >= 100) {
            enc_position_rapid(buf, gps.lat, gps.lon);
            send_frame(node, PGN_POSITION_RAPID, 2, SRC_GPS, buf, 8);
            enc_heading(buf, gps.sid, gps.hdg_deg);
            send_frame(node, PGN_VESSEL_HEADING, 2, SRC_GPS, buf, 8);
            t_rapid = now;
        }

        /* ── GPS 250 ms ── */
        if (now - t_cog >= 250) {
            enc_cog_sog(buf, gps.sid, gps.cog_deg, gps.sog_kn);
            send_frame(node, PGN_COG_SOG, 2, SRC_GPS, buf, 8);
            t_cog = now;
        }

        /* ── GPS 1000 ms ── */
        if (now - t_pos >= 1000) {
            uint8_t n = enc_gnss_position(buf, &gps, days, secs_x10k);
            send_fast_packet(node, PGN_GNSS_POSITION, 3, SRC_GPS, buf, n, seq++);
            t_pos = now;
        }
        if (now - t_dop >= 1000) {
            enc_dops(buf, gps.sid, gps.hdop, gps.pdop, gps.tdop);
            send_frame(node, PGN_GNSS_DOPS, 6, SRC_GPS, buf, 8);
            t_dop = now;
        }
        if (now - t_time >= 1000) {
            uint8_t n = enc_time_date(buf, days, secs);
            send_fast_packet(node, PGN_TIME_DATE, 3, SRC_GPS, buf, n, seq++);
            enc_system_time(buf, gps.sid, days, secs);
            send_frame(node, PGN_SYSTEM_TIME, 3, SRC_GPS, buf, 8);
            t_time = now;
        }

        /* ── Engine 100 ms (RPM) ── */
        if (now - t_eng_r >= 100) {
            enc_engine_rapid(buf, engine.sid, engine.rpm);
            send_frame(node, PGN_ENGINE_RAPID, 2, SRC_ENGINE, buf, 8);
            t_eng_r = now;
        }

        /* ── Engine 500 ms (dynamic) ── */
        if (now - t_eng_d >= 500) {
            enc_engine_dynamic(buf, engine.sid,
                               engine.oil_pres_bar, engine.oil_temp_c,
                               engine.coolant_c, 0);
            send_frame(node, PGN_ENGINE_DYNAMIC, 6, SRC_ENGINE, buf, 8);
            t_eng_d = now;
        }

        /* ── Engine 1000 ms ── */
        if (now - t_fluid >= 1000) {
            enc_fluid_level(buf, engine.sid, 0, engine.fuel_pct); /* fuel */
            send_frame(node, PGN_FLUID_LEVEL, 6, SRC_ENGINE, buf, 8);
            t_fluid = now;
        }
        if (now - t_batt >= 1000) {
            enc_battery(buf, engine.sid, engine.battery_v);
            send_frame(node, PGN_BATTERY_STATUS, 6, SRC_ENGINE, buf, 8);
            t_batt = now;
        }
        if (now - t_depth >= 1000) {
            enc_water_depth(buf, engine.sid, engine.depth_m);
            send_frame(node, PGN_WATER_DEPTH, 6, SRC_ENGINE, buf, 8);
            t_depth = now;
        }
        if (now - t_env >= 1000) {
            enc_environment(buf, engine.sid, engine.water_temp_c);
            send_frame(node, PGN_ENVIRONMENT, 6, SRC_ENGINE, buf, 8);
            enc_speed_water(buf, engine.sid, engine.speed_water_kn);
            send_frame(node, PGN_SPEED_WATER, 2, SRC_ENGINE, buf, 8);
            t_env = now;
        }

        /* ── AIS positions toutes les 3 s ── */
        if (now - t_ais_pos >= 3000) {
            for (int i = 0; i < (int)N_VESSELS; i++) {
                ais_update(&ais_vessels[i], 3.0f);
                uint8_t ais_buf[28];
                uint8_t n = enc_ais_class_b(ais_buf, &ais_vessels[i]);
                send_fast_packet(node, PGN_AIS_CLASS_B, 4, SRC_AIS, ais_buf, n, seq++);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            t_ais_pos = now;
        }

        /* ── AIS static toutes les 30 s ── */
        if (now - t_ais_stat >= 30000) {
            ais_vessel_t *v = &ais_vessels[ais_stat_idx % N_VESSELS];
            uint8_t ais_buf1[24];
            uint8_t n1 = enc_ais_static_b1(ais_buf1, v);
            send_fast_packet(node, PGN_AIS_STATIC_B1, 6, SRC_AIS, ais_buf1, n1, seq++);
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t ais_buf2[37];
            uint8_t n2 = enc_ais_static_b2(ais_buf2, v);
            send_fast_packet(node, PGN_AIS_STATIC_B2, 6, SRC_AIS, ais_buf2, n2, seq++);
            ais_stat_idx++;
            t_ais_stat = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}