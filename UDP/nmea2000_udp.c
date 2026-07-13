/*
 * NMEA2000 Récepteur + Wi-Fi + UDP broadcast NMEA0183
 * ESP32-S3 + SN65HVD230  |  TWAI 250 kbit/s
 *
 * - Reçoit les trames NMEA2000 via CAN
 * - Décode les PGNs GPS
 * - Convertit en phrases NMEA0183
 * - Broadcast UDP sur port 10110 (standard NMEA réseau)
 *
 * GPIO5 = TX CAN
 * GPIO4 = RX CAN
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ─── Configuration ──────────────────────────────────────────────────────── */
#define CAN_TX_GPIO     GPIO_NUM_5
#define CAN_RX_GPIO     GPIO_NUM_4
#define CAN_BITRATE     250000
#define RX_POOL_DEPTH   64

#define WIFI_SSID       "SARTI"        /* ← modifier */
#define WIFI_PASSWORD   "123456" /* ← modifier */
#define WIFI_MAX_RETRY  10

#define UDP_PORT        10110               /* port NMEA standard */
#define UDP_BROADCAST   "255.255.255.255"

static const char *TAG = "NMEA_UDP";

/* ─── PGNs ───────────────────────────────────────────────────────────────── */
#define PGN_GNSS_POSITION    129029U
#define PGN_COG_SOG          129026U
#define PGN_VESSEL_HEADING   127250U
#define PGN_SPEED            128259U
#define PGN_TIME_DATE        129033U
#define PGN_GNSS_DOPS        129539U
#define PGN_ADDRESS_CLAIM     60928U

/* ─── Wi-Fi ──────────────────────────────────────────────────────────────── */
static EventGroupHandle_t wifi_events;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retry_count = 0;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGW(TAG, "Reconnexion Wi-Fi (%d/%d)...", retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_events, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP obtenue : " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h1, h2;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h1));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &h2));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connexion à %s...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connecté !");
        return true;
    }
    ESP_LOGE(TAG, "Connexion Wi-Fi échouée");
    return false;
}

/* ─── Socket UDP broadcast ───────────────────────────────────────────────── */
static int udp_sock = -1;
static struct sockaddr_in udp_dest;

static bool udp_init(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Erreur création socket UDP");
        return false;
    }
    int broadcast = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&udp_dest, 0, sizeof(udp_dest));
    udp_dest.sin_family      = AF_INET;
    udp_dest.sin_port        = htons(UDP_PORT);
    udp_dest.sin_addr.s_addr = inet_addr(UDP_BROADCAST);

    ESP_LOGI(TAG, "UDP broadcast prêt → %s:%d", UDP_BROADCAST, UDP_PORT);
    return true;
}

static void udp_send(const char *sentence)
{
    if (udp_sock < 0) return;
    sendto(udp_sock, sentence, strlen(sentence), 0,
           (struct sockaddr *)&udp_dest, sizeof(udp_dest));
    ESP_LOGD(TAG, "UDP TX: %s", sentence);
}

/* ─── Checksum NMEA0183 ──────────────────────────────────────────────────── */
static uint8_t nmea_checksum(const char *s)
{
    uint8_t cs = 0;
    while (*s) cs ^= (uint8_t)*s++;
    return cs;
}

/* ─── État GPS ───────────────────────────────────────────────────────────── */
typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, heading_deg;
    float   hdop, vdop;
    uint8_t num_svs;
    char    utc[16];     /* HHMMss.ss */
    char    date[8];     /* DDMMYY */
    uint8_t fix;
    uint8_t updated;     /* flags de mise à jour */
} gps_t;

static gps_t g = {
    .hdop = 99.9f, .vdop = 99.9f, .num_svs = 0,
};

/* ─── Conversion NMEA0183 ────────────────────────────────────────────────── */

/* Degrés décimaux → DDmm.mmmmm */
static void deg_to_nmea(double deg, char *buf, char *dir, char pos, char neg)
{
    if (deg < 0) { *dir = neg; deg = -deg; } else { *dir = pos; }
    int d = (int)deg;
    double m = (deg - d) * 60.0;
    sprintf(buf, "%02d%08.5f", d, m);
}

/* $GPGGA – Fix GPS */
static void send_gpgga(void)
{
    if (!g.fix) return;
    char lat_s[16], lon_s[16];
    char lat_d, lon_d;
    deg_to_nmea(g.lat, lat_s, &lat_d, 'N', 'S');
    deg_to_nmea(g.lon, lon_s, &lon_d, 'E', 'W');

    char body[128];
    snprintf(body, sizeof(body),
             "GPGGA,%s,%s,%c,%s,%c,1,%02d,%.2f,%.1f,M,0.0,M,,",
             g.utc, lat_s, lat_d, lon_s, lon_d,
             g.num_svs, g.hdop, g.alt);

    char sentence[160];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
             body, nmea_checksum(body));
    udp_send(sentence);
    ESP_LOGI(TAG, "TX GPGGA");
}

/* $GPRMC – Position et vitesse recommandées */
static void send_gprmc(void)
{
    if (!g.fix) return;
    char lat_s[16], lon_s[16];
    char lat_d, lon_d;
    deg_to_nmea(g.lat, lat_s, &lat_d, 'N', 'S');
    deg_to_nmea(g.lon, lon_s, &lon_d, 'E', 'W');

    char body[160];
    snprintf(body, sizeof(body),
             "GPRMC,%s,A,%s,%c,%s,%c,%.2f,%.2f,%s,,,A",
             g.utc, lat_s, lat_d, lon_s, lon_d,
             g.sog_kn, g.cog_deg, g.date);

    char sentence[200];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
             body, nmea_checksum(body));
    udp_send(sentence);
    ESP_LOGI(TAG, "TX GPRMC");
}

/* $GPVTG – Cap et vitesse */
static void send_gpvtg(void)
{
    char body[128];
    snprintf(body, sizeof(body),
             "GPVTG,%.2f,T,%.2f,M,%.2f,N,%.2f,K,A",
             g.cog_deg, g.cog_deg,
             g.sog_kn, g.sog_kn * 1.852f);

    char sentence[160];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
             body, nmea_checksum(body));
    udp_send(sentence);
}

/* $GPHDG – Cap magnétique */
static void send_gphdg(void)
{
    char body[64];
    snprintf(body, sizeof(body), "GPHDG,%.2f,,,,", g.heading_deg);
    char sentence[96];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
             body, nmea_checksum(body));
    udp_send(sentence);
}

/* $GPGSA – DOP et satellites actifs */
static void send_gpgsa(void)
{
    char body[128];
    snprintf(body, sizeof(body),
             "GPGSA,A,3,,,,,,,,,,,,,%.2f,%.2f,1.00",
             g.hdop, g.vdop);
    char sentence[160];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
             body, nmea_checksum(body));
    udp_send(sentence);
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

/* ─── Fast-Packet réassembleur ───────────────────────────────────────────── */
#define FP_MAX_STREAMS  4
#define FP_MAX_PAYLOAD  223

typedef struct {
    uint8_t  active;
    uint8_t  src;
    uint32_t pgn;
    uint8_t  seq_id;
    uint8_t  total;
    uint8_t  received;
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
        if (fp_streams[i].active &&
            fp_streams[i].src    == src &&
            fp_streams[i].pgn    == pgn &&
            fp_streams[i].seq_id == seq_id) {
            s = &fp_streams[i]; break;
        }
    }

    if (frame_num == 0) {
        for (int i = 0; i < FP_MAX_STREAMS; i++) {
            if (!fp_streams[i].active) { s = &fp_streams[i]; break; }
        }
        if (!s) s = &fp_streams[0];
        memset(s, 0, sizeof(*s));
        s->active = 1; s->src = src; s->pgn = pgn; s->seq_id = seq_id;
        s->total  = frame[1];
        uint8_t n = s->total < 6 ? s->total : 6;
        memcpy(s->buf, &frame[2], n);
        s->received = 1;
    } else {
        if (!s) return NULL;
        uint8_t offset = 6 + (frame_num - 1) * 7;
        uint8_t remain = s->total > offset ? s->total - offset : 0;
        uint8_t n = remain < 7 ? remain : 7;
        if (offset + n <= FP_MAX_PAYLOAD)
            memcpy(&s->buf[offset], &frame[1], n);
        s->received++;
    }

    uint8_t n_frames = (s->total <= 6) ? 1 : (1 + (s->total - 6 + 6) / 7);
    if (s->received >= n_frames) {
        *out_len = s->total;
        s->active = 0;
        return s->buf;
    }
    return NULL;
}

/* ─── Décodeurs PGN → mise à jour g ─────────────────────────────────────── */

static void decode_cog_sog(const uint8_t *d, uint8_t len)
{
    if (len < 6) return;
    uint16_t cog_i = d[2] | ((uint16_t)d[3] << 8);
    uint16_t sog_i = d[4] | ((uint16_t)d[5] << 8);
    g.cog_deg = cog_i * 1e-4f * 180.0f / M_PI;
    g.sog_kn  = sog_i * 1e-4f / 0.514444f;
}

static void decode_vessel_heading(const uint8_t *d, uint8_t len)
{
    if (len < 4) return;
    uint16_t hdg_i = d[2] | ((uint16_t)d[3] << 8);
    g.heading_deg  = hdg_i * 1e-4f * 180.0f / M_PI;
}

static void decode_gnss_dops(const uint8_t *d, uint8_t len)
{
    if (len < 8) return;
    g.hdop = (int16_t)(d[2] | ((uint16_t)d[3] << 8)) * 0.01f;
    g.vdop = (int16_t)(d[4] | ((uint16_t)d[5] << 8)) * 0.01f;
}

static void decode_gnss_position(const uint8_t *d, uint8_t len)
{
    if (len < 27) return;
    int64_t lat_i = 0, lon_i = 0;
    for (int i = 0; i < 8; i++) lat_i |= ((int64_t)d[11+i] << (8*i));
    for (int i = 0; i < 8; i++) lon_i |= ((int64_t)d[19+i] << (8*i));
    if (lat_i & ((int64_t)1 << 55)) lat_i |= ~(((int64_t)1 << 56) - 1);
    if (lon_i & ((int64_t)1 << 55)) lon_i |= ~(((int64_t)1 << 56) - 1);
    g.lat    = lat_i * 1e-7;
    g.lon    = lon_i * 1e-7;
    g.fix    = 1;
    g.num_svs = len > 28 ? d[28] : 8;
}

static void decode_time_date(const uint8_t *d, uint8_t len)
{
    if (len < 10) return;
    uint16_t days  = d[0] | ((uint16_t)d[1] << 8);
    uint64_t sx    = 0;
    for (int i = 0; i < 8; i++) sx |= ((uint64_t)d[2+i] << (8*i));
    uint32_t secs  = (uint32_t)(sx / 10000);
    uint8_t  h = secs / 3600, m = (secs % 3600) / 60, s = secs % 60;
    snprintf(g.utc,  sizeof(g.utc),  "%02u%02u%02u.00", h, m, s);

    /* Convertir days depuis 1970 en DDMMYY */
    /* Approximation simple */
    uint32_t epoch = days * 86400UL;
    struct tm t;
    time_t tt = (time_t)epoch;
    gmtime_r(&tt, &t);
    snprintf(g.date, sizeof(g.date), "%02d%02d%02d",
             t.tm_mday, t.tm_mon + 1, (t.tm_year + 1900) % 100);
}

/* ─── Pool de réception CAN ──────────────────────────────────────────────── */
typedef struct {
    twai_frame_t frame;
    uint8_t      data[8];
} rx_item_t;

static rx_item_t        rx_pool[RX_POOL_DEPTH];
static SemaphoreHandle_t free_slots;
static SemaphoreHandle_t pending_frames;
static int write_idx = 0, read_idx = 0;

static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *ctx)
{
    (void)edata; (void)ctx;
    BaseType_t woken = pdFALSE;
    if (xSemaphoreTakeFromISR(free_slots, &woken) != pdTRUE) return woken;
    if (twai_node_receive_from_isr(handle, &rx_pool[write_idx].frame) == ESP_OK) {
        write_idx = (write_idx + 1) % RX_POOL_DEPTH;
        xSemaphoreGiveFromISR(pending_frames, &woken);
    }
    return woken;
}

static bool IRAM_ATTR on_error(twai_node_handle_t h,
                                const twai_error_event_data_t *e, void *ctx)
{
    (void)h; (void)ctx;
    ESP_EARLY_LOGW(TAG, "CAN err=0x%08"PRIX32, e->err_flags.val);
    return false;
}

/* ─── Tâche UDP broadcast ────────────────────────────────────────────────── */
static void udp_task(void *arg)
{
    while (!udp_init()) vTaskDelay(pdMS_TO_TICKS(1000));

    uint32_t t_last = 0;
    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - t_last >= 1000) {        /* broadcast toutes les secondes */
            send_gpgga();
            send_gprmc();
            send_gpvtg();
            send_gphdg();
            send_gpgsa();
            t_last = now;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ─── app_main ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== NMEA2000 → Wi-Fi UDP NMEA0183 ===");

    /* NVS (requis pour Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Wi-Fi */
    bool wifi_ok = wifi_init();

    /* TWAI / CAN */
    free_slots     = xSemaphoreCreateCounting(RX_POOL_DEPTH, RX_POOL_DEPTH);
    pending_frames = xSemaphoreCreateCounting(RX_POOL_DEPTH, 0);
    for (int i = 0; i < RX_POOL_DEPTH; i++) {
        rx_pool[i].frame.buffer     = rx_pool[i].data;
        rx_pool[i].frame.buffer_len = 8;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO, .rx = CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing     = { .bitrate = CAN_BITRATE },
        .tx_queue_depth = 4,
        .flags          = { .enable_listen_only = false },
    };

    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &node));

    twai_mask_filter_config_t accept_all = {
        .id = 0, .mask = 0, .is_ext = true, .no_fd = true,
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node, 0, &accept_all));

    twai_event_callbacks_t cbs = {
        .on_rx_done = on_rx_done,
        .on_error   = on_error,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));

    ESP_LOGI(TAG, "TWAI démarré – RX=GPIO%d TX=GPIO%d @ %d bps",
             CAN_RX_GPIO, CAN_TX_GPIO, CAN_BITRATE);

    /* Tâche UDP si Wi-Fi OK */
    if (wifi_ok) {
        xTaskCreate(udp_task, "udp_task", 4096, NULL, 5, NULL);
    }

    /* Boucle CAN → décodage */
    while (1) {
        if (xSemaphoreTake(pending_frames, pdMS_TO_TICKS(100)) == pdTRUE) {
            rx_item_t *item = &rx_pool[read_idx];
            read_idx = (read_idx + 1) % RX_POOL_DEPTH;

            uint8_t prio, src; uint32_t pgn;
            parse_can_id(item->frame.header.id, &prio, &pgn, &src);
            const uint8_t *raw = item->frame.buffer;
            uint8_t dlc = item->frame.header.dlc;
            xSemaphoreGive(free_slots);

            const uint8_t *payload = NULL;
            uint8_t plen = 0;

            if (pgn == PGN_GNSS_POSITION || pgn == PGN_TIME_DATE) {
                payload = fp_feed(src, pgn, raw, &plen);
                if (!payload) continue;
            } else {
                payload = raw; plen = dlc;
            }

            switch (pgn) {
            case PGN_COG_SOG:        decode_cog_sog(payload, plen);        break;
            case PGN_VESSEL_HEADING: decode_vessel_heading(payload, plen);  break;
            case PGN_GNSS_DOPS:      decode_gnss_dops(payload, plen);       break;
            case PGN_GNSS_POSITION:  decode_gnss_position(payload, plen);   break;
            case PGN_TIME_DATE:      decode_time_date(payload, plen);       break;
            default: break;
            }
        }
    }
}
