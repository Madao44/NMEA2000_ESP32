/*
 * NMEA2000 Emetteur – GPS Sportnav SPO25 Simulator
 * ESP32-S3 + SN65HVD230  |  TWAI 250 kbit/s
 *
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

/* ─── Configuration ──────────────────────────────────────────────────────── */
#define CAN_TX_GPIO     GPIO_NUM_5
#define CAN_RX_GPIO     GPIO_NUM_4
#define CAN_BITRATE     250000
#define SRC_ADDR        0x23        /* adresse simulée du SPO25 */

static const char *TAG = "NMEA2000_TX";

/* ─── PGNs ───────────────────────────────────────────────────────────────── */
#define PGN_GNSS_POSITION    129029U
#define PGN_COG_SOG          129026U
#define PGN_VESSEL_HEADING   127250U
#define PGN_SPEED            128259U
#define PGN_TIME_DATE        129033U
#define PGN_GNSS_DOPS        129539U
#define PGN_ADDRESS_CLAIM     60928U

/* ─── Construction ID CAN 29 bits NMEA2000 ───────────────────────────────── */
static uint32_t build_can_id(uint8_t prio, uint32_t pgn, uint8_t src)
{
    uint8_t pf = (pgn >> 8) & 0xFF;
    uint32_t can_pgn = (pf < 0xF0) ? (pgn & 0x1FF00U) : (pgn & 0x1FFFFU);
    uint32_t id  = ((uint32_t)(prio & 0x07)) << 26;
    id |= (can_pgn & 0x3FFFFU) << 8;
    id |= src;
    return id;
}

/* ─── Envoi trame unique ──────────────────────────────────────────────────── */
static void send_frame(twai_node_handle_t node,
                       uint32_t pgn, uint8_t prio,
                       const uint8_t *payload, uint8_t len)
{
    uint8_t buf[8];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, payload, len < 8 ? len : 8);

    twai_frame_t frame = {
        .header = {
            .id  = build_can_id(prio, pgn, SRC_ADDR),
            .ide = 1,
            .rtr = 0,
            .fdf = 0,
            .dlc = 8,
        },
        .buffer     = buf,
        .buffer_len = 8,
    };

    esp_err_t err = twai_node_transmit(node, &frame, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX PGN %"PRIu32" err=%s", pgn, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "TX PGN %"PRIu32" ID=0x%08"PRIX32" %02X%02X%02X%02X%02X%02X%02X%02X",
                 pgn, frame.header.id,
                 buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    }
}

/* ─── Fast-Packet : fragmentation ────────────────────────────────────────── */
static void send_fast_packet(twai_node_handle_t node,
                             uint32_t pgn, uint8_t prio,
                             const uint8_t *payload, uint8_t total,
                             uint8_t seq_id)
{
    uint32_t can_id = build_can_id(prio, pgn, SRC_ADDR);
    uint8_t frame_num = 0;
    uint8_t offset    = 0;

    while (offset < total || frame_num == 0) {
        uint8_t buf[8];
        memset(buf, 0xFF, sizeof(buf));
        buf[0] = ((seq_id & 0x07) << 5) | (frame_num & 0x1F);

        if (frame_num == 0) {
            buf[1] = total;
            uint8_t n = (total < 6) ? total : 6;
            memcpy(&buf[2], &payload[offset], n);
            offset += n;
        } else {
            uint8_t remain = total - offset;
            uint8_t n = remain < 7 ? remain : 7;
            memcpy(&buf[1], &payload[offset], n);
            offset += n;
        }

        twai_frame_t frame = {
            .header = {
                .id  = can_id,
                .ide = 1,
                .rtr = 0,
                .fdf = 0,
                .dlc = 8,
            },
            .buffer     = buf,
            .buffer_len = 8,
        };
        twai_node_transmit(node, &frame, pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "TX FP  PGN %"PRIu32" f[%d] %02X%02X%02X%02X%02X%02X%02X%02X",
                 pgn, frame_num,
                 buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

        frame_num++;
        vTaskDelay(pdMS_TO_TICKS(2));

        if (offset >= total && frame_num > 0) break;
    }
}

/* ─── Encodeurs PGN ──────────────────────────────────────────────────────── */

static void encode_cog_sog(uint8_t *buf, float cog_deg, float sog_kn, uint8_t sid)
{
    uint16_t cog_i = (uint16_t)(cog_deg * M_PI / 180.0f / 1e-4f);
    uint16_t sog_i = (uint16_t)(sog_kn * 0.514444f / 1e-4f);
    buf[0] = sid;
    buf[1] = 0;   /* COG ref = True */
    buf[2] = cog_i & 0xFF;
    buf[3] = (cog_i >> 8) & 0xFF;
    buf[4] = sog_i & 0xFF;
    buf[5] = (sog_i >> 8) & 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFF;
}

static void encode_vessel_heading(uint8_t *buf, float hdg_deg, uint8_t sid)
{
    uint16_t hdg_i = (uint16_t)(hdg_deg * M_PI / 180.0f / 1e-4f);
    buf[0] = sid;
    buf[1] = 1;   /* ref = Magnetic */
    buf[2] = hdg_i & 0xFF;
    buf[3] = (hdg_i >> 8) & 0xFF;
    buf[4] = 0xFF; buf[5] = 0xFF;
    buf[6] = 0xFF; buf[7] = 0x00;
}

static void encode_speed(uint8_t *buf, float spd_kn, uint8_t sid)
{
    uint16_t sw = (uint16_t)(spd_kn * 0.514444f / 0.01f);
    buf[0] = sid;
    buf[1] = sw & 0xFF;
    buf[2] = (sw >> 8) & 0xFF;
    buf[3] = sw & 0xFF;
    buf[4] = (sw >> 8) & 0xFF;
    buf[5] = 0xFF; buf[6] = 0xFF; buf[7] = 0xFF;
}

static void encode_gnss_dops(uint8_t *buf, float hdop, float vdop, float tdop, uint8_t sid)
{
    int16_t h = (int16_t)(hdop / 0.01f);
    int16_t v = (int16_t)(vdop / 0.01f);
    int16_t t = (int16_t)(tdop / 0.01f);
    buf[0] = sid;
    buf[1] = (3 & 0x07) | ((3 & 0x07) << 3);  /* mode 3D */
    buf[2] = h & 0xFF; buf[3] = (h >> 8) & 0xFF;
    buf[4] = v & 0xFF; buf[5] = (v >> 8) & 0xFF;
    buf[6] = t & 0xFF; buf[7] = (t >> 8) & 0xFF;
}

static uint8_t encode_gnss_position(uint8_t *buf,
                                     double lat, double lon, double alt,
                                     uint16_t days, uint64_t secs_x10k,
                                     uint8_t sid)
{
    int64_t lat_i = (int64_t)(lat / 1e-7);
    int64_t lon_i = (int64_t)(lon / 1e-7);
    int32_t alt_i = (int32_t)(alt / 1e-6);
    int16_t hdop  = 90;   /* 0.90 */
    int16_t pdop  = 140;  /* 1.40 */
    int16_t geo   = 4800; /* 48.00 m */

    uint8_t idx = 0;
    buf[idx++] = sid;
    buf[idx++] = days & 0xFF;
    buf[idx++] = (days >> 8) & 0xFF;
    /* secs (8 bytes little-endian) */
    for (int i = 0; i < 8; i++) buf[idx++] = (secs_x10k >> (8*i)) & 0xFF;
    /* lat (8 bytes) */
    for (int i = 0; i < 8; i++) buf[idx++] = (lat_i >> (8*i)) & 0xFF;
    /* lon (8 bytes) */
    for (int i = 0; i < 8; i++) buf[idx++] = (lon_i >> (8*i)) & 0xFF;
    /* alt (4 bytes) */
    for (int i = 0; i < 4; i++) buf[idx++] = (alt_i >> (8*i)) & 0xFF;
    buf[idx++] = (0) | (1 << 4);   /* GPS, fix 3D */
    buf[idx++] = 0;                 /* integrity */
    buf[idx++] = 10;                /* num SVs */
    buf[idx++] = hdop & 0xFF; buf[idx++] = (hdop>>8) & 0xFF;
    buf[idx++] = pdop & 0xFF; buf[idx++] = (pdop>>8) & 0xFF;
    buf[idx++] = geo  & 0xFF; buf[idx++] = (geo >>8) & 0xFF;
    buf[idx++] = 0; buf[idx++] = 0xFF;
    return idx;
}

static uint8_t encode_time_date(uint8_t *buf, uint16_t days, uint64_t secs_x10k)
{
    buf[0] = days & 0xFF;
    buf[1] = (days >> 8) & 0xFF;
    for (int i = 0; i < 8; i++) buf[i+2] = (secs_x10k >> (8*i)) & 0xFF;
    return 10;
}

static void encode_address_claim(uint8_t *buf)
{
    uint64_t name = 0;
    name |= (uint64_t)(0x1A2B3C & 0x1FFFFF);
    name |= (uint64_t)(0x0099   & 0x07FF) << 21;
    name |= (uint64_t)(145      & 0xFF)   << 40;
    name |= (uint64_t)(60       & 0x7F)   << 48;
    name |= (uint64_t)(4        & 0x07)   << 60;
    name |= (uint64_t)(1        & 0x01)   << 63;
    for (int i = 0; i < 8; i++) buf[i] = (name >> (8*i)) & 0xFF;
}

/* ─── Simulation GPS (cercle autour de Barcelone) ────────────────────────── */
typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, heading;
    float   hdop, pdop, tdop;
    uint8_t sid;
    uint32_t tick;
} gps_t;

static void gps_update(gps_t *g)
{
    float t  = g->tick * 0.1f;          /* 100 ms par tick */
    float R  = 500.0f;
    float om = 2.0f * M_PI / 120.0f;

    g->lat     = 41.3809 + (R * cosf(om * t)) / 111320.0;
    g->lon     = 2.1734  + (R * sinf(om * t)) / (111320.0 * cosf(41.38f * M_PI / 180.0f));
    g->cog_deg = fmodf(om * t * 180.0f / M_PI + 90.0f, 360.0f);
    g->heading = g->cog_deg;
    g->sid     = (g->sid + 1) & 0xFF;
    g->tick++;
}

/* ─── Tâche principale ───────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== NMEA2000 Emetteur – SPO25 Simulator ===");

    /* Init TWAI */
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out  = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = { .bitrate = CAN_BITRATE },
        .tx_queue_depth = 32,
        .flags = { .enable_listen_only = false },
    };

    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &node));
    ESP_ERROR_CHECK(twai_node_enable(node));
    ESP_LOGI(TAG, "TWAI démarré – TX=GPIO%d RX=GPIO%d @ %d bps",
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);

    /* Address Claim au boot */
    uint8_t ac_buf[8];
    encode_address_claim(ac_buf);
    send_frame(node, PGN_ADDRESS_CLAIM, 6, ac_buf, 8);
    vTaskDelay(pdMS_TO_TICKS(250));

    gps_t gps = {
        .alt = 2.0f, .sog_kn = 6.5f,
        .hdop = 0.9f, .pdop = 1.4f, .tdop = 1.1f,
    };

    uint8_t  seq_id   = 0;
    uint32_t t_pos    = 0, t_cogsog = 0, t_hdg = 0;
    uint32_t t_spd    = 0, t_time   = 0, t_dop = 0;

    /* Heure simulée : 12h00 UTC, jour 20000 depuis 1970 */
    uint16_t days       = 20000;
    uint64_t base_secs  = (uint64_t)(12 * 3600) * 10000ULL;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        gps_update(&gps);
        uint64_t secs_x10k = base_secs + (uint64_t)(gps.tick * 100) * 100ULL;

        uint8_t buf[64];

        /* PGN 129026 – COG & SOG  (250 ms) */
        if (now - t_cogsog >= 250) {
            encode_cog_sog(buf, gps.cog_deg, gps.sog_kn, gps.sid);
            send_frame(node, PGN_COG_SOG, 2, buf, 8);
            t_cogsog = now;
        }

        /* PGN 127250 – Vessel Heading  (100 ms) */
        if (now - t_hdg >= 100) {
            encode_vessel_heading(buf, gps.heading, gps.sid);
            send_frame(node, PGN_VESSEL_HEADING, 2, buf, 8);
            t_hdg = now;
        }

        /* PGN 129029 – GNSS Position  (1000 ms) */
        if (now - t_pos >= 1000) {
            uint8_t plen = encode_gnss_position(buf,
                               gps.lat, gps.lon, gps.alt,
                               days, secs_x10k, gps.sid);
            send_fast_packet(node, PGN_GNSS_POSITION, 3, buf, plen, seq_id++);
            t_pos = now;
        }

        /* PGN 128259 – Speed  (1000 ms) */
        if (now - t_spd >= 1000) {
            encode_speed(buf, gps.sog_kn, gps.sid);
            send_frame(node, PGN_SPEED, 2, buf, 8);
            t_spd = now;
        }

        /* PGN 129033 – Time & Date  (1000 ms) */
        if (now - t_time >= 1000) {
            uint8_t tlen = encode_time_date(buf, days, secs_x10k);
            send_fast_packet(node, PGN_TIME_DATE, 3, buf, tlen, seq_id++);
            t_time = now;
        }

        /* PGN 129539 – GNSS DOPs  (1000 ms) */
        if (now - t_dop >= 1000) {
            encode_gnss_dops(buf, gps.hdop, gps.pdop, gps.tdop, gps.sid);
            send_frame(node, PGN_GNSS_DOPS, 6, buf, 8);
            t_dop = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}