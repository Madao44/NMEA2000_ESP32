/*
 * NMEA2000 Emetteur – GPS Sportnav SPO25 Simulator v2
 * Simule exactement les PGNs émis par le vrai SPO25 observés sur le bus :
 *
 *   PGN 129025 – Position Rapid Update       (100 ms)  single frame
 *   PGN 129026 – COG & SOG Rapid Update      (250 ms)  single frame
 *   PGN 127250 – Vessel Heading              (100 ms)  single frame
 *   PGN 129029 – GNSS Position Data         (1000 ms)  fast-packet
 *   PGN 129539 – GNSS DOPs                  (1000 ms)  single frame
 *   PGN 126992 – System Time                (1000 ms)  single frame
 *   PGN 128267 – Water Depth                (1000 ms)  single frame
 *   PGN 129540 – GNSS Sats in View          (1000 ms)  fast-packet
 *   PGN  60928 – ISO Address Claim          (boot)     single frame
 *
 * Adresse source : 0x15 (comme le vrai SPO25 observé)
 * ESP32-S3 + SN65HVD230  |  TWAI 250 kbit/s
 * GPIO5 = TX CAN  |  GPIO4 = RX CAN
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
#define SRC_ADDR     0x15U   /* adresse source = 0x15 comme le vrai SPO25 */

static const char *TAG = "SPO25_SIM";

/* ─── PGNs observés sur le bus ──────────────────────────────────────────── */
#define PGN_POSITION_RAPID   129025U
#define PGN_COG_SOG          129026U
#define PGN_VESSEL_HEADING   127250U
#define PGN_GNSS_POSITION    129029U
#define PGN_GNSS_DOPS        129539U
#define PGN_SYSTEM_TIME      126992U
#define PGN_WATER_DEPTH      128267U
#define PGN_GNSS_SATS        129540U
#define PGN_ADDRESS_CLAIM     60928U

/* ─── Construction ID CAN 29 bits ──────────────────────────────────────── */
static uint32_t build_can_id(uint8_t prio, uint32_t pgn, uint8_t src)
{
    uint8_t  pf      = (pgn >> 8) & 0xFF;
    uint32_t can_pgn = (pf < 0xF0) ? (pgn & 0x1FF00U) : (pgn & 0x1FFFFU);
    uint32_t id      = ((uint32_t)(prio & 0x07)) << 26;
    id |= (can_pgn & 0x3FFFFU) << 8;
    id |= src;
    return id;
}

static esp_err_t send_frame(twai_node_handle_t node,
                             uint32_t pgn, uint8_t prio,
                             const uint8_t *data, uint8_t len)
{
    static uint8_t buf[8];   /* static pour survivre à l'ISR */
    memset(buf, 0xFF, sizeof(buf));
    if (len > 8) len = 8;
    memcpy(buf, data, len);

    twai_frame_t frame = {
        .header = {
            .id  = build_can_id(prio, pgn, SRC_ADDR),
            .ide = 1, .rtr = 0, .fdf = 0, .dlc = 8,
        },
        .buffer = buf, .buffer_len = 8,
    };
    esp_err_t err = twai_node_transmit(node, &frame, pdMS_TO_TICKS(50));
    if (err == ESP_OK)
        ESP_LOGI(TAG, "TX PGN=%-6"PRIu32" %02X%02X%02X%02X%02X%02X%02X%02X",
                 pgn, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    return err;
}

/* ─── Fast-Packet ───────────────────────────────────────────────────────── */
static void send_fast_packet(twai_node_handle_t node,
                              uint32_t pgn, uint8_t prio,
                              const uint8_t *payload, uint8_t total,
                              uint8_t seq_id)
{
    uint32_t can_id    = build_can_id(prio, pgn, SRC_ADDR);
    uint8_t  frame_num = 0, offset = 0;

    while (frame_num == 0 || offset < total) {
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
            uint8_t n = (remain < 7) ? remain : 7;
            memcpy(&buf[1], &payload[offset], n);
            offset += n;
        }

        twai_frame_t frame = {
            .header = { .id=can_id, .ide=1, .rtr=0, .fdf=0, .dlc=8 },
            .buffer = buf, .buffer_len = 8,
        };
        twai_node_transmit(node, &frame, pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "TX  PGN=%-6"PRIu32" f[%02d] "
                 "%02X%02X%02X%02X%02X%02X%02X%02X",
                 pgn, frame_num,
                 buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

        frame_num++;
        vTaskDelay(pdMS_TO_TICKS(2));
        if (offset >= total) break;
    }
}

/* ─── GPS simulé ────────────────────────────────────────────────────────── */
typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, hdg_deg;
    float   hdop, pdop, tdop;
    float   depth_m;
    uint8_t num_svs;
    uint8_t sid;
    uint32_t tick;
} gps_t;

static void gps_update(gps_t *g)
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

/* ─── Encodeurs ─────────────────────────────────────────────────────────── */

/* PGN 129025 – Position Rapid Update */
static void enc_position_rapid(uint8_t *b, double lat, double lon)
{
    int32_t la = (int32_t)(lat / 1e-7);
    int32_t lo = (int32_t)(lon / 1e-7);
    b[0]= la&0xFF; b[1]=(la>>8)&0xFF; b[2]=(la>>16)&0xFF; b[3]=(la>>24)&0xFF;
    b[4]= lo&0xFF; b[5]=(lo>>8)&0xFF; b[6]=(lo>>16)&0xFF; b[7]=(lo>>24)&0xFF;
}

/* PGN 129026 – COG & SOG Rapid Update */
static void enc_cog_sog(uint8_t *b, uint8_t sid, float cog_deg, float sog_kn)
{
    uint16_t c = (uint16_t)(cog_deg * (float)M_PI / 180.0f / 1e-4f);
    uint16_t s = (uint16_t)(sog_kn * 0.514444f / 1e-4f);
    b[0]=sid; b[1]=0x00;
    b[2]=c&0xFF; b[3]=(c>>8)&0xFF;
    b[4]=s&0xFF; b[5]=(s>>8)&0xFF;
    b[6]=0xFF; b[7]=0xFF;
}

/* PGN 127250 – Vessel Heading */
static void enc_heading(uint8_t *b, uint8_t sid, float hdg_deg)
{
    uint16_t h = (uint16_t)(hdg_deg * (float)M_PI / 180.0f / 1e-4f);
    b[0]=sid; b[1]=0x01;
    b[2]=h&0xFF; b[3]=(h>>8)&0xFF;
    b[4]=0xFF; b[5]=0xFF; b[6]=0xFF; b[7]=0x00;
}

/* PGN 129539 – GNSS DOPs */
static void enc_dops(uint8_t *b, uint8_t sid, float hdop, float pdop, float tdop)
{
    int16_t h=(int16_t)(hdop/0.01f), p=(int16_t)(pdop/0.01f), t2=(int16_t)(tdop/0.01f);
    b[0]=sid; b[1]=(3&7)|((3&7)<<3);
    b[2]=h&0xFF; b[3]=(h>>8)&0xFF;
    b[4]=p&0xFF; b[5]=(p>>8)&0xFF;
    b[6]=t2&0xFF; b[7]=(t2>>8)&0xFF;
}

/* PGN 126992 – System Time */
static void enc_system_time(uint8_t *b, uint8_t sid,
                             uint16_t days, uint32_t secs)
{
    uint64_t sx = (uint64_t)secs * 10000ULL;
    b[0]=sid; b[1]=0xF0;
    b[2]=days&0xFF; b[3]=(days>>8)&0xFF;
    b[4]=(uint8_t)(sx&0xFF); b[5]=(uint8_t)((sx>>8)&0xFF);
    b[6]=(uint8_t)((sx>>16)&0xFF); b[7]=(uint8_t)((sx>>24)&0xFF);
}

/* PGN 128267 – Water Depth */
static void enc_water_depth(uint8_t *b, uint8_t sid, float depth_m)
{
    uint32_t d = (uint32_t)(depth_m / 0.01f);
    b[0]=sid;
    b[1]=d&0xFF; b[2]=(d>>8)&0xFF; b[3]=(d>>16)&0xFF; b[4]=(d>>24)&0xFF;
    b[5]=0x00; b[6]=0xFD; b[7]=0xFF;
}

/* PGN 129029 – GNSS Position Data (fast-packet) */
static uint8_t enc_gnss_position(uint8_t *b, const gps_t *g,
                                  uint16_t days, uint64_t secs_x10k)
{
    int64_t la = (int64_t)(g->lat / 1e-7);
    int64_t lo = (int64_t)(g->lon / 1e-7);
    int32_t al = (int32_t)(g->alt / 1e-6);
    int16_t hd = (int16_t)(g->hdop / 0.01f);
    int16_t pd = (int16_t)(g->pdop / 0.01f);
    int16_t ge = 4800;

    uint8_t i = 0;
    b[i++] = g->sid;
    b[i++] = days & 0xFF; b[i++] = (days>>8) & 0xFF;
    for (int j=0;j<8;j++) b[i++] = (uint8_t)((secs_x10k>>(8*j)) & 0xFF);
    for (int j=0;j<8;j++) b[i++] = (uint8_t)((la>>(8*j)) & 0xFF);
    for (int j=0;j<8;j++) b[i++] = (uint8_t)((lo>>(8*j)) & 0xFF);
    for (int j=0;j<4;j++) b[i++] = (uint8_t)((al>>(8*j)) & 0xFF);
    b[i++] = (0) | (1<<4);
    b[i++] = 0x00;
    b[i++] = g->num_svs;
    b[i++] = hd&0xFF; b[i++] = (hd>>8)&0xFF;
    b[i++] = pd&0xFF; b[i++] = (pd>>8)&0xFF;
    b[i++] = ge&0xFF; b[i++] = (ge>>8)&0xFF;
    b[i++] = 0x00; b[i++] = 0xFF;
    return i;
}

/* PGN 129540 – GNSS Sats in View (fast-packet) */
static uint8_t enc_gnss_sats(uint8_t *b, uint8_t sid, uint8_t num_svs)
{
    uint8_t i = 0;
    b[i++] = sid;
    b[i++] = 0xFF;
    b[i++] = num_svs;
    for (uint8_t s = 0; s < num_svs && i + 12 <= 220; s++) {
        b[i++] = s + 1;
        b[i++] = (uint8_t)(45 * 4);
        uint16_t az = (uint16_t)((s * 30 % 360) * 10000U * 314U / 18000U);
        b[i++] = az & 0xFF; b[i++] = (az>>8) & 0xFF;
        uint16_t snr = (uint16_t)(35.0f / 0.01f);
        b[i++] = snr & 0xFF; b[i++] = (snr>>8) & 0xFF;
        b[i++]=0x00; b[i++]=0x00; b[i++]=0x00; b[i++]=0x00;
        b[i++] = 0x07;
        b[i++] = 0xFF;
    }
    return i;
}

/* PGN 60928 – ISO Address Claim */
static void enc_address_claim(uint8_t *b)
{
    uint64_t name = 0;
    name |= (uint64_t)(0x1A2B3C & 0x1FFFFF);
    name |= (uint64_t)(0x0099   & 0x07FF) << 21;
    name |= (uint64_t)(145      & 0xFF)   << 40;
    name |= (uint64_t)(60       & 0x7F)   << 48;
    name |= (uint64_t)(4        & 0x07)   << 60;
    name |= (uint64_t)(1        & 0x01)   << 63;
    for (int i=0;i<8;i++) b[i] = (uint8_t)((name>>(8*i)) & 0xFF);
}

/* ─── app_main ──────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== SPO25 Simulator v2 – Real PGNs ===");
    ESP_LOGI(TAG, "SRC=0x%02X  TX=GPIO%d  RX=GPIO%d  @ %d bps",
             SRC_ADDR, CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);

    twai_onchip_node_config_t ncfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO, .rx = CAN_RX_GPIO,
            .quanta_clk_out    = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing     = { .bitrate = CAN_BITRATE },
        .tx_queue_depth = 32,
        .flags          = { .enable_listen_only = false },
    };

    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&ncfg, &node));
    ESP_ERROR_CHECK(twai_node_enable(node));

    /* Address Claim */
    uint8_t buf[256];
    enc_address_claim(buf);
    send_frame(node, PGN_ADDRESS_CLAIM, 6, buf, 8);
    vTaskDelay(pdMS_TO_TICKS(250));

    gps_t gps = {
        .alt     = 2.0f,
        .sog_kn  = 0.0f,
        .hdop    = 1.1f,
        .pdop    = 1.5f,
        .tdop    = 1.0f,
        .depth_m = 4.5f,
        .num_svs = 11,
    };

    uint8_t  seq    = 0;
    uint16_t days   = 20000;
    uint32_t base_s = 12 * 3600;

    uint32_t t_rapid=0, t_cog=0,  t_hdg=0;
    uint32_t t_pos=0,   t_dop=0,  t_time=0;
    uint32_t t_depth=0, t_sats=0;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        gps_update(&gps);

        uint32_t secs      = base_s + gps.tick / 10;
        uint64_t secs_x10k = (uint64_t)secs * 10000ULL;

        /* 100 ms */
        if (now - t_rapid >= 100) {
            enc_position_rapid(buf, gps.lat, gps.lon);
            send_frame(node, PGN_POSITION_RAPID, 2, buf, 8);
            t_rapid = now;
        }
        if (now - t_hdg >= 100) {
            enc_heading(buf, gps.sid, gps.hdg_deg);
            send_frame(node, PGN_VESSEL_HEADING, 2, buf, 8);
            t_hdg = now;
        }

        /* 250 ms */
        if (now - t_cog >= 250) {
            enc_cog_sog(buf, gps.sid, gps.cog_deg, gps.sog_kn);
            send_frame(node, PGN_COG_SOG, 2, buf, 8);
            t_cog = now;
        }

        /* 1000 ms */
        if (now - t_depth >= 1000) {
            enc_water_depth(buf, gps.sid, gps.depth_m);
            send_frame(node, PGN_WATER_DEPTH, 6, buf, 8);
            t_depth = now;
        }
        if (now - t_pos >= 1000) {
            uint8_t n = enc_gnss_position(buf, &gps, days, secs_x10k);
            send_fast_packet(node, PGN_GNSS_POSITION, 3, buf, n, seq++);
            t_pos = now;
        }
        if (now - t_dop >= 1000) {
            enc_dops(buf, gps.sid, gps.hdop, gps.pdop, gps.tdop);
            send_frame(node, PGN_GNSS_DOPS, 6, buf, 8);
            t_dop = now;
        }
        if (now - t_time >= 1000) {
            enc_system_time(buf, gps.sid, days, secs);
            send_frame(node, PGN_SYSTEM_TIME, 3, buf, 8);
            t_time = now;
        }
        if (now - t_sats >= 1000) {
            uint8_t n = enc_gnss_sats(buf, gps.sid, gps.num_svs);
            send_fast_packet(node, PGN_GNSS_SATS, 6, buf, n, seq++);
            t_sats = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}