/*
 * NMEA2000 → Wi-Fi Access Point + WebSocket JSON (esp_http_server natif)
 * ESP32-S3 + SN65HVD230  |  TWAI 250 kbit/s
 *
 * Hotspot : SSID=NMEA2000  Password=sportnav
 * Dashboard : http://192.168.4.1
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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ─── Configuration ─────────────────────────────────────────────────── */
#define CAN_TX_GPIO   GPIO_NUM_5
#define CAN_RX_GPIO   GPIO_NUM_4
#define CAN_BITRATE   250000
#define RX_POOL_DEPTH 64
#define AP_SSID       "NMEA2000"
#define AP_PASSWORD   "sportnav"
#define AP_MAX_CONN   4

static const char *TAG = "NMEA_AP";

/* ─── PGNs ──────────────────────────────────────────────────────────── */
#define PGN_GNSS_POSITION  129029U
#define PGN_COG_SOG        129026U
#define PGN_VESSEL_HEADING 127250U
#define PGN_SPEED          128259U
#define PGN_TIME_DATE      129033U
#define PGN_GNSS_DOPS      129539U
#define PGN_POSITION_RAPID  129025U

/* ─── État GPS ──────────────────────────────────────────────────────── */
typedef struct {
    double  lat, lon, alt;
    float   cog_deg, sog_kn, heading_deg;
    float   hdop, vdop, tdop;
    uint8_t num_svs;
    char    utc[16];
    uint8_t fix;
} gps_t;

static gps_t g_gps = { .hdop = 99.9f, .vdop = 99.9f };
static SemaphoreHandle_t g_gps_mutex;

/* ─── WebSocket clients ─────────────────────────────────────────────── */
#define MAX_WS_CLIENTS 4
static int g_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t g_ws_mutex;
static httpd_handle_t g_server = NULL;

static void ws_init(void) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) g_ws_fds[i] = -1;
}

static void ws_add(int fd) {
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_fds[i] == -1) { g_ws_fds[i] = fd; break; }
    }
    xSemaphoreGive(g_ws_mutex);
    ESP_LOGI(TAG, "WS client connecte fd=%d", fd);
}

static void ws_remove(int fd) {
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_fds[i] == fd) { g_ws_fds[i] = -1; break; }
    }
    xSemaphoreGive(g_ws_mutex);
    ESP_LOGI(TAG, "WS client deconnecte fd=%d", fd);
}

static void ws_broadcast(const char *json) {
    if (!g_server) return;
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = g_ws_fds[i];
        if (fd < 0) continue;
        httpd_ws_frame_t frame = {
            .type    = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json,
            .len     = strlen(json),
            .final   = true,
        };
        esp_err_t err = httpd_ws_send_frame_async(g_server, fd, &frame);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WS send error fd=%d err=%s", fd, esp_err_to_name(err));
            g_ws_fds[i] = -1;
        }
    }
    xSemaphoreGive(g_ws_mutex);
}

/* ─── JSON builder ──────────────────────────────────────────────────── */
static void build_json(char *buf, size_t len) {
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    snprintf(buf, len,
        "{\"fix\":%s,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,"
        "\"sog\":%.2f,\"cog\":%.1f,\"hdg\":%.1f,"
        "\"hdop\":%.2f,\"vdop\":%.2f,\"svs\":%d,\"utc\":\"%s\"}",
        g_gps.fix ? "true" : "false",
        g_gps.lat, g_gps.lon, g_gps.alt,
        g_gps.sog_kn, g_gps.cog_deg, g_gps.heading_deg,
        g_gps.hdop, g_gps.vdop, g_gps.num_svs, g_gps.utc);
    xSemaphoreGive(g_gps_mutex);
}

/* ─── Parseur ID CAN 29 bits ────────────────────────────────────────── */
static void parse_can_id(uint32_t can_id, uint8_t *prio,
                         uint32_t *pgn, uint8_t *src) {
    *prio = (can_id >> 26) & 0x07;
    uint32_t pgn_raw = (can_id >> 8) & 0x3FFFF;
    *src  = can_id & 0xFF;
    uint8_t pf = (pgn_raw >> 8) & 0xFF;
    *pgn  = (pf < 0xF0) ? (pgn_raw & 0x1FF00U) : pgn_raw;
}

/* ─── Fast-Packet ───────────────────────────────────────────────────── */
#define FP_MAX_STREAMS  4
#define FP_MAX_PAYLOAD  223
typedef struct {
    uint8_t  active, src, seq_id, total, received;
    uint32_t pgn;
    uint8_t  buf[FP_MAX_PAYLOAD];
} fp_stream_t;
static fp_stream_t fp_streams[FP_MAX_STREAMS];

static const uint8_t *fp_feed(uint8_t src, uint32_t pgn,
                               const uint8_t *frame, uint8_t *out_len) {
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
        s->active = 1; s->src = src; s->pgn = pgn;
        s->seq_id = seq_id; s->total = frame[1];
        uint8_t n = s->total < 6 ? s->total : 6;
        memcpy(s->buf, &frame[2], n);
        s->received = 1;
    } else {
        if (!s) return NULL;
        uint8_t offset = 6 + (frame_num - 1) * 7;
        uint8_t remain = s->total > offset ? s->total - offset : 0;
        uint8_t n = remain < 7 ? remain : 7;
        if (offset + n <= FP_MAX_PAYLOAD) memcpy(&s->buf[offset], &frame[1], n);
        s->received++;
    }
    uint8_t nf = (s->total <= 6) ? 1 : (1 + (s->total - 6 + 6) / 7);
    if (s->received >= nf) { *out_len = s->total; s->active = 0; return s->buf; }
    return NULL;
}

/* ─── Décodeurs PGN ─────────────────────────────────────────────────── */
static void decode_cog_sog(const uint8_t *d, uint8_t l) {
    if (l < 6) return;
    uint16_t ci = d[2] | ((uint16_t)d[3] << 8);
    uint16_t si = d[4] | ((uint16_t)d[5] << 8);
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    g_gps.cog_deg = ci * 1e-4f * 180.0f / (float)M_PI;
    g_gps.sog_kn  = si * 1e-4f / 0.514444f;
    xSemaphoreGive(g_gps_mutex);
}
static void decode_heading(const uint8_t *d, uint8_t l) {
    if (l < 4) return;
    uint16_t hi = d[2] | ((uint16_t)d[3] << 8);
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    g_gps.heading_deg = hi * 1e-4f * 180.0f / (float)M_PI;
    xSemaphoreGive(g_gps_mutex);
}
static void decode_dops(const uint8_t *d, uint8_t l) {
    if (l < 8) return;
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    g_gps.hdop = (int16_t)(d[2] | ((uint16_t)d[3] << 8)) * 0.01f;
    g_gps.vdop = (int16_t)(d[4] | ((uint16_t)d[5] << 8)) * 0.01f;
    g_gps.tdop = (int16_t)(d[6] | ((uint16_t)d[7] << 8)) * 0.01f;
    xSemaphoreGive(g_gps_mutex);
}
static void decode_position(const uint8_t *d, uint8_t l) {
    if (l < 27) return;
    int64_t lat = 0, lon = 0;
    for (int i = 0; i < 8; i++) lat |= ((int64_t)d[11+i] << (8*i));
    for (int i = 0; i < 8; i++) lon |= ((int64_t)d[19+i] << (8*i));
    if (lat & ((int64_t)1 << 55)) lat |= ~(((int64_t)1 << 56) - 1);
    if (lon & ((int64_t)1 << 55)) lon |= ~(((int64_t)1 << 56) - 1);
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    g_gps.lat = lat * 1e-7; g_gps.lon = lon * 1e-7; g_gps.fix = 1;
    g_gps.num_svs = l > 28 ? d[28] : 8;
    xSemaphoreGive(g_gps_mutex);
}
static void decode_time(const uint8_t *d, uint8_t l) {
    if (l < 10) return;
    uint64_t sx = 0;
    for (int i = 0; i < 8; i++) sx |= ((uint64_t)d[2+i] << (8*i));
    uint32_t s = (uint32_t)(sx / 10000);
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    snprintf(g_gps.utc, sizeof(g_gps.utc), "%02lu:%02lu:%02lu",
         (unsigned long)(s/3600), (unsigned long)((s%3600)/60), (unsigned long)(s%60));
    xSemaphoreGive(g_gps_mutex);
}

/* ─── Dashboard HTML (sans internet) ───────────────────────────────── */
static const char DASHBOARD_HTML[] =
"<!DOCTYPE html><html lang='fr'><head>"
"<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>NMEA2000</title><style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{font-family:'Segoe UI',sans-serif;background:#0a1628;color:#e0e8f0}"
"header{background:#0d2137;padding:10px 16px;display:flex;align-items:center;gap:10px;border-bottom:1px solid #1e3a5f}"
"header h1{font-size:1rem;color:#7dd3fc;flex:1}"
".dot{width:10px;height:10px;border-radius:50%;background:#ef4444;flex-shrink:0}"
".dot.ok{background:#22c55e;box-shadow:0 0 8px #22c55e}"
"#slbl{font-size:.75rem;color:#94a3b8}"
".body{display:flex;flex-wrap:wrap;gap:0}"
".left{flex:1;min-width:280px;display:flex;align-items:center;justify-content:center;padding:16px;background:#060f1e}"
".right{width:300px;padding:12px;display:flex;flex-direction:column;gap:10px}"
"@media(max-width:650px){.right{width:100%}}"
".card{background:#0a1e35;border:1px solid #1e3a5f;border-radius:8px;padding:10px}"
".card h2{font-size:.65rem;text-transform:uppercase;letter-spacing:1.5px;color:#60a5fa;margin-bottom:8px}"
".row{display:flex;justify-content:space-between;align-items:baseline;padding:4px 0;border-bottom:1px solid #1e3a5f18}"
".row:last-child{border:none}"
".lbl{font-size:.73rem;color:#94a3b8}"
".val{font-size:.92rem;font-weight:600}"
".big{font-size:1.6rem;color:#7dd3fc}"
".ok{color:#22c55e}.no{color:#ef4444}"
"canvas{display:block}"
"footer{padding:5px 14px;font-size:.65rem;color:#334155;border-top:1px solid #1e3a5f;text-align:center}"
"</style></head><body>"
"<header><div class='dot' id='dot'></div><h1>NMEA2000 Live</h1><span id='slbl'>Connecting...</span></header>"
"<div class='body'>"
"<div class='left'>"
"<div style='display:flex;flex-direction:column;gap:16px;align-items:center'>"
"<canvas id='compass' width='160' height='160'></canvas>"
"<canvas id='track' width='260' height='260'></canvas>"
"</div>"
"</div>"
"<div class='right'>"
"<div class='card'><h2>Position GPS</h2>"
"<div class='row'><span class='lbl'>Fix</span><span class='val' id='vfix'>-</span></div>"
"<div class='row'><span class='lbl'>Latitude</span><span class='val' id='vlat'>-</span></div>"
"<div class='row'><span class='lbl'>Longitude</span><span class='val' id='vlon'>-</span></div>"
"<div class='row'><span class='lbl'>Altitude</span><span class='val' id='valt'>-</span></div>"
"<div class='row'><span class='lbl'>Satellites</span><span class='val' id='vsvs'>-</span></div>"
"<div class='row'><span class='lbl'>UTC</span><span class='val' id='vutc'>-</span></div>"
"</div>"
"<div class='card'><h2>Navigation</h2>"
"<div class='row'><span class='lbl'>Speed (SOG)</span><span class='val big' id='vsog'>-</span></div>"
"<div class='row'><span class='lbl'>Course (COG)</span><span class='val' id='vcog'>-</span></div>"
"<div class='row'><span class='lbl'>Heading</span><span class='val' id='vhdg'>-</span></div>"
"</div>"
"<div class='card'><h2>Quality Signal</h2>"
"<div class='row'><span class='lbl'>HDOP</span><span class='val' id='vhdop'>-</span></div>"
"<div class='row'><span class='lbl'>VDOP</span><span class='val' id='vvdop'>-</span></div>"
"</div>"
"</div></div>"
"<footer>ESP32-S3 &bull; NMEA2000 &bull; 250 kbps &bull; <span id='ts'>-</span></footer>"
"<script>"
/* compas */
"var cc=document.getElementById('compass'),cctx=cc.getContext('2d');"
"function drawCompass(h){"
"var cx=80,cy=80,r=74;"
"cctx.clearRect(0,0,160,160);"
"cctx.beginPath();cctx.arc(cx,cy,r,0,6.283);cctx.fillStyle='#060f1e';cctx.fill();"
"cctx.strokeStyle='#1e3a5f';cctx.lineWidth=2;cctx.stroke();"
"var cards=['N','E','S','O'],colors=['#ef4444','#94a3b8','#94a3b8','#94a3b8'];"
"cards.forEach(function(c,i){"
"var a=(i*90-90)*Math.PI/180;"
"cctx.fillStyle=colors[i];cctx.font='bold 13px Segoe UI';"
"cctx.textAlign='center';cctx.textBaseline='middle';"
"cctx.fillText(c,cx+(r-18)*Math.cos(a),cy+(r-18)*Math.sin(a));});"
"for(var i=0;i<36;i++){"
"var a2=i*10*Math.PI/180,tick=i%3===0?10:5;"
"cctx.beginPath();cctx.moveTo(cx+(r-2)*Math.cos(a2),cy+(r-2)*Math.sin(a2));"
"cctx.lineTo(cx+(r-2-tick)*Math.cos(a2),cy+(r-2-tick)*Math.sin(a2));"
"cctx.strokeStyle='#334155';cctx.lineWidth=1;cctx.stroke();}"
"var a3=(h-90)*Math.PI/180;"
"cctx.beginPath();"
"cctx.moveTo(cx+r*0.55*Math.cos(a3),cy+r*0.55*Math.sin(a3));"
"cctx.lineTo(cx+r*0.18*Math.cos(a3+2.7),cy+r*0.18*Math.sin(a3+2.7));"
"cctx.lineTo(cx+r*0.18*Math.cos(a3-2.7),cy+r*0.18*Math.sin(a3-2.7));"
"cctx.closePath();cctx.fillStyle='#3b82f6';cctx.fill();"
"cctx.beginPath();cctx.arc(cx,cy,5,0,6.283);cctx.fillStyle='#e0e8f0';cctx.fill();"
"cctx.fillStyle='#e0e8f0';cctx.font='bold 14px Segoe UI';"
"cctx.textAlign='center';cctx.fillText(Math.round(h)+'deg',cx,cy+r-16);}"
"drawCompass(0);"
/* trace trajectoire */
"var tc=document.getElementById('track'),tctx=tc.getContext('2d');"
"var trail=[],originLat=null,originLon=null;"
"function toXY(lat,lon){"
"if(originLat===null){originLat=lat;originLon=lon;}"
"var dx=(lon-originLon)*111320*Math.cos(originLat*Math.PI/180);"
"var dy=(lat-originLat)*111320;"
"return [dx,dy];}"
"function drawTrack(hdg){"
"tctx.clearRect(0,0,260,260);"
"tctx.fillStyle='#060f1e';tctx.fillRect(0,0,260,260);"
"tctx.strokeStyle='#1e3a5f22';tctx.lineWidth=1;"
"for(var g=0;g<=260;g+=26){tctx.beginPath();tctx.moveTo(g,0);tctx.lineTo(g,260);tctx.stroke();"
"tctx.beginPath();tctx.moveTo(0,g);tctx.lineTo(260,g);tctx.stroke();}"
"tctx.strokeStyle='#1e3a5f';tctx.lineWidth=1;"
"tctx.strokeRect(0,0,260,260);"
"if(trail.length<1){tctx.fillStyle='#334155';tctx.font='12px Segoe UI';"
"tctx.textAlign='center';tctx.fillText('Waiting for GPS...',130,130);return;}"
"var xs=trail.map(function(p){return p[0];}),ys=trail.map(function(p){return p[1];});"
"var minx=Math.min.apply(null,xs),maxx=Math.max.apply(null,xs);"
"var miny=Math.min.apply(null,ys),maxy=Math.max.apply(null,ys);"
"var range=Math.max(maxx-minx,miny-miny+20,50)*1.4;"
"var cx2=130,cy2=130;"
"function px(p){return[cx2+(p[0]-(minx+maxx)/2)/range*220,"
"cy2-(p[1]-(miny+maxy)/2)/range*220];}"
"tctx.strokeStyle='#1d4ed8';tctx.lineWidth=1.5;tctx.beginPath();"
"trail.forEach(function(p,i){var q=px(p);if(i===0)tctx.moveTo(q[0],q[1]);else tctx.lineTo(q[0],q[1]);});"
"tctx.stroke();"
"var last=px(trail[trail.length-1]);"
"tctx.save();tctx.translate(last[0],last[1]);tctx.rotate((hdg||0)*Math.PI/180);"
"tctx.beginPath();tctx.moveTo(0,-10);tctx.lineTo(7,8);tctx.lineTo(0,4);tctx.lineTo(-7,8);tctx.closePath();"
"tctx.fillStyle='#3b82f6';tctx.fill();tctx.strokeStyle='#fff';tctx.lineWidth=1;tctx.stroke();"
"tctx.restore();"
"tctx.fillStyle='#475569';tctx.font='10px Segoe UI';tctx.textAlign='left';"
"tctx.fillText(trail.length+' pts',4,254);}"
"drawTrack(0);"
/* helpers */
"function sv(id,v){var e=document.getElementById(id);if(e)e.textContent=(v!==null&&v!==undefined)?v:'-';}"
/* WebSocket */
"var ws,rt,lastHdg=0;"
"function conn(){"
"ws=new WebSocket('ws://'+location.hostname+'/ws');"
"ws.onopen=function(){document.getElementById('dot').classList.add('ok');sv('slbl','Connected');clearTimeout(rt);};"
"ws.onclose=function(){document.getElementById('dot').classList.remove('ok');sv('slbl','Reconnecting...');rt=setTimeout(conn,2000);};"
"ws.onmessage=function(ev){try{"
"var d=JSON.parse(ev.data);"
"var fe=document.getElementById('vfix');"
"if(fe){fe.textContent=d.fix?'OUI':'NON';fe.className='val '+(d.fix?'ok':'no');}"
"sv('vlat',d.lat?d.lat.toFixed(6)+'deg':null);"
"sv('vlon',d.lon?d.lon.toFixed(6)+'deg':null);"
"sv('valt',d.alt!=null?d.alt.toFixed(1)+' m':null);"
"sv('vsvs',d.svs);"
"sv('vutc',d.utc);"
"sv('vsog',d.sog!=null?d.sog.toFixed(2)+' kn':null);"
"sv('vcog',d.cog!=null?d.cog.toFixed(1)+'deg':null);"
"sv('vhdg',d.hdg!=null?d.hdg.toFixed(1)+'deg':null);"
"sv('vhdop',d.hdop!=null?d.hdop.toFixed(2):null);"
"sv('vvdop',d.vdop!=null?d.vdop.toFixed(2):null);"
"var h=(d.hdg!=null)?d.hdg:(d.cog!=null?d.cog:lastHdg);lastHdg=h;"
"drawCompass(h);"
"if(d.lat&&d.lon){var p=toXY(d.lat,d.lon);trail.push(p);if(trail.length>500)trail.shift();drawTrack(h);}"
"sv('ts',new Date().toLocaleTimeString());"
"}catch(e){}};"
"}conn();"
"</script></body></html>";

/* ─── Handlers HTTP ─────────────────────────────────────────────────── */
static esp_err_t handle_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_ws(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ws_add(httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    /* Lire et ignorer les frames entrantes (ping/pong gérés par httpd) */
    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    uint8_t buf[32] = {0};
    frame.payload = buf;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
    if (err != ESP_OK) {
        int fd = httpd_req_to_sockfd(req);
        ws_remove(fd);
    }
    return err;
}

/* ─── Démarrage HTTP ────────────────────────────────────────────────── */
static void http_start(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port    = 80;
    cfg.max_open_sockets = 7;
    ESP_ERROR_CHECK(httpd_start(&g_server, &cfg));

    httpd_uri_t root = { .uri="/",    .method=HTTP_GET, .handler=handle_root };
    httpd_uri_t wsep = { .uri="/ws",  .method=HTTP_GET, .handler=handle_ws,
                         .is_websocket=true };
    httpd_register_uri_handler(g_server, &root);
    httpd_register_uri_handler(g_server, &wsep);
    ESP_LOGI(TAG, "HTTP + WebSocket demarres sur port 80");
}

/* ─── Wi-Fi Access Point ────────────────────────────────────────────── */
static void wifi_ap_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t ap = {
        .ap = {
            .ssid           = AP_SSID,
            .password       = AP_PASSWORD,
            .ssid_len       = strlen(AP_SSID),
            .channel        = 6,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .max_connection = AP_MAX_CONN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Hotspot: SSID=%s  IP=192.168.4.1", AP_SSID);
}

/* ─── Pool RX CAN ───────────────────────────────────────────────────── */
typedef struct { twai_frame_t frame; uint8_t data[8]; } rx_item_t;
static rx_item_t         rx_pool[RX_POOL_DEPTH];
static SemaphoreHandle_t free_slots, pending_frames;
static int write_idx = 0, read_idx = 0;

static bool IRAM_ATTR on_rx(twai_node_handle_t h,
                             const twai_rx_done_event_data_t *e, void *ctx) {
    (void)e; (void)ctx;
    BaseType_t w = pdFALSE;
    if (xSemaphoreTakeFromISR(free_slots, &w) != pdTRUE) return w;
    if (twai_node_receive_from_isr(h, &rx_pool[write_idx].frame) == ESP_OK) {
        write_idx = (write_idx + 1) % RX_POOL_DEPTH;
        xSemaphoreGiveFromISR(pending_frames, &w);
    }
    return w;
}
static bool IRAM_ATTR on_err(twai_node_handle_t h,
                              const twai_error_event_data_t *e, void *ctx) {
    (void)h; (void)e; (void)ctx;
    return false;
}

/* ─── Tâche broadcast JSON ──────────────────────────────────────────── */
static void broadcast_task(void *arg) {
    char json[256];
    while (1) {
        build_json(json, sizeof(json));
        ws_broadcast(json);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void decode_position_rapid(const uint8_t *d, uint8_t l) {
    if (l < 8) return;
    int32_t lat_i = (int32_t)(d[0] | ((uint32_t)d[1]<<8) | ((uint32_t)d[2]<<16) | ((uint32_t)d[3]<<24));
    int32_t lon_i = (int32_t)(d[4] | ((uint32_t)d[5]<<8) | ((uint32_t)d[6]<<16) | ((uint32_t)d[7]<<24));
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    g_gps.lat = lat_i * 1e-7;
    g_gps.lon = lon_i * 1e-7;
    g_gps.fix = 1;
    xSemaphoreGive(g_gps_mutex);
}
/* ─── app_main ──────────────────────────────────────────────────────── */
void app_main(void) {
    ESP_LOGI(TAG, "=== NMEA2000 AP + WebSocket JSON ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_gps_mutex = xSemaphoreCreateMutex();
    g_ws_mutex  = xSemaphoreCreateMutex();
    ws_init();

    wifi_ap_init();
    http_start();

    xTaskCreate(broadcast_task, "broadcast", 4096, NULL, 5, NULL);

    /* CAN */
    free_slots     = xSemaphoreCreateCounting(RX_POOL_DEPTH, RX_POOL_DEPTH);
    pending_frames = xSemaphoreCreateCounting(RX_POOL_DEPTH, 0);
    for (int i = 0; i < RX_POOL_DEPTH; i++) {
        rx_pool[i].frame.buffer     = rx_pool[i].data;
        rx_pool[i].frame.buffer_len = 8;
    }

    twai_onchip_node_config_t ncfg = {
        .io_cfg = { .tx=CAN_TX_GPIO, .rx=CAN_RX_GPIO,
                    .quanta_clk_out=GPIO_NUM_NC,
                    .bus_off_indicator=GPIO_NUM_NC },
        .bit_timing     = { .bitrate = CAN_BITRATE },
        .tx_queue_depth = 4,
        .flags          = { .enable_listen_only = false },
    };
    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&ncfg, &node));

    twai_mask_filter_config_t af = { .id=0, .mask=0, .is_ext=true, .no_fd=true };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node, 0, &af));

    twai_event_callbacks_t cbs = { .on_rx_done=on_rx, .on_error=on_err };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));

    ESP_LOGI(TAG, "TWAI GPIO TX=%d RX=%d @ %d bps", CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);
    ESP_LOGI(TAG, ">>> Wi-Fi: %s / %s  <<<", AP_SSID, AP_PASSWORD);
    ESP_LOGI(TAG, ">>> Ouvre: http://192.168.4.1  <<<");

    while (1) {
        if (xSemaphoreTake(pending_frames, pdMS_TO_TICKS(100)) == pdTRUE) {
            rx_item_t *item = &rx_pool[read_idx];
            read_idx = (read_idx + 1) % RX_POOL_DEPTH;
            uint8_t prio, src; uint32_t pgn;
            parse_can_id(item->frame.header.id, &prio, &pgn, &src);
            const uint8_t *raw = item->frame.buffer;
            uint8_t dlc = item->frame.header.dlc;
            xSemaphoreGive(free_slots);

            const uint8_t *payload = NULL; uint8_t plen = 0;
            if (pgn == PGN_GNSS_POSITION || pgn == PGN_TIME_DATE) {
                payload = fp_feed(src, pgn, raw, &plen);
                if (!payload) continue;
            } else { payload = raw; plen = dlc; }

    switch (pgn) {
    case PGN_COG_SOG:
        decode_cog_sog(payload, plen);
        ESP_LOGI(TAG, "RX COG=%.1f SOG=%.2fkn", g_gps.cog_deg, g_gps.sog_kn);
        break;
    case PGN_VESSEL_HEADING:
        decode_heading(payload, plen);
        ESP_LOGI(TAG, "RX HDG=%.1f", g_gps.heading_deg);
        break;
    case PGN_GNSS_DOPS:
        decode_dops(payload, plen);
        break;
    case PGN_GNSS_POSITION:
        decode_position(payload, plen);
        ESP_LOGI(TAG, "RX POS lat=%.7f lon=%.7f", g_gps.lat, g_gps.lon);
        break;
    case PGN_TIME_DATE:
        decode_time(payload, plen);
        break;
    case PGN_POSITION_RAPID:
        decode_position_rapid(payload, plen);
        ESP_LOGI(TAG, "RX POS_RAPID lat=%.7f lon=%.7f", g_gps.lat, g_gps.lon);
        break;
    default:
        ESP_LOGI(TAG, "RX PGN=%"PRIu32" src=0x%02X data=%02X%02X%02X%02X%02X%02X%02X%02X",
                pgn, src,
                raw[0],raw[1],raw[2],raw[3],
                raw[4],raw[5],raw[6],raw[7]);
        break;
    }
        }
    }
}