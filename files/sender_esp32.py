"""
╔══════════════════════════════════════════════════════════════════════════════╗
║        ESP32 – ÉMETTEUR NMEA2000  (GPS Sportnav SPO25 Simulator)           ║
║                                                                              ║
║  Matériel :                                                                  ║
║    • ESP32 (WROOM-32 ou S3)                                                  ║
║    • Transceiver CAN : MCP2551 ou SN65HVD230                                ║
║      - TX  → GPIO 4   (configurable via CAN_TX_PIN)                         ║
║      - RX  → GPIO 5   (configurable via CAN_RX_PIN)                         ║
║      - Bus CAN 250 kbit/s (norme NMEA2000)                                  ║
║                                                                              ║
║  Firmware : MicroPython ≥ 1.22  avec driver machine.CAN                     ║
║  Flashage : mpremote copy sender_esp32.py :main.py                          ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import struct
import math
import time
from machine import CAN, Pin

# ─── Configuration matérielle ─────────────────────────────────────────────────
CAN_TX_PIN   = 4
CAN_RX_PIN   = 5
CAN_BAUDRATE = 250_000          # NMEA2000 = 250 kbit/s
MY_SRC_ADDR  = 0x23             # Adresse source simulée du SPO25

# ─── Constantes NMEA2000 ──────────────────────────────────────────────────────
PGN_GNSS_POSITION    = 129029
PGN_COG_SOG          = 129026
PGN_VESSEL_HEADING   = 127250
PGN_SPEED            = 128259
PGN_TIME_DATE        = 129033
PGN_GNSS_DOPS        = 129539
PGN_ISO_ADDRESS_CLAIM= 60928

PRIORITY = {
    PGN_GNSS_POSITION:    3,
    PGN_COG_SOG:          2,
    PGN_VESSEL_HEADING:   2,
    PGN_SPEED:            2,
    PGN_TIME_DATE:        3,
    PGN_GNSS_DOPS:        6,
    PGN_ISO_ADDRESS_CLAIM:6,
}

INTERVAL_MS = {
    PGN_GNSS_POSITION:    1000,
    PGN_COG_SOG:          250,
    PGN_VESSEL_HEADING:   100,
    PGN_SPEED:            1000,
    PGN_TIME_DATE:        1000,
    PGN_GNSS_DOPS:        1000,
    PGN_ISO_ADDRESS_CLAIM:5000,
}


# ═══════════════════════════════════════════════════════════════════════════════
# Données GPS simulées (trajectoire circulaire autour de Barcelone)
# ═══════════════════════════════════════════════════════════════════════════════

class SimulatedGPS:
    """Simule un GPS se déplaçant autour du port de Barcelone."""

    def __init__(self):
        self.lat       = 41.3809     # Port de Barcelone
        self.lon       = 2.1734
        self.alt       = 2.0         # mètres
        self.sog_kn    = 6.5         # vitesse sur le fond (nœuds)
        self.cog_deg   = 0.0         # cap sur le fond (degrés vrais)
        self.heading   = 0.0
        self.hdop      = 0.9
        self.pdop      = 1.4
        self.tdop      = 1.1
        self.num_svs   = 10
        self.sid       = 0
        self._t0       = time.ticks_ms()

    def update(self):
        """Met à jour la position simulée (cercle de 500 m de rayon)."""
        t_s        = time.ticks_diff(time.ticks_ms(), self._t0) / 1000.0
        R          = 500.0           # rayon en mètres
        omega      = 2 * math.pi / 120.0   # tour en 120 s
        # Coordonnées en mètres → conversion approximative en degrés
        self.lat   = 41.3809 + (R * math.cos(omega * t_s)) / 111320.0
        self.lon   = 2.1734  + (R * math.sin(omega * t_s)) / (111320.0 * math.cos(math.radians(41.38)))
        self.cog_deg  = (math.degrees(omega * t_s) + 90) % 360
        self.heading  = self.cog_deg
        self.sid      = (self.sid + 1) & 0xFF

    def utc_days_and_secs(self):
        """Retourne (jours depuis 1970, secondes depuis minuit)."""
        # MicroPython : utilise time.gmtime()
        try:
            gmt  = time.gmtime()
            # Approximation jours depuis 1970 (pas de module datetime)
            days = int(time.time() // 86400)
            secs = gmt[3] * 3600 + gmt[4] * 60 + gmt[5]
        except Exception:
            days = 19900   # ~2024
            secs = 43200
        return days, float(secs)


# ═══════════════════════════════════════════════════════════════════════════════
# Encodeurs inline (version allégée pour MicroPython sans modules externes)
# ═══════════════════════════════════════════════════════════════════════════════

def _deg2rad(d):
    return d * math.pi / 180.0

def encode_cog_sog(gps):
    cog_i = int(_deg2rad(gps.cog_deg) / 1e-4) & 0xFFFF
    sog_i = int((gps.sog_kn * 0.514444) / 1e-4) & 0xFFFF
    return struct.pack("<BBHH2s", gps.sid, 0, cog_i, sog_i, b'\xFF\xFF')

def encode_vessel_heading(gps):
    hdg = int(_deg2rad(gps.heading) / 1e-4) & 0xFFFF
    return struct.pack("<BBHHHx", gps.sid, 1, hdg, 0, 0)

def encode_speed(gps):
    sw = int((gps.sog_kn * 0.514444) / 0.01) & 0xFFFF
    sg = sw
    return struct.pack("<BHH3s", gps.sid, sw, sg, b'\xFF\xFF\xFF')

def encode_gnss_dops(gps):
    mode = (3 & 0x07) | ((3 & 0x07) << 3)
    return struct.pack("<BBhhh",
        gps.sid, mode,
        int(gps.hdop / 0.01),
        int(gps.pdop / 0.01),
        int(gps.tdop / 0.01),
    )[:8].ljust(8, b'\xFF')

def encode_address_claim():
    """NAME 64 bits du SPO25 fictif."""
    name  =  (0x1A2B3C   & 0x1FFFFF)
    name |=  (0x0099     & 0x07FF) << 21
    name |=  (0          & 0xFF)   << 32
    name |=  (145        & 0xFF)   << 40
    name |=  (60         & 0x7F)   << 48
    name |=  (0          & 0x0F)   << 56
    name |=  (4          & 0x07)   << 60
    name |=  (1          & 0x01)   << 63
    return struct.pack("<Q", name)

def encode_gnss_position(gps):
    """Encode PGN 129029 – payload 43 bytes (fast-packet)."""
    days, secs = gps.utc_days_and_secs()
    lat_i  = int(gps.lat / 1e-7)
    lon_i  = int(gps.lon / 1e-7)
    alt_i  = int(gps.alt / 1e-6)
    secs_i = int(secs * 10000)
    hdop_i = int(gps.hdop / 0.01)
    pdop_i = int(gps.pdop / 0.01)
    geo_i  = int(48.0    / 0.01)

    payload = struct.pack(
        "<BHQqqihhhhB",
        gps.sid, days & 0xFFFF, secs_i,
        lat_i, lon_i, alt_i,
        (0) | (1 << 4),    # GPS, fixe 3D
        0, gps.num_svs, hdop_i,
    )
    payload += struct.pack("<hhhB", pdop_i, geo_i, 0, 0xFF)
    return payload[:43].ljust(43, b'\xFF')

def encode_time_date(gps):
    days, secs = gps.utc_days_and_secs()
    secs_i = int(secs * 10000)
    return struct.pack("<HQ", days & 0xFFFF, secs_i)[:8].ljust(8, b'\xFF')


# ═══════════════════════════════════════════════════════════════════════════════
# Fast-Packet fragmentation
# ═══════════════════════════════════════════════════════════════════════════════

def fragment_fast_packet(payload, seq_id=0):
    seq_id    = seq_id & 0x07
    total_len = len(payload)
    frames    = []
    frame_num = 0
    offset    = 0

    # Frame 0
    header = bytes([(seq_id << 5) | (frame_num & 0x1F), total_len & 0xFF])
    chunk  = payload[offset:offset + 6].ljust(6, b'\xFF')
    frames.append(header + chunk)
    offset += 6; frame_num += 1

    while offset < total_len:
        seq_byte = (seq_id << 5) | (frame_num & 0x1F)
        chunk    = payload[offset:offset + 7].ljust(7, b'\xFF')
        frames.append(bytes([seq_byte]) + chunk)
        offset += 7; frame_num += 1

    return frames


# ═══════════════════════════════════════════════════════════════════════════════
# Construction de l'ID CAN 29 bits
# ═══════════════════════════════════════════════════════════════════════════════

def build_can_id(priority, pgn, src, dst=0xFF):
    pf = (pgn >> 8) & 0xFF
    if pf < 0xF0:
        can_pgn = (pgn & 0x1FF00) | (dst & 0xFF)
    else:
        can_pgn = pgn & 0x1FFFF

    can_id  = (priority & 0x07) << 26
    can_id |= (can_pgn  & 0x3FFFF) << 8
    can_id |= (src       & 0xFF)
    return can_id


# ═══════════════════════════════════════════════════════════════════════════════
# Envoi sur le bus CAN
# ═══════════════════════════════════════════════════════════════════════════════

def send_single_frame(can_bus, pgn, payload, src, priority, dst=0xFF):
    """Envoie un message CAN frame unique (≤ 8 bytes)."""
    can_id = build_can_id(priority, pgn, src, dst)
    data   = payload[:8].ljust(8, b'\xFF')
    can_bus.send(data, can_id, extframe=True)
    print(f"  TX  PGN {pgn:6d}  ID=0x{can_id:08X}  {data.hex()}")

def send_fast_packet(can_bus, pgn, payload, src, priority, seq_id=0):
    """Envoie un message NMEA2000 multi-trames (Fast-Packet)."""
    frames = fragment_fast_packet(payload, seq_id)
    can_id = build_can_id(priority, pgn, src)
    for i, frame in enumerate(frames):
        can_bus.send(frame, can_id, extframe=True)
        print(f"  TX  PGN {pgn:6d}  frame[{i}]  ID=0x{can_id:08X}  {frame.hex()}")
        time.sleep_ms(2)            # délai inter-frame réglementaire


# ═══════════════════════════════════════════════════════════════════════════════
# Boucle principale
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print(" ESP32 – Émetteur NMEA2000  (Sportnav SPO25 Simulator)")
    print("=" * 60)

    # Initialisation du bus CAN
    can = CAN(0,
              mode=CAN.NORMAL,
              baudrate=CAN_BAUDRATE,
              tx=Pin(CAN_TX_PIN),
              rx=Pin(CAN_RX_PIN),
              extframe=True)          # trames 29 bits étendues

    gps    = SimulatedGPS()
    seq_id = 0                        # séquence fast-packet

    # Horodatages de dernière émission par PGN
    last_sent = {pgn: -INTERVAL_MS[pgn] for pgn in INTERVAL_MS}

    # ── Address Claim au démarrage ─────────────────────────────────────────────
    print("\n[INIT] ISO Address Claim…")
    send_single_frame(can,
                      PGN_ISO_ADDRESS_CLAIM,
                      encode_address_claim(),
                      MY_SRC_ADDR,
                      PRIORITY[PGN_ISO_ADDRESS_CLAIM])
    time.sleep_ms(250)

    print("[INIT] Démarrage de la simulation GPS\n")

    while True:
        now = time.ticks_ms()
        gps.update()

        for pgn, interval in INTERVAL_MS.items():
            if pgn == PGN_ISO_ADDRESS_CLAIM:
                continue
            if time.ticks_diff(now, last_sent[pgn]) >= interval:

                print(f"[{time.ticks_ms():8d} ms]  Émission PGN {pgn}")

                if pgn == PGN_GNSS_POSITION:
                    send_fast_packet(can, pgn, encode_gnss_position(gps),
                                     MY_SRC_ADDR, PRIORITY[pgn], seq_id)
                    seq_id = (seq_id + 1) & 0x07

                elif pgn == PGN_COG_SOG:
                    send_single_frame(can, pgn, encode_cog_sog(gps),
                                      MY_SRC_ADDR, PRIORITY[pgn])

                elif pgn == PGN_VESSEL_HEADING:
                    send_single_frame(can, pgn, encode_vessel_heading(gps),
                                      MY_SRC_ADDR, PRIORITY[pgn])

                elif pgn == PGN_SPEED:
                    send_single_frame(can, pgn, encode_speed(gps),
                                      MY_SRC_ADDR, PRIORITY[pgn])

                elif pgn == PGN_TIME_DATE:
                    send_fast_packet(can, pgn, encode_time_date(gps),
                                     MY_SRC_ADDR, PRIORITY[pgn], seq_id)
                    seq_id = (seq_id + 1) & 0x07

                elif pgn == PGN_GNSS_DOPS:
                    send_single_frame(can, pgn, encode_gnss_dops(gps),
                                      MY_SRC_ADDR, PRIORITY[pgn])

                last_sent[pgn] = now

        time.sleep_ms(10)            # granularité de la boucle principale


if __name__ == "__main__":
    main()
