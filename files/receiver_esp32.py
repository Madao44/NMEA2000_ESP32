"""
╔══════════════════════════════════════════════════════════════════════════════╗
║        ESP32 – RÉCEPTEUR NMEA2000  (Décodeur CAN / GPS Sportnav SPO25)     ║
║                                                                              ║
║  Matériel :                                                                  ║
║    • ESP32 (WROOM-32 ou S3)                                                  ║
║    • Transceiver CAN : MCP2551 ou SN65HVD230                                ║
║      - TX  → GPIO 4   (configurable via CAN_TX_PIN)                         ║
║      - RX  → GPIO 5   (configurable via CAN_RX_PIN)                         ║
║    • (optionnel) écran OLED SSD1306 sur I2C pour affichage live              ║
║                                                                              ║
║  Firmware : MicroPython ≥ 1.22  avec driver machine.CAN                     ║
║  Flashage : mpremote copy receiver_esp32.py :main.py                        ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import struct
import math
import time
from machine import CAN, Pin

# ─── Configuration matérielle ─────────────────────────────────────────────────
CAN_TX_PIN   = 4
CAN_RX_PIN   = 5
CAN_BAUDRATE = 250_000

# ─── Constantes PGN ───────────────────────────────────────────────────────────
PGN_GNSS_POSITION    = 129029
PGN_COG_SOG          = 129026
PGN_VESSEL_HEADING   = 127250
PGN_SPEED            = 128259
PGN_TIME_DATE        = 129033
PGN_GNSS_DOPS        = 129539
PGN_ISO_ADDRESS_CLAIM= 60928

PGN_NAMES = {
    PGN_GNSS_POSITION:    "GNSS Position",
    PGN_COG_SOG:          "COG & SOG",
    PGN_VESSEL_HEADING:   "Vessel Heading",
    PGN_SPEED:            "Speed",
    PGN_TIME_DATE:        "Time & Date",
    PGN_GNSS_DOPS:        "GNSS DOPs",
    PGN_ISO_ADDRESS_CLAIM:"ISO Addr Claim",
}

# PGNs qui utilisent le protocole Fast-Packet (payload > 8 bytes)
FAST_PACKET_PGNS = {PGN_GNSS_POSITION, PGN_TIME_DATE}


# ═══════════════════════════════════════════════════════════════════════════════
# Parseur de l'ID CAN 29 bits
# ═══════════════════════════════════════════════════════════════════════════════

def parse_can_id(can_id):
    priority = (can_id >> 26) & 0x07
    pgn_raw  = (can_id >> 8)  & 0x3FFFF
    src      =  can_id        & 0xFF
    pf       = (pgn_raw >> 8) & 0xFF

    if pf < 0xF0:
        dst = pgn_raw & 0xFF
        pgn = pgn_raw & 0x1FF00
    else:
        dst = 0xFF
        pgn = pgn_raw

    return priority, pgn, src, dst


# ═══════════════════════════════════════════════════════════════════════════════
# Réassembleur Fast-Packet
# ═══════════════════════════════════════════════════════════════════════════════

class FastPacketAssembler:
    def __init__(self):
        self._buf = {}

    def feed(self, src, pgn, frame):
        if len(frame) < 2:
            return None
        seq_id    = (frame[0] >> 5) & 0x07
        frame_num =  frame[0]       & 0x1F
        key       = (src, pgn, seq_id)

        if frame_num == 0:
            total                = frame[1]
            self._buf[key]       = {"total": total, "frames": {0: frame[2:]}}
        else:
            if key not in self._buf:
                return None
            self._buf[key]["frames"][frame_num] = frame[1:]

        buf        = self._buf[key]
        total      = buf["total"]
        n_expected = self._n_frames(total)

        if len(buf["frames"]) < n_expected:
            return None

        payload = b""
        for i in range(n_expected):
            payload += buf["frames"].get(i, b"")
        del self._buf[key]
        return payload[:total]

    @staticmethod
    def _n_frames(total):
        if total <= 6:
            return 1
        return 1 + math.ceil((total - 6) / 7)


# ═══════════════════════════════════════════════════════════════════════════════
# Décodeurs de PGN
# ═══════════════════════════════════════════════════════════════════════════════

def _rad2deg(r):
    return r * 180.0 / math.pi

def decode_gnss_position(payload):
    """PGN 129029 – Position GNSS complète."""
    if len(payload) < 28:
        return None
    try:
        sid, days, secs_i = struct.unpack_from("<BHQ", payload, 0)
        lat_i, lon_i, alt_i = struct.unpack_from("<qqq", payload, 11)[0:3]
        # Nota : les entiers signés 64 bits font exactement 8 bytes
        lat_i = struct.unpack_from("<q", payload, 11)[0]
        lon_i = struct.unpack_from("<q", payload, 19)[0]
        # alt sur 4 bytes dans la spec réelle ; ici on lit en 32 bits signé
        alt_i = struct.unpack_from("<i", payload, 27)[0] if len(payload) > 30 else 0
        return {
            "sid"  : sid,
            "days" : days,
            "secs" : secs_i / 10000.0,
            "lat"  : lat_i  * 1e-7,
            "lon"  : lon_i  * 1e-7,
            "alt"  : alt_i  * 1e-6,
        }
    except Exception as e:
        return {"error": str(e)}

def decode_cog_sog(payload):
    """PGN 129026 – COG & SOG."""
    try:
        sid, cog_ref, cog_i, sog_i = struct.unpack_from("<BBHH", payload, 0)
        return {
            "sid"    : sid,
            "cog_ref": "True" if cog_ref == 0 else "Magnetic",
            "cog_deg": _rad2deg(cog_i * 1e-4),
            "sog_kn" : (sog_i * 1e-4) / 0.514444,
        }
    except Exception as e:
        return {"error": str(e)}

def decode_vessel_heading(payload):
    """PGN 127250 – Cap du navire."""
    try:
        sid, ref, hdg_i, dev_i, var_i = struct.unpack_from("<BBHHHx", payload, 0)
        return {
            "sid"       : sid,
            "ref"       : "True" if ref == 0 else "Magnetic",
            "heading_deg": _rad2deg(hdg_i * 1e-4),
            "dev_deg"   : _rad2deg(dev_i * 1e-4),
            "var_deg"   : _rad2deg(var_i * 1e-4),
        }
    except Exception as e:
        return {"error": str(e)}

def decode_speed(payload):
    """PGN 128259 – Vitesse."""
    try:
        sid, sw_i, sg_i = struct.unpack_from("<BHH", payload, 0)
        return {
            "sid"          : sid,
            "speed_water_kn" : (sw_i * 0.01) / 0.514444,
            "speed_ground_kn": (sg_i * 0.01) / 0.514444,
        }
    except Exception as e:
        return {"error": str(e)}

def decode_time_date(payload):
    """PGN 129033 – Date & heure UTC."""
    try:
        days, secs_i = struct.unpack_from("<HQ", payload, 0)
        secs         = secs_i / 10000.0
        h = int(secs // 3600)
        m = int((secs % 3600) // 60)
        s = secs % 60
        return {
            "days"     : days,
            "utc_time" : f"{h:02d}:{m:02d}:{s:05.2f}",
        }
    except Exception as e:
        return {"error": str(e)}

def decode_gnss_dops(payload):
    """PGN 129539 – DOPs."""
    try:
        sid, mode, hdop_i, vdop_i, tdop_i = struct.unpack_from("<BBhhh", payload, 0)
        return {
            "sid"  : sid,
            "hdop" : hdop_i * 0.01,
            "vdop" : vdop_i * 0.01,
            "tdop" : tdop_i * 0.01,
        }
    except Exception as e:
        return {"error": str(e)}

def decode_address_claim(payload):
    """PGN 60928 – ISO Address Claim / NAME 64 bits."""
    try:
        name = struct.unpack_from("<Q", payload, 0)[0]
        return {
            "unique_num"    : name         & 0x1FFFFF,
            "manufacturer"  : (name >> 21) & 0x07FF,
            "device_func"   : (name >> 40) & 0xFF,
            "device_class"  : (name >> 48) & 0x7F,
            "industry_group": (name >> 60) & 0x07,
        }
    except Exception as e:
        return {"error": str(e)}

DECODERS = {
    PGN_GNSS_POSITION:    decode_gnss_position,
    PGN_COG_SOG:          decode_cog_sog,
    PGN_VESSEL_HEADING:   decode_vessel_heading,
    PGN_SPEED:            decode_speed,
    PGN_TIME_DATE:        decode_time_date,
    PGN_GNSS_DOPS:        decode_gnss_dops,
    PGN_ISO_ADDRESS_CLAIM:decode_address_claim,
}


# ═══════════════════════════════════════════════════════════════════════════════
# Affichage console formaté
# ═══════════════════════════════════════════════════════════════════════════════

def print_decoded(pgn, src, data):
    name = PGN_NAMES.get(pgn, f"PGN {pgn}")
    print(f"  ┌─ {name}  (src=0x{src:02X})")
    if isinstance(data, dict):
        for k, v in data.items():
            if k == "error":
                print(f"  │   ⚠ {v}")
            elif isinstance(v, float):
                print(f"  │   {k:20s} = {v:.6f}")
            else:
                print(f"  │   {k:20s} = {v}")
    print("  └" + "─" * 40)


# ═══════════════════════════════════════════════════════════════════════════════
# État GPS courant (mis à jour à chaque réception)
# ═══════════════════════════════════════════════════════════════════════════════

class GPSState:
    """Agrège les données GPS reçues sur le bus."""
    lat = lon = alt = cog = sog = heading = 0.0
    hdop = vdop = tdop = 99.9
    utc_time = "??:??:??"
    fix = False
    src = 0xFF

    def update_from_pgn(self, pgn, data):
        if "error" in data:
            return
        if pgn == PGN_GNSS_POSITION:
            self.lat = data.get("lat", self.lat)
            self.lon = data.get("lon", self.lon)
            self.alt = data.get("alt", self.alt)
            self.fix = True
        elif pgn == PGN_COG_SOG:
            self.cog = data.get("cog_deg", self.cog)
            self.sog = data.get("sog_kn",  self.sog)
        elif pgn == PGN_VESSEL_HEADING:
            self.heading = data.get("heading_deg", self.heading)
        elif pgn == PGN_GNSS_DOPS:
            self.hdop = data.get("hdop", self.hdop)
            self.vdop = data.get("vdop", self.vdop)
            self.tdop = data.get("tdop", self.tdop)
        elif pgn == PGN_TIME_DATE:
            self.utc_time = data.get("utc_time", self.utc_time)

    def display_summary(self):
        fix_str = "FIX OK" if self.fix else "NO FIX"
        print(f"\n  ╔══ GPS Summary [{fix_str}] ══════════════════════")
        print(f"  ║  Position : {self.lat:.6f}°N  {self.lon:.6f}°E  alt={self.alt:.1f}m")
        print(f"  ║  COG={self.cog:.1f}°  SOG={self.sog:.2f}kn  HDG={self.heading:.1f}°")
        print(f"  ║  HDOP={self.hdop:.2f}  VDOP={self.vdop:.2f}  UTC={self.utc_time}")
        print(f"  ╚{'═'*48}\n")


# ═══════════════════════════════════════════════════════════════════════════════
# Boucle principale
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print(" ESP32 – Récepteur NMEA2000  (Décodeur CAN)")
    print("=" * 60)

    can = CAN(0,
              mode=CAN.NORMAL,
              baudrate=CAN_BAUDRATE,
              tx=Pin(CAN_TX_PIN),
              rx=Pin(CAN_RX_PIN),
              extframe=True)

    assembler = FastPacketAssembler()
    gps_state = GPSState()
    last_summary = time.ticks_ms()
    SUMMARY_INTERVAL = 5000         # afficher le résumé toutes les 5 s

    print("[INFO] En attente de trames NMEA2000…\n")

    while True:
        # ── Lecture non-bloquante : peut renvoyer None ─────────────────────────
        msg = can.recv(timeout=100)   # timeout 100 ms

        if msg is not None:
            can_id, rtr, fmi, data = msg
            priority, pgn, src, dst = parse_can_id(can_id)

            print(f"[{time.ticks_ms():8d} ms]  RX  ID=0x{can_id:08X}  PGN={pgn:6d}  src=0x{src:02X}  {bytes(data).hex()}")

            # ── Fast-Packet ou frame unique ? ────────────────────────────────
            if pgn in FAST_PACKET_PGNS:
                payload = assembler.feed(src, pgn, bytes(data))
                if payload is None:
                    continue        # attente de la suite du message
            else:
                payload = bytes(data)

            # ── Décodage ─────────────────────────────────────────────────────
            decoder = DECODERS.get(pgn)
            if decoder:
                decoded = decoder(payload)
                print_decoded(pgn, src, decoded)
                gps_state.src = src
                gps_state.update_from_pgn(pgn, decoded)
            else:
                print(f"  ⚠ PGN {pgn} non supporté  payload={payload.hex()}")

        # ── Résumé périodique ─────────────────────────────────────────────────
        if time.ticks_diff(time.ticks_ms(), last_summary) >= SUMMARY_INTERVAL:
            gps_state.display_summary()
            last_summary = time.ticks_ms()


if __name__ == "__main__":
    main()
