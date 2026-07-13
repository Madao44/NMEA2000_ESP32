"""
╔══════════════════════════════════════════════════════════════════════════════╗
║    Simulation PC  –  Test de bout en bout sans matériel                     ║
║                                                                              ║
║  Ce script tourne sur CPython (bureau / PC) et simule l'intégralité du      ║
║  pipeline :                                                                  ║
║    1. Simulation GPS (trajectoire circulaire – Port de Barcelone)            ║
║    2. Encodage NMEA2000 (PGNs du Sportnav SPO25)                             ║
║    3. Fragmentation Fast-Packet                                               ║
║    4. Réassemblage Fast-Packet                                                ║
║    5. Décodage des PGNs                                                      ║
║    6. Affichage des résultats                                                 ║
║                                                                              ║
║  Dépendances : aucune (stdlib uniquement)                                    ║
║  Usage       : python simulation_pc.py [--cycles N] [--delay S]             ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import argparse
import math
import struct
import time
import sys
from datetime import datetime, timezone, date as date_t
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Callable

# ─── Imports des modules du projet ────────────────────────────────────────────
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent))

from common.nmea2000_pgn import (
    PGN_GNSS_POSITION, PGN_COG_SOG, PGN_VESSEL_HEADING,
    PGN_SPEED, PGN_TIME_DATE, PGN_GNSS_DOPS, PGN_ISO_ADDRESS_CLAIM,
    PRIORITY, SEND_INTERVAL_MS, PGN_NAMES,
    build_can_id, parse_can_id,
)
from common.nmea2000_encoder import (
    encode_gnss_position, encode_cog_sog, encode_vessel_heading,
    encode_speed, encode_time_date, encode_gnss_dops, encode_address_claim,
)
from common.fast_packet import fragment_fast_packet, FastPacketAssembler


# ═══════════════════════════════════════════════════════════════════════════════
# Couleurs ANSI (terminal)
# ═══════════════════════════════════════════════════════════════════════════════
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    CYAN   = "\033[36m"
    GREEN  = "\033[32m"
    YELLOW = "\033[33m"
    RED    = "\033[31m"
    BLUE   = "\033[34m"
    GREY   = "\033[90m"
    MAG    = "\033[35m"

def colored(text, *colors):
    return "".join(colors) + str(text) + C.RESET


# ═══════════════════════════════════════════════════════════════════════════════
# GPS simulé
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SimGPS:
    lat:       float = 41.3809
    lon:       float = 2.1734
    alt:       float = 2.0
    sog_kn:    float = 6.5
    cog_deg:   float = 0.0
    heading:   float = 0.0
    hdop:      float = 0.9
    pdop:      float = 1.4
    tdop:      float = 1.1
    num_svs:   int   = 10
    sid:       int   = 0
    _t0:       float = field(default_factory=time.time)

    def update(self):
        t      = time.time() - self._t0
        R      = 500.0
        omega  = 2 * math.pi / 120.0
        self.lat    = 41.3809 + (R * math.cos(omega * t)) / 111320.0
        self.lon    = 2.1734  + (R * math.sin(omega * t)) / (111320.0 * math.cos(math.radians(41.38)))
        self.cog_deg  = (math.degrees(omega * t) + 90) % 360
        self.heading  = self.cog_deg
        self.sid      = (self.sid + 1) & 0xFF

    def utc_days_and_secs(self):
        dt   = datetime.now(timezone.utc)
        days = (dt.date() - date_t(1970, 1, 1)).days
        secs = dt.hour * 3600 + dt.minute * 60 + dt.second + dt.microsecond / 1e6
        return days, secs


# ═══════════════════════════════════════════════════════════════════════════════
# Bus CAN virtuel (en mémoire)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class CANFrame:
    can_id:    int
    data:      bytes
    timestamp: float = field(default_factory=time.time)

class VirtualCANBus:
    """Simule un bus CAN en mémoire (pas de matériel nécessaire)."""

    def __init__(self):
        self._queue:    List[CANFrame] = []
        self.tx_count:  int = 0
        self.rx_count:  int = 0

    def send(self, data: bytes, can_id: int):
        frame = CANFrame(can_id=can_id, data=data[:8].ljust(8, b'\xFF'))
        self._queue.append(frame)
        self.tx_count += 1

    def recv(self) -> Optional[CANFrame]:
        if self._queue:
            self.rx_count += 1
            return self._queue.pop(0)
        return None

    def pending(self) -> int:
        return len(self._queue)


# ═══════════════════════════════════════════════════════════════════════════════
# Émetteur
# ═══════════════════════════════════════════════════════════════════════════════

class NMEASender:
    SRC = 0x23     # adresse simulée du SPO25

    FAST_PACKET_PGNS = {PGN_GNSS_POSITION, PGN_TIME_DATE}

    def __init__(self, bus: VirtualCANBus):
        self.bus     = bus
        self.gps     = SimGPS()
        self.seq_id  = 0
        self._last   = {pgn: -SEND_INTERVAL_MS[pgn] / 1000.0
                        for pgn in SEND_INTERVAL_MS}

    def _send_frame(self, pgn: int, payload: bytes):
        prio   = PRIORITY[pgn]
        can_id = build_can_id(prio, pgn, self.SRC)

        if pgn in self.FAST_PACKET_PGNS:
            frames = fragment_fast_packet(pgn, payload, self.seq_id)
            self.seq_id = (self.seq_id + 1) & 0x07
            for frame in frames:
                self.bus.send(frame, can_id)
                print(colored(f"  ↑ TX FP  PGN {pgn:6d}  ID=0x{can_id:08X}  {frame.hex()}", C.BLUE))
        else:
            data = payload[:8].ljust(8, b'\xFF')
            self.bus.send(data, can_id)
            print(colored(f"  ↑ TX     PGN {pgn:6d}  ID=0x{can_id:08X}  {data.hex()}", C.CYAN))

    def tick(self):
        """Appeler régulièrement pour émettre les PGNs selon leur intervalle."""
        now = time.time()
        self.gps.update()
        days, secs = self.gps.utc_days_and_secs()

        schedule = {
            PGN_GNSS_POSITION:   lambda: encode_gnss_position(
                                      self.gps.lat, self.gps.lon, self.gps.alt,
                                      self.gps.sid, days, secs,
                                      num_svs=self.gps.num_svs,
                                      hdop=self.gps.hdop, pdop=self.gps.pdop),
            PGN_COG_SOG:         lambda: encode_cog_sog(self.gps.cog_deg, self.gps.sog_kn, self.gps.sid),
            PGN_VESSEL_HEADING:  lambda: encode_vessel_heading(self.gps.heading, sid=self.gps.sid),
            PGN_SPEED:           lambda: encode_speed(self.gps.sog_kn, self.gps.sog_kn, self.gps.sid),
            PGN_TIME_DATE:       lambda: encode_time_date(),
            PGN_GNSS_DOPS:       lambda: encode_gnss_dops(self.gps.hdop, self.gps.pdop, self.gps.tdop, self.gps.sid),
        }

        for pgn, builder in schedule.items():
            interval = SEND_INTERVAL_MS[pgn] / 1000.0
            if now - self._last[pgn] >= interval:
                pgn_name = PGN_NAMES.get(pgn, str(pgn))
                print(colored(f"\n[SENDER  {now:.3f}s] {pgn_name}", C.BOLD, C.CYAN))
                self._send_frame(pgn, builder())
                self._last[pgn] = now

    def send_address_claim(self):
        pgn    = PGN_ISO_ADDRESS_CLAIM
        prio   = PRIORITY[pgn]
        can_id = build_can_id(prio, pgn, self.SRC)
        data   = encode_address_claim()
        self.bus.send(data, can_id)
        print(colored(f"  ↑ TX     PGN {pgn:6d}  ID=0x{can_id:08X}  {data.hex()} [Address Claim]", C.MAG))


# ═══════════════════════════════════════════════════════════════════════════════
# Récepteur
# ═══════════════════════════════════════════════════════════════════════════════

def _rad2deg(r): return r * 180.0 / math.pi

class NMEAReceiver:
    """Lit le bus CAN virtuel et décode les PGNs NMEA2000."""

    FAST_PGNS = {PGN_GNSS_POSITION, PGN_TIME_DATE}

    def __init__(self, bus: VirtualCANBus):
        self.bus      = bus
        self.asm      = FastPacketAssembler()
        self.gps_state: Dict = {
            "lat": None, "lon": None, "alt": None,
            "cog": None, "sog": None, "heading": None,
            "hdop": None, "utc": None,
        }

    def tick(self):
        """Draine toutes les trames CAN disponibles."""
        while (frame := self.bus.recv()) is not None:
            self._process(frame)

    def _process(self, frame: CANFrame):
        info   = parse_can_id(frame.can_id)
        pgn    = info["pgn"]
        src    = info["src"]
        data   = frame.data

        if pgn in self.FAST_PGNS:
            payload = self.asm.feed(src, pgn, data)
            if payload is None:
                print(colored(f"  ↓ RX FP  PGN {pgn:6d}  [fragment en attente]", C.GREY))
                return
        else:
            payload = data

        print(colored(f"  ↓ RX     PGN {pgn:6d}  src=0x{src:02X}  {payload.hex()}", C.GREEN))
        self._decode(pgn, src, payload)

    def _decode(self, pgn: int, src: int, payload: bytes):
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")
        result = {}

        try:
            if pgn == PGN_GNSS_POSITION:
                sid, days, secs_i = struct.unpack_from("<BHQ", payload, 0)
                lat_i = struct.unpack_from("<q", payload, 11)[0]
                lon_i = struct.unpack_from("<q", payload, 19)[0]
                alt_i = struct.unpack_from("<i", payload, 27)[0] if len(payload) > 30 else 0
                result = {
                    "lat": f"{lat_i * 1e-7:.7f}°",
                    "lon": f"{lon_i * 1e-7:.7f}°",
                    "alt": f"{alt_i * 1e-6:.2f} m",
                    "utc": f"{days}j + {secs_i/10000:.4f}s",
                }
                self.gps_state.update({
                    "lat": lat_i * 1e-7,
                    "lon": lon_i * 1e-7,
                    "alt": alt_i * 1e-6,
                })

            elif pgn == PGN_COG_SOG:
                sid, cog_ref, cog_i, sog_i = struct.unpack_from("<BBHH", payload, 0)
                cog = _rad2deg(cog_i * 1e-4)
                sog = (sog_i * 1e-4) / 0.514444
                result = {"COG": f"{cog:.2f}°", "SOG": f"{sog:.2f} kn",
                          "ref": "True" if cog_ref == 0 else "Mag"}
                self.gps_state.update({"cog": cog, "sog": sog})

            elif pgn == PGN_VESSEL_HEADING:
                sid, ref, hdg_i = struct.unpack_from("<BBH", payload, 0)
                hdg = _rad2deg(hdg_i * 1e-4)
                result = {"heading": f"{hdg:.2f}°",
                          "ref": "True" if ref == 0 else "Mag"}
                self.gps_state["heading"] = hdg

            elif pgn == PGN_SPEED:
                sid, sw, sg = struct.unpack_from("<BHH", payload, 0)
                result = {"speed_water": f"{(sw*0.01)/0.514444:.2f} kn",
                          "speed_ground": f"{(sg*0.01)/0.514444:.2f} kn"}

            elif pgn == PGN_TIME_DATE:
                days, secs_i = struct.unpack_from("<HQ", payload, 0)
                secs = secs_i / 10000.0
                h, m = int(secs // 3600), int((secs % 3600) // 60)
                s = secs % 60
                utc_str = f"{h:02d}:{m:02d}:{s:05.2f}"
                result = {"date_days": days, "utc": utc_str}
                self.gps_state["utc"] = utc_str

            elif pgn == PGN_GNSS_DOPS:
                sid, mode, hdop, vdop, tdop = struct.unpack_from("<BBhhh", payload, 0)
                result = {"HDOP": hdop*0.01, "VDOP": vdop*0.01, "TDOP": tdop*0.01}
                self.gps_state["hdop"] = hdop * 0.01

            elif pgn == PGN_ISO_ADDRESS_CLAIM:
                name_i = struct.unpack_from("<Q", payload, 0)[0]
                result = {
                    "manuf_code": f"0x{(name_i>>21)&0x7FF:04X}",
                    "device_func": (name_i>>40)&0xFF,
                    "device_class": (name_i>>48)&0x7F,
                }

        except Exception as e:
            result = {"⚠ error": str(e)}

        # Affichage
        print(colored(f"      ┌─ {name}  (src=0x{src:02X})", C.YELLOW, C.BOLD))
        for k, v in result.items():
            print(colored(f"      │  {k:<20} = {v}", C.YELLOW))
        print(colored(f"      └{'─'*40}", C.YELLOW))

    def print_summary(self):
        s = self.gps_state
        print(colored("\n  ╔══════════════ GPS State ═══════════════════╗", C.GREEN, C.BOLD))
        if s["lat"] is not None:
            print(colored(f"  ║  Pos : {s['lat']:.7f}°N  {s['lon']:.7f}°E", C.GREEN))
            print(colored(f"  ║  Alt : {s['alt']:.2f} m", C.GREEN))
        if s["cog"] is not None:
            print(colored(f"  ║  COG : {s['cog']:.2f}°   SOG : {s['sog']:.2f} kn", C.GREEN))
        if s["heading"] is not None:
            print(colored(f"  ║  HDG : {s['heading']:.2f}°", C.GREEN))
        if s["hdop"] is not None:
            print(colored(f"  ║  HDOP: {s['hdop']:.2f}", C.GREEN))
        if s["utc"] is not None:
            print(colored(f"  ║  UTC : {s['utc']}", C.GREEN))
        print(colored("  ╚═══════════════════════════════════════════╝\n", C.GREEN, C.BOLD))


# ═══════════════════════════════════════════════════════════════════════════════
# Point d'entrée
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Simulation NMEA2000 – SPO25 sur bus CAN virtuel")
    ap.add_argument("--cycles", type=int, default=20,   help="Nombre de cycles de simulation")
    ap.add_argument("--delay",  type=float, default=0.25, help="Délai entre cycles (secondes)")
    args = ap.parse_args()

    print(colored("=" * 65, C.BOLD))
    print(colored("  Simulation NMEA2000 – GPS Sportnav SPO25", C.BOLD, C.MAG))
    print(colored("  Bus CAN virtuel  |  250 kbit/s  |  29 bits extended", C.GREY))
    print(colored("=" * 65, C.BOLD))

    bus      = VirtualCANBus()
    sender   = NMEASender(bus)
    receiver = NMEAReceiver(bus)

    # Address claim au démarrage
    print(colored("\n[INIT] ISO Address Claim…", C.MAG, C.BOLD))
    sender.send_address_claim()
    receiver.tick()

    for cycle in range(1, args.cycles + 1):
        print(colored(f"\n{'─'*65}", C.GREY))
        print(colored(f" Cycle {cycle:3d}/{args.cycles}", C.BOLD))
        print(colored(f"{'─'*65}", C.GREY))

        sender.tick()
        receiver.tick()

        if cycle % 8 == 0 or cycle == args.cycles:
            receiver.print_summary()

        time.sleep(args.delay)

    print(colored("\n[FIN]  Statistiques bus CAN virtuel", C.BOLD))
    print(f"  Trames TX : {bus.tx_count}")
    print(f"  Trames RX : {bus.rx_count}")
    print(colored("Simulation terminée.\n", C.GREEN, C.BOLD))


if __name__ == "__main__":
    main()
