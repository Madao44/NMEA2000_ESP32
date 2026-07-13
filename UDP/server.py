"""
Serveur pont UDP → WebSocket
Reçoit les phrases NMEA0183 de l'ESP32 via UDP (port 10110)
et les retransmet aux navigateurs connectés via WebSocket (port 8765)

Dépendances :
    pip install websockets

Usage :
    python server.py
"""

import asyncio
import socket
import websockets
import json
import re
from datetime import datetime

UDP_PORT = 10110
WS_PORT  = 8765

# ─── Clients WebSocket connectés ──────────────────────────────────────────────
connected_clients = set()

# ─── État GPS agrégé ──────────────────────────────────────────────────────────
gps_state = {
    "lat": None, "lon": None, "alt": None,
    "cog": None, "sog": None, "hdg": None,
    "hdop": None, "vdop": None,
    "fix": False, "utc": None, "date": None,
    "num_svs": 0,
    "sentences": [],     # dernières phrases brutes
}

# ─── Parseurs NMEA0183 ────────────────────────────────────────────────────────

def nmea_checksum_ok(sentence: str) -> bool:
    if "*" not in sentence:
        return True   # pas de checksum, on accepte
    body, cs = sentence.strip().lstrip("$").rsplit("*", 1)
    calc = 0
    for c in body:
        calc ^= ord(c)
    return calc == int(cs[:2], 16)

def nmea_lat(s: str, d: str) -> float | None:
    if not s or len(s) < 4:
        return None
    deg = float(s[:2])
    mn  = float(s[2:])
    v   = deg + mn / 60.0
    return -v if d == "S" else v

def nmea_lon(s: str, d: str) -> float | None:
    if not s or len(s) < 5:
        return None
    deg = float(s[:3])
    mn  = float(s[3:])
    v   = deg + mn / 60.0
    return -v if d == "W" else v

def nmea_utc(s: str) -> str | None:
    if not s or len(s) < 6:
        return None
    return f"{s[:2]}:{s[2:4]}:{s[4:6]} UTC"

def parse_gpgga(fields):
    gps_state["utc"]     = nmea_utc(fields[1])
    gps_state["lat"]     = nmea_lat(fields[2], fields[3])
    gps_state["lon"]     = nmea_lon(fields[4], fields[5])
    gps_state["fix"]     = fields[6] in ("1", "2", "4", "5")
    gps_state["num_svs"] = int(fields[7]) if fields[7] else 0
    gps_state["hdop"]    = float(fields[8]) if fields[8] else None
    gps_state["alt"]     = float(fields[9]) if fields[9] else None

def parse_gprmc(fields):
    gps_state["utc"]  = nmea_utc(fields[1])
    gps_state["fix"]  = fields[2] == "A"
    gps_state["lat"]  = nmea_lat(fields[3], fields[4])
    gps_state["lon"]  = nmea_lon(fields[5], fields[6])
    gps_state["sog"]  = float(fields[7]) if fields[7] else None
    gps_state["cog"]  = float(fields[8]) if fields[8] else None
    gps_state["date"] = fields[9] if fields[9] else None

def parse_gpvtg(fields):
    gps_state["cog"] = float(fields[1]) if fields[1] else gps_state["cog"]
    gps_state["sog"] = float(fields[5]) if fields[5] else gps_state["sog"]

def parse_gphdg(fields):
    gps_state["hdg"] = float(fields[1]) if fields[1] else None

def parse_gpgsa(fields):
    gps_state["hdop"] = float(fields[16]) if len(fields) > 16 and fields[16] else gps_state["hdop"]
    gps_state["vdop"] = float(fields[17].split("*")[0]) if len(fields) > 17 and fields[17] else gps_state["vdop"]

PARSERS = {
    "GPGGA": parse_gpgga,
    "GPRMC": parse_gprmc,
    "GPVTG": parse_gpvtg,
    "GPHDG": parse_gphdg,
    "GPGSA": parse_gpgsa,
}

def process_sentence(sentence: str):
    sentence = sentence.strip()
    if not sentence.startswith("$"):
        return
    if not nmea_checksum_ok(sentence):
        print(f"  ⚠ Checksum invalide : {sentence}")
        return

    inner = sentence.lstrip("$").split("*")[0]
    fields = inner.split(",")
    tag = fields[0]

    if tag in PARSERS:
        try:
            PARSERS[tag](fields)
        except Exception as e:
            print(f"  ⚠ Erreur parse {tag}: {e}")

    # Garder les 10 dernières phrases
    gps_state["sentences"] = (gps_state["sentences"] + [sentence])[-10:]

# ─── Récepteur UDP ────────────────────────────────────────────────────────────

async def udp_receiver():
    loop = asyncio.get_event_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)

    print(f"[UDP] Écoute sur port {UDP_PORT}...")

    while True:
        try:
            data, addr = await loop.sock_recvfrom(sock, 1024)
            text = data.decode("ascii", errors="ignore")
            for line in text.splitlines():
                if line.startswith("$"):
                    print(f"[UDP] {addr[0]} → {line}")
                    process_sentence(line)

            # Broadcast aux clients WebSocket
            global connected_clients
            if connected_clients:
                msg = json.dumps({
                    "type":  "gps",
                    "data":  gps_state,
                    "ts":    datetime.utcnow().isoformat(),
                })
                dead = set()
                for ws in connected_clients:
                    try:
                        await ws.send(msg)
                    except Exception:
                        dead.add(ws)
                connected_clients -= dead

        except BlockingIOError:
            await asyncio.sleep(0.05)
        except Exception as e:
            print(f"[UDP] Erreur: {e}")
            await asyncio.sleep(0.1)

# ─── Serveur WebSocket ────────────────────────────────────────────────────────

async def ws_handler(websocket):
    print(f"[WS] Client connecté : {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        # Envoie l'état courant immédiatement
        await websocket.send(json.dumps({
            "type": "gps",
            "data": gps_state,
            "ts":   datetime.utcnow().isoformat(),
        }))
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        print(f"[WS] Client déconnecté : {websocket.remote_address}")

# ─── Point d'entrée ───────────────────────────────────────────────────────────

async def main():
    print("=" * 55)
    print("  Serveur NMEA0183 UDP → WebSocket")
    print(f"  UDP  écoute  : 0.0.0.0:{UDP_PORT}")
    print(f"  WS   écoute  : ws://0.0.0.0:{WS_PORT}")
    print(f"  Ouvrir       : http://localhost:{WS_PORT}")
    print("=" * 55)

    ws_server = await websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    await asyncio.gather(
        udp_receiver(),
        ws_server.wait_closed(),
    )

if __name__ == "__main__":
    asyncio.run(main())
