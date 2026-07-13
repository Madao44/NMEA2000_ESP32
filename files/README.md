# NMEA2000 ESP32 Simulator – GPS Sportnav SPO25

Simulation complète de l'émission et réception de données **NMEA2000** via **CAN bus** sur deux ESP32, avec émulation du GPS **Sportnav SPO25**.

---

## Architecture

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│   ESP32 #1  (ÉMETTEUR)      │         │   ESP32 #2  (RÉCEPTEUR)     │
│                             │         │                             │
│  SimulatedGPS               │         │  FastPacketAssembler        │
│  → encode PGNs NMEA2000     │   CAN   │  → décode PGNs NMEA2000     │
│  → Fast-Packet fragment     │◄───────►│  → affiche valeurs GPS      │
│  → tx CAN 29 bits 250kbps   │  bus    │  → mise à jour GPSState     │
│                             │         │                             │
│  GPIO4 → CAN TX             │         │  GPIO4 → CAN TX             │
│  GPIO5 ← CAN RX             │         │  GPIO5 ← CAN RX             │
│  + MCP2551 / SN65HVD230     │         │  + MCP2551 / SN65HVD230     │
└─────────────────────────────┘         └─────────────────────────────┘
```

---

## PGNs émis (Sportnav SPO25)

| PGN    | Nom                    | Intervalle | Protocole   |
|--------|------------------------|------------|-------------|
| 129029 | GNSS Position Data     | 1 000 ms   | Fast-Packet |
| 129026 | COG & SOG Rapid Update | 250 ms     | Single      |
| 127250 | Vessel Heading         | 100 ms     | Single      |
| 128259 | Speed                  | 1 000 ms   | Single      |
| 129033 | Time & Date            | 1 000 ms   | Fast-Packet |
| 129539 | GNSS DOPs              | 1 000 ms   | Single      |
| 60928  | ISO Address Claim      | Au boot    | Single      |

---

## Structure du projet

```
nmea2000_esp32/
├── common/
│   ├── nmea2000_pgn.py       ← Constantes PGN + construction ID CAN
│   ├── nmea2000_encoder.py   ← Encodeurs binaires des PGNs
│   └── fast_packet.py        ← Fragmentation / réassemblage Fast-Packet
├── sender/
│   └── sender_esp32.py       ← MicroPython pour ESP32 émetteur
├── receiver/
│   └── receiver_esp32.py     ← MicroPython pour ESP32 récepteur
└── simulation_pc.py          ← Simulation PC complète (pas de matériel)
```

---

## Utilisation

### 1. Simulation PC (sans matériel)

```bash
python simulation_pc.py --cycles 40 --delay 0.2
```

### 2. Flash sur ESP32 (MicroPython ≥ 1.22)

```bash
# Installer mpremote
pip install mpremote

# ESP32 émetteur (port COM3 sur Windows, /dev/ttyUSB0 sur Linux)
mpremote connect /dev/ttyUSB0 cp sender/sender_esp32.py :main.py

# ESP32 récepteur
mpremote connect /dev/ttyUSB1 cp receiver/receiver_esp32.py :main.py
```

### 3. Câblage CAN

```
ESP32 GPIO4 → TXD du MCP2551/SN65HVD230
ESP32 GPIO5 ← RXD du MCP2551/SN65HVD230
CANH ──────── CANH (bus partagé)
CANL ──────── CANL (bus partagé)

Résistances de terminaison : 120Ω aux deux extrémités du bus
Vitesse : 250 kbit/s (standard NMEA2000)
```

---

## Format de l'ID CAN NMEA2000 (29 bits)

```
Bits 28-26 : Priorité (0-7)
Bits 25-8  : PGN (18 bits)
Bits 7-0   : Adresse source
```

## Fast-Packet Protocol

```
Frame 0 : [seq|0x00] [total_len] [data 0..5]
Frame N : [seq|N   ] [data 0..6]
```

---

## Dépendances

- **ESP32** : MicroPython ≥ 1.22 avec module `machine.CAN`
- **PC**    : Python ≥ 3.10 (stdlib uniquement, aucune dépendance externe)
