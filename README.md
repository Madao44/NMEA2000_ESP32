# NMEA2000 ESP32

Pipeline collecting NMEA2000 data from a boat into a remote database (OGC SensorThings API), with a local real-time web dashboard. Built as part of an international internship at **OBSEA / UPC SARTI** (Vilanova i la Geltrú, Spain).

![status](https://img.shields.io/badge/status-production-brightgreen)

## Overview

An ESP32-S3 reads the boat's NMEA2000 bus (GPS, AIS, engine, depth/speed) over CAN, publishes the data via MQTT/HTTP to a **FROST-Server** (OGC SensorThings API), and serves a local web dashboard for real-time monitoring.

Two test contexts: `Mar de Caroba` (dev/lab) and `Sei Whale` (production deployment).

## Architecture

```
NMEA2000 bus (CAN 250kbps)
        │
   SN65HVD230 (transceiver)
        │
     ESP32-S3  ──Wi-Fi (4G router)──▶  FROST-Server (Docker, PostgreSQL/PostGIS)
        │                                     https://nmea2k.obsea.es/FROST-Server/v1.1/
        └──▶ Local dashboard (http://192.168.4.1)
```

## Project structure

This repo holds the firmware's development history. **`NMEA2000_ESP32_FROST` is the final, production version**; the other folders are the development steps and test prototypes that led to it.

| Folder | Role |
|---|---|
| **`NMEA2000_ESP32_FROST`** | ✅ **Final version, deployed in production.** NMEA2000 reading + publishing to FROST-Server (MQTT/HTTP) + local dashboard. |
| `NMEA_Final` | Build stage preceding `NMEA2000_ESP32_FROST`: base NMEA2000 parsing logic before FROST-Server integration. |
| `nmea2000_GSM` | Test prototype for GSM/4G connectivity (D-Link DWR-960 router). |
| `nmea2000_ap` | Test prototype for the Wi-Fi access-point mode (base of the local dashboard at `192.168.4.1`). |
| `nmea2000_receiver` | Test prototype for receiving CAN frames (read-only NMEA2000 bus access). |
| `nmea2000_sender/nmea2000_sender` | Test prototype for sending CAN frames (simulator/validation). |
| `nmea2000_udp` | Test prototype for UDP data transmission. |
| `nmea2000_gsm.c`, `raw data.txt`, `test.txt` | Standalone test/debug files, not used in production. |
| `ESP32_GENERIC_S3-*.bin` | Precompiled MicroPython firmware for flashing the ESP32-S3. |
| `Internship_report_NMEA2000_UPC_SARTI.docx` | Internship report. |

> To develop or reproduce the project, work only inside `NMEA2000_ESP32_FROST` — the other folders are kept for historical reference.

## Hardware

| Component | Reference |
|---|---|
| Microcontroller | ESP32-S3 |
| CAN transceiver | SN65HVD230 (RS→GND for 250 kbps mode) |
| GPS | Sportnav SPO25F |
| AIS | em-trak B921 |
| Engine | SELVA outboard (DLC-Plus display) |
| Depth/speed | KTD-TM520 |
| Connectivity | D-Link DWR-960 (4G router, 2.4 GHz) |

NMEA2000 wiring (M12 5-pin connector, A-coded):

| Pin | Function |
|---|---|
| 1 | Shield |
| 2 | V+ (red) |
| 3 | GND (black) |
| 4 | CAN H (white) |
| 5 | CAN L (blue) |

> V+ should not be connected when components have their own power supply. GND connection is mandatory as a common voltage reference. Galvanic isolation is recommended.

## Software stack

- **Firmware**: ESP-IDF v5.5.4, TWAI driver, [ttlappalainen/NMEA2000_esp32](https://github.com/ttlappalainen/NMEA2000_esp32) library, PubSubClient (MQTT), ArduinoJson
- **Backend**: FROST-Server (OGC SensorThings API v1.1), Docker Compose, PostgreSQL/PostGIS, Apache2 reverse proxy

## Build & flash

```bash
# Environment: ESP-IDF v5.5.4
cd NMEA2000_ESP32_FROST
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## Local dashboard

Connect to the Wi-Fi network broadcast by the ESP32:

- SSID: `NMEA2000`
- URL: `http://192.168.4.1`

Displays GPS, heading, engine parameters, and AIS targets in real time.

## Supported NMEA2000 PGNs

| PGN | Data | Type |
|---|---|---|
| 129025 | Position Rapid Update (lat/lon) | standard frame |
| 129026 | COG/SOG | standard frame |
| 129029 | Full GNSS | fast-packet |
| 129033 | GPS date/time | standard frame (unreliable → NTP fallback) |
| 129039 | AIS target (em-trak B921) | fast-packet, native binary format |
| 129540 / 126992 | GNSS satellites / system time | — |

> The SELVA engine is not connected to the NMEA2000 bus on this boat: no engine PGN (1274xx/1275xx) is transmitted. Only the GPS (src=0x15) is an active source. An engine-to-NMEA2000 gateway would be required to acquire this data.

## Server deployment (FROST-Server)

```bash
docker compose up -d
```

- The full entity graph (`Thing`, `Sensor`, `ObservedProperty`, `Datastream`, `Location`) must be bootstrapped via HTTP REST (curl) — MQTT can only create `Observations`.
- The Docker port must be bound as `0.0.0.0:8080` (not `127.0.0.1`) for the Apache reverse proxy to route traffic correctly.

## Related repos

- [`Madao44/NMEA2000_ESP32`](https://github.com/Madao44/NMEA2000_ESP32) — ESP32 firmware
- [`Madao44/UPC_SARTI`](https://github.com/Madao44/UPC_SARTI) — server infrastructure / bootstrap

## Author

Nathan — international internship, ISEN Brest (Yncréa) x OBSEA/UPC SARTI

## License

MIT
