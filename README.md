# NMEA2000 ESP32

Pipeline de collecte de données NMEA2000 d'un bateau vers une base de données distante (OGC SensorThings API), avec dashboard web local temps réel. Projet réalisé dans le cadre d'un stage international à l'**OBSEA / UPC SARTI** (Vilanova i la Geltrú, Espagne).

![status](https://img.shields.io/badge/status-production-brightgreen)

## Aperçu

Un ESP32-S3 lit le bus NMEA2000 du bateau (GPS, AIS, moteur, profondeur/vitesse) via CAN, publie les données en MQTT/HTTP vers un serveur **FROST-Server** (OGC SensorThings API) et sert un dashboard web local pour le monitoring en direct.

Deux contextes de test : `Mar de Caroba` (dev/labo) et `Sei Whale` (déploiement production).

## Architecture

```
NMEA2000 bus (CAN 250kbps)
        │
   SN65HVD230 (transceiver)
        │
     ESP32-S3  ──Wi-Fi (4G router)──▶  FROST-Server (Docker, PostgreSQL/PostGIS)
        │                                     https://nmea2k.obsea.es/FROST-Server/v1.1/
        └──▶ Dashboard local (http://192.168.4.1)
```

## Hardware

| Composant | Référence |
|---|---|
| Microcontrôleur | ESP32-S3 |
| Transceiver CAN | SN65HVD230 (RS→GND pour mode 250 kbps) |
| GPS | Sportnav SPO25F |
| AIS | em-trak B921 |
| Moteur | SELVA outboard (display DLC-Plus) |
| Profondeur/vitesse | KTD-TM520 |
| Connectivité | D-Link DWR-960 (routeur 4G, 2.4 GHz) |

Câblage NMEA2000 (connecteur M12 5 broches, code A) :

| Pin | Fonction |
|---|---|
| 1 | Shield |
| 2 | V+ (rouge) |
| 3 | GND (noir) |
| 4 | CAN H (blanc) |
| 5 | CAN L (bleu) |

> V+ non connecté si les composants ont leur propre alimentation. GND obligatoire (référence de tension commune). Isolation galvanique recommandée.

## Stack logicielle

- **Firmware** : ESP-IDF v5.5.4, driver TWAI, lib [ttlappalainen/NMEA2000_esp32](https://github.com/ttlappalainen/NMEA2000_esp32), PubSubClient (MQTT), ArduinoJson
- **Backend** : FROST-Server (OGC SensorThings API v1.1), Docker Compose, PostgreSQL/PostGIS, Apache2 reverse proxy

## Build & flash

```bash
# Environnement : ESP-IDF v5.5.4
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## Dashboard local

Se connecter au Wi-Fi émis par l'ESP32 :

- SSID : `NMEA2000`
- URL : `http://192.168.4.1`

Affiche GPS, cap, paramètres moteur et cibles AIS en temps réel.

## PGN NMEA2000 supportés

| PGN | Donnée | Type |
|---|---|---|
| 129025 | Position Rapid Update (lat/lon) | standard frame |
| 129026 | COG/SOG | standard frame |
| 129029 | GNSS complet | fast-packet |
| 129033 | Date/heure GPS | standard frame (non fiable → fallback NTP) |
| 129039 | Cible AIS (em-trak B921) | fast-packet, format binaire natif |
| 129540 / 126992 | Satellites GNSS / heure système | — |

> Le moteur SELVA n'est pas raccordé au bus NMEA2000 sur ce bateau : aucun PGN moteur (1274xx/1275xx) n'est émis. Seul le GPS (src=0x15) est source active. Une passerelle moteur→NMEA2000 serait nécessaire pour exploiter ces données.

## Déploiement serveur (FROST-Server)

```bash
docker compose up -d
```

- Bootstrap de l'entity graph (`Thing`, `Sensor`, `ObservedProperty`, `Datastream`, `Location`) via HTTP REST (curl) — MQTT ne peut créer que des `Observations`.
- Le port Docker doit être bindé en `0.0.0.0:8080` (pas `127.0.0.1`) pour que le reverse proxy Apache route correctement.

## Repos liés

- [`Madao44/NMEA2000_ESP32`](https://github.com/Madao44/NMEA2000_ESP32) — firmware ESP32
- [`Madao44/UPC_SARTI`](https://github.com/Madao44/UPC_SARTI) — infra serveur / bootstrap

## Auteur

Nathan — stage international ISEN Brest (Yncréa) x OBSEA/UPC SARTI

## Licence

MIT
