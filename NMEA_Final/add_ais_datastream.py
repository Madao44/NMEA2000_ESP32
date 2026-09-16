#!/usr/bin/env python3
"""
Ajoute UNIQUEMENT le Datastream AIS a une base FROST-Server qui contient
deja les Datastreams 1-15 (GPS + moteur).

A n'executer qu'une seule fois. Affiche a la fin l'identifiant a placer
dans FROST_AIS_DATASTREAM_ID du firmware.

Usage:
    python add_ais_datastream.py
"""
import requests

BASE_URL = "https://nmea2k.obsea.es/FROST-Server/v1.1"
VESSEL_NAME = "Sei Whale"
THING_ID = 1
SENSOR_ID = 1
AUTH = None

# result = tableau JSON de cibles, donc type d'observation generique
OM_OBSERVATION = "http://www.opengis.net/def/observationType/OGC-OM/2.0/OM_Observation"


def post(path, payload):
    r = requests.post(f"{BASE_URL}/{path}", json=payload, auth=AUTH, timeout=15)
    r.raise_for_status()
    return int(r.headers["Location"].rstrip("/").split("(")[-1].rstrip(")"))


def main():
    print(f"Connexion a {BASE_URL} ...")

    op_id = post("ObservedProperties", {
        "name": "AISTargets",
        "definition": "https://obsea.es/nmea2000#AISTargets",
        "description": "Cibles AIS environnantes (tableau JSON)",
    })
    print(f"ObservedProperty cree : @iot.id={op_id}")

    ds_id = post("Datastreams", {
        "name": f"{VESSEL_NAME} - AISTargets",
        "description": "Cibles AIS environnantes, tableau JSON "
                       "(mmsi, lat, lon, sog, cog, name, call, status)",
        "observationType": OM_OBSERVATION,
        "unitOfMeasurement": {"name": "n/a", "symbol": "", "definition": "n/a"},
        "Thing": {"@iot.id": THING_ID},
        "Sensor": {"@iot.id": SENSOR_ID},
        "ObservedProperty": {"@iot.id": op_id},
    })

    print("\n" + "=" * 52)
    print("A placer dans le firmware (nmea2000_gsm.c) :")
    print("=" * 52)
    print(f"\n#define FROST_AIS_DATASTREAM_ID {ds_id}\n")


if __name__ == "__main__":
    main()
