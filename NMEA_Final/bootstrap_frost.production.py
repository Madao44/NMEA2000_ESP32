#!/usr/bin/env python3
"""
Cree les entites SensorThings API (Thing / Sensor / ObservedProperties /
Datastreams) necessaires pour enregistrer les donnees GPS + moteur du bateau
sur le serveur FROST-Server.

A executer UNE SEULE FOIS, apres que FROST-Server soit deploye et accessible
(voir DEPLOY.md). Le script affiche a la fin la liste des identifiants de
Datastreams a copier dans le firmware ESP32 (FROST_DATASTREAM_IDS).

Usage:
    pip install requests
    python3 bootstrap_frost.py
"""
import requests

# --- A ADAPTER SI BESOIN ---
# MODE TEST LOCAL : pointe vers ton PC (sur le meme reseau Wi-Fi que l'ESP32,
# le DWR-960), pas vers le serveur de production nmea2k.obsea.es.
# Remplace 10.5.159.131 par l'IP de ton PC trouvee avec "ipconfig".
BASE_URL = "https://nmea2k.obsea.es/FROST-Server/v1.1"
VESSEL_NAME = "Sei Whale"
AUTH = None   # ex: ("utilisateur", "motdepasse") si une authentification Basic est activee sur le serveur

OM_MEASUREMENT = "http://www.opengis.net/def/observationType/OGC-OM/2.0/OM_Measurement"

# (nom technique, description, unite: nom, symbole, URI de definition)
# ATTENTION : l'ordre de cette liste DOIT correspondre a l'ordre du tableau
# FROST_DATASTREAM_IDS[] et des "values[]" dans le firmware ESP32.
PARAMETERS = [
    ("Latitude",           "Latitude GPS du bateau",                "degree",                "deg",  "http://www.qudt.org/qudt/owl/1.0.0/unit/Degree"),
    ("Longitude",          "Longitude GPS du bateau",                "degree",                "deg",  "http://www.qudt.org/qudt/owl/1.0.0/unit/Degree"),
    ("SpeedOverGround",    "Vitesse fond (SOG)",                     "knot",                  "kn",   "http://www.qudt.org/qudt/owl/1.0.0/unit/Knot"),
    ("CourseOverGround",   "Cap fond (COG)",                         "degree",                "deg",  "http://www.qudt.org/qudt/owl/1.0.0/unit/Degree"),
    ("Heading",            "Cap compas (HDG)",                       "degree",                "deg",  "http://www.qudt.org/qudt/owl/1.0.0/unit/Degree"),
    ("HDOP",               "Dilution horizontale de precision GPS",  "dimensionless",         "-",    "http://www.qudt.org/qudt/owl/1.0.0/unit/Unitless"),
    ("EngineRPM",          "Regime moteur",                          "revolution per minute", "rpm",  "http://www.qudt.org/qudt/owl/1.0.0/unit/RevolutionPerMinute"),
    ("CoolantTemperature", "Temperature liquide de refroidissement",  "degree Celsius",       "degC", "http://www.qudt.org/qudt/owl/1.0.0/unit/DegreeCelsius"),
    ("OilPressure",        "Pression huile moteur",                  "bar",                   "bar",  "http://www.qudt.org/qudt/owl/1.0.0/unit/Bar"),
    ("OilTemperature",     "Temperature huile moteur",                "degree Celsius",       "degC", "http://www.qudt.org/qudt/owl/1.0.0/unit/DegreeCelsius"),
    ("BatteryVoltage",     "Tension batterie moteur",                 "volt",                 "V",    "http://www.qudt.org/qudt/owl/1.0.0/unit/Volt"),
    ("FuelLevel",          "Niveau de carburant",                     "percent",              "%",    "http://www.qudt.org/qudt/owl/1.0.0/unit/Percent"),
    ("WaterDepth",         "Profondeur d'eau sous la coque",          "metre",                "m",    "http://www.qudt.org/qudt/owl/1.0.0/unit/Meter"),
    ("WaterTemperature",   "Temperature de l'eau",                    "degree Celsius",       "degC", "http://www.qudt.org/qudt/owl/1.0.0/unit/DegreeCelsius"),
    ("SpeedThroughWater",  "Vitesse surface (loch)",                  "knot",                 "kn",   "http://www.qudt.org/qudt/owl/1.0.0/unit/Knot"),
]


def post(path, payload):
    r = requests.post(f"{BASE_URL}/{path}", json=payload, auth=AUTH, timeout=15)
    r.raise_for_status()
    location = r.headers["Location"]
    entity_id = location.rstrip("/").split("(")[-1].rstrip(")")
    return int(entity_id)


def main():
    print(f"Connexion a {BASE_URL} ...")

    thing_id = post("Things", {
        "name": VESSEL_NAME,
        "description": f"Passerelle NMEA2000/ESP32 embarquee sur {VESSEL_NAME}",
        "properties": {"gateway": "ESP32-S3 NMEA2000"},
    })
    print(f"Thing cree   : @iot.id={thing_id}")

    sensor_id = post("Sensors", {
        "name": "ESP32 NMEA2000 Gateway",
        "description": "Passerelle NMEA2000 (GPS + moteur) vers SensorThings API",
        "encodingType": "text/html",
        "metadata": "https://obsea.es/",
    })
    print(f"Sensor cree  : @iot.id={sensor_id}")

    print("\nCreation des ObservedProperties + Datastreams ...\n")
    results = []
    for name, desc, unit_name, unit_symbol, unit_def in PARAMETERS:
        op_id = post("ObservedProperties", {
            "name": name,
            "definition": f"https://obsea.es/nmea2000#{name}",
            "description": desc,
        })
        ds_id = post("Datastreams", {
            "name": f"{VESSEL_NAME} - {name}",
            "description": desc,
            "observationType": OM_MEASUREMENT,
            "unitOfMeasurement": {"name": unit_name, "symbol": unit_symbol, "definition": unit_def},
            "Thing": {"@iot.id": thing_id},
            "Sensor": {"@iot.id": sensor_id},
            "ObservedProperty": {"@iot.id": op_id},
        })
        results.append((name, ds_id))
        print(f"  {name:20s} -> ObservedProperty={op_id:<4d} Datastream={ds_id}")

    print("\n" + "=" * 60)
    print("Copie ce tableau dans le firmware ESP32")
    print("(remplace FROST_DATASTREAM_IDS[] dans nmea2000_gsm.c) :")
    print("=" * 60 + "\n")
    print("static const int FROST_DATASTREAM_IDS[] = {")
    for name, ds_id in results:
        print(f"    {ds_id},  /* {name} */")
    print("};")


if __name__ == "__main__":
    main()
