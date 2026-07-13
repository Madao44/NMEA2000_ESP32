"""
NMEA2000 PGN Definitions - GPS Sportnav SPO25
Définitions des PGNs émis par le GPS Sportnav SPO25
"""

# ─── PGN courants du SPO25 ────────────────────────────────────────────────────
PGN_GNSS_POSITION        = 129029  # Position GPS (lat/lon/alt)
PGN_COG_SOG              = 129026  # Cap et vitesse sur le fond
PGN_VESSEL_HEADING       = 127250  # Cap du navire
PGN_SPEED                = 128259  # Vitesse dans l'eau / sur le fond
PGN_TIME_DATE            = 129033  # Date et heure UTC
PGN_GNSS_DOPS            = 129539  # Dilution de précision (DOP)
PGN_GNSS_SATS_IN_VIEW    = 129540  # Satellites en vue
PGN_ISO_ADDRESS_CLAIM    = 60928   # Adressage réseau NMEA2000

# ─── Priorités CAN ────────────────────────────────────────────────────────────
PRIORITY = {
    PGN_GNSS_POSITION:     3,
    PGN_COG_SOG:           2,
    PGN_VESSEL_HEADING:    2,
    PGN_SPEED:             2,
    PGN_TIME_DATE:         3,
    PGN_GNSS_DOPS:         6,
    PGN_GNSS_SATS_IN_VIEW: 6,
    PGN_ISO_ADDRESS_CLAIM: 6,
}

# ─── Intervalles d'émission recommandés (ms) ─────────────────────────────────
SEND_INTERVAL_MS = {
    PGN_GNSS_POSITION:     1000,
    PGN_COG_SOG:           250,
    PGN_VESSEL_HEADING:    100,
    PGN_SPEED:             1000,
    PGN_TIME_DATE:         1000,
    PGN_GNSS_DOPS:         1000,
    PGN_GNSS_SATS_IN_VIEW: 1000,
}

# ─── Taille des messages (bytes, 0 = fast-packet) ────────────────────────────
MSG_LENGTH = {
    PGN_GNSS_POSITION:     8,   # fast-packet (multi-frame)
    PGN_COG_SOG:           8,   # single frame
    PGN_VESSEL_HEADING:    8,   # single frame
    PGN_SPEED:             8,   # single frame
    PGN_TIME_DATE:         8,   # fast-packet
    PGN_GNSS_DOPS:         8,   # single frame
    PGN_GNSS_SATS_IN_VIEW: 8,   # fast-packet (multi-frame)
    PGN_ISO_ADDRESS_CLAIM: 8,   # single frame
}

# ─── Construction de l'ID CAN 29 bits NMEA2000 ───────────────────────────────
def build_can_id(priority: int, pgn: int, src: int, dst: int = 0xFF) -> int:
    """
    Construit l'identifiant CAN 29 bits au format NMEA2000.

    Structure :
        bits 28-26 : priorité (0-7)
        bits 25-8  : PGN (18 bits, inclut PDU format/spécifique)
        bits 7-0   : adresse source

    Pour les PGN > 0xF000 (PDU2 / broadcast), dst est ignoré dans l'ID.
    Pour les PGN < 0xF000 (PDU1 / adressé), l'octet de destination
    remplace les bits 15-8 du PGN dans l'ID.
    """
    pf = (pgn >> 8) & 0xFF          # PDU Format
    if pf < 0xF0:                   # PDU1 – message adressé
        can_pgn = (pgn & 0x1FF00) | (dst & 0xFF)
    else:                           # PDU2 – broadcast
        can_pgn = pgn & 0x1FFFF

    can_id  = (priority & 0x07) << 26
    can_id |= (can_pgn  & 0x3FFFF) << 8
    can_id |= (src       & 0xFF)
    return can_id


def parse_can_id(can_id: int) -> dict:
    """Décode un identifiant CAN 29 bits NMEA2000."""
    priority = (can_id >> 26) & 0x07
    pgn_raw  = (can_id >> 8)  & 0x3FFFF
    src      =  can_id        & 0xFF
    pf       = (pgn_raw >> 8) & 0xFF

    if pf < 0xF0:                   # PDU1
        dst = pgn_raw & 0xFF
        pgn = pgn_raw & 0x1FF00
    else:                           # PDU2
        dst = 0xFF
        pgn = pgn_raw

    return {"priority": priority, "pgn": pgn, "src": src, "dst": dst}


PGN_NAMES = {
    PGN_GNSS_POSITION:     "GNSS Position Data",
    PGN_COG_SOG:           "COG & SOG, Rapid Update",
    PGN_VESSEL_HEADING:    "Vessel Heading",
    PGN_SPEED:             "Speed",
    PGN_TIME_DATE:         "Time & Date",
    PGN_GNSS_DOPS:         "GNSS DOPs",
    PGN_GNSS_SATS_IN_VIEW: "GNSS Sats in View",
    PGN_ISO_ADDRESS_CLAIM: "ISO Address Claim",
}
