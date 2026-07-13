"""
NMEA2000 Encodeurs de PGN – GPS Sportnav SPO25
Encode les données GPS en trames binaires conformes NMEA2000.

Référence : NMEA 2000 Appendix B (PGN definitions)
            Canboat / OpenPlotter format tables
"""

import struct
import math
from datetime import datetime, timezone, date as date_type


# ─── Constantes de résolution NMEA2000 ────────────────────────────────────────
LAT_LON_RES   = 1e-7     # degrés  (resolution 1e-7 deg)
ALT_RES       = 1e-6     # mètres  (resolution 1e-6 m)
COG_SOG_RES   = 1e-4     # rad & m/s
HEADING_RES   = 1e-4     # radians
SPEED_RES     = 1e-2     # m/s (resolution 0.01 m/s)
DOP_RES       = 1e-2     # sans unité


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


# ─── PGN 129029 – GNSS Position Data (fast-packet, 43 bytes) ─────────────────
def encode_gnss_position(
    lat: float,
    lon: float,
    alt: float,
    sid: int,
    days_since_1970: int,
    seconds_since_midnight: float,
    gnss_type: int = 0,          # 0=GPS, 1=GLONASS, 2=GPS+GLONASS
    method: int = 1,             # 1=GNSS fix
    integrity: int = 0,
    num_svs: int = 8,
    hdop: float = 1.0,
    pdop: float = 1.8,
    geoid_sep: float = 48.0,
    ref_stations: int = 0,
) -> bytes:
    """Encode PGN 129029 – Position GNSS complète (lat/lon/alt + métadonnées)."""
    lat_i  = int(lat  / LAT_LON_RES)
    lon_i  = int(lon  / LAT_LON_RES)
    alt_i  = int(alt  / ALT_RES)
    hdop_i = int(hdop / DOP_RES)
    pdop_i = int(pdop / DOP_RES)
    geo_i  = int(geoid_sep / DOP_RES)
    secs_i = int(seconds_since_midnight * 10000)   # 0.0001 s resolution

    payload = struct.pack(
        "<BHQqqihBBh",
        sid & 0xFF,
        days_since_1970 & 0xFFFF,
        secs_i,
        lat_i,
        lon_i,
        alt_i,
        (gnss_type & 0x0F) | ((method & 0x0F) << 4),
        integrity & 0xFF,
        num_svs & 0xFF,
        hdop_i,
    )
    # Compléter jusqu'à 43 bytes (fast-packet standard)
    payload += struct.pack("<hhhB", pdop_i, geo_i, ref_stations, 0xFF)
    return payload[:43].ljust(43, b'\xFF')


# ─── PGN 129026 – COG & SOG, Rapid Update (8 bytes) ─────────────────────────
def encode_cog_sog(
    cog_deg: float,
    sog_knots: float,
    sid: int = 0,
    cog_ref: int = 0,           # 0=True, 1=Magnetic
) -> bytes:
    """Encode PGN 129026 – Cap sur le fond (COG) et vitesse sur le fond (SOG)."""
    cog_rad = _deg_to_rad(cog_deg)
    sog_ms  = sog_knots * 0.514444          # nœuds → m/s
    cog_i   = int(cog_rad / COG_SOG_RES) & 0xFFFF
    sog_i   = int(sog_ms  / COG_SOG_RES) & 0xFFFF

    return struct.pack(
        "<BBHH2s",
        sid & 0xFF,
        cog_ref & 0x03,
        cog_i,
        sog_i,
        b'\xFF\xFF',            # réservé
    )


# ─── PGN 127250 – Vessel Heading (8 bytes) ───────────────────────────────────
def encode_vessel_heading(
    heading_deg: float,
    deviation_deg: float = 0.0,
    variation_deg: float = 0.0,
    sid: int = 0,
    ref: int = 1,               # 0=True, 1=Magnetic
) -> bytes:
    """Encode PGN 127250 – Cap du navire."""
    hdg = int(_deg_to_rad(heading_deg) / HEADING_RES) & 0xFFFF
    dev = int(_deg_to_rad(deviation_deg) / HEADING_RES) & 0xFFFF
    var = int(_deg_to_rad(variation_deg) / HEADING_RES) & 0xFFFF

    return struct.pack(
        "<BBHHHx",
        sid & 0xFF,
        ref & 0x03,
        hdg,
        dev,
        var,
    )


# ─── PGN 128259 – Speed (8 bytes) ─────────────────────────────────────────────
def encode_speed(
    speed_water_knots: float,
    speed_ground_knots: float = 0.0,
    sid: int = 0,
) -> bytes:
    """Encode PGN 128259 – Vitesse dans l'eau et sur le fond."""
    sw = int(speed_water_knots  * 0.514444 / SPEED_RES) & 0xFFFF
    sg = int(speed_ground_knots * 0.514444 / SPEED_RES) & 0xFFFF

    return struct.pack(
        "<BHH3s",
        sid & 0xFF,
        sw,
        sg,
        b'\xFF\xFF\xFF',
    )


# ─── PGN 129033 – Time & Date (8 bytes) ──────────────────────────────────────
def encode_time_date(dt: datetime = None) -> bytes:
    """
    Encode PGN 129033 – Date et heure UTC.
    Utilise l'heure courante si dt est None.
    """
    if dt is None:
        dt = datetime.now(timezone.utc)
    epoch = date_type(1970, 1, 1)
    days  = (dt.date() - epoch).days
    secs  = dt.hour * 3600 + dt.minute * 60 + dt.second + dt.microsecond / 1e6
    secs_i = int(secs * 10000)     # 0.0001 s resolution

    return struct.pack(
        "<HQ",
        days & 0xFFFF,
        secs_i,
    )[:8].ljust(8, b'\xFF')


# ─── PGN 129539 – GNSS DOPs (8 bytes) ────────────────────────────────────────
def encode_gnss_dops(
    hdop: float = 1.0,
    vdop: float = 1.5,
    tdop: float = 1.2,
    sid: int = 0,
    desired_mode: int = 3,      # 3 = 3D
    actual_mode: int = 3,
) -> bytes:
    """Encode PGN 129539 – Dilutions de précision GNSS."""
    mode = (desired_mode & 0x07) | ((actual_mode & 0x07) << 3)
    return struct.pack(
        "<BBhhh",
        sid & 0xFF,
        mode,
        int(hdop / DOP_RES),
        int(vdop / DOP_RES),
        int(tdop / DOP_RES),
    )[:8].ljust(8, b'\xFF')


# ─── PGN 60928 – ISO Address Claim (8 bytes) ─────────────────────────────────
def encode_address_claim(
    unique_number: int = 0x1A2B3C,
    manufacturer_code: int = 0x0099,   # code fictif SPO25
    device_instance: int = 0,
    device_function: int = 145,        # 145 = Ownship Attitude
    device_class: int = 60,            # 60 = Navigation
    system_instance: int = 0,
    industry_group: int = 4,           # 4 = Marine
    arbitrary_addr: int = 1,
) -> bytes:
    """Encode PGN 60928 – Revendication d'adresse ISO/NMEA2000."""
    # NAME field: 64 bits
    name  =  (unique_number    & 0x1FFFFF)
    name |=  (manufacturer_code & 0x07FF) << 21
    name |=  (device_instance   & 0xFF)   << 32
    name |=  (device_function   & 0xFF)   << 40
    name |=  (device_class      & 0x7F)   << 48
    name |=  (system_instance   & 0x0F)   << 56
    name |=  (industry_group    & 0x07)   << 60
    name |=  (arbitrary_addr    & 0x01)   << 63
    return struct.pack("<Q", name)
