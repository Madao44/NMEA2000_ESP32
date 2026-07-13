"""
NMEA2000 Fast-Packet Protocol
Fragmentation / réassemblage des messages NMEA2000 > 8 bytes sur le bus CAN.

Norme : NMEA 2000 § 3.3 (Fast Packet Protocol)
Chaque trame CAN = 8 bytes :
  - Byte 0 (frame 0) : [seq_id 3 bits | frame_counter 5 bits]
                        frame_counter = 0 pour le premier frame
  - Byte 1 (frame 0) : longueur totale du message (bytes)
  - Bytes 2-7 (frame 0) : 6 premiers bytes de données
  - Bytes 1-7 (frames suivants) : 7 bytes de données
"""

from typing import List, Tuple


def fragment_fast_packet(pgn: int, payload: bytes, seq_id: int = 0) -> List[bytes]:
    """
    Fragmente un payload NMEA2000 en une liste de trames CAN 8 bytes
    conformes au protocole Fast-Packet.

    Args:
        pgn      : PGN du message (utilisé pour la doc, non encodé ici)
        payload  : données brutes à fragmenter (n'importe quelle longueur ≤ 223 bytes)
        seq_id   : identifiant de séquence (0-7) pour différencier plusieurs streams

    Returns:
        Liste de bytes de 8 bytes chacun.
    """
    seq_id     = seq_id & 0x07
    total_len  = len(payload)
    frames     = []
    frame_num  = 0
    offset     = 0

    # ── Frame 0 : entête ──────────────────────────────────────────────────────
    header     = bytes([(seq_id << 5) | (frame_num & 0x1F), total_len & 0xFF])
    chunk      = payload[offset:offset + 6]
    chunk      = chunk.ljust(6, b'\xFF')     # padding si payload court
    frames.append(header + chunk)
    offset    += 6
    frame_num += 1

    # ── Frames suivants : données ─────────────────────────────────────────────
    while offset < total_len:
        seq_byte = (seq_id << 5) | (frame_num & 0x1F)
        chunk    = payload[offset:offset + 7]
        chunk    = chunk.ljust(7, b'\xFF')
        frames.append(bytes([seq_byte]) + chunk)
        offset    += 7
        frame_num += 1

    return frames


class FastPacketAssembler:
    """
    Réassemble des trames CAN Fast-Packet en payload NMEA2000 complet.

    Usage:
        asm = FastPacketAssembler()
        for frame in received_frames:
            result = asm.feed(src_addr, pgn, frame_data)
            if result is not None:
                payload = result  # message complet !
    """

    def __init__(self):
        # clé : (src, pgn, seq_id) → {"total": int, "frames": {num: bytes}}
        self._buffers: dict = {}

    def feed(self, src: int, pgn: int, frame: bytes) -> bytes | None:
        """
        Ingère une trame CAN de 8 bytes.
        Retourne le payload complet si le message est terminé, None sinon.
        """
        if len(frame) < 2:
            return None

        seq_id    = (frame[0] >> 5) & 0x07
        frame_num =  frame[0]       & 0x1F
        key       = (src, pgn, seq_id)

        if frame_num == 0:
            # Premier frame : lire la longueur totale et 6 bytes de données
            total    = frame[1]
            data     = frame[2:]
            self._buffers[key] = {"total": total, "frames": {0: data}}
        else:
            if key not in self._buffers:
                return None             # frame orphelin
            self._buffers[key]["frames"][frame_num] = frame[1:]

        buf = self._buffers[key]

        # ── Vérifier si tous les frames sont reçus ────────────────────────────
        total      = buf["total"]
        n_expected = self._expected_frames(total)
        if len(buf["frames"]) < n_expected:
            return None

        # ── Reconstituer le payload ───────────────────────────────────────────
        payload = b""
        for i in range(n_expected):
            payload += buf["frames"].get(i, b"")
        payload = payload[:total]
        del self._buffers[key]
        return payload

    @staticmethod
    def _expected_frames(total_len: int) -> int:
        """Calcule le nombre de trames attendues pour un payload de total_len bytes."""
        if total_len <= 6:
            return 1
        return 1 + math.ceil((total_len - 6) / 7)


# ── Import tardif pour éviter la circularité ──────────────────────────────────
import math
