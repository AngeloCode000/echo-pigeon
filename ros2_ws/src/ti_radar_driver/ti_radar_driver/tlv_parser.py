"""Streaming parser for the TI mmWave SDK 3.x out-of-box demo UART format.

Frame layout (little-endian):
    magic word          8 bytes   02 01 04 03 06 05 08 07
    version             uint32
    totalPacketLen      uint32    includes the magic word
    platform            uint32
    frameNumber         uint32
    timeCpuCycles       uint32
    numDetectedObj      uint32
    numTLVs             uint32
    subFrameNumber      uint32
    TLVs                numTLVs x [type uint32, length uint32, payload]

TLV type 1 (DETECTED_POINTS): numDetectedObj x (x, y, z, doppler) float32,
meters and m/s, in the TI board frame (x right, y boresight, z up).
TLV type 7 (SIDE_INFO_FOR_DETECTED_POINTS): numDetectedObj x (snr, noise)
int16 in 0.1 dB units. Optional — only present when enabled by guiMonitor.

Pure module — must not import rclpy or serial so tests run anywhere.
"""

import struct
from dataclasses import dataclass, field
from typing import List, Optional

MAGIC_WORD = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
HEADER_LENGTH = 40  # magic word + 8 uint32 fields
TLV_HEADER_LENGTH = 8

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

# Cap accepted packet length so a corrupt length field cannot stall the
# stream waiting for data that will never arrive.
MAX_PACKET_LENGTH = 65536


@dataclass
class DetectedPoint:
    """One point in TI board coordinates, SNR/noise in dB when available."""

    x: float
    y: float
    z: float
    doppler: float
    snr_db: float = 0.0
    noise_db: float = 0.0


@dataclass
class Frame:
    frame_number: int
    num_detected_obj: int
    points: List[DetectedPoint] = field(default_factory=list)


class TlvParser:
    """Feed raw serial bytes in, get complete frames out.

    Tolerates partial reads, leading garbage, unknown TLV types, and
    corrupt length fields by resynchronizing on the magic word.
    """

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames = []
        while True:
            frame = self._try_parse_one()
            if frame is None:
                break
            if frame is not SKIPPED:
                frames.append(frame)
        return frames

    def _try_parse_one(self):
        start = self._buffer.find(MAGIC_WORD)
        if start < 0:
            # Keep a tail that could be a partial magic word.
            if len(self._buffer) > len(MAGIC_WORD):
                del self._buffer[:-len(MAGIC_WORD)]
            return None
        if start > 0:
            del self._buffer[:start]

        if len(self._buffer) < HEADER_LENGTH:
            return None

        (_version, total_len, _platform, frame_number, _cycles,
         num_obj, num_tlvs, _subframe) = struct.unpack_from(
            '<8I', self._buffer, len(MAGIC_WORD))

        if total_len < HEADER_LENGTH or total_len > MAX_PACKET_LENGTH:
            # Corrupt length: discard this magic word and resync.
            del self._buffer[:len(MAGIC_WORD)]
            return SKIPPED
        if len(self._buffer) < total_len:
            return None

        packet = bytes(self._buffer[:total_len])
        del self._buffer[:total_len]
        try:
            return self._parse_packet(packet, frame_number, num_obj, num_tlvs)
        except (struct.error, ValueError):
            return SKIPPED

    def _parse_packet(self, packet, frame_number, num_obj, num_tlvs):
        frame = Frame(frame_number=frame_number, num_detected_obj=num_obj)
        offset = HEADER_LENGTH
        side_info: Optional[list] = None

        for _ in range(num_tlvs):
            if offset + TLV_HEADER_LENGTH > len(packet):
                break
            tlv_type, tlv_len = struct.unpack_from('<2I', packet, offset)
            offset += TLV_HEADER_LENGTH
            if offset + tlv_len > len(packet):
                break

            if tlv_type == TLV_DETECTED_POINTS:
                count = tlv_len // 16
                for i in range(count):
                    x, y, z, doppler = struct.unpack_from(
                        '<4f', packet, offset + 16 * i)
                    frame.points.append(DetectedPoint(x, y, z, doppler))
            elif tlv_type == TLV_SIDE_INFO:
                count = tlv_len // 4
                side_info = [
                    struct.unpack_from('<2h', packet, offset + 4 * i)
                    for i in range(count)
                ]
            # Unknown TLV types are skipped by length.
            offset += tlv_len

        if side_info is not None:
            for point, (snr, noise) in zip(frame.points, side_info):
                point.snr_db = snr * 0.1
                point.noise_db = noise * 0.1
        return frame


# Sentinel distinguishing "consumed corrupt data, keep going" from
# "need more bytes" inside the parse loop.
SKIPPED = object()


def build_frame(frame_number, points, include_side_info=True,
                extra_tlvs=()):
    """Serialize a Frame back to wire bytes. Used by tests and simulators."""
    tlvs = []
    detected = b''.join(
        struct.pack('<4f', p.x, p.y, p.z, p.doppler) for p in points)
    tlvs.append(struct.pack('<2I', TLV_DETECTED_POINTS, len(detected))
                + detected)
    if include_side_info:
        side = b''.join(
            struct.pack('<2h', int(round(p.snr_db * 10)),
                        int(round(p.noise_db * 10))) for p in points)
        tlvs.append(struct.pack('<2I', TLV_SIDE_INFO, len(side)) + side)
    for tlv_type, payload in extra_tlvs:
        tlvs.append(struct.pack('<2I', tlv_type, len(payload)) + payload)

    body = b''.join(tlvs)
    total_len = HEADER_LENGTH + len(body)
    header = MAGIC_WORD + struct.pack(
        '<8I', 0x03060000, total_len, 0x6843, frame_number, 0,
        len(points), len(tlvs), 0)
    return header + body
