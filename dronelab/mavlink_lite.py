"""Just enough MAVLink (v1 + v2) to discover vehicles: parse frames, decode
HEARTBEAT, and build a GCS HEARTBEAT. No pymavlink dependency so the GUI
starts fast and the code is testable anywhere."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

STX_V1 = 0xFE
STX_V2 = 0xFD
MSG_HEARTBEAT = 0
CRC_EXTRA = {MSG_HEARTBEAT: 50}

MAV_TYPE_GCS = 6
MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_AUTOPILOT_INVALID = 8
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_AUTOPILOT_PX4 = 12
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

MAV_TYPE_NAMES = {
    1: "fixed wing", 2: "quadrotor", 3: "coaxial", 4: "helicopter", 6: "GCS", 10: "ground rover",
    11: "boat", 12: "submarine", 13: "hexarotor", 14: "octorotor", 15: "tricopter",
    18: "onboard computer", 19: "VTOL", 20: "VTOL", 21: "VTOL", 22: "VTOL", 26: "gimbal", 27: "ADSB",
}
AUTOPILOT_NAMES = {0: "generic", 3: "ArduPilot", 12: "PX4", 8: "-"}

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
    9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE",
}
COPTER_TYPES = {2, 3, 4, 13, 14, 15}


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclass
class Heartbeat:
    sysid: int
    compid: int
    custom_mode: int
    mav_type: int
    autopilot: int
    base_mode: int
    system_status: int
    version: int  # 1 or 2 (frame version)

    @property
    def armed(self) -> bool:
        return bool(self.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def is_vehicle(self) -> bool:
        return self.autopilot != MAV_AUTOPILOT_INVALID and self.mav_type not in (MAV_TYPE_GCS,
                                                                                  MAV_TYPE_ONBOARD_CONTROLLER)

    @property
    def mode_name(self) -> str:
        if self.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA and self.mav_type in COPTER_TYPES:
            return COPTER_MODES.get(self.custom_mode, f"MODE{self.custom_mode}")
        return f"MODE{self.custom_mode}"

    @property
    def type_name(self) -> str:
        return MAV_TYPE_NAMES.get(self.mav_type, f"type {self.mav_type}")

    @property
    def autopilot_name(self) -> str:
        return AUTOPILOT_NAMES.get(self.autopilot, f"ap {self.autopilot}")


def _decode_heartbeat(payload: bytes, sysid: int, compid: int, version: int) -> Heartbeat:
    payload = payload.ljust(9, b"\x00")  # v2 truncates trailing zero bytes
    custom_mode, mtype, ap, base, status, _ver = struct.unpack("<IBBBBB", payload[:9])
    return Heartbeat(sysid, compid, custom_mode, mtype, ap, base, status, version)


def iter_frames(buf: bytes) -> Iterator[Tuple[int, int, int, bytes, int]]:
    """Yield (msgid, sysid, compid, payload, version) for every CRC-valid frame in a datagram.

    Only messages listed in CRC_EXTRA can be CRC-checked; others are skipped.
    """
    i, n = 0, len(buf)
    while i < n:
        stx = buf[i]
        if stx == STX_V2 and i + 10 <= n:
            plen, incompat = buf[i + 1], buf[i + 2]
            sysid, compid = buf[i + 5], buf[i + 6]
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            end = i + 10 + plen + 2 + (13 if incompat & 0x01 else 0)
            if end > n:
                return
            if msgid in CRC_EXTRA:
                crc = x25_crc(buf[i + 1:i + 10 + plen] + bytes([CRC_EXTRA[msgid]]))
                if crc == struct.unpack_from("<H", buf, i + 10 + plen)[0]:
                    yield msgid, sysid, compid, bytes(buf[i + 10:i + 10 + plen]), 2
            i = end
        elif stx == STX_V1 and i + 6 <= n:
            plen = buf[i + 1]
            sysid, compid, msgid = buf[i + 3], buf[i + 4], buf[i + 5]
            end = i + 6 + plen + 2
            if end > n:
                return
            if msgid in CRC_EXTRA:
                crc = x25_crc(buf[i + 1:i + 6 + plen] + bytes([CRC_EXTRA[msgid]]))
                if crc == struct.unpack_from("<H", buf, i + 6 + plen)[0]:
                    yield msgid, sysid, compid, bytes(buf[i + 6:i + 6 + plen]), 1
            i = end
        else:
            i += 1


def heartbeats(buf: bytes) -> Iterator[Heartbeat]:
    for msgid, sysid, compid, payload, ver in iter_frames(buf):
        if msgid == MSG_HEARTBEAT:
            yield _decode_heartbeat(payload, sysid, compid, ver)


def pack_heartbeat(seq: int = 0, sysid: int = 255, compid: int = 190, mav_type: int = MAV_TYPE_GCS,
                   autopilot: int = MAV_AUTOPILOT_INVALID, base_mode: int = 0, custom_mode: int = 0,
                   system_status: int = 4, version: int = 2) -> bytes:
    payload = struct.pack("<IBBBBB", custom_mode, mav_type, autopilot, base_mode, system_status, 3)
    if version == 1:
        header = bytes([len(payload), seq & 0xFF, sysid, compid, MSG_HEARTBEAT])
        crc = x25_crc(header + payload + bytes([CRC_EXTRA[MSG_HEARTBEAT]]))
        return bytes([STX_V1]) + header + payload + struct.pack("<H", crc)
    # v2: truncate trailing zeros (at least one byte stays)
    p = payload.rstrip(b"\x00") or payload[:1]
    header = bytes([len(p), 0, 0, seq & 0xFF, sysid, compid, 0, 0, 0])
    crc = x25_crc(header + p + bytes([CRC_EXTRA[MSG_HEARTBEAT]]))
    return bytes([STX_V2]) + header + p + struct.pack("<H", crc)


def first_vehicle_heartbeat(buf: bytes) -> Optional[Heartbeat]:
    for hb in heartbeats(buf):
        if hb.is_vehicle:
            return hb
    return None
