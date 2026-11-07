"""ArduPilot JSON SITL physics backend (libraries/SITL/SIM_JSON.cpp).

SITL sends a binary servo packet each frame to UDP 9002 and blocks
(lockstep) until the physics replies with one JSON line describing the
vehicle after that frame. Frame time is 1/frame_rate; a repeated
frame_count means SITL resent the packet, so the previous reply is sent
again without stepping.
"""
from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass

MAGIC_16 = 18458
MAGIC_32 = 29569
HDR = struct.Struct("<HHI")


@dataclass
class ServoPacket:
    frame_rate: int
    frame_count: int
    pwm: tuple[int, ...]


def parse_servo_packet(data: bytes) -> ServoPacket | None:
    if len(data) < HDR.size:
        return None
    magic, rate, count = HDR.unpack_from(data)
    if magic == MAGIC_16:
        n = 16
    elif magic == MAGIC_32:
        n = 32
    else:
        return None
    if len(data) < HDR.size + 2 * n:
        return None
    pwm = struct.unpack_from(f"<{n}H", data, HDR.size)
    return ServoPacket(rate, count, pwm)


def build_servo_packet(rate: int, count: int, pwm: list[int]) -> bytes:
    """Used by tests to impersonate SITL."""
    pwm = list(pwm) + [0] * (16 - len(pwm))
    return HDR.pack(MAGIC_16, rate, count) + struct.pack("<16H", *pwm[:16])


def state_json(timestamp: float, gyro, accel_body, position, quaternion, velocity) -> bytes:
    msg = {
        "timestamp": round(timestamp, 6),
        "imu": {"gyro": [float(x) for x in gyro],
                "accel_body": [float(x) for x in accel_body]},
        "position": [float(x) for x in position],
        "quaternion": [float(x) for x in quaternion],
        "velocity": [float(x) for x in velocity],
    }
    return ("\n" + json.dumps(msg, separators=(",", ":")) + "\n").encode()


class JsonBridge:
    def __init__(self, port: int = 9002, timeout_s: float = 30.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(timeout_s)
        self.peer = None
        self.last_count: int | None = None
        self.last_reply: bytes | None = None
        self.resets = 0

    def receive(self) -> tuple[ServoPacket, bool] | None:
        """Next packet and whether it is new (False for a resend)."""
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                return None
            pkt = parse_servo_packet(data)
            if pkt is None:
                continue
            self.peer = addr
            if self.last_count is not None and pkt.frame_count == self.last_count:
                return pkt, False
            if self.last_count is not None and pkt.frame_count < self.last_count:
                self.resets += 1  # SITL restarted; caller decides what to do
            self.last_count = pkt.frame_count
            return pkt, True

    def reply(self, payload: bytes | None = None):
        if payload is not None:
            self.last_reply = payload
        if self.peer is not None and self.last_reply is not None:
            self.sock.sendto(self.last_reply, self.peer)

    def close(self):
        self.sock.close()
