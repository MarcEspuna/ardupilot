#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import random
import socket
import struct
import time
from typing import Iterable, List, Sequence, Tuple


Vector3 = Tuple[float, float, float]


class RTLSLinkBeaconSerialSim:
    MAGIC = b"RB"
    VERSION = 1

    MSG_HELLO = 1
    MSG_ANCHOR = 2
    MSG_POSITION = 3
    MSG_TDOA = 4
    MSG_CONFIG_END = 5
    MSG_ACK = 0x80

    ACK_OK = 0

    def __init__(
        self,
        address: Tuple[str, int],
        anchors: Sequence[Vector3] | None = None,
        sample_rate_hz: float = 20.0,
        position_error_m: float = 0.5,
        tdoa_sigma_m: float = 0.15,
        tdoa_noise_m: float = 0.0,
        tdoa_bias_m: float = 0.0,
        tdoa_dropout_pct: float = 0.0,
        tdoa_outlier_pct: float = 0.0,
        tdoa_outlier_m: float = 0.0,
        startup_position_s: float = 10.0,
        startup_position_max_abs_down_m: float = 1.0,
        seed: int = 1,
    ) -> None:
        self.address = address
        self.anchors: List[Vector3] = list(anchors or cube_anchors())
        self.sample_period = 1.0 / sample_rate_hz
        self.position_error_m = position_error_m
        self.tdoa_sigma_m = tdoa_sigma_m
        self.tdoa_noise_m = tdoa_noise_m
        self.tdoa_bias_m = tdoa_bias_m
        self.tdoa_dropout_pct = tdoa_dropout_pct
        self.tdoa_outlier_pct = tdoa_outlier_pct
        self.tdoa_outlier_m = tdoa_outlier_m
        self.rng = random.Random(seed)
        self.startup_position_s = startup_position_s
        self.startup_position_max_abs_down_m = startup_position_max_abs_down_m
        self.startup_position_complete = False
        self.sock: socket.socket | None = None
        self.seq = 0
        self.config_accepted = False
        self.config_accepted_time = 0.0
        self.last_config_time = 0.0
        self.last_sample_time = 0.0
        self.next_tdoa_pair = 0
        self.tdoa_pairs = [
            (anchor_a, anchor_b)
            for anchor_a in range(len(self.anchors))
            for anchor_b in range(anchor_a + 1, len(self.anchors))
        ]
        self.sent_positions = 0
        self.sent_tdoa = 0
        self.dropped_tdoa = 0
        self.rx = bytearray()

    def connect(self, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        while True:
            try:
                sock = socket.create_connection(self.address, timeout=1.0)
                sock.setblocking(False)
                self.sock = sock
                return
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.1)

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def service(self, position_ned: Vector3, now: float | None = None) -> None:
        if self.sock is None:
            raise RuntimeError("simulator socket is not connected")
        now = time.time() if now is None else now
        self._read_acks(now)
        if not self.config_accepted and now - self.last_config_time >= 0.5:
            self.send_config()
            self.last_config_time = now
            return
        if self.config_accepted and now - self.last_sample_time >= self.sample_period:
            if self._should_send_startup_position(position_ned, now):
                self.send_position(position_ned)
            self.send_tdoa(position_ned)
            self.last_sample_time = now

    def _should_send_startup_position(self, position_ned: Vector3, now: float) -> bool:
        if self.startup_position_complete:
            return False
        if now - self.config_accepted_time > self.startup_position_s:
            self.startup_position_complete = True
            return False
        if abs(position_ned[2]) >= self.startup_position_max_abs_down_m:
            self.startup_position_complete = True
            return False
        return True

    def send_config(self) -> None:
        self._send_frame(self.MSG_HELLO, struct.pack("<BBB", self.VERSION, 0, len(self.anchors)))
        for idx, pos in enumerate(self.anchors):
            self._send_frame(self.MSG_ANCHOR, struct.pack("<Biii", idx, *[meters_to_mm(v) for v in pos]))
        self._send_frame(self.MSG_CONFIG_END, struct.pack("<B", len(self.anchors)))

    def send_position(self, position_ned: Vector3) -> None:
        payload = struct.pack(
            "<iiiH",
            meters_to_mm(position_ned[0]),
            meters_to_mm(position_ned[1]),
            meters_to_mm(position_ned[2]),
            max(1, min(65535, meters_to_mm(self.position_error_m))),
        )
        self._send_frame(self.MSG_POSITION, payload)
        self.sent_positions += 1

    def send_tdoa(self, position_ned: Vector3) -> None:
        if not self.tdoa_pairs:
            return
        sigma_mm = max(1, min(65535, meters_to_mm(self.tdoa_sigma_m)))
        anchor_a, anchor_b = self.tdoa_pairs[self.next_tdoa_pair]
        self.next_tdoa_pair = (self.next_tdoa_pair + 1) % len(self.tdoa_pairs)
        if self.rng.random() * 100.0 < self.tdoa_dropout_pct:
            self.dropped_tdoa += 1
            return
        da = distance(position_ned, self.anchors[anchor_a])
        db = distance(position_ned, self.anchors[anchor_b])
        diff = db - da + self.tdoa_bias_m + self.rng.gauss(0.0, self.tdoa_noise_m)
        if self.rng.random() * 100.0 < self.tdoa_outlier_pct:
            diff += self.rng.choice((-1.0, 1.0)) * self.tdoa_outlier_m
        payload = struct.pack("<BBiH", anchor_a, anchor_b, meters_to_mm(diff), sigma_mm)
        self._send_frame(self.MSG_TDOA, payload)
        self.sent_tdoa += 1

    def _send_frame(self, msg_id: int, payload: bytes) -> None:
        if self.sock is None:
            return
        header = self.MAGIC + bytes((msg_id, len(payload), self.seq))
        self.seq = (self.seq + 1) & 0xFF
        frame_no_crc = header + payload
        self.sock.sendall(frame_no_crc + struct.pack("<H", crc16_ccitt(frame_no_crc)))

    def _read_acks(self, now: float | None = None) -> None:
        if self.sock is None:
            return
        now = time.time() if now is None else now
        while True:
            try:
                chunk = self.sock.recv(128)
            except BlockingIOError:
                break
            if not chunk:
                break
            self.rx.extend(chunk)

        while True:
            magic_idx = self.rx.find(self.MAGIC)
            if magic_idx < 0:
                self.rx.clear()
                return
            if magic_idx:
                del self.rx[:magic_idx]
            if len(self.rx) < 7:
                return
            msg_id = self.rx[2]
            payload_len = self.rx[3]
            frame_len = 2 + 3 + payload_len + 2
            if len(self.rx) < frame_len:
                return
            frame = bytes(self.rx[:frame_len])
            del self.rx[:frame_len]
            expected = struct.unpack("<H", frame[-2:])[0]
            if crc16_ccitt(frame[:-2]) != expected or msg_id != self.MSG_ACK or payload_len < 3:
                continue
            acked, status, version = frame[5], frame[6], frame[7]
            if acked == self.MSG_CONFIG_END and status == self.ACK_OK and version == self.VERSION:
                if not self.config_accepted:
                    self.config_accepted_time = now
                self.config_accepted = True


def rectangle_anchors() -> List[Vector3]:
    return [(7.5, 15.0, 0.0), (-2.5, 15.0, 0.0), (-2.5, -5.0, 0.0), (7.5, -5.0, 0.0)]


def cube_anchors() -> List[Vector3]:
    return [
        ((-10.0 if anchor_id & 0x01 else 10.0),
         (-10.0 if anchor_id & 0x02 else 10.0),
         (10.0 if anchor_id & 0x04 else -10.0))
        for anchor_id in range(8)
    ]


def distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def meters_to_mm(meters: float) -> int:
    if not math.isfinite(meters):
        return 0
    return int(round(meters * 1000.0))


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_position(value: str) -> Vector3:
    parts = [float(v.strip()) for v in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("position must be N,E,D")
    return (parts[0], parts[1], parts[2])


def main() -> None:
    parser = argparse.ArgumentParser(description="RTLS Link beacon/TDoA serial simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--geometry", choices=("rectangle", "cube"), default="cube")
    parser.add_argument("--position", type=parse_position, default=(0.0, 0.0, 0.0))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--noise", type=float, default=0.0)
    parser.add_argument("--sigma", type=float, default=0.15)
    parser.add_argument("--bias", type=float, default=0.0)
    parser.add_argument("--dropout-pct", type=float, default=0.0)
    parser.add_argument("--outlier-pct", type=float, default=0.0)
    parser.add_argument("--outlier-m", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--startup-position-max-abs-down", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    anchors = rectangle_anchors() if args.geometry == "rectangle" else cube_anchors()
    sim = RTLSLinkBeaconSerialSim(
        (args.host, args.port),
        anchors=anchors,
        sample_rate_hz=args.rate,
        tdoa_sigma_m=args.sigma,
        tdoa_noise_m=args.noise,
        tdoa_bias_m=args.bias,
        tdoa_dropout_pct=args.dropout_pct,
        tdoa_outlier_pct=args.outlier_pct,
        tdoa_outlier_m=args.outlier_m,
        startup_position_max_abs_down_m=args.startup_position_max_abs_down,
        seed=args.seed,
    )
    sim.connect()
    try:
        start = time.time()
        while time.time() - start < args.duration:
            sim.service(args.position)
            time.sleep(0.01)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
