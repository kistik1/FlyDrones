"""Original DJI Mavic Air bridge client.

DJI Mobile SDK runs on Android, not on this Python process.  This backend talks
to the Android bridge over a private Wi-Fi network.  The phone remains the
authority for arming Virtual Stick mode and rejects stale or unauthenticated
commands.
"""

from __future__ import annotations

import io
import json
import math
import secrets
import socket
import struct
import threading
import time
from dataclasses import asdict
from typing import Any

import numpy as np
from PIL import Image

from ..motor.command import AXES, FlightCommand
from ..safety import Telemetry
from .base import Drone

PROTOCOL_VERSION = 1
COMMAND_PORT = 45900
VIDEO_PORT = 45902
MAX_CONTROL_DATAGRAM = 4096
MAX_VIDEO_FRAME = 2_000_000
ALLOWED_MESSAGE_TYPES = {"hello", "status", "command", "disarm", "video_hello", "error"}


class BridgeProtocolError(ValueError):
    """A bridge packet is malformed or belongs to another protocol version."""


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BridgeProtocolError(f"{field} must be a finite number")
    return float(value)


def encode_bridge_message(message: dict[str, Any]) -> bytes:
    if not isinstance(message, dict):
        raise BridgeProtocolError("message must be an object")
    if message.get("version") != PROTOCOL_VERSION:
        raise BridgeProtocolError(f"version must be {PROTOCOL_VERSION}")
    if message.get("type") not in ALLOWED_MESSAGE_TYPES:
        raise BridgeProtocolError("unknown message type")
    raw = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(raw) > MAX_CONTROL_DATAGRAM:
        raise BridgeProtocolError("control packet is too large")
    return raw


def decode_bridge_message(data: bytes) -> dict[str, Any]:
    if len(data) > MAX_CONTROL_DATAGRAM:
        raise BridgeProtocolError("control packet is too large")
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BridgeProtocolError("invalid JSON") from exc
    if not isinstance(value, dict):
        raise BridgeProtocolError("message must be an object")
    if value.get("version") != PROTOCOL_VERSION:
        raise BridgeProtocolError(f"unsupported protocol version: {value.get('version')!r}")
    if value.get("type") not in ALLOWED_MESSAGE_TYPES:
        raise BridgeProtocolError("unknown message type")
    return value


def make_hello(client_nonce: str) -> dict[str, Any]:
    return {
        "version": PROTOCOL_VERSION,
        "type": "hello",
        "client_nonce": client_nonce,
    }


def make_command(token: str, seq: int, command: FlightCommand, timestamp_ms: int | None = None) -> dict[str, Any]:
    axes = {}
    for axis in AXES:
        value = _finite_number(getattr(command, axis), axis)
        axes[axis] = max(-1.0, min(1.0, value))
    return {
        "version": PROTOCOL_VERSION,
        "type": "command",
        "token": token,
        "seq": int(seq),
        "timestamp_ms": int(time.monotonic() * 1000 if timestamp_ms is None else timestamp_ms),
        **axes,
    }


class MavicAirDrone(Drone):
    """FlyDrones adapter for the Android Mavic Air bridge.

    ``takeoff`` and ``land`` never ask the aircraft to move.  Version 1 requires
    the operator to take off and land with the S01A remote.  ``land`` sends a
    zero command followed by a bridge disarm request.
    """

    name = "mavic-air"
    has_camera = True

    def __init__(
        self,
        host: str,
        port: int = COMMAND_PORT,
        video_port: int = VIDEO_PORT,
        connect_timeout_s: float = 3.0,
        status_timeout_s: float = 0.75,
    ):
        if not host:
            raise ValueError("Mavic Air bridge host is required")
        self.addr = (socket.gethostbyname(host), int(port))
        self.connect_timeout_s = float(connect_timeout_s)
        self.status_timeout_s = float(status_timeout_s)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)

        self.video_addr = (self.addr[0], int(video_port))
        self._video_sock: socket.socket | None = None
        self._video_buffer = bytearray()
        self._next_video_connect_at = 0.0
        self._frame: np.ndarray | None = None
        self._last_frame_at = 0.0

        self.client_nonce = secrets.token_hex(16)
        self.token: str | None = None
        self.seq = 0
        self._command_lock = threading.Lock()
        self._latest_command = FlightCommand.hover("heartbeat")
        self._latest_command_at = 0.0
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._last_status_at = 0.0
        self._bridge_armed = False
        self._tel = Telemetry()
        self._product = ""

    @property
    def bridge_armed(self) -> bool:
        return self._bridge_armed and self._status_fresh()

    @property
    def product(self) -> str:
        return self._product

    def _status_fresh(self) -> bool:
        return bool(self.token) and time.monotonic() - self._last_status_at <= self.status_timeout_s

    def _send_message(self, message: dict[str, Any]) -> None:
        self.sock.sendto(encode_bridge_message(message), self.addr)

    def _poll_control(self) -> None:
        while True:
            try:
                data, source = self.sock.recvfrom(MAX_CONTROL_DATAGRAM + 1)
            except BlockingIOError:
                return
            if source != self.addr:
                continue
            try:
                message = decode_bridge_message(data)
            except BridgeProtocolError:
                continue
            if message["type"] != "status" or message.get("client_nonce") != self.client_nonce:
                continue
            token = message.get("token")
            if not isinstance(token, str) or len(token) < 16:
                continue
            if self.token != token:
                self._drop_video_client()
            self.token = token
            self._last_status_at = time.monotonic()
            self._bridge_armed = bool(message.get("armed", False))
            self._product = str(message.get("product", ""))
            self._tel = Telemetry(
                t=self._last_status_at,
                alt_m=self._optional_number(message.get("alt_m")),
                vz_mps=self._optional_number(message.get("vz_mps")),
                yaw_deg=self._optional_number(message.get("yaw_deg")),
                yaw_rate_dps=self._optional_number(message.get("yaw_rate_dps")) or 0.0,
                battery_pct=self._optional_number(message.get("battery_pct")),
                flying=bool(message.get("flying", False)),
            )

    @staticmethod
    def _optional_number(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return _finite_number(value, "telemetry")
        except BridgeProtocolError:
            return None

    def connect(self) -> None:
        deadline = time.monotonic() + self.connect_timeout_s
        next_hello = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hello:
                self._send_message(make_hello(self.client_nonce))
                next_hello = now + 0.25
            self._poll_control()
            if self._status_fresh():
                product = self.product or "unknown DJI product"
                print(f"Mavic Air bridge connected: {product}; phone-side arm is {self._bridge_armed}")
                self._start_heartbeat()
                return
            time.sleep(0.02)
        raise ConnectionError(f"no Mavic Air bridge response from {self.addr[0]}:{self.addr[1]}")

    def takeoff(self) -> None:
        print("Mavic Air bridge: take off manually with the S01A remote, then arm on the phone.")

    def land(self) -> None:
        if not self.token:
            return
        self.send(FlightCommand.hover("disarm"))
        self._send_message({"version": PROTOCOL_VERSION, "type": "disarm", "token": self.token, "reason": "python stop"})
        self._bridge_armed = False
        print("Mavic Air bridge disarmed. Land with the S01A remote.")

    def emergency_stop(self) -> None:
        # Cutting motors in the air is not exposed for this aircraft.
        self.land()

    def send(self, cmd: FlightCommand) -> None:
        self._poll_control()
        if not self.token:
            return
        if not self._status_fresh():
            cmd = FlightCommand.hover("bridge status timeout")
            self._bridge_armed = False
        with self._command_lock:
            self._latest_command = cmd
            self._latest_command_at = time.monotonic()
            self._send_command_locked(cmd)

    def _send_command_locked(self, cmd: FlightCommand) -> None:
        if not self.token:
            return
        self.seq += 1
        self._send_message(make_command(self.token, self.seq, cmd))

    def _start_heartbeat(self) -> None:
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="mavic-air-heartbeat", daemon=True
        )
        self._heartbeat_thread.start()

    def _heartbeat_loop(self) -> None:
        """Feed the phone watchdog independently of video and brain rendering.

        Repeat a fresh command, but fall back to hover if the main control loop
        has not produced one for 250 ms. The Android watchdog remains the final
        fail-closed guard at 300 ms.
        """
        while not self._heartbeat_stop.wait(0.1):
            with self._command_lock:
                fresh = time.monotonic() - self._latest_command_at <= 0.25
                cmd = self._latest_command if fresh and self._status_fresh() else FlightCommand.hover("heartbeat stale")
                try:
                    self._send_command_locked(cmd)
                except OSError:
                    return

    def telemetry(self) -> Telemetry:
        self._poll_control()
        if not self._status_fresh():
            return Telemetry(t=time.monotonic(), flying=False)
        return Telemetry(**asdict(self._tel))

    def _drop_video_client(self) -> None:
        if self._video_sock is not None:
            self._video_sock.close()
        self._video_sock = None
        self._video_buffer.clear()
        self._next_video_connect_at = time.monotonic() + 0.25

    def _poll_video(self) -> None:
        if self._video_sock is None:
            if not self.token or time.monotonic() < self._next_video_connect_at:
                return
            try:
                client = socket.create_connection(self.video_addr, timeout=0.2)
                hello = encode_bridge_message({"version": PROTOCOL_VERSION, "type": "video_hello", "token": self.token})
                client.sendall(struct.pack(">I", len(hello)) + hello)
                client.setblocking(False)
                self._video_sock = client
            except OSError:
                self._next_video_connect_at = time.monotonic() + 0.25
                return
        eof = False
        while True:
            try:
                chunk = self._video_sock.recv(65536)
            except BlockingIOError:
                break
            except OSError:
                self._drop_video_client()
                return
            if not chunk:
                eof = True
                break
            self._video_buffer.extend(chunk)
        while len(self._video_buffer) >= 4:
            size = struct.unpack(">I", self._video_buffer[:4])[0]
            if size == 0 or size > MAX_VIDEO_FRAME:
                self._drop_video_client()
                return
            if len(self._video_buffer) < 4 + size:
                break
            payload = bytes(self._video_buffer[4:4 + size])
            del self._video_buffer[:4 + size]
            try:
                rgb = np.asarray(Image.open(io.BytesIO(payload)).convert("RGB"))
            except (OSError, ValueError):
                continue
            self._frame = np.ascontiguousarray(rgb[..., ::-1])
            self._last_frame_at = time.monotonic()
        if eof:
            if self._video_sock is not None:
                self._video_sock.close()
            self._video_sock = None
            self._video_buffer.clear()
            self._next_video_connect_at = time.monotonic() + 0.25

    def frame(self) -> np.ndarray | None:
        self._poll_video()
        if time.monotonic() - self._last_frame_at > 0.5:
            return None
        return self._frame

    def close(self) -> None:
        self._heartbeat_stop.set()
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=0.5)
            self._heartbeat_thread = None
        if self.token and self._bridge_armed:
            self.land()
        self._drop_video_client()
        self.sock.close()
