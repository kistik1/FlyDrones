import io
import socket
import struct
import threading
import time

import numpy as np
import pytest
from PIL import Image

from flydrones.drones.mavic_air import (
    BridgeProtocolError,
    MavicAirDrone,
    decode_bridge_message,
    encode_bridge_message,
    make_command,
)
from flydrones.motor import FlightCommand


class MockBridge:
    def __init__(self, armed=True, video=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.armed = armed
        self.video = video
        self.send_status = True
        self.commands = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.token = "0123456789abcdef0123456789abcdef"
        self.video_sock = None
        self.video_port = 9
        self.video_thread = None
        if video:
            self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.video_sock.bind(("127.0.0.1", 0))
            self.video_sock.listen(1)
            self.video_sock.settimeout(0.05)
            self.video_port = self.video_sock.getsockname()[1]
            self.video_thread = threading.Thread(target=self.send_video, daemon=True)

    def __enter__(self):
        self.thread.start()
        if self.video_thread is not None:
            self.video_thread.start()
        return self

    def __exit__(self, *_):
        self.stop.set()
        self.thread.join(timeout=1)
        self.sock.close()
        if self.video_sock is not None:
            self.video_sock.close()
        if self.video_thread is not None:
            self.video_thread.join(timeout=1)

    def run(self):
        client = nonce = None
        last_status = 0.0
        while not self.stop.is_set():
            try:
                data, source = self.sock.recvfrom(4097)
                msg = decode_bridge_message(data)
                if msg["type"] == "hello":
                    client, nonce = source, msg["client_nonce"]
                elif msg["type"] == "command":
                    self.commands.append(msg)
                elif msg["type"] == "disarm":
                    self.armed = False
            except TimeoutError:
                pass
            now = time.monotonic()
            if client and self.send_status and now - last_status > 0.03:
                status = {
                    "version": 1, "type": "status", "client_nonce": nonce, "token": self.token,
                    "connected": True, "armed": self.armed, "flying": self.armed,
                    "product": "Mock Mavic Air", "alt_m": 1.25, "yaw_rate_dps": 2.5, "battery_pct": 80.0,
                }
                self.sock.sendto(encode_bridge_message(status), client)
                last_status = now

    @staticmethod
    def recv_exact(sock, size):
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise OSError("unexpected EOF")
            data.extend(chunk)
        return bytes(data)

    def send_video(self):
        video = None
        while not self.stop.is_set() and video is None:
            try:
                video, _ = self.video_sock.accept()
            except TimeoutError:
                pass
        if video is None:
            return
        size = struct.unpack(">I", self.recv_exact(video, 4))[0]
        hello = decode_bridge_message(self.recv_exact(video, size))
        if hello.get("type") != "video_hello" or hello.get("token") != self.token:
            video.close()
            return
        rgb = np.zeros((8, 12, 3), dtype=np.uint8)
        rgb[:, :, 0] = 255
        data = io.BytesIO()
        Image.fromarray(rgb).save(data, format="PNG")
        payload = data.getvalue()
        video.sendall(struct.pack(">I", len(payload)) + payload)
        video.close()


def test_protocol_rejects_bad_messages():
    with pytest.raises(BridgeProtocolError):
        decode_bridge_message(b"not json")
    with pytest.raises(BridgeProtocolError):
        decode_bridge_message(b'{"version":2,"type":"hello"}')
    with pytest.raises(BridgeProtocolError):
        encode_bridge_message({"version": 1, "type": "unknown"})
    with pytest.raises(BridgeProtocolError):
        make_command("token-token-token", 1, FlightCommand(throttle=float("nan")))


def test_command_clips_axes():
    msg = make_command("0123456789abcdef", 7, FlightCommand(throttle=2, yaw=-2), timestamp_ms=100)
    assert msg["throttle"] == 1.0
    assert msg["yaw"] == -1.0
    assert msg["seq"] == 7


def test_adapter_handshake_command_and_telemetry():
    with MockBridge() as bridge:
        drone = MavicAirDrone("127.0.0.1", port=bridge.port, video_port=bridge.video_port, connect_timeout_s=1)
        try:
            drone.connect()
            assert drone.product == "Mock Mavic Air"
            assert drone.bridge_armed
            drone.send(FlightCommand(throttle=0.2, yaw=-0.1))
            deadline = time.monotonic() + 1
            while not bridge.commands and time.monotonic() < deadline:
                time.sleep(0.01)
            assert bridge.commands[-1]["throttle"] == pytest.approx(0.2)
            tel = drone.telemetry()
            assert tel.alt_m == pytest.approx(1.25)
            assert tel.battery_pct == pytest.approx(80)
        finally:
            drone.close()


def test_stale_status_forces_zero_command():
    with MockBridge() as bridge:
        drone = MavicAirDrone("127.0.0.1", port=bridge.port, video_port=bridge.video_port, connect_timeout_s=1, status_timeout_s=0.02)
        try:
            drone.connect()
            bridge.send_status = False
            time.sleep(0.04)
            drone.send(FlightCommand(throttle=0.2, yaw=-0.1))
            deadline = time.monotonic() + 1
            while not bridge.commands and time.monotonic() < deadline:
                time.sleep(0.01)
            assert bridge.commands[-1]["throttle"] == 0
            assert bridge.commands[-1]["yaw"] == 0
            assert not drone.bridge_armed
        finally:
            drone.close()


def test_heartbeat_repeats_then_zeroes_a_stale_command():
    with MockBridge() as bridge:
        drone = MavicAirDrone("127.0.0.1", port=bridge.port, video_port=bridge.video_port, connect_timeout_s=1)
        try:
            drone.connect()
            drone.send(FlightCommand(yaw=0.2))
            time.sleep(0.18)
            assert any(command["yaw"] == pytest.approx(0.2) for command in bridge.commands)
            time.sleep(0.25)
            assert bridge.commands[-1]["yaw"] == 0
        finally:
            drone.close()


def test_adapter_receives_authenticated_video():
    with MockBridge(video=True) as bridge:
        drone = MavicAirDrone("127.0.0.1", port=bridge.port, video_port=bridge.video_port, connect_timeout_s=1)
        try:
            drone.connect()
            frame = None
            deadline = time.monotonic() + 1
            while frame is None and time.monotonic() < deadline:
                frame = drone.frame()
                time.sleep(0.01)
            assert frame is not None
            assert frame.shape == (8, 12, 3)
            assert frame[0, 0, 2] == 255  # bridge returns BGR
        finally:
            drone.close()
