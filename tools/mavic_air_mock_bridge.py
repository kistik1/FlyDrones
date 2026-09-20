#!/usr/bin/env python3
"""Mock the Android Mavic Air bridge without DJI hardware.

This is a development tool.  It never talks to a drone.  It accepts bridge
packets, prints accepted commands, returns synthetic telemetry, and optionally
streams a generated JPEG test pattern.
"""

from __future__ import annotations

import argparse
import io
import secrets
import socket
import struct
import time

import numpy as np
from PIL import Image, ImageDraw

from flydrones.drones.mavic_air import BridgeProtocolError, decode_bridge_message, encode_bridge_message


def status(nonce: str, token: str, armed: bool) -> dict:
    return {
        "version": 1,
        "type": "status",
        "client_nonce": nonce,
        "token": token,
        "connected": True,
        "armed": armed,
        "flying": armed,
        "product": "Mock Mavic Air",
        "battery_pct": 87.0,
        "alt_m": 1.0 if armed else 0.0,
        "vz_mps": 0.0,
        "yaw_deg": 0.0,
        "yaw_rate_dps": 0.0,
    }


def packet(payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + payload


def read_packet(sock: socket.socket) -> bytes:
    header = sock.recv(4)
    if len(header) != 4:
        raise OSError("incomplete video hello")
    size = struct.unpack(">I", header)[0]
    if size <= 0 or size > 4096:
        raise OSError("invalid video hello size")
    payload = bytearray()
    while len(payload) < size:
        chunk = sock.recv(size - len(payload))
        if not chunk:
            raise OSError("incomplete video hello")
        payload.extend(chunk)
    return bytes(payload)


def test_frame(number: int) -> bytes:
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    rgb[:, :, 1] = 32
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    draw.rectangle((20, 20, 300, 220), outline=(57, 255, 136), width=5)
    draw.text((36, 100), f"MOCK MAVIC AIR  {number:04d}", fill=(232, 238, 245))
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=70)
    return out.getvalue()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=45900)
    parser.add_argument("--video-port", type=int, default=45902)
    parser.add_argument("--armed", action="store_true", help="simulate the phone's local ARM button")
    parser.add_argument("--video", action="store_true", help="send a synthetic camera stream")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.1)
    print(f"mock bridge listening on {args.host}:{args.port}; armed={args.armed}")
    video_server = None
    if args.video:
        video_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        video_server.bind((args.host, args.video_port))
        video_server.listen(1)
        video_server.setblocking(False)
        print(f"mock video listening on {args.host}:{args.video_port}")
    client = None
    nonce = token = None
    video_client = None
    last_status = 0.0
    last_video = 0.0
    frame_number = 0
    try:
        while True:
            try:
                data, source = sock.recvfrom(4097)
                message = decode_bridge_message(data)
                if message["type"] == "hello":
                    client = source
                    nonce = message["client_nonce"]
                    token = secrets.token_hex(24)
                    if video_client is not None:
                        video_client.close()
                        video_client = None
                    print(f"session from {source[0]}:{source[1]}")
                elif message["type"] == "command" and message.get("token") == token:
                    axes = " ".join(f"{key}={message[key]:+.2f}" for key in ("throttle", "yaw", "forward", "lateral"))
                    print(f"command {message['seq']:05d} {axes} {'ACCEPT' if args.armed else 'REJECT(disarmed)'}")
                elif message["type"] == "disarm" and message.get("token") == token:
                    args.armed = False
                    print(f"disarmed: {message.get('reason', '')}")
            except TimeoutError:
                pass
            except BridgeProtocolError as exc:
                print(f"ignored packet: {exc}")

            now = time.monotonic()
            if client and nonce and token and now - last_status >= 0.1:
                sock.sendto(encode_bridge_message(status(nonce, token, args.armed)), client)
                last_status = now
            if video_server is not None and client and token and video_client is None:
                candidate = None
                try:
                    candidate, source = video_server.accept()
                    candidate.settimeout(0.2)
                    hello = decode_bridge_message(read_packet(candidate))
                    if source[0] != client[0] or hello.get("type") != "video_hello" or hello.get("token") != token:
                        raise OSError("invalid video client")
                    candidate.settimeout(None)
                    video_client = candidate
                    print(f"video client authenticated from {source[0]}:{source[1]}")
                except BlockingIOError:
                    pass
                except (OSError, BridgeProtocolError):
                    if candidate is not None:
                        candidate.close()
            if video_client is not None and now - last_video >= 0.1:
                try:
                    video_client.sendall(packet(test_frame(frame_number)))
                    frame_number += 1
                    last_video = now
                except OSError:
                    video_client.close()
                    video_client = None
    except KeyboardInterrupt:
        print("\nmock bridge stopped")
    finally:
        if video_client is not None:
            video_client.close()
        if video_server is not None:
            video_server.close()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
