# Original Mavic Air Android bridge

This integration targets the U11X Mavic Air and S01A remote. It does not apply
to the Air 2, Air 2S, Air 3, or later aircraft.

The bridge has passed a propeller-off connected bench test. It has not completed
a flight test. Keep the propellers removed during development.

## Design

The Android phone owns the USB connection to the S01A remote and runs DJI
Mobile SDK v4.18. FlyDrones communicates with the phone across a private laptop
hotspot:

```text
FlyDrones -- UDP control/status --> Android -- DJI Mobile SDK --> S01A --> U11X
FlyDrones -- authenticated TCP connection --> Android camera decoder
FlyDrones <-- JPEG frames ------------------ Android camera decoder
```

The phone listens for video clients on TCP port 45902. The laptop opens that
connection after the UDP handshake and authenticates it with the session token.

The phone creates a random session token for every laptop handshake. It only
forwards commands after the operator presses ARM on the phone. A missing command
for 300 ms, an invalid packet, a replayed sequence, or a closed bridge sends a
zero command and disables Virtual Stick mode.

`timestamp_ms` is the laptop's monotonic timestamp. The phone checks that it
increases, but it does not compare it to the phone clock. Freshness comes from
the phone's local 300 ms receive watchdog.

## Test with no DJI hardware

Install the base package and developer dependencies, then start the mock Android
bridge:

```bash
source .venv/bin/activate
python tools/mavic_air_mock_bridge.py --armed --video
```

In a second terminal, run FlyDrones against the mock:

```bash
source .venv/bin/activate
flydrones fly \
  --drone mavic-air \
  --bridge-host 127.0.0.1 \
  --config configs/mavic_air.yaml \
  --input camera \
  --seconds 5 \
  --send
```

Here `--send` sends packets only to the local mock process. The mock prints each
accepted command and generates a synthetic camera frame.

## Configure the Android project

The project is in `android/mavic-air-bridge`.

1. In DJI Developer, register an Android Mobile SDK v4 application with package
   name `com.flydrones.mavicairbridge`.
2. Copy `local.properties.example` to `local.properties`.
3. Set `sdk.dir` and `DJI_API_KEY` in that local file. Git ignores it.
4. Open the directory in Android Studio. Use JDK 17 and install Android SDK 34.
5. Build and install the debug APK on an ARM Android phone.

Keep the phone unlocked with the bridge activity in the foreground while using
it. The activity keeps the screen awake. If Android enters Doze or battery
saver while the phone is locked, the OS can block inbound bridge packets; the
aircraft remains disarmed.

The project pins DJI Mobile SDK 4.18, the latest official v4 Android release.
It uses the SDK's documented `dji-sdk` and `dji-sdk-provided` Maven artifacts.

## Laptop hotspot

Create a password-protected hotspot on the laptop and connect the phone. Find
the phone's hotspot IP from the laptop's client list. The Android app listens on
UDP port 45900 and TCP port 45902.

Do not expose either port to the internet. Protocol v1 authenticates a session
with a random token, but it does not encrypt traffic.

## Current safety limits

`configs/mavic_air.yaml` sets these initial limits:

- normalized vertical and yaw command: 0.20
- normalized forward and lateral command: 0.15
- altitude ceiling: 1.5 m
- maximum session: 30 seconds
- low-battery threshold: 35 percent
- automatic takeoff: disabled

The Android app independently clamps every normalized axis to 0.25. It maps a
full normalized horizontal command to 2 m/s, vertical to 1 m/s, and yaw to 45
degrees per second. The stricter Python configuration wins in normal use.

## Connected bench-test status

- All Python tests pass.
- The mock bridge survives disconnect, replay, malformed-packet, and stale-link tests.
- Android Studio builds the app with the user's DJI key.
- The phone displays the expected Mavic Air product name.
- The aircraft cannot take off from a laptop message.
- ARM can only be pressed on the phone.
- Closing the laptop or disabling Wi-Fi disarms Virtual Stick.
- The authenticated camera stream delivers 320x240 frames over TCP.

The connected test used status requests and zero commands only. A first flight
requires a separate safety review and test plan.
