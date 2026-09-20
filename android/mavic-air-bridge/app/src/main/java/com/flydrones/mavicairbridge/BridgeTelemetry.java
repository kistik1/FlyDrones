package com.flydrones.mavicairbridge;

final class BridgeTelemetry {
    volatile boolean connected;
    volatile boolean armed;
    volatile boolean flying;
    volatile String product = "No DJI product";
    volatile Double batteryPct;
    volatile Double altitudeM;
    volatile Double verticalSpeedMps;
    volatile Double yawDeg;
    volatile Double yawRateDps;
}
