package com.flydrones.mavicairbridge;

import android.graphics.SurfaceTexture;

interface DroneTransport {
    interface ResultCallback {
        void onResult(boolean success, String message);
    }

    void start(ResultCallback callback);
    void attachVideoSurface(SurfaceTexture surface, int width, int height);
    void detachVideoSurface();
    void setArmed(boolean armed, ResultCallback callback);
    void send(BridgeCommand command);
    void zeroAndDisarm(String reason);
    BridgeTelemetry telemetry();
    byte[] latestJpeg();
    void close();
}
