package com.flydrones.mavicairbridge;

import org.json.JSONException;
import org.json.JSONObject;

final class BridgeCommand {
    static final float PHONE_AXIS_LIMIT = 0.25f;

    final long sequence;
    final long clientTimestampMs;
    final float throttle;
    final float yaw;
    final float forward;
    final float lateral;

    private BridgeCommand(long sequence, long clientTimestampMs, float throttle, float yaw, float forward, float lateral) {
        this.sequence = sequence;
        this.clientTimestampMs = clientTimestampMs;
        this.throttle = throttle;
        this.yaw = yaw;
        this.forward = forward;
        this.lateral = lateral;
    }

    static BridgeCommand parse(JSONObject json) throws JSONException {
        long sequence = json.getLong("seq");
        long timestamp = json.getLong("timestamp_ms");
        if (sequence < 1 || timestamp < 0) throw new JSONException("invalid sequence or timestamp");
        return new BridgeCommand(
                sequence,
                timestamp,
                axis(json, "throttle"),
                axis(json, "yaw"),
                axis(json, "forward"),
                axis(json, "lateral")
        );
    }

    static BridgeCommand zero(long sequence) {
        return new BridgeCommand(sequence, 0, 0, 0, 0, 0);
    }

    private static float axis(JSONObject json, String name) throws JSONException {
        double value = json.getDouble(name);
        if (!Double.isFinite(value)) throw new JSONException(name + " is not finite");
        return (float) Math.max(-PHONE_AXIS_LIMIT, Math.min(PHONE_AXIS_LIMIT, value));
    }
}
