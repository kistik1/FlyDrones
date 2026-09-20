package com.flydrones.mavicairbridge;

import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

final class BridgeServer implements AutoCloseable {
    private static final String TAG = "FlyDronesBridge";
    interface Listener { void onBridgeStatus(String message); }

    static final int PROTOCOL_VERSION = 1;
    static final int COMMAND_PORT = 45900;
    static final int VIDEO_PORT = 45902;
    static final long DEADMAN_MS = 300;
    static final int MAX_DATAGRAM = 4096;
    static final int MAX_VIDEO_FRAME = 2_000_000;

    private final DroneTransport transport;
    private final Listener listener;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final SecureRandom random = new SecureRandom();
    private DatagramChannel channel;
    private Thread serverThread;
    private ServerSocket videoServer;
    private Thread videoThread;
    private volatile Socket activeVideo;
    private volatile Session session;
    private volatile long lastValidCommandAt;

    BridgeServer(DroneTransport transport, Listener listener) {
        this.transport = transport;
        this.listener = listener;
    }

    void start() throws IOException {
        if (!running.compareAndSet(false, true)) return;
        channel = DatagramChannel.open(StandardProtocolFamily.INET);
        channel.configureBlocking(false);
        channel.bind(new InetSocketAddress("0.0.0.0", COMMAND_PORT));
        videoServer = new ServerSocket();
        videoServer.setReuseAddress(true);
        videoServer.bind(new InetSocketAddress("0.0.0.0", VIDEO_PORT));
        videoServer.setSoTimeout(500);
        serverThread = new Thread(this::run, "mavic-air-bridge");
        videoThread = new Thread(this::runVideo, "mavic-air-video");
        serverThread.start();
        videoThread.start();
        listener.onBridgeStatus("Bridge listening on UDP " + COMMAND_PORT + " and TCP " + VIDEO_PORT);
    }

    private void run() {
        Log.i(TAG, "control loop started");
        ByteBuffer buffer = ByteBuffer.allocate(MAX_DATAGRAM + 1);
        long lastStatusAt = 0;
        while (running.get()) {
            try {
                buffer.clear();
                SocketAddress source = channel.receive(buffer);
                if (source != null) {
                    InetSocketAddress sender = (InetSocketAddress) source;
                    int length = buffer.position();
                    Log.d(TAG, "received " + length + " bytes from " + sender.getAddress().getHostAddress() + ":" + sender.getPort());
                    if (length > MAX_DATAGRAM) throw new JSONException("packet too large");
                    String text = new String(buffer.array(), 0, length, StandardCharsets.UTF_8);
                    handle(new JSONObject(text), sender.getAddress(), sender.getPort());
                } else {
                    Thread.sleep(10);
                }
            } catch (JSONException | IOException error) {
                Log.w(TAG, "rejected control packet", error);
                failClosed("invalid bridge packet: " + error.getMessage());
            } catch (InterruptedException error) {
                Thread.currentThread().interrupt();
                break;
            }

            long now = android.os.SystemClock.elapsedRealtime();
            if (transport.telemetry().armed && now - lastValidCommandAt > DEADMAN_MS) {
                failClosed("command watchdog expired");
            }
            if (session != null && now - lastStatusAt >= 100) {
                sendStatus(session);
                lastStatusAt = now;
            }
        }
    }

    private void handle(JSONObject json, InetAddress address, int port) throws JSONException {
        if (json.getInt("version") != PROTOCOL_VERSION) throw new JSONException("unsupported version");
        String type = json.getString("type");
        if ("hello".equals(type)) {
            String nonce = boundedString(json, "client_nonce", 16, 128);
            Session replacement = new Session(address, port, nonce, randomToken());
            Log.i(TAG, "hello from " + address.getHostAddress() + ":" + port);
            session = replacement;
            lastValidCommandAt = android.os.SystemClock.elapsedRealtime();
            sendStatus(replacement);
            listener.onBridgeStatus("Laptop session " + address.getHostAddress() + "; DISARMED until local ARM");
            return;
        }
        Session current = session;
        if (current == null || !current.address.equals(address) || current.port != port) throw new JSONException("no session");
        if (!current.token.equals(json.optString("token"))) throw new JSONException("bad token");
        if ("disarm".equals(type)) {
            failClosed("laptop requested disarm");
            return;
        }
        if (!"command".equals(type)) throw new JSONException("unknown type");
        BridgeCommand command = BridgeCommand.parse(json);
        if (command.sequence <= current.lastSequence || command.clientTimestampMs < current.lastClientTimestamp) {
            throw new JSONException("replayed command");
        }
        current.lastSequence = command.sequence;
        current.lastClientTimestamp = command.clientTimestampMs;
        lastValidCommandAt = android.os.SystemClock.elapsedRealtime();
        if (transport.telemetry().armed) transport.send(command);
    }

    private void sendStatus(Session target) {
        BridgeTelemetry telemetry = transport.telemetry();
        try {
            JSONObject json = new JSONObject();
            json.put("version", PROTOCOL_VERSION);
            json.put("type", "status");
            json.put("client_nonce", target.nonce);
            json.put("token", target.token);
            json.put("connected", telemetry.connected);
            json.put("armed", telemetry.armed);
            json.put("flying", telemetry.flying);
            json.put("product", telemetry.product);
            putNullable(json, "battery_pct", telemetry.batteryPct);
            putNullable(json, "alt_m", telemetry.altitudeM);
            putNullable(json, "vz_mps", telemetry.verticalSpeedMps);
            putNullable(json, "yaw_deg", telemetry.yawDeg);
            putNullable(json, "yaw_rate_dps", telemetry.yawRateDps);
            byte[] data = json.toString().getBytes(StandardCharsets.UTF_8);
            channel.send(ByteBuffer.wrap(data), new InetSocketAddress(target.address, target.port));
        } catch (JSONException | IOException error) {
            Log.w(TAG, "status send failed", error);
            listener.onBridgeStatus("Status send failed: " + error.getMessage());
        }
    }

    private void runVideo() {
        Log.i(TAG, "video server started on TCP " + VIDEO_PORT);
        while (running.get()) {
            try {
                Socket video = videoServer.accept();
                authenticateAndStreamVideo(video);
            } catch (SocketTimeoutException ignored) {
            } catch (Exception error) {
                if (running.get()) Log.w(TAG, "video connection rejected", error);
            }
        }
    }

    private void authenticateAndStreamVideo(Socket video) throws IOException, JSONException, InterruptedException {
        try (Socket connection = video) {
            connection.setSoTimeout(1000);
            InetAddress address = connection.getInetAddress();
            DataInputStream input = new DataInputStream(connection.getInputStream());
            int size = input.readInt();
            if (size <= 0 || size > MAX_DATAGRAM) throw new IOException("invalid video hello size");
            byte[] payload = new byte[size];
            input.readFully(payload);
            JSONObject hello = new JSONObject(new String(payload, StandardCharsets.UTF_8));
            Session target = session;
            if (target == null || !target.address.equals(address)) throw new IOException("video client has no control session");
            if (hello.getInt("version") != PROTOCOL_VERSION || !"video_hello".equals(hello.getString("type"))) {
                throw new IOException("invalid video hello");
            }
            if (!target.token.equals(hello.optString("token"))) throw new IOException("bad video token");

            Socket previous = activeVideo;
            activeVideo = connection;
            if (previous != null && previous != connection) {
                try { previous.close(); } catch (IOException ignored) { }
            }
            connection.setSoTimeout(0);
            DataOutputStream output = new DataOutputStream(new BufferedOutputStream(connection.getOutputStream()));
            listener.onBridgeStatus("Video streaming to " + address.getHostAddress());
            while (running.get() && session == target && activeVideo == connection) {
                byte[] jpeg = transport.latestJpeg();
                if (jpeg != null && jpeg.length > 0 && jpeg.length <= MAX_VIDEO_FRAME) writeFrame(output, jpeg);
                Thread.sleep(100);
            }
        } finally {
            if (activeVideo == video) activeVideo = null;
        }
    }

    private static void writeFrame(DataOutputStream output, byte[] payload) throws IOException {
        output.writeInt(payload.length);
        output.write(payload);
        output.flush();
    }

    private void failClosed(String reason) {
        transport.zeroAndDisarm(reason);
        listener.onBridgeStatus("DISARMED: " + reason);
    }

    private String randomToken() {
        byte[] bytes = new byte[24];
        random.nextBytes(bytes);
        StringBuilder value = new StringBuilder(bytes.length * 2);
        for (byte item : bytes) value.append(String.format(Locale.US, "%02x", item & 0xff));
        return value.toString();
    }

    private static String boundedString(JSONObject json, String field, int min, int max) throws JSONException {
        String value = json.getString(field);
        if (value.length() < min || value.length() > max) throw new JSONException("invalid " + field);
        return value;
    }

    private static void putNullable(JSONObject json, String field, Double value) throws JSONException {
        json.put(field, value == null || !Double.isFinite(value) ? JSONObject.NULL : value);
    }

    @Override
    public void close() {
        running.set(false);
        failClosed("bridge closed");
        if (channel != null) {
            try { channel.close(); } catch (IOException ignored) { }
        }
        if (activeVideo != null) {
            try { activeVideo.close(); } catch (IOException ignored) { }
        }
        if (videoServer != null) {
            try { videoServer.close(); } catch (IOException ignored) { }
        }
        if (serverThread != null) {
            try { serverThread.join(1000); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
        }
        if (videoThread != null) {
            try { videoThread.join(1000); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
        }
    }

    private static final class Session {
        final InetAddress address;
        final int port;
        final String nonce;
        final String token;
        volatile long lastSequence;
        volatile long lastClientTimestamp;

        Session(InetAddress address, int port, String nonce, String token) {
            this.address = address;
            this.port = port;
            this.nonce = nonce;
            this.token = token;
        }
    }
}
