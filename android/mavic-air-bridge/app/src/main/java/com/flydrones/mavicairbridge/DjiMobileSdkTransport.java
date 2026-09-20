package com.flydrones.mavicairbridge;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.view.TextureView;

import java.io.ByteArrayOutputStream;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

final class DjiMobileSdkTransport implements DroneTransport {
    private static final float MAX_HORIZONTAL_MPS = 2.0f;
    private static final float MAX_VERTICAL_MPS = 1.0f;
    private static final float MAX_YAW_DPS = 45.0f;

    private final Context context;
    private final TextureView preview;
    private final BridgeTelemetry telemetry = new BridgeTelemetry();
    private volatile FlightController flightController;
    private volatile DJICodecManager codecManager;
    private volatile VideoFeeder.VideoDataListener videoListener;

    DjiMobileSdkTransport(Context context, TextureView preview) {
        this.context = context.getApplicationContext();
        this.preview = preview;
    }

    @Override
    public void start(ResultCallback callback) {
        DJISDKManager.getInstance().registerApp(context, new DJISDKManager.SDKManagerCallback() {
            @Override public void onRegister(DJIError error) {
                if (error == DJISDKError.REGISTRATION_SUCCESS) {
                    DJISDKManager.getInstance().startConnectionToProduct();
                    callback.onResult(true, "DJI SDK registered");
                } else {
                    callback.onResult(false, "DJI registration failed: " + error.getDescription());
                }
            }

            @Override public void onProductDisconnect() { bindProduct(null); }
            @Override public void onProductConnect(BaseProduct product) { bindProduct(product); }
            @Override public void onProductChanged(BaseProduct product) { bindProduct(product); }
            @Override public void onComponentChange(BaseProduct.ComponentKey key, BaseComponent oldComponent, BaseComponent newComponent) {
                bindProduct(DJISDKManager.getInstance().getProduct());
            }
            @Override public void onInitProcess(DJISDKInitEvent event, int totalProcess) { }
            @Override public void onDatabaseDownloadProgress(long current, long total) { }
        });
    }

    private synchronized void bindProduct(BaseProduct product) {
        telemetry.connected = product != null && product.isConnected() && product instanceof Aircraft;
        telemetry.product = product != null && product.getModel() != null ? product.getModel().getDisplayName() : "No DJI product";
        if (!(product instanceof Aircraft)) {
            flightController = null;
            telemetry.armed = false;
            telemetry.flying = false;
            return;
        }
        Aircraft aircraft = (Aircraft) product;
        flightController = aircraft.getFlightController();
        if (flightController != null) {
            flightController.setStateCallback(state -> {
                telemetry.flying = state.isFlying();
                telemetry.altitudeM = state.getAircraftLocation() == null ? null : (double) state.getAircraftLocation().getAltitude();
                telemetry.verticalSpeedMps = (double) state.getVelocityZ();
                telemetry.yawDeg = state.getAttitude() == null ? null : (double) state.getAttitude().yaw;
            });
        }
        if (aircraft.getBattery() != null) {
            aircraft.getBattery().setStateCallback(state -> telemetry.batteryPct = (double) state.getChargeRemainingInPercent());
        }
        startVideoFeed();
    }

    @Override
    public synchronized void attachVideoSurface(SurfaceTexture surface, int width, int height) {
        detachVideoSurface();
        codecManager = new DJICodecManager(context, surface, width, height);
        startVideoFeed();
    }

    private synchronized void startVideoFeed() {
        if (codecManager == null || videoListener != null) return;
        VideoFeeder feeder = VideoFeeder.getInstance();
        if (feeder == null) return;
        VideoFeeder.VideoFeed feed = feeder.getPrimaryVideoFeed();
        if (feed == null) return;
        videoListener = (buffer, size) -> {
            DJICodecManager codec = codecManager;
            if (codec != null) codec.sendDataToDecoder(buffer, size);
        };
        feed.addVideoDataListener(videoListener);
    }

    @Override
    public synchronized void detachVideoSurface() {
        VideoFeeder feeder = VideoFeeder.getInstance();
        VideoFeeder.VideoFeed feed = feeder == null ? null : feeder.getPrimaryVideoFeed();
        if (feed != null && videoListener != null) feed.removeVideoDataListener(videoListener);
        videoListener = null;
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
    }

    @Override
    public synchronized void setArmed(boolean armed, ResultCallback callback) {
        FlightController controller = flightController;
        if (controller == null || !telemetry.connected) {
            callback.onResult(false, "Mavic Air is not connected");
            return;
        }
        if (!armed) {
            send(BridgeCommand.zero(0));
            controller.setVirtualStickModeEnabled(false, error -> {
                telemetry.armed = false;
                callback.onResult(error == null, error == null ? "Virtual Stick disarmed" : error.getDescription());
            });
            return;
        }
        controller.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        controller.setVerticalControlMode(VerticalControlMode.VELOCITY);
        controller.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        controller.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        controller.setVirtualStickModeEnabled(true, error -> {
            telemetry.armed = error == null;
            callback.onResult(error == null, error == null ? "Virtual Stick armed" : error.getDescription());
        });
    }

    @Override
    public void send(BridgeCommand command) {
        FlightController controller = flightController;
        if (controller == null || !telemetry.armed) return;
        FlightControlData data = new FlightControlData(
                command.forward * MAX_HORIZONTAL_MPS,
                command.lateral * MAX_HORIZONTAL_MPS,
                command.yaw * MAX_YAW_DPS,
                command.throttle * MAX_VERTICAL_MPS
        );
        controller.sendVirtualStickFlightControlData(data, error -> { });
    }

    @Override
    public void zeroAndDisarm(String reason) {
        if (!telemetry.armed) return;
        setArmed(false, (success, message) -> { });
    }

    @Override public BridgeTelemetry telemetry() { return telemetry; }

    @Override
    public byte[] latestJpeg() {
        if (!preview.isAvailable()) return null;
        Bitmap bitmap = preview.getBitmap(320, 240);
        if (bitmap == null) return null;
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.JPEG, 65, output);
        bitmap.recycle();
        return output.toByteArray();
    }

    @Override
    public void close() {
        zeroAndDisarm("app close");
        detachVideoSurface();
        DJISDKManager.getInstance().stopConnectionToProduct();
    }
}
