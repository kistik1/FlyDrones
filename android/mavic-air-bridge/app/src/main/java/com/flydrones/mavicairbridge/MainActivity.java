package com.flydrones.mavicairbridge;

import android.Manifest;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.view.TextureView;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public final class MainActivity extends AppCompatActivity implements TextureView.SurfaceTextureListener {
    private static final int PERMISSION_REQUEST = 1001;
    private static final String[] REQUIRED_PERMISSIONS = {
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.READ_PHONE_STATE
    };

    private TextureView preview;
    private TextView djiStatus;
    private TextView bridgeStatus;
    private Button armButton;
    private DjiMobileSdkTransport transport;
    private BridgeServer bridge;
    private boolean sdkRegistered;

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        buildUi();
        transport = new DjiMobileSdkTransport(this, preview);
        armButton.setOnClickListener(view -> toggleArm());
        requestPermissionsOrStart();
    }

    private void buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(28, 36, 28, 28);
        root.setBackgroundColor(Color.rgb(5, 7, 11));

        TextView title = text("FlyDrones · Mavic Air bridge", 22, Color.rgb(57, 255, 136));
        root.addView(title);
        root.addView(text("U11X aircraft · S01A remote · Mobile SDK v4", 13, Color.rgb(138, 152, 168)));
        djiStatus = text("DJI SDK not started", 15, Color.WHITE);
        bridgeStatus = text("Bridge stopped", 14, Color.rgb(76, 201, 240));
        root.addView(djiStatus);
        root.addView(bridgeStatus);

        preview = new TextureView(this);
        preview.setSurfaceTextureListener(this);
        root.addView(preview, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));

        armButton = new Button(this);
        armButton.setText("ARM VIRTUAL STICK LOCALLY");
        armButton.setEnabled(false);
        root.addView(armButton, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));
        root.addView(text("Arming is local to this phone. Wi-Fi loss or 300 ms without a valid command sends zero and disables Virtual Stick. Take off and land with the physical remote.", 12, Color.rgb(255, 176, 32)));
        setContentView(root);
    }

    private TextView text(String value, int size, int color) {
        TextView view = new TextView(this);
        view.setText(value);
        view.setTextSize(size);
        view.setTextColor(color);
        view.setPadding(0, 8, 0, 8);
        return view;
    }

    private void requestPermissionsOrStart() {
        List<String> missing = new ArrayList<>();
        for (String permission : REQUIRED_PERMISSIONS) {
            if (ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED) missing.add(permission);
        }
        if (missing.isEmpty()) startServices();
        else ActivityCompat.requestPermissions(this, missing.toArray(new String[0]), PERMISSION_REQUEST);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        if (requestCode != PERMISSION_REQUEST) return;
        for (int result : results) {
            if (result != PackageManager.PERMISSION_GRANTED) {
                djiStatus.setText("Required DJI SDK permission denied");
                return;
            }
        }
        startServices();
    }

    private void startServices() {
        transport.start((success, message) -> runOnUiThread(() -> {
            sdkRegistered = success;
            djiStatus.setText(message);
            armButton.setEnabled(success);
        }));
        try {
            bridge = new BridgeServer(transport, message -> runOnUiThread(() -> {
                bridgeStatus.setText(message);
                if (message.startsWith("DISARMED")) {
                    djiStatus.setText("Virtual Stick disarmed");
                    armButton.setText("ARM VIRTUAL STICK LOCALLY");
                    armButton.setEnabled(sdkRegistered);
                }
            }));
            bridge.start();
        } catch (IOException error) {
            bridgeStatus.setText("Bridge failed: " + error.getMessage());
        }
    }

    private void toggleArm() {
        boolean requested = !transport.telemetry().armed;
        armButton.setEnabled(false);
        transport.setArmed(requested, (success, message) -> runOnUiThread(() -> {
            djiStatus.setText(message);
            armButton.setText(transport.telemetry().armed ? "DISARM VIRTUAL STICK" : "ARM VIRTUAL STICK LOCALLY");
            armButton.setEnabled(true);
        }));
    }

    @Override public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surface, int width, int height) { transport.attachVideoSurface(surface, width, height); }
    @Override public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surface, int width, int height) { }
    @Override public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surface) { transport.detachVideoSurface(); return true; }
    @Override public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surface) { }

    @Override
    protected void onDestroy() {
        if (bridge != null) bridge.close();
        if (transport != null) transport.close();
        super.onDestroy();
    }
}
