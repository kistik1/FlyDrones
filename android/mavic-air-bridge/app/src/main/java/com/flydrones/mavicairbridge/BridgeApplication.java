package com.flydrones.mavicairbridge;

import android.app.Application;
import android.content.Context;

import androidx.multidex.MultiDex;

/** Installs DJI's protected MSDK v4 runtime before any SDK API class is loaded. */
public final class BridgeApplication extends Application {
    @Override
    protected void attachBaseContext(Context base) {
        super.attachBaseContext(base);
        MultiDex.install(this);
        com.cySdkyc.clx.Helper.install(this);
    }
}
