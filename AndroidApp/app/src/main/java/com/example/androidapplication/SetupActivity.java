package com.example.androidapplication;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.*;
import android.bluetooth.le.*;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;

@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
public class SetupActivity extends AppCompatActivity {
    private BluetoothAdapter bluetoothAdapter;
    private Handler mHandler;

    private static final int ENABLE_BLUETOOTH_REQUEST_CODE = 1;
    private boolean mScanning;
    private static final long SCAN_PERIOD = 10000;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        final BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();
    }
    
}