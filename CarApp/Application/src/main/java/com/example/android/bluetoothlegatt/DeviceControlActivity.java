/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.bluetoothlegatt;

import android.app.Activity;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.IBinder;
import android.support.annotation.RequiresApi;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ExpandableListView;
import android.widget.SimpleExpandableListAdapter;
import android.widget.Spinner;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/**
 * For a given BLE device, this Activity provides the user interface to connect, display data,
 * and display GATT services and characteristics supported by the device.  The Activity
 * communicates with {@code BluetoothLeService}, which in turn interacts with the
 * Bluetooth LE API.
 */
public class DeviceControlActivity extends Activity implements AdapterView.OnItemSelectedListener {
    private final static String TAG = DeviceControlActivity.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";

    private TextView mConnectionState;
    private TextView mDataField;
    private String mDeviceName;
    private String mDeviceAddress;
    private ExpandableListView mGattServicesList;
    private BluetoothLeService mBluetoothLeService;
    private ArrayList<ArrayList<BluetoothGattCharacteristic>> mGattCharacteristics =
            new ArrayList<ArrayList<BluetoothGattCharacteristic>>();
    private boolean mConnected = false;
    private BluetoothGattCharacteristic mNotifyCharacteristic;

    private final String LIST_NAME = "NAME";
    private final String LIST_UUID = "UUID";

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
//            mBluetoothLeService.connect(mDeviceAddress);

        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                updateConnectionState(R.string.connected);
                invalidateOptionsMenu();
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                updateConnectionState(R.string.disconnected);
                invalidateOptionsMenu();
                clearUI();
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                // Show all the supported services and characteristics on the user interface.
                displayGattServices(mBluetoothLeService.getSupportedGattServices());

                // Here the connection is fully established and the user is taken to the custom main menu that will be used to control the GoPiGo.
                goToMenu();

            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                displayData(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
            }
        }
    };

    // If a given GATT characteristic is selected, check for supported features.  This sample
    // demonstrates 'Read' and 'Notify' features.  See
    // http://d.android.com/reference/android/bluetooth/BluetoothGatt.html for the complete
    // list of supported characteristic features.
    private final ExpandableListView.OnChildClickListener servicesListClickListner =
            new ExpandableListView.OnChildClickListener() {
                @Override
                public boolean onChildClick(ExpandableListView parent, View v, int groupPosition,
                                            int childPosition, long id) {
                    if (mGattCharacteristics != null) {
                        final BluetoothGattCharacteristic characteristic =
                                mGattCharacteristics.get(groupPosition).get(childPosition);
                        final int charaProp = characteristic.getProperties();
                        if ((charaProp | BluetoothGattCharacteristic.PROPERTY_READ) > 0) {
                            // If there is an active notification on a characteristic, clear
                            // it first so it doesn't update the data field on the user interface.
                            if (mNotifyCharacteristic != null) {
                                mBluetoothLeService.setCharacteristicNotification(
                                        mNotifyCharacteristic, false);
                                mNotifyCharacteristic = null;
                            }
                            mBluetoothLeService.readCharacteristic(characteristic);
                        }
                        if ((charaProp | BluetoothGattCharacteristic.PROPERTY_NOTIFY) > 0) {
                            mNotifyCharacteristic = characteristic;
                            mBluetoothLeService.setCharacteristicNotification(
                                    characteristic, true);
                        }
                        return true;
                    }
                    return false;
                }
    };

    private void clearUI() {
        mGattServicesList.setAdapter((SimpleExpandableListAdapter) null);
        mDataField.setText(R.string.no_data);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.gatt_services_characteristics);

        final Intent intent = getIntent();
        mDeviceName = intent.getStringExtra(EXTRAS_DEVICE_NAME);
        mDeviceAddress = intent.getStringExtra(EXTRAS_DEVICE_ADDRESS);

        // Sets up UI references.
        ((TextView) findViewById(R.id.device_address)).setText(mDeviceAddress);
        mGattServicesList = (ExpandableListView) findViewById(R.id.gatt_services_list);
        mGattServicesList.setOnChildClickListener(servicesListClickListner);
        mConnectionState = (TextView) findViewById(R.id.connection_state);
        mDataField = (TextView) findViewById(R.id.data_value);

        getActionBar().setTitle(mDeviceName);
        getActionBar().setDisplayHomeAsUpEnabled(true);
        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);

        // Set an default list with locations:
        for (int i = 0; i < 3; i++) {
            stationArray.add(stationArray.size());
        }
        // Set an default list with colors:
        colorArray.add("Pink");
        colorArray.add("Yellow");
        colorArray.add("Blue");
        colorArray.add("Purple");

    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    protected void onResume() {
        super.onResume();
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(mDeviceAddress);
            Log.d(TAG, "Connect request result=" + result);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        unregisterReceiver(mGattUpdateReceiver);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.gatt_services, menu);
        if (mConnected) {
            menu.findItem(R.id.menu_connect).setVisible(false);
            menu.findItem(R.id.menu_disconnect).setVisible(true);
        } else {
            menu.findItem(R.id.menu_connect).setVisible(true);
            menu.findItem(R.id.menu_disconnect).setVisible(false);
        }
        return true;
    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch(item.getItemId()) {
            case R.id.menu_connect:
                mBluetoothLeService.connect(mDeviceAddress);
                return true;
            case R.id.menu_disconnect:
                mBluetoothLeService.disconnect();
                return true;
            case android.R.id.home:
                onBackPressed();
                return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private void updateConnectionState(final int resourceId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mConnectionState.setText(resourceId);
            }
        });
    }

    private void displayData(String data) {
        if (data != null) {
            mDataField.setText(data);
        }
    }

    // Demonstrates how to iterate through the supported GATT Services/Characteristics.
    // In this sample, we populate the data structure that is bound to the ExpandableListView
    // on the UI.
    private void displayGattServices(List<BluetoothGattService> gattServices) {
        if (gattServices == null) return;
        String uuid = null;
        String unknownServiceString = getResources().getString(R.string.unknown_service);
        String unknownCharaString = getResources().getString(R.string.unknown_characteristic);
        ArrayList<HashMap<String, String>> gattServiceData = new ArrayList<HashMap<String, String>>();
        ArrayList<ArrayList<HashMap<String, String>>> gattCharacteristicData
                = new ArrayList<ArrayList<HashMap<String, String>>>();
        mGattCharacteristics = new ArrayList<ArrayList<BluetoothGattCharacteristic>>();

        // Loops through available GATT Services.
        for (BluetoothGattService gattService : gattServices) {
            HashMap<String, String> currentServiceData = new HashMap<String, String>();
            uuid = gattService.getUuid().toString();
            currentServiceData.put(
                    LIST_NAME, SampleGattAttributes.lookup(uuid, unknownServiceString));
            currentServiceData.put(LIST_UUID, uuid);
            gattServiceData.add(currentServiceData);

            ArrayList<HashMap<String, String>> gattCharacteristicGroupData =
                    new ArrayList<HashMap<String, String>>();
            List<BluetoothGattCharacteristic> gattCharacteristics =
                    gattService.getCharacteristics();
            ArrayList<BluetoothGattCharacteristic> charas =
                    new ArrayList<BluetoothGattCharacteristic>();

            // Loops through available Characteristics.
            for (BluetoothGattCharacteristic gattCharacteristic : gattCharacteristics) {
                charas.add(gattCharacteristic);
                HashMap<String, String> currentCharaData = new HashMap<String, String>();
                uuid = gattCharacteristic.getUuid().toString();
                currentCharaData.put(
                        LIST_NAME, SampleGattAttributes.lookup(uuid, unknownCharaString));
                currentCharaData.put(LIST_UUID, uuid);
                gattCharacteristicGroupData.add(currentCharaData);
            }
            mGattCharacteristics.add(charas);
            gattCharacteristicData.add(gattCharacteristicGroupData);
        }

        SimpleExpandableListAdapter gattServiceAdapter = new SimpleExpandableListAdapter(
                this,
                gattServiceData,
                android.R.layout.simple_expandable_list_item_2,
                new String[] {LIST_NAME, LIST_UUID},
                new int[] { android.R.id.text1, android.R.id.text2 },
                gattCharacteristicData,
                android.R.layout.simple_expandable_list_item_2,
                new String[] {LIST_NAME, LIST_UUID},
                new int[] { android.R.id.text1, android.R.id.text2 }
        );
        mGattServicesList.setAdapter(gattServiceAdapter);
    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }


    /**
     * Down below is the new code to control the GoPiGo
     */
    // New list with used variables
    private int destination;
    private int color;
    private ArrayList<Integer> stationArray = new ArrayList<>(Arrays.asList(0));
    private ArrayList<ArrayList<Integer>> stationConnections = new ArrayList<ArrayList<Integer>>();
    private ArrayList<String> colorArray = new ArrayList<String>();


    /**
     *  General methods
     */
    // When a device is connected, show the main_menu:
    public void goToMenu() {
        setContentView(R.layout.main_menu);
    }

    // When switching between modes, go back to the main_menu. This will also send the cancel command to stop current orders.
    public void returnMenu(View view) {
        writeValue(0, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        setContentView(R.layout.main_menu);
    }

    // The function to write to the custom service. This can be used for both the first and second characteristic. This
    public void writeValue(int value, String uuid, String characterstic) {
        if(mBluetoothLeService != null) {
            mBluetoothLeService.writeCustomCharacteristic(value, uuid, characterstic);
        }
    }


    /**
     *  Methods for the settings page
     */
    // A menu for the settings was supposed to be implemented, but this was removed due to time constraints.
    public void goToSettings(View view) {
        setContentView(R.layout.settings);
//        TextView connectionInfo = (TextView) findViewById(R.id.connectionInfo);
//        connectionInfo.setText("Connection info:\nDevice name: " + mDeviceName + "\nAddress: " + mDeviceAddress);
    }

    // This method was supposed to be used to change the intersection and station layout for the line following mode.
    public void changeStations(View view) {
        // TODO: add method to change the station layout
        // use addStation & removeStation
    }

    // This method was supposed to be used to change the connected device, however, this function was not necessary because the BLE example already had this option.
    public void searchDevices(View view) {
        // TODO: add method to change the connected device
        //What connections/settings should be terminated/changed and how to get back to DeviceScanActivity?
    }

    /**
     *  Methods for mode 1: line following
     */
    // This function shows the user the line following menu and sets up the dropdown menu.
    public void lineFollowing(View view) {
        setContentView(R.layout.mode_line);

        // Setup for the spinner
        Spinner spinner = (Spinner) findViewById(R.id.stationDestination);
        ArrayAdapter<Integer> spinnerAdapter = new ArrayAdapter<Integer>(DeviceControlActivity.this, android.R.layout.simple_spinner_item, stationArray);
        spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(spinnerAdapter);
        spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                destination = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                destination = 0;
            }
        });
    }

    // This function sends the GoPiGo a message that specifies the line following mode and destination.
    public void goToDestination(View view) {
        final Button goToStationButton = findViewById(R.id.goToStation);

        writeValue(1, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        writeValue(destination, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "db5e19fd-0800-4f27-bbf2-6e91ec9c37d2");

        goToStationButton.setBackgroundColor(Color.GREEN);
        System.out.println("Car goes to station: " + destination);
    }

    // This sends the cancel command to the GoPiGO.
    public void stopCar(View view) {
        writeValue(0, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        final Button goToStationButton = findViewById(R.id.goToStation);
        goToStationButton.setBackgroundColor(Color.RED);
        System.out.println("All orders are cancelled!");
    }

    // The code below were some initial tries to create a matrix/map with all the intersections/stations and their (colored) connections to other intersections/stations.
    // Not enough time was left to complete this functionality.

//    public void addStation(int red, int blue, int green, int yellow) {
//        int i = 0;
//        while (stationArray.contains(i)) {
//            i++;
//        }
//        stationArray.add(i);
//        ArrayList<Integer> connections = new ArrayList<>(Arrays.asList(red, blue, green, yellow));
//        stationConnections.add(i, connections);
//
//        // TODO: create connection from other side as well
//    }
//
//    public void removeStation(View view, int station) {
//        stationArray.remove(station);
//        stationConnections.remove(station);
//
//        for (int i = station; i < stationArray.size(); i++) {
//            // TODO: remove connection from other side as well
//        }
//    }

    /**
     *  Methods for mode 2: person following
     */

    // This function shows the user the person/color following menu and sets up the dropdown menu.
    public void personFollowing(View view) {
        setContentView(R.layout.mode_person);

        // Setup for the spinner
        Spinner spinnerColor = (Spinner) findViewById(R.id.spinnerColor);
        ArrayAdapter<String> spinnerAdapterColor = new ArrayAdapter<String>(DeviceControlActivity.this, android.R.layout.simple_spinner_item, colorArray);
        spinnerAdapterColor.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerColor.setAdapter(spinnerAdapterColor);
        spinnerColor.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                color = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                color = 0;
            }
        });
    }

    // A removed function in which we wanted to give the user a screenshot from the GoPiGo with the tracked object, so the user had the
    // option to scan for a person/color again if the scan did not track the right person/color.
    // However, this was later on removed, because the GoPiGo sends no screenshot and the user can then just use the go button to scan again.
    public void scanPerson(View view) {
        writeValue(2, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        writeValue(color, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "db5e19fd-0800-4f27-bbf2-6e91ec9c37d2");
    }

    // This function sends the GoPiGo a message that specifies the person/color following mode and color.
    public void goFollowPerson(View view) {
        writeValue(2, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        writeValue(color, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "db5e19fd-0800-4f27-bbf2-6e91ec9c37d2");

        final Button goFollowPersonButton = findViewById(R.id.goFollowPerson);
        goFollowPersonButton.setBackgroundColor(Color.GREEN);
        System.out.println("Car follows color: " + color);

    }

    // This sends the cancel command to the GoPiGO.
    public void stopFollowPerson(View view) {
        writeValue(0, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");

        final Button goFollowPersonButton = findViewById(R.id.goFollowPerson);
        goFollowPersonButton.setBackgroundColor(Color.RED);
    }

    /**
     *  Methods for mode 3: car following
     *
     *  This is basically a copy of the person follow part, because the car follow mode and person follow mode were supposed to be two different modes.
     *  However, eventually both the modes scan for colors, so the modes were combined and renamed to color following in the app.
     */
    public void carFollowing(View view) {
        setContentView(R.layout.mode_car);
    }

    public void scanCar(View view) {
        writeValue(3, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        writeValue(color, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "db5e19fd-0800-4f27-bbf2-6e91ec9c37d2");
    }

    public void goFollowCar(View view) {
        writeValue(4, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        writeValue(color, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "db5e19fd-0800-4f27-bbf2-6e91ec9c37d2");

        final Button goFollowPersonButton = findViewById(R.id.goFollowCar);
        goFollowPersonButton.setBackgroundColor(Color.GREEN);
    }

    public void stopFollowCar(View view) {
        writeValue(0, "a7ead335-61e5-4d23-a4ce-bd0a956d5952", "1b4a5e34-54bf-4196-abe2-5dc7b590a415");

        final Button goFollowPersonButton = findViewById(R.id.goFollowCar);
        goFollowPersonButton.setBackgroundColor(Color.RED);
    }

    @Override
    public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {

    }

    @Override
    public void onNothingSelected(AdapterView<?> adapterView) {

    }
}
