package com.example.androidapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Color;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class MainActivity extends AppCompatActivity implements AdapterView.OnItemSelectedListener {

    private HashMap<String, BLEDevice> mBLEDevicesHashMap;
    private ArrayList<BLEDevice> mBLEDevicesArrayList;
    private int destination;
    private ArrayList<Integer> stationArray = new ArrayList<>(Arrays.asList(0));



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //TODO check permissions and set BLE features. Start scan and add devices to list.

        mBLEDevicesHashMap = new HashMap<>();
        mBLEDevicesArrayList = new ArrayList<>();

        // Set an default list with locations:
        for (int i = 0; i < 3; i++){
            stationArray.add(stationArray.size());
        }

    }

    /**
     *  General methods
     */
    public void returnMenu(View view) {
        setContentView(R.layout.main_menu);
    }

    public void skipScan(View view) {
        setContentView(R.layout.main_menu);
    }


    /**
     *  Methods for mode 1: line following
     */
    public void lineFollowing(View view) {
        setContentView(R.layout.mode_line);

        // Setup for the spinner
        Spinner spinner = (Spinner) findViewById(R.id.stationDestination);
        ArrayAdapter<Integer> spinnerAdapter = new ArrayAdapter<Integer>(MainActivity.this, android.R.layout.simple_spinner_item, stationArray);
        spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(spinnerAdapter);
        spinner.setOnItemSelectedListener(this);
    }

    @Override
    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
        setDestination(view, position);
    }

    @Override
    public void onNothingSelected(AdapterView<?> parent) {
        // TODO: Auto-generated method stub
    }

    public void setDestination(View view, int location) {
        destination = location;
    }

    public void goToDestination(View view) {
        // TODO: add method to give the car the order
        final Button goToStationButton = findViewById(R.id.goToStation);
        goToStationButton.setBackgroundColor(Color.GREEN);
        System.out.println("Car goes to station: " + destination);
    }

    public void stopCar(View view) {
        final Button goToStationButton = findViewById(R.id.goToStation);
        goToStationButton.setBackgroundColor(getResources().getColor(R.color.design_default_color_primary));
        System.out.println("All orders are cancelled!");
    }

    /**
     *  Methods for mode 2: person following
     */
    public void personFollowing(View view) {
        setContentView(R.layout.mode_person);
    }
        //TODO: implement functions and screens for person following

    public void scanPerson(View view) {
        //TODO: let the Pi scan for persons, send a picture/videofeed of the choice to the app, so the user can try a new scan or start the following
    }

    public void setFollowingDistance(View view) {
        // TODO: somehow use the text input to set the distance. (use default if no distance is specified and set limits on the input)
    }

    public void goFollowPerson(View view) {
        final Button goFollowPersonButton = findViewById(R.id.goFollowPerson);
        goFollowPersonButton.setBackgroundColor(Color.GREEN);
    }

    public void stopFollowPerson(View view) {
        final Button goFollowPersonButton = findViewById(R.id.goFollowPerson);
        goFollowPersonButton.setBackgroundColor(getResources().getColor(R.color.design_default_color_primary));
    }

    /**
     *  Methods for mode 3: car following
     */


}