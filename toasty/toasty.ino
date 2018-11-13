// TOASTY!!! A living toaster.

#include "gyro.cpp"

/*
    === PIN LAYOUT ===
    13 - LEDs (possibly to transistor switch)
    11 - Sound output and Timer 2 Override (Pin 3 should no longer be used)
    12 - Solenoid 1 : Main toaster movement
    10 - Motor : Weighted for purring
     8 - PIR sensor reading : For detecting humans.
     7 - Solenoid 2 : Possibly to drive additional movement.
    A5 - Gyro/Accelerometer SCL output (Using wire to interpret)
    A4 - Gyro/Accelerometer SDA output (Using wire to interpret)

*/

#define LED_PIN 13

GyroAccel gyro;

void setup() {

    // initialize serial
    Serial.begin(9600);

    // initialize gyroscope/accelerometer
    gyro.start();
    gyro.isOnline();
    // Serial.print("Gyro Status:")

    // led pin
    pinMode(LED_PIN, OUTPUT);

}

void loop() {

    gyro.isLevel();

}
