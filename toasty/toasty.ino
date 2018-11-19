/* TOASTY!!! A living toaster.
 *
 */
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

#define LED_PIN 9

#include <Wire.h>
#include <MPU6050_tockn.h>

// MPU6050 is the main chip on the Osepp BAL-01 sensor
  MPU6050 accelgyro(Wire);

// This will be the gyroscope controller
class BalanceController
{
private:

  bool running = false;
  float inX = 0.0;
  float inY = 0.0;
  float inZ = 0.0;
  float accX = 0.0;
  float accY = 0.0;
  float accZ = 0.0;
  float sensitivity = 10.0; // in degrees
  float idleSen = 1.5;
  int idleTimeout = 3; // in seconds
  unsigned long idleTime = 0;

public:
    BalanceController()
    {
      Wire.begin();
    }

    void start()
    {
    if (!running)
    {
      // initialize MPU6050
      accelgyro.begin();
      accelgyro.calcGyroOffsets(true);

      running = true;

      inX = accelgyro.getAngleX();
      inY = accelgyro.getAngleY();
      inZ = accelgyro.getAngleZ();
    }

    }

    // test connection to sensor
    bool isOnline()
    {
        return true;
    }

    void update() {
        accelgyro.update();

        bool idle = true;

        float diffX = abs(accelgyro.getGyroX() - accX);
        float diffY = abs(accelgyro.getGyroY() - accY);
        float diffZ = abs(accelgyro.getGyroZ() - accZ);

        if (diffX > idleSen || diffY > idleSen || diffZ > idleSen) {
            idle = false;
        }

        if (!idle) {
            idleTime = millis();
        }

        accX = accelgyro.getGyroX();
        accY = accelgyro.getGyroY();
        accZ = accelgyro.getGyroZ();
    }

    bool isLevel()
    {
        bool flag = true;

        float diffX = abs(accelgyro.getAngleX() - inX);
        float diffY = abs(accelgyro.getAngleY() - inY);
        float diffZ = abs(accelgyro.getAngleZ() - inZ);

        if (diffX > sensitivity || diffY > sensitivity || diffZ > sensitivity) {
            flag = false;
        }

        return flag;
    }

    bool isIdle() {

        bool flag = false;
        if (millis() - idleTime > idleTimeout * 1000) {
            flag = true;
        }

        return flag;
    }

    bool isUpsideDown()
    {
        // if the gravity becomes reversed from the original orientation
        return (accelgyro.getAccY() < 0.0);
    }
};


/// Motion sensor
class MotionController
{
private:

    //the time we give the sensor to calibrate (10-60 secs according to the datasheet)
    int calibrationTime = 30;

    int pirPin = 8;    //the digital pin connected to the PIR sensor's output
    int asyncDelay = 50;
    int asyncTimer = 0;

    bool motionDetected = false;

public:
    MotionController()
    {

    }

    void start()
    {
        pinMode(pirPin, INPUT);
        digitalWrite(pirPin, LOW);

        //give the sensor some time to calibrate
        Serial.print("calibrating sensor ");
        for(int i = 0; i < calibrationTime; i++){
            Serial.print(".");
            delay(1000);
        }
        Serial.println(" done");
        Serial.println("SENSOR ACTIVE");
        delay(50);

    }

    void update() {

        if (millis() - asyncTimer < asyncDelay) {
            return;
        }

        if(digitalRead(pirPin) == HIGH){
            // Serial.println("Motion");
            motionDetected = true;
        }

        if (digitalRead(pirPin) == LOW) {
            // Serial.println("no motion");
            motionDetected = false;

        }

    }

    bool motion() {
        return motionDetected;
    }

};

class MotorController
{
private:
    unsigned long mTimer = 0;
    int motorPin = 10;

    int purrGap = 2000; // in milliseconds
    int purrLen = 1000; // in milliseconds
    unsigned long delayTimer = 0;

public:
    MotorController()
    {

    }

    void start() {
        pinMode(motorPin, OUTPUT);
    }

    void update(bool iRun)
    {
        if (millis() - delayTimer > 50) {
          if (iRun) {
  //            Serial.println("run");
              // Run purring motor
              if (millis() - mTimer < purrLen) {
  //                Serial.println("purr");
                  digitalWrite(motorPin, HIGH);
              } else if (millis() - mTimer < purrLen + purrGap) {
  //              Serial.println("purr silent");
                  digitalWrite(motorPin, LOW);
              } else {
  //              Serial.println("=========================== RESET =====================");
                  mTimer = millis();
              }

          } else {
  //          Serial.println("stop");
              digitalWrite(motorPin, LOW);
              mTimer = millis();
          }

          delayTimer = millis();
        }
    }
};


#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "sounddata.h"

#define SAMPLE_RATE 8000

int speakerPin = 11; // Can be either 3 or 11, two PWM outputs connected to Timer 2
volatile uint16_t sample;
byte lastSample;
bool playing = false;
int *sounddata_length;
signed char * sounddata_data;

void stopPlayback()
{
    // Disable playback per-sample interrupt.
//    TIMSK1 &= ~_BV(OCIE1A);
//
//    // Disable the per-sample timer completely.
//    TCCR1B &= ~_BV(CS10);
//
//    // Disable the PWM timer.
//    TCCR2B &= ~_BV(CS10);
//
//    digitalWrite(speakerPin, LOW);
//    OCR2A = 0;
    playing = false;
}

void killPlayback()
{
    // Disable playback per-sample interrupt.
   TIMSK1 &= ~_BV(OCIE1A);

   // Disable the per-sample timer completely.
   TCCR1B &= ~_BV(CS10);

   // Disable the PWM timer.
   TCCR2B &= ~_BV(CS10);

   digitalWrite(speakerPin, LOW);
   OCR2A = 0;
    playing = false;
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= *sounddata_length || !playing) {
//        if (sample == sounddata_length + lastSample) {
            stopPlayback();
//        }
//        else {
//            if(speakerPin==11){
//                // Ramp down to zero to reduce the click at the end of playback.
//                OCR2A = sounddata_length + lastSample - sample;
//            } else {
//                OCR2B = sounddata_length + lastSample - sample;
//            }
//        }
        OCR2A = char(127);
    }
    else {
        if(speakerPin==11){
            OCR2A = pgm_read_byte(&sounddata_data[sample]);
        } else {
            OCR2B = pgm_read_byte(&sounddata_data[sample]);
        }
    }

    ++sample;
//    if (sample == 1) {
//      sample = 0;
//    } else {
//      sample = 1;
//    }

}

void startPlayback(const signed char * data, int *d_length)
{
    sounddata_data = data;
    sounddata_length = d_length;

    // Serial.println("sounddata:: ");
//    Serial.println(*data + 0);
    // Serial.println(pgm_read_byte(data[9255]));
    // Serial.println(pgm_read_byte(&sounddata_data));


    playing = true;
    pinMode(speakerPin, OUTPUT);

    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    if(speakerPin==11){
        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
        TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sounddata_data[0]);
    } else {
        // Do non-inverting PWM on pin OC2B (p.155)
        // On the Arduino this is pin 3.
        TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
        TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2B = pgm_read_byte(&sounddata_data[0]);
    }

    // ***********************************************
    // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&sounddata_data[*sounddata_length -1] );
    // Serial.print("lastSample: ");
    // Serial.println(lastSample);
    sample = 0;
    sei();
}



// =========
// MAIN APP
// =========

BalanceController balControl;
MotionController moControl;
MotorController purrMotor;

unsigned long masterTimer = 0;
unsigned long iTimer = 0;
bool jumping = false;
int solenoidPin = 12;

void setup() {

    // initialize serial
    Serial.begin(9600);

    moControl.start();
    // initialize gyroscope/accelerometer
    balControl.start();

    // setup motor for purring
    purrMotor.start();

    // led pin
    pinMode(LED_PIN, OUTPUT);
    pinMode(solenoidPin, OUTPUT);

    // Sound setup
    delay(1000);
    // Serial.println("Register TCCR2A Values");
    // Serial.print("TCCR2A: ");
    // Serial.println(TCCR2A, BIN);
    // Serial.print("_BV(COM2A1): ");
    // Serial.println(_BV(COM2A1), BIN);
    // Serial.print("(TCCR2A | _BV(COM2A1)): ");
    // Serial.println((TCCR2A | _BV(COM2A1)), BIN);
    // Serial.print("~_BV(COM2A0): ");
    // Serial.println(~_BV(COM2A0), BIN);
    // Serial.print("(TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0): ");
    // Serial.println( (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0), BIN);

    masterTimer = millis();

}

void jump() {
    digitalWrite(solenoidPin, HIGH);
    delay(150);
    digitalWrite(solenoidPin, LOW);
    int r = random(75, 300);
    delay(100);
}

void loop() {

    // must update PIR sensor
    moControl.update();
    // must update gyro
    balControl.update();

    if (moControl.motion() && millis() - masterTimer > 3000) {
        digitalWrite(LED_PIN, HIGH);

//        if (balControl.isLevel()) {
          int r = random(6, 13);
          for (int i = 0; i < r; i++) {
              startPlayback( sd_chicken_data, &sd_chicken_length );
              delay(406);
          }
          killPlayback();
  
          r = random(2, 6);
          for (int i = 0; i < r; i++) {
              jump();
  //            delay(406);
          }
          
//        } else {
//
//          int r = random(6, 10);
//          for (int i = 0; i < r; i++) {
//              digitalWrite(10, HIGH);
//              delay(1500);
//              digitalWrite(10, LOW);
//              delay(1000);
//          }
//          
//        }

        masterTimer = millis();
        iTimer = millis();

    } else {
        digitalWrite(LED_PIN, LOW);
//        iTimer = millis() - iTimer;

        if (millis() - iTimer > 10000) {

            startPlayback( sd_meow_data, &sd_meow_length );
            delay(528);
            killPlayback();
            iTimer = millis();
        }
    }

    // if (!jumping && moControl.motion() && !balControl.isLevel() && !balControl.isIdle() ) {
    //     // purr
    //     startPlayback( sd_purr_data, &sd_purr_length );
    //     purrMotor.update(true);
    // } else {
    //     purrMotor.update( false );
    // }

    // if (balControl.isLevel() && balControl.isIdle()) {
    //     if (millis() - masterTimer > 5000) {
    //         digitalWrite(solenoidPin, HIGH);
    //         delay(200);
    //         digitalWrite(solenoidPin, LOW);
    //         delay(200);
    //         digitalWrite(solenoidPin, HIGH);
    //         delay(200);
    //         digitalWrite(solenoidPin, LOW);
    //         delay(200);
    //         digitalWrite(solenoidPin, HIGH);
    //         delay(200);
    //         digitalWrite(solenoidPin, LOW);
    //         masterTimer = millis();
    //     }
    // } else {
    //     masterTimer = millis();
    // }



}

/*

=== BEHAVIOR ===
if (not level and not idle)
    Then purr

if (level and idle)
    do the chicken
    erratic

if (motion)
    turn on lights

if (purring)
    pulse lights

*/
