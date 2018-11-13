// This will be the gyroscope controller

#include <Arduino.h>

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


class GyroAccel
{
private:

	// MPU6050 is the main chip on the Osepp BAL-01 sensor
	MPU6050 accelgyro;
	bool running = false;

	int16_t ax, ay, az;	// Accelerometer values
	int16_t gx, gy, gz;	// Gyroscope values

public:
	GyroAccel()
	{
		// join I@C bus (I2Cdev library doesn't do this automatically)
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		    Wire.begin();
		#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		    Fastwire::setup(400, true);
		#endif

	}

	void start()
	{
		if (!running)
		{
			// initialize MPU6050
			Serial.println("Initializing I2C device...");
			accelgyro.initialize();

			running = true;
		}

	}

	// test connection to sensor
	bool isOnline()
	{
		bool connection = false;

		if (running)
		{
			// verify connection
			Serial.println("Testing device connection...");
			connection = accelgyro.testConnection();
			Serial.println(connection ? "MPU6050 connection successful" : "MPU6050 connection failed");

      if (connection) {
          Serial.println("Updating internal sensor offsets...");
          // -76 -2359 1688 0 0 0
          Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
          Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
          Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
          Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
          Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
          Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
          Serial.print("\n");
      }
		}

		return connection;
	}

	// Set current values as level.
	void orient()
	{
		// TODO
	}

	bool isLevel()
	{
		// read raw accel/gyro measurements from device
	    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	    Serial.print("a/g:\t");
	    Serial.print(float(ax)/16384.0); Serial.print("\t");
	    Serial.print(float(ay)/16384.0); Serial.print("\t");
	    Serial.print(float(az)/16384.0); Serial.print("\t");
	    Serial.print(float(gx)/250); Serial.print("\t");
	    Serial.print(float(gy)/250); Serial.print("\t");
	    Serial.println(float(gz)/250);

	    return true;
	}

	bool isUpsideDown()
	{
		// if the gravity becomes reversed from the original orientation
		return (az > 5000);
	}
};
