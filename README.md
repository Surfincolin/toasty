# toasty
A Dancing Toaster!


Using a solenoid, motor, LEDs, Gyroscope/Accelerometer, speaker, and motion sensor, the hope is to have a dancing toaster. All will be controlled by an Arduino and powered by 9v batteries.

### Dev Notes
- Install the MPU6050 and I2CDev libraries https://github.com/jrowberg/i2cdevlib.git
- Install this helper lib https://github.com/Tockn/MPU6050_tockn
- Install Rogue Code Tone library https://code.google.com/archive/p/rogue-code/downloads
- Change line 26 in Tone.cpp from `#include <wiring.h>` to `#include <Arduino.h>`

- https://www.youtube.com/watch?v=tUapZ_JdHLE
- http://afrotechmods.com/tutorials/2017/01/17/how-to-make-a-simple-1-watt-audio-amplifier-lm386-based/

#### wav2c instructions
- `git submodule update --init --recursive`
- Got to the [readme](https://github.com/olleolleolle/wav2c) for instructions on building.
- Original samples can be found [here](https://www.dropbox.com/sh/3hajf4vuec6rkwt/AADmnkM_vvla61nuUuk2IhuMa?dl=0)
- `wav2c/wav2c robot-sad-01.wav robot-sad.h sounddata`
