/*
  Pinouts on Adafruit's page are wrong.
  Reference the Playground source:
  https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/70ff084b16e3a77aac7294190ab51d0af6ad7c5f/libraries/Bluefruit52Lib/examples/Peripheral/bluefruit_playground/bluefruit_playground.ino
*/

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <InternalFileSystem.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_LIS3MDL.h> // Magnetometer
#include <Arduino_LSM6DS3.h> // Accelerometer and gyroscope
// #include <Adafruit_Sensor_Calibration.h>

#define NEOPIXEL_NUM 3
#define SERVO_SHIELD_PIN 0
#define NUM_PIXELS 0
#define BRIGHTNESS 10

#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  800 // this is the 'maximum' pulse length count (out of 4096)

// Sensor calibration
// #define FILE_SENSOR_CALIB "sensor_calib.json"
// Adafruit_Sensor_Calibration_SDFat cal;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool flippedUpState = false;

#include "servo.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);   // for nrf52840 with native usb

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  /* Servo motor */
  pwm.begin();
  pwm.setPWMFreq(60);

  strip.begin();
  strip.setPixelColor(0, strip.Color(255, 5, 5, BRIGHTNESS));
  strip.show();

  flipUp();
}

void loop() {
  delay(10);
  const int position = pwm.getPWM(SERVO_SHIELD_PIN);

  if (position != 0) {
    Serial.println(position);
  }

  float ax, ay, az, gx, gy, gz;
  bool gySteady = true; // gy > -0.07 && gy < 0.07;

  // TODO implement "psuedo gesture" to control visor
  // IF head tilting down + accelerating forward THEN move visor down
  // IF head tilting up + accelerating backward THEN move visor up

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    Serial.println("Accelerometer values: ");
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.println(az);
  } else {
    Serial.println("Error witH IMU accelerometer.");

    ax = 0.5; // Temp workaround for IMU flipping out
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    Serial.println("Gyroscope values: ");
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.println(gz);
  } else {
    Serial.println("Error witH IMU gyroscope.");
  }

 if (ax > 0.5 && gySteady) {
   strip.setPixelColor(0, strip.Color(0, 15, 0, BRIGHTNESS));
   strip.show();

   flipDown();

   delay(10);
 }

 if (ax < 0.5 && gySteady) {
   strip.setPixelColor(0, strip.Color(15, 0, 0, BRIGHTNESS));
   strip.show();

   flipUp();

   delay(10);
 }
}
