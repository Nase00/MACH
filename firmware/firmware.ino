#include "FastLED.h"
#include <Wire.h>
#include <SPI.h>

#include <Servo.h>

#define NEOPIXEL_PIN 12
#define NEOPIXEL_NUM 3
#define SERVO_PIN 9
#define NUM_PIXELS 0
#define DISENGAGED_POS 0
#define ENGAGED_POS 180

Servo servo;
CRGB leds[NEOPIXEL_NUM];

void setup() {
    Serial.begin(9600);

    // servo.attach(SERVO_PIN);
    // servo.write(DISENGAGED_POS);

    FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, NEOPIXEL_NUM);
}

void loop() {
    leds[0] = CRGB::White; FastLED.show(); delay(30); 
    leds[2] = CRGB::White; FastLED.show(); delay(30); 

    leds[0] = CRGB::Black; FastLED.show(); delay(30);
    leds[2] = CRGB::Black; FastLED.show(); delay(30);
}