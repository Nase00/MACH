void flipUp() {
  if (flippedUpState == true) {
    Serial.println("Already flipped up");
    return;
  }

  Serial.println("Flipping up");

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(SERVO_SHIELD_PIN, 0, pulselen);
    delay(1);
  }

  flippedUpState = true;

  delay(100);
}

void flipDown() {
  if (flippedUpState != true) {
    Serial.println("Already flipped down");
    return;
  }

  Serial.println("Flipping down");

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(SERVO_SHIELD_PIN, 0, pulselen);
    delay(1);
  }

  flippedUpState = false;

  delay(100);
}

void toggle() {
  if (flippedUpState) {
    flipDown();
  } else {
    flipUp();
  }

  flippedUpState = !flippedUpState;
}