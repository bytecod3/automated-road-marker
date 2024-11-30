#include <Arduino.h>
#include "defs.h"
#include "motor_driver.h"

Motor mtr_1(EN1_A, EN1_B, IN1_1, IN1_2, IN1_3, IN1_4);
Motor mtr_2(EN2_A, EN2_B, IN2_1, IN2_2, IN2_3, IN2_4);

void setup() {
    Serial.begin(BAUD_RATE);

    // initialize motors
    mtr_1.init_motor_pins();
    mtr_2.init_motor_pins();

    // init ultrasonic sensors



}

void loop() {

}
