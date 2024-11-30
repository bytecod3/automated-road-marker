/**
 * Defines functions to control the motors
 *
 */

#include "motor_driver.h"

/**
 * @brief Constructor initialise the motor's pins
 * @param ena   enable A
 * @param enb   enable B
 * @param in1   Direction control 1
 * @param in2   Direction control 2
 * @param in3   Direction control 3
 * @param in4   Direction control 4
 */
Motor::Motor(int ena, int enb, int in1, int in2, int in3, int in4) {
    this->_ena = ena;
    this->_enb = enb;
    this->_in1 = in1;
    this->_in2 = in2;
    this->_in3 = in3;
    this->_in4 = in4;
}

/**
 * initial speed
 */
 int Motor::speed = 150;

/**
 * @brief Initialize motor pin-modes
 */
void Motor::init_motor_pins() {
    pinMode(this->_ena, OUTPUT);
    pinMode(this->_enb, OUTPUT);
    pinMode(this->_in1, OUTPUT);
    pinMode(this->_in2, OUTPUT);
    pinMode(this->_in3, OUTPUT);
    pinMode(this->_in4, OUTPUT);

    // turn off all motors initially
    this->stop();
}

/**
 * @brief get the motor speed
 */
int Motor::get_speed() {
    return speed;
}

/**
 * @brief Start the motors
 */
void Motor::start() {
    analogWrite(this->_ena, this->get_speed());
    analogWrite(this->_enb, this->get_speed());

    // turn on motor A and B
    digitalWrite(this->_in1, HIGH);
    digitalWrite(this->_in2, LOW);
    digitalWrite(this->_in3, HIGH);
    digitalWrite(this->_in4, LOW);
}

/**
 * @brief turn left
 */
void Motor::turn_left() {
    // turn on motor A and B
    digitalWrite(this->_in1, HIGH); // off
    digitalWrite(this->_in2, HIGH); // off
    digitalWrite(this->_in3, HIGH);
    digitalWrite(this->_in4, LOW);
}

/**
 * @brief turn right
 */
void Motor::turn_right() {
    // turn on motor A and B
    digitalWrite(this->_in1, HIGH);
    digitalWrite(this->_in2, LOW);
    digitalWrite(this->_in3, HIGH); // off
    digitalWrite(this->_in4, HIGH); // off
}

/**
 * @brief move forward
 */
void Motor::move_forward() {
    digitalWrite(this->_in1, LOW);
    digitalWrite(this->_in2, HIGH);
    digitalWrite(this->_in3, LOW);
    digitalWrite(this->_in4, HIGH);
}

/**
 * @brief reverse
 */
void Motor::reverse() {
    // turn on motor A and B
    digitalWrite(this->_in1, HIGH);
    digitalWrite(this->_in2, LOW);
    digitalWrite(this->_in3, HIGH);
    digitalWrite(this->_in4, LOW);
}

/**
 * @brief stop the motor
 */
void Motor::stop() {
    digitalWrite(this->_in1, LOW);
    digitalWrite(this->_in2, LOW);
    digitalWrite(this->_in3, LOW);
    digitalWrite(this->_in4, LOW);
}

/*
 * @brief Decelerate motor
 */
void Motor::decelerate() {
    int s = this->get_speed();

    // decelerate from speed s to 0
    for(int i = s; i > 0; i-- ) {
        analogWrite(this->_ena, i);
        analogWrite(this->_enb, i);
        delay(DECEL_DELAY);
    }

    // turn off the motors after deceleration
    this->stop();

}
