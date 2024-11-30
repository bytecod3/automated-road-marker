/**
 * Header file for the motor driver
 */

#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

class Motor {
private:
    uint8_t _ena, _enb, _in1, _in2, _in3, _in4;
    static int speed;

public:
    Motor(int, int, int, int, int, int); // constructor
    void init_motor_pins();
    void start();
    void stop();
    void turn_left();
    void turn_right();
    void reverse();
    void move_forward();
    void set_start_speed();
    static int get_speed();

};


#endif
