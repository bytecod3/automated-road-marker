#include "defines.h"



/**
 * 
 * states 
 IDLE - on but not painting, buzz twice,blink at 1 second
 PAINTING - actively checking straight line and running paint system, blink at 200ms
 length between IDLE and START_PAINT ~10 seconds 
 measure distance painted?
 * FLOW
 * wait for 10 seconds after setup 
 * Read the IMU 
 * Activate the paint system while checking the straightness of the line 
 * Apply PID to motor while painting 
 * 
 * For demo: paint for 30 seconds then stop 
 * deactivate paint system 
 * state == DONE_PAINTING
 * buzz 3 times at 400ms
 * blink at 1Hz
 */

// function prototypes 

long int now = 0;
long int last_millis = 0;
long int current_millis = 0;
uint16_t interval = 300;

boolean buzz_state = LOW;
boolean user_led_state = LOW;

void initialize_hardware_pins();
void blink_non_blocking(uint16_t);
void buzz_non_blocking(uint16_t);
void onboard_blink_non_blocking();

/**
 * Sensor read functions 
 */
void read_gps_cordinates();
void read_accelerometer_values();
void get_flow_rate();

/**
 * Relay control functions 
 */
void activate_solenoid_relay();
void activate_spray_gun_relay();
void activate_pump_one_relay();
void activate_pump_two;
void deactivate_solenoid_relay();
void deactivate_spray_gun_relay();
void deactivate_pump_one_relay();
void deactivate_pump_two;

/**
 * Motor control functions 
 */
 void robot_start();
 void robot_stop();
 void robot_forward();
 void robot_turn_left();
 void robot_turn_right();
 void robot_reverse();

 /**
  * Other functions
  */
void check_straight_line();
void compute_RTK();

void blink_non_blocking(uint16_t i) {
  current_millis = millis();
  if(current_millis - last_millis >= i) {
    last_millis = current_millis;
    user_led_state = !user_led_state;
    digitalWrite(USER_LED, user_led_state);
  }
}

//void buzz_non_blocking(uint16_t j) {
//  current_millis = millis();
//  if(current_millis - la;st_millis >= j) {
//    last_millis = current_millis;
//    buzz_state = !buzz_state;
//    digitalWrite(BUZZER, buzz_state);
//  }
//}

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(USER_LED, OUTPUT);
}


void loop() {

  blink_non_blocking(interval);
  //buzz_non_blocking(interval);
  
}
