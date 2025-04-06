#include "defines.h"
#include <Wire.h>
#include "I2Cdev.h"
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
void activate_pump_two();
void deactivate_solenoid_relay();
void deactivate_spray_gun_relay();
void deactivate_pump_one_relay();
void deactivate_pump_two();

/**
 * Motor control functions 
 */
void robot_start();
void robot_stop();
void robot_forward();
void robot_turn_left();
void robot_turn_right();
void robot_reverse();
void check_straight_line();
void compute_RTK();

void initialize_hardware_pins();
void blink_non_blocking(uint16_t);
void buzz_non_blocking(uint16_t);
void onboard_blink_non_blocking();
void mpu_callibrate();
void calc_yaw();



#define MPU_ADDRESS 0x68
#define MPU6050_RA_GYRO_ZOUT_H 0x47 //z values register adress
#define MPU_ACCEL_RANGE 16
#define GYRO_RANGE 250
float gyro_z; // raw z register values 

unsigned long last_time = 0;
float dt; // differential time 
long gyro_z0 = 0; // gyro osffset = mean value 
float yaw;
float yaw_old = 0;
float gyro_angle_z = 0; // angle variable

void mpu_init() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6b); // PWR_MNGMT REGISTER 
  Wire.write(0); // wake up
  Wire.endTransmission(true);

  delay(2000);
  debugln("********** straight line *****************");

  mpu_callibrate();
}


// motor control variables
unsigned long t1 = 0;
unsigned long t2 = 0;


long int now = 0;
long int last_millis = 0;
long int current_millis = 0;
uint16_t interval = 300;

boolean buzz_state = LOW;
boolean user_led_state = LOW;

 /**
  * Other functions
  */

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

  //blink_non_blocking(interval);
  //buzz_non_blocking(interval);
  t1 = millis();
  t2 = t1;

//  while(abs(t2-t1) < 2500) {
//    forward(my_speed);
//    t2 = millis();
//  }
  
}

void blink_non_blocking(uint16_t i) {
  current_millis = millis();
  if(current_millis - last_millis >= i) {
    last_millis = current_millis;
    user_led_state = !user_led_state;
    digitalWrite(USER_LED, user_led_state);
  }
}

void mpu_callibrate() {
  unsigned short times = 100; // sampling times
  for(int i=0; i < times; i++) {
    gyro_z = get_z_rotation();
    gyro_z0 += gyro_z; // sum all measured values of gyro_z
  }

  gyro_z0 /= times; // compute the mean
}

int16_t get_z_rotation() {
  uint8_t buffer[14];
  I2Cdev::readBytes(MPU_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
  return ((int16_t) buffer[0] << 8 | buffer[1]);
}

void calc_yaw() {
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  gyro_z = get_z_rotation();

  float angular_z = (gyro_z - gyro_z0) / 131.0 * dt;
  if(fabs(angular_z) < 0.05) {
    angular_z = 0.00;
  }

  gyro_angle_z += angular_z; // return the z axis rotation integral
  yaw = -gyro_angle_z;
  
}
