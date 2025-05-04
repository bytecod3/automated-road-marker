#include "defines.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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
void init_motor_pins();
void robot_start();
void robot_stop();
void robot_forward();
void robot_turn_left();
void robot_turn_right();
void robot_reverse();
void robot_set_speed(uint8_t);
void robot_motor_test();
void check_straight_line();
void compute_RTK();

void initialize_hardware_pins();
void blink_non_blocking(uint16_t);
void buzz_non_blocking(uint16_t);
void onboard_blink_non_blocking();
void mpu_callibrate();
void calc_yaw();
void corr_speed();

Adafruit_MPU6050 mpu;

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

// motor control variables
unsigned long t1 = 0;
unsigned long t2 = 0;

// timing variables
long int now = 0;
long int last_millis = 0;
long int current_millis = 0;
uint16_t interval = 300;

boolean buzz_state = LOW;
boolean user_led_state = LOW;

// motor variables 
uint8_t motor_speed = 200;

// motor driver functions 
void init_motor_pins() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // turn off motor - initial state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/**
 * set motor speed
 */
void robot_set_speed(uint8_t speed) {
  analogWrite(ENA, speed); 
  analogWrite(ENB, speed);
}

/**
 * stop robot
 */
void robot_stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/**
 * move robot forward
 */
void robot_forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/**
 * reverse robot
 */
void robot_reverse() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

/**
 * test the robot motors
 */
void robot_motor_test() {
  robot_forward();
  delay(3000);
  robot_reverse();
}

 /**
  * Other functions
  */

/**
 * non blocking buzz
 */
void buzz_non_blocking(uint16_t j) {
  current_millis = millis();
  if(current_millis - la;st_millis >= j) {
    last_millis = current_millis;
    buzz_state = !buzz_state;
    digitalWrite(BUZZER, buzz_state);
  }
}

/**
 * non blocking blink
 */
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
  debug("gyro mean: "); debugln(gyro_z0);
  
}

void mpu_init() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu_callibrate();
}

int16_t get_z_rotation() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z;
}

void calc_yaw() {
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  gyro_z = get_z_rotation();

  //float angular_z = (gyro_z - gyro_z0) / 131.0 * dt;
  float angular_z = (gyro_z - gyro_z0) * dt;
  if(fabs(angular_z) < 0.05) {
    angular_z = 0.00;
  }

  gyro_angle_z += angular_z; // return the z axis rotation integral
  yaw = -gyro_angle_z;
  
}

void corr_speed() {
  calc_yaw();
}

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);
  pinMode(USER_LED, OUTPUT);
  mpu_init();
  init_motor_pins();
  robot_set_speed(100);
  robot_motor_test();
}


void loop() {
  //debugln(get_z_rotation());
  calc_yaw();
  debug("Yaw: "); debugln(yaw);
}
