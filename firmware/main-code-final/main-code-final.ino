#include "defines.h"
#include <Wire.h>
#include "I2Cdev.h"

#define MOTOR_ENABLE 1 // must be set to 1 for the motors to run
#define MPU_ENABLE 1  // must be set to 1 for the MPU to be read
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

void initialize_hardware_pins();
void blink_non_blocking(uint16_t);
void buzz_non_blocking(uint16_t);
void onboard_blink_non_blocking();


#if MOTOR_ENABLE 
/**
 * Motor control functions 
 */
  void init_motor_pins();
  void robot_start();
  void robot_stop();
  void robot_forward(uint8_t speed);
  void robot_turn_left(uint8_t speed);
  void robot_turn_right(uint8_t speed);
  void robot_reverse();
  void robot_set_speed(uint8_t);
  void drive_straight(uint8_t speed);
  void robot_motor_test();
  void correct_speed(int my_speed);
  void compute_RTK();
#endif

#if MPU_ENABLE 
  //===================== MPU FUNCTIONS AND VARIABLES ======================
  #define MPU_ADDRESS 0x68
  #define MPU6050_RA_GYRO_ZOUT_H 0x47 //z values register adress
  #define MPU_ACCEL_RANGE 16
  #define GYRO_RANGE 250
  
  float gyro_z; // raw z register values 
  unsigned long last_time = 0;
  float dt; // differential time 
  long gyro_z0 = 221; // gyro osffset = mean value - get this value from the callibration process
  float yaw;
  float yaw_old = 0;
  float gyro_angle_z = 0; // angle variable

  void mpu_init();
  void mpu_callibrate();
  int16_t get_z_rotation();
  void calc_yaw();
  void check_straight_line();
  void read_accelerometer_values();


  // initialize mpu
  void mpu_init() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0); // wake up
    Wire.endTransmission(true);
    delay(1000);
  }

  // callibrate the MPU
  void mpu_callibrate() {
    // get callibratio samples
    debugln("Getting callibration samples...");
    unsigned short times = 100; // sampling times
    for(int i=0; i < times; i++) {
      gyro_z = get_z_rotation();
      gyro_z0 += gyro_z; // sum all measured values of gyro_z
    }

    debugln("Acquired samples...");
  
    gyro_z0 /= times; // compute the mean - to get the offset of the gyroscope for compensation
    debug("Z-axis Mean: "); debugln(gyro_z0);
  }

  
  // read z axis rotation
  int16_t get_z_rotation() {
    uint8_t buffer[14];
    I2Cdev::readBytes(MPU_ADDRESS, MPU6050_RA_GYRO_ZOUT_H,2, buffer);
    return ( ((uint16_t)buffer[0]) << 8) | buffer[1];
    
  }
  
  void calc_yaw() {
    unsigned long current_time = millis(); // current time (ms)
    dt = (current_time - last_time) / 1000.0; // differential time (seconds)
    last_time = current_time; // last sampling time (ms)
  
    gyro_z = get_z_rotation();
  
    float angular_z = (gyro_z - gyro_z0) / 131.0 * dt;
    if(fabs(angular_z) < 0.05) {
      angular_z = 0.00;
    }
  
    gyro_angle_z += angular_z; // return the z axis absolute rotation integral
    yaw = -gyro_angle_z;

    // debug
    //debug("GyroZ Value: "); debug(gyro_z); debug("       GyroAngle: "); debugln(gyro_angle_z);    
  }
  
  //======================= END OF MPU FUCNTIONS AND VARIABLES ==============

#endif

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

#if MOTOR_ENABLE // must be set to 1 to run motors
  
  // motor variables 
  uint8_t high_speed = 55;
  uint8_t low_speed = 20;
  uint8_t my_speed = 45;
  uint8_t correct_speed_right;
  uint8_t correct_speed_left;
  
  // motor driver functions 
  void init_motor_pins() {
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
  
    // turn off motor - initial state
    //digitalWrite(IN1, LOW);
    //digitalWrite(IN2, LOW);
    //digitalWrite(IN3, LOW);
    //digitalWrite(IN4, LOW);
  }
  
  /**
   * set motor speed
   */
  void robot_set_speed(uint8_t speed) {
    analogWrite(ENA, speed); 
    analogWrite(ENB, speed);
  }

  /**
   * Turn Left
   */
   void robot_left(uint8_t speed) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, LOW); // right motor drive forward
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, speed); 
    digitalWrite(IN3, HIGH); // reverse left motor, the robot will turn left
    digitalWrite(IN4, LOW); 
   }

   /**
    * Turn Right
    */
  void robot_right(uint8_t speed) {
    analogWrite(ENB, speed);
    digitalWrite(IN3, LOW); // left motor drive forward
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, speed); 
    digitalWrite(IN1, HIGH); // reverse right motor, the robot will turn right
    digitalWrite(IN2, LOW); 
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
  void robot_forward(uint8_t speed) {

    drive_straight(speed);

    analogWrite(ENA, correct_speed_right); // right motor
    analogWrite(ENB, correct_speed_left); // left motor
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  /**
   * reverse robot
   */
  void robot_reverse() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  /**
   * test the robot motors
   */
  void robot_motor_test() {
    robot_forward(100);
    delay(3000);
    robot_reverse();
    delay(3000);
    robot_stop();
  }

  void correct_speed(int my_speed){
    calc_yaw();
    int kp = 10; // Proportional constant 
    correct_speed_right = my_speed + (yaw - yaw_old) * kp; // maintain speed by speeding up right motor
    
    if(correct_speed_right > high_speed) {
      correct_speed_right = high_speed; // high_speed is the maximum speed of the robot
    } else if(correct_speed_right < low_speed) {
      correct_speed_right = low_speed; // low_speed is the minimum speed of the car
    }

    // if direction changes, correct speed  = my_speed + (yaw_old - yaw) * kp;
    correct_speed_left = my_speed - (yaw - yaw_old) * kp;
    if(correct_speed_left > high_speed){
      correct_speed_left = high_speed;
    } else if(correct_speed_left < low_speed) {
      correct_speed_left = low_speed;
    }
    
  }

  void drive_straight(uint8_t speed) {
    static unsigned long onTime;
    // to compute corr_speed, the yaw data is acquired at startup, and updated every 10 ms
    if(millis() - onTime > 10) {
      correct_speed(speed);
      onTime = millis();
    } 
  }

#endif

 /**
  * Other functions
  */

/**
 * non blocking buzz
 */
void buzz_non_blocking(uint16_t j) {
  current_millis = millis();
  if(current_millis - last_millis >= j) {
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

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);
  pinMode(USER_LED, OUTPUT);

  #if MPU_ENABLE
    mpu_init();
    mpu_callibrate();
  #endif

  #if MOTOR_ENABLE
    // initialize motor hardware
    init_motor_pins();
    //robot_set_speed(70);
    //robot_motor_test();
  #endif
}


void loop() {
  //=========== GO FOWARD ================
  t1 = millis();
  t2 = t1;
//  
//  robot_forward(my_speed); // go forward
//  t2 = millis();
//  
//  // debug speeds
//  debug("YAW: "); debug(yaw); debug(" Right-speed: "); debug(correct_speed_right); debug(" Left-speed: "); debugln(correct_speed_left);
//  yaw_old = yaw;

  //=========== GO REVERSE ================

  //calc_yaw();
  //debugln(yaw);

  // test left and right procedures 
  robot_left(55);
  delay(4000);
  robot_right(55);
  delay(4000);
  
  blink_non_blocking(300);
  
}
