/*
 * init_functions.h
 *
 * Created: 2/24/2025 3:26:14 PM
 *  Author: PC
 */ 

// initialization functions - ULTRASONIC, MPU, GPS, FLOW METER, 
// MOTORS, UART, ADC, PWM, TIMERS, RESET SYSTEM,
// Solenoid ports
#ifndef INIT_FUNCTIONS_H_
#define INIT_FUNCTIONS_H_

void uart_init(void);
void uart_transmit(unsigned char c);
void uart_debug(char* s);
void uart_receive();

void buzzer_init();
void led_user_init();

void adc_init();
void timer1_init();

void motor_system_init();
void ultrasonic_init();
void mpu_init();
void gps_init();
void flowmeter_init();
void spraygun_init();
void pump_init();


#endif /* INIT_FUNCTIONS_H_ */