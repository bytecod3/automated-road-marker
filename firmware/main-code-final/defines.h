#ifndef DEFINES_H
#define DEFINES_H

#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)


#define BUZZER A11
#define USER_LED A12


// MOTOR DRIVER PINS
#define ENA 11
#define ENB 12
#define IN1 14
#define IN2 15
#define IN3 16
#define IN4 17

// paint system pins
#define PUMP_ONE_RELAY_PIN        A13
#define PUMP_TWO_RELAY_PIN        A10
#define SOLENOID_VALVE_RELAY_PIN  A14


#endif
