# automated-road-marker
Automated road marker project


## Battery sizing and component voltages 
| Component    | Voltage rating (V) | Current rating
| -------- | ------- | ------- |
| Arduino MEGA 2560  | 7-12    | -
| TT 130 accelerator/decelerator motor  | 3-6     | 170mA(at 4.5V)
| HC-SR04 ultrasonic sensor    | 5    | 4mA
|  12V Solenoid valve G1/2 DN15 BSPP Pressure Type   | 12    | 416mA
| 12V DC 385 high temperature water pump    | 12    | 400mA |
| YF-S401 Water Flow Sensor   | 5    | 15 mA (at 5V)

Recomended Battery = 12V >2000mAh

## firmware dependencies
The following libraries need to intalled in Arduino 
1. LiquidCrystal I2C library by Frank de Brabander
2. I2Cdev

## ultrasonic designations 

US1 -> Front ultrasonic
US2 -> Right ultrasonic
US3 -> Left ultrasonic
US4 -> Resorvoir level ultrasonic

### Ultrasonic Conclusion
Ultrasonic avidance is not necesssary in the real-world. Why are you painting a road with obstacles??

## Flow meter interrupt pin change
The schematic show flow meter signal connected to pin 10. From tests, pin 10 cannot support
interrupts. Please use pin 2 for the flow meter signal.( See code on flow meter Pin -> in defines. h. If you wish 
to change the pin to pin number 3).

Only pin 2 and 3 can be used for the flow meter.

# LCD library installation
If you cannot find the Arduino-LiquidCrystal-I2C on Arduino Library manager, copy the Arduino-LiquidCrystal-I2C folder into your Arduino installation folder.
You should be able to compile the code with no errors.


