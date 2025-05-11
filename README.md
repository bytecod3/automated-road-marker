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
The following libraries need to installed in Arduino 
1. LiquidCrystal I2C library by Frank de Brabander
2. I2Cdev

## Following a straight line 
The robot follows a heading that is referenced to where it starts. To do this, 
1. place the robot on the surface you want to paint
2. press the reset button WHILE THE ROBOT IS STATIONARY! There should be no movements or shaking. 
3. The robot should follow a straight heading.
4. To stop the robot, press the ON/OFF button, or disconenct the battery 

5. If using LCD, the screen should show the flow rate.

## Paint system 
The paint sequence is as follows:
1. Once the robot is turned on, it paints at intervals of 5 seconds( this can be changed in the 
code. See code!). This paint sequence produces a dotted line 
2. Pump 1, pump 2 and solenoid are turned on at the same time.
3. Flow rate is measured and displayed. 
4. Process repeats.
5. You can increase the length of the dotted line by increasing the paint interval. (Again, see code 
comments for this)

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


