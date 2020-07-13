# Peltier_driver_PID

This project is a piece of final year paper in my Engineering degree concluded in June 2019.

This is a simple Peltier driver using a MOSFET with two relays in order to invert the current flow and control the heat/cool side. An LM35 sensor was used to get the current temperature on one side of the Peltier plate. With the input temperature, the output was controled using the PWM and the MOSFET. The power supply is a 5A 12VCC.

![alt text](https://github.com/rcharaba/Peltier_driver_PID/blob/master/peltier_driver_circuit.PNG)  
 
 
Also to properly tune the PID, a GUI was implemented in the Arduino code in order to send/receive the PID data:  
![alt text](https://github.com/rcharaba/Peltier_driver_PID/blob/master/pid_control.PNG)   

source: https://playground.arduino.cc/Code/PIDLibrary/
