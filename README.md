# Arduino BGA Rework Station PID Controller
Code for Arduino based BGA Rework Station PID controller using a solid state relay and a k-type thermocouple. 

This controller has several functions:
1. It reads the temperature from a k-type thermocouple amplified by a MAX6675 amplifier
1. Compares the input temperature with a desired temperature and uses a PID controller library to calculate the on time of a heating element to correct it.
1. Uses a solid state relay to turn on/off the actual heating element
1. Communicate with a PC through serial console to receive bga profiles (NOT FINISHED)

The code is based on the work by Norcal Reballer from the http://bgamods.com forum and his code helped me a lot to understand how PID controllers should work.

This is not a finished product. This software is a part of a bigger plan for building a BGA rework station, consisting of the station itself, controlled by an Arduino running this software and a Java desktop application for command and control. 

There is still work to be done (and probably bugs), but maybe someone will find use for it.
