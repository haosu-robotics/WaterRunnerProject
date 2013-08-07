WaterRunnerCode
===============

Arduino and computer interfacing code for a water runner robot

libraries
---------

The libraries folder contains commonly used classes:

* Filter.h - Filter Interface Class
    * RunningAvg.h - Class implementing the Filter interface to smooth measurements using a running average
* PID.h - PID control loop 
* QueueArray.h - Library implementing a generic, dynamic queue (array version). Used by Running Average Algorithm.
* TypeDefs.h - Contains Type Definitions for angular speed and acceleration. 
* encoder.h - Has implementation of calculating and returning angular speed given angular position measurement. Implementation of angular position measurement is encoder-counter-dependent, and therefore left to derived classes.
    *  HCTL2032SC.h - Derived from the encoder class, this class implements a method for obtaining the relative angular position of a quadrature encoder connected to an [Avago HCTL-2032-SC][1] encoder counter. 

trigint
-------

[trigint][2] is an integer-based trigonometry library for ANSI C. The trigonometry functions in the standard C library use floating point data types (double or float), which may be too slow or unavailable in an embedded environment. This library uses only integer parameters, return values, and calculations. It has been tested on the iPhone with Xcode and on an Atmel AVR microcontroller with avr-gcc, but it is written in ANSI C99, and should work anywhere with a modern C compiler.

Because the standard C library uses radians as the angle unit, they must be represented with a floating point types. To avoid using floating point for angles, a new angle unit is used, trigint\_angle\_t. This splits the circle into 16,384 angle units, instead of 2\*PI radians or 360 degrees. Thus, the angle parameters can be safetly stored in an unsigned 16-bit data type.

encTest
-------

Very early Arduino code for interfacing with the encoder counter.

encPlot
-------

An initial attempt to plot encoder data in real time using the matplotlib python library. Matplotlib proved to be too slow to perform real time plotting. This code interfacing with the encoder counter found in encTest.ino served as the basis for the encoder.h and HCTL2032SC.h library functions.

encPlot2
--------

A second attempt to plot encoder data in real time, now using Qt, which can plot more quickly than Matplotlib. mtrPlot.py attempted to use an infinite loop to poll for data from the Arduino, and therefore ran poorly. testPlot.py uses a Qt timer to obtain data at a set frequency, and thus ran much more smoothly. The Arduino code makes use of the libraries.

imuPlot 
-------

Interfaces the Arduino and [YEI 3Space Embedded IMU][3] and plots the orientation of the imu on the computer using matplotlib. Experiments with sending data from the Arduino to the computer in binary format along with a checksum.

mtrTest
-------

Small program to test the [L298][4] motor driver.

[1]: http://www.avagotech.com/docs/AV02-0096EN                           "Avago HCTL-2032-SC documentation"
[2]: http://www.dribin.org/dave/trigint/                                 "trigint library website"
[3]: http://www.yeitechnology.com/productdisplay/3-space-embedded-0      "YEI 3 Space Embedded Website"
[4]: https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf      "L298 Motor Driver documentation"
