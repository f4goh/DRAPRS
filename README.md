#ARDUINO APRS and DRA818 module library #
F4GOH Anthony f4goh@orange.fr <br>

June 2015

Use this library freely with Arduino 1.0.6

## Installation ##
To use the DRAPRS library:  
- Go to https://github.com/f4goh/DRAPRS, click the [Download ZIP](https://github.com/f4goh/DRAPRS/archive/master.zip) button and save the ZIP file to a convenient location on your PC.
- Uncompress the downloaded file.  This will result in a folder containing all the files for the library, that has a name that includes the branch name, usually DRAPS-master.
- Rename the folder to  DRAPRS.
- Copy the renamed folder to the Arduino sketchbook\libraries folder.

- you must add Arduino TimerOne library : <br>
  Go to http://playground.arduino.cc/Code/Timer1

## Usage notes ##


To use the 4x20 LCD char, the LiquidCrystal_I2C and WIRE libraries must also be included.


```c++
#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DRAPRS.h>
```
