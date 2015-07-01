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


To use 4x20 characters LCD Display, the LiquidCrystal_I2C and WIRE libraries must also be included.


```c++
#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DRAPRS.h>
```

Change your callsign into unsigned char array

```c++
// track char array
unsigned  char     track[72]={'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,0x60,              //avant APTT4 7 octets (0-6)
                           'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,('0' + 12) << 1,     //F4GOH-11 7 octets (7-13)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'1'<<1,' '<<1,('0' + 1) << 1,      //WIDE1-1 7 octets (14-20)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'2'<<1,' '<<1,('0' + 1) << 1 | 1 , //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
                           0x03,0xf0,                                                     //ctrl, pid 2 octets (28-29)
                           '/','1','5','0','4','5','2','h',      //heure 8 (30-37)
                           '4','8','5','1','.','2','0','N','/','0','0','2','2','0','.','9','2','E',      //lat, long 18 octets (38-55)
                           '>','7','3',' ','A','n','t','h','o','n','y',' ',' ',' ',' ',' '};               //commentaire 15 car
```
change GPS speed to 4800 bauds if you need

```c++
gps.begin(9600);
```
change frequency into dra_serial.println...

```c++
// Format˖AT+DMOSETGROUP=BWˈTX_FˈRX_FˈTx_subaudioˈSQˈRx_subaudio
// AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000
// SQ 0 à 8

byte configDra818()
{
char ack[3];
int n;
digitalWrite(PwDw, HIGH);
delay(2000);  //wait 2000ms to be awake
dra_serial.println(F("AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000"));
ack[2]=0;
while (ack[2]!=0xa)
{
if (dra_serial.available()>0) {
                               ack[0]=ack[1];
                               ack[1]=ack[2];
                               ack[2]=dra_serial.read();
                               }
}
digitalWrite(PwDw, LOW);
return (ack[0]==0x30) ? 1 : 0;
}
```
