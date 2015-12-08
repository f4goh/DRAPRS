/*
    AFSK APRS with ID51

    Copyright (C) 2015 Anthony LE CREN <f4goh@orange.fr>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <Wire.h>

#include <DRAPRS.h>


#define PTT     6    // PTT pin. This is active low.

#define bfPin 5     //afsk output must be PD5 
#define ledPin 13     //led for GPS fix (not used here)


#define txPin 9      //tx pin into RX ID51 connection  (you must add rs232 shift level converter between arduino and ID51)
#define rxPin 8      //rx pin into TX ID51 connection  (you must add rs232 shift level converter between arduino and ID51)

#define TX_interval 10    //interval between TX seconds


SoftwareSerial id51(rxPin, txPin);  // RX, TX for ID51


// track char array
unsigned  char     track[86]={'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,0x60,              //avant APTT4 7 octets (0-6)
                           'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,('0' + 12) << 1,     //F4GOH-11 7 octets (7-13)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'1'<<1,' '<<1,('0' + 1) << 1,      //WIDE1-1 7 octets (14-20)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'2'<<1,' '<<1,('0' + 1) << 1 | 1 , //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
                           0x03,0xf0,                                                     //ctrl, pid 2 octets (28-29)
                           '/','1','5','0','4','5','2','h',      //heure 8 (30-37)
                           '4','8','5','1','.','2','0','N','/','0','0','2','2','0','.','9','2','E',      //lat, long 18 octets (38-55)
                           '>','7','3',' ','A','n','t','h','o','n','y',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};               //commentaire 20 car octets (56-
                           

typedef struct  {   //34 bytes
  byte notused[6];
  byte latitude[5];
  byte longitude[6];
  byte altitude[4];
  byte cap[2];
  byte speed[3];
  byte date[4];
  byte hour[3];
  byte end;
  }GpsId51Struct;
  GpsId51Struct gpsData;    //declare la structure GPS
  
typedef struct  {   //19 bytes
  byte notused[6];
  byte call[12];
  byte end;
  }CallId51Struct;
  CallId51Struct CallData;    //declare la structure CALL

typedef struct  {   //27 bytes
  byte notused[6];
  byte msg[20];
  byte end;
  }MsgId51Struct;
  MsgId51Struct MsgData;    //declare la structure  MSG
    

byte TX_enable=0;

void setup(){
        Serial.begin(9600);
        id51.begin(9600);
        Beacon.begin(bfPin,ledPin);
        pinMode(bfPin, OUTPUT);
        pinMode(PTT, OUTPUT);
        digitalWrite(PTT, LOW);
        Timer1.initialize(76);    //µs  fe=13200 hz so TE=76µs
}



void loop()
{
ID51_cmd_GPS();
if (TX_enable==1) {
      ID51_cmd_CALL();
      ID51_cmd_MSG();
      txing();
      TX_enable=0;
}
}

/*
 // for debug redirect id51 info to serial
void loop()
{
char car;
if (Serial.available()>0){
                            car=Serial.read();
                            if (car=='$') {
                                            txing();
                                          }
                            else
                            if (car=='*') {
                                                    ID51_cmd_CALL();
                                                    ID51_cmd_MSG();
                                                    ID51_cmd_GPS(); 
                            }
                            else        
                            id51.write(car);  
                            }
  
  if (id51.available()>0){
                            Serial.write(id51.read());  
                            }
}

*/

// id51 GPS command
void ID51_cmd_GPS()
{
delay(100);
byte Cmd[]={0xFE,0xFE,0x86,0xE0,0x23,0x00,0xFD};
for (int n=0;n<sizeof(Cmd);n++) id51.write(Cmd[n]);
byte car=0;
int ptr=0;
while (car!=0xFD){
   if (id51.available()>0){
          car=id51.read();
          *((char*)&gpsData + ptr) = car;
          ptr++;
   }
}
if (ptr>8) {
   print_data(gpsData.latitude,5,0);
   print_data(gpsData.longitude,6,0);
   print_data(gpsData.hour,3,0);

   track[31]=(gpsData.hour[0]>>4)+0x30;
   track[32]=(gpsData.hour[0]&0xf)+0x30;

   track[33]=(gpsData.hour[1]>>4)+0x30;
   track[34]=(gpsData.hour[1]&0xf)+0x30;
      
   track[35]=(gpsData.hour[2]>>4)+0x30;
   track[36]=(gpsData.hour[2]&0xf)+0x30;

if (gpsData.hour[2]%TX_interval==0) TX_enable=1;

   track[38]=(gpsData.latitude[0]>>4)+0x30;
   track[39]=(gpsData.latitude[0]&0xf)+0x30;

   track[40]=(gpsData.latitude[1]>>4)+0x30;
   track[41]=(gpsData.latitude[1]&0xf)+0x30;

   track[43]=(gpsData.latitude[2]>>4)+0x30;
   track[44]=(gpsData.latitude[2]&0xf)+0x30;
 
   if (gpsData.latitude[4]==0x01) track[45]='N'; else track[45]='S';
  
 
   track[47]=(gpsData.longitude[0]&0xf)+0x30;

   track[48]=(gpsData.longitude[1]>>4)+0x30;
   track[49]=(gpsData.longitude[1]&0xf)+0x30;

   track[50]=(gpsData.longitude[2]>>4)+0x30;
   track[51]=(gpsData.longitude[2]&0xf)+0x30;

   track[53]=(gpsData.longitude[3]>>4)+0x30;
   track[54]=(gpsData.longitude[3]&0xf)+0x30;

   if (gpsData.longitude[5]==0x01) track[55]='E'; else track[55]='W';

//for (int n=0;n<sizeof(track);n++) {Serial.print(track[n],HEX);Serial.print(",");}    //print char array track (for  info)
//for (int n=30;n<sizeof(track);n++) {Serial.print((char) track[n]);}    //print char array track (for  info)   
}
else Serial.println("GPS not enabled");
}

// id51 CALL command
void ID51_cmd_CALL()
{
  delay(10);
byte Cmd[]={0xFE,0xFE,0x86,0xE0,0x1F,0x00,0xFD};
for (int n=0;n<sizeof(Cmd);n++) id51.write(Cmd[n]);
byte car=0;
int ptr=0;
while (car!=0xFD){
   if (id51.available()>0){
          car=id51.read();
          *((char*)&CallData + ptr) = car;
          ptr++;
   }
}
if (ptr>8) {   
   print_data(CallData.call,6,1);
   
   for (int m=0;m<6;m++)
   {
   track[0+m]=(CallData.call[m]<<1);
   track[7+m]=(CallData.call[m]<<1);
   }
}
else Serial.println("no Call");
}

// id51 MSG command
void ID51_cmd_MSG()
{
delay(10);
byte Cmd[]={0xFE,0xFE,0x86,0xE0,0x1F,0x02,0xFD};
for (int n=0;n<sizeof(Cmd);n++) id51.write(Cmd[n]);
byte car=0;
int ptr=0;
while (car!=0xFD){
   if (id51.available()>0){
          car=id51.read();
          *((char*)&MsgData + ptr) = car;
          ptr++;
   }
}
if (ptr>8) {
   print_data(MsgData.msg,20,1);
   for (int m=0;m<20;m++)
   {
   track[57+m]=MsgData.msg[m];   
   }
}
else Serial.println("no Msg");
}


// print buffer array
void print_data(byte * array, int nb, byte mode) {
   for (int n =0; n < nb; n ++) {    
      if (mode==0) Serial.print(array[n],HEX);
      else Serial.print((char)array[n]);
   }
   Serial.println();
}



// pwm configuration and txing APRS
void txing()
{
byte save_TIMSK0;
byte save_PCICR;
delay(2000);  //wait 2000ms to be awake
digitalWrite(PTT, HIGH);            //ptt on
delay(500);                         //delay before sending data
TCCR0B = TCCR0B & 0b11111000 | 1;    //switch to 62500 HZ PWM frequency
save_TIMSK0=TIMSK0;                  //save Timer 0 register
TIMSK0 =0;                           //disable Timer 0
save_PCICR=PCICR;                    //save external pin interrupt register
PCICR = 0;                           //disable external pin interrupt
Timer1.attachInterrupt(sinus_irq);   //warp interrupt in library
Beacon.sendpacket(track,sizeof(track));  //send packet
digitalWrite(PTT, LOW);              //PTT off
Timer1.detachInterrupt();            //disable timer1 interrupt
analogWrite(bfPin,0);                //PWM at 0
TCCR0B = TCCR0B & 0b11111000 | 3;    //register return to normal
TIMSK0 =save_TIMSK0;
PCICR = save_PCICR;
}


void sinus_irq()    //warp timer1 irq into DRAPRS lib
{
Beacon.sinus();
}



/*
to know how much interrupt are used (to disabled it during TX)

    Serial.print("EIMSK External Interrupt Mask Register ");
    Serial.println(EIMSK,HEX);

    Serial.print("PCICR Pin Change Interrupt Control Register ");
    Serial.println(PCICR,HEX);

    Serial.print("PCMSK2 Pin Change Mask Register 2 ");
    Serial.println(PCMSK2,HEX);
    
    Serial.print("PCMSK1 Pin Change Mask Register 1 ");
    Serial.println(PCMSK1,HEX);

    Serial.print("PCMSK0 Pin Change Mask Register 0 ");
    Serial.println(PCMSK0,HEX);

    Serial.print("TIMSK0  Timer/Counter Interrupt Mask Register ");
    Serial.println(TIMSK0,HEX);

    Serial.print("TIMSK1 Timer/Counter1 Interrupt Mask Register ");
    Serial.println(TIMSK1,HEX);

    Serial.print("TIMSK2 Timer/Counter1 Interrupt Mask Register ");
    Serial.println(TIMSK2,HEX);

    Serial.print("SPCR SPI Control Register ");
    Serial.println(SPCR,HEX);

    Serial.print("UCSRnB USART Control and Status Register n B ");
    Serial.println(UCSR0B,HEX);

    Serial.print("TWCR TWI Control Register ");
    Serial.println(TWCR,HEX);

    Serial.print("ACSR Analog Comparator Control and Status Register ");
    Serial.println(ACSR,HEX);

    Serial.print("ADCSRA ADC Control and Status Register A ");
    Serial.println(ADCSRA,HEX);
*/




/*
command FE FE 86 E0 23 00 FD
Answer if if GPS not enable : FE FE E0 86 23 00 FF FD
Answer if if GPS enable :

                  |   lat      |    long          |    alt    |cap  |speed   | date      | hour|   
                  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27  
FE FE E0 86 23 00 47 53 42 00 01 00 00 16 61 00 01 00 07 20 00 FF FF FF FF FF 20 15 12 05 10 16 33 FD
FE FE E0 86 23 00 47 53 42 00 01 00 00 16 61 00 01 00 07 20 00 FF FF FF FF FF 20 15 12 05 12 46 05 FD
FE FE E0 86 23 00 47 53 42 00 00 00 00 16 61 00 00 00 07 20 00 FF FF FF FF FF 20 15 12 05 12 47 09 FD
FE FE E0 86 23 00 47 53 42 00 01 00 00 16 61 00 01 00 07 20 00 FF FF FF FF FF 20 15 12 05 12 18 09 FD
0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
01 : E N
00 : W S

command call
FE FE 86 E0 1F 00 FD
Answer
FE FE E0 86 1F 00 46 34 47 4F 48 20 20 20 49 44 35 31 FD
                  F  4  G  O   H          I  D  5  1



command msg
FE FE 86 E0 1F 02 FD
Answer
FE FE E0 86 1F 02 37 33 27 73 20 64 65 20 46 34 47 4F 48 20 20 20 20 20 20 20 FD
                  7  3   '  s     d  e     f  4  g  o  h 
 */
