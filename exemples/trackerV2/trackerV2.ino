/*
    AFSK APRS with DRA818

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
#include <LiquidCrystal_I2C.h>

#include <DRAPRS.h>


#define PTT     A2    // PTT pin. This is active low.
#define PwDw      A3  // Power Down pin. This need to start low, then be set high before programming.


#define DRA_RXD 7   // The Arduino IO line that is connected to the DRA818's TXD pin. 3.3V only
                    // We *receive* TTL 3.3V Serial from the DRA818 on this pin.

#define DRA_TXD 6   // The Arduino IO line connected to the DRA818's RXD pin.
                    // We *send* TTL serial to the DRA818 on this pin.

#define bfPin 5     //afsk output must be PD5 
#define ledPin 13     //led for GPS fix


#define txPin 9      //tx pin into RX GPS connection
#define rxPin 8      //rx pin into TX GPS connection

SoftwareSerial dra_serial(DRA_RXD, DRA_TXD);    //for DRA818

SoftwareSerial gps(rxPin, txPin);  // RX, TX for GPS

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    //4 lines *20 columns lcd char
LiquidCrystal_I2C lcd(0x27, 20, 4);    //4 lines *20 columns lcd char


// track char array
unsigned  char     track[72]={'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,0x60,              //avant APTT4 7 octets (0-6)
                           'F'<<1,'4'<<1,'G'<<1,'O'<<1,'H'<<1,' '<<1,('0' + 12) << 1,     //F4GOH-11 7 octets (7-13)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'1'<<1,' '<<1,('0' + 1) << 1,      //WIDE1-1 7 octets (14-20)
                           'W'<<1,'I'<<1,'D'<<1,'E'<<1,'2'<<1,' '<<1,('0' + 1) << 1 | 1 , //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
                           0x03,0xf0,                                                     //ctrl, pid 2 octets (28-29)
                           '/','1','5','0','4','5','2','h',      //heure 8 (30-37)
                           '4','8','5','1','.','2','0','N','/','0','0','2','2','0','.','9','2','E',      //lat, long 18 octets (38-55)
                           '>','7','3',' ','A','n','t','h','o','n','y',' ',' ',' ',' ',' '};               //commentaire 15 car octets (56-71)
                           
char txString[50];
int ptrChaine;


void setup() {
  Serial.begin(57600);
  dra_serial.begin(9600);
  gps.begin(4800);
  Beacon.begin(bfPin, ledPin, 1200, 2200, 350);   //analog pin, led pin, freq1, freq2, shift freq

  lcd.begin();  //4 lines *20 columns lcd char

  lcd.setBacklight(HIGH);    
  lcd.clear();
  lcd.print(F("  APRS TRACKER v1.0"));    //intro
  lcd.setCursor(0, 2);
  lcd.print(F("     F4GOH 2015"));
  delay(4000);
  lcd.clear();

  pinMode(PwDw, OUTPUT);        //config dra818 ctrl lines
  pinMode(bfPin, OUTPUT);
  digitalWrite(PwDw, LOW);

  pinMode(PTT, OUTPUT);
  digitalWrite(PTT, LOW);

  Timer1.initialize(76);    //µs  fe=13200 hz so TE=76µs 13157.9 mesured


  Beacon.GPGGA.pperiod =10;   //interval between two tx
  Beacon.GPGGA.debug = true;   //allow debug print char
  Beacon.GPGGA.dumpNmea = false;   //allow print nmea sentence

    //uncomment if you want to config dra818 frequency     
  //    if (configDra818()==1) Serial.println("freq update"); else Serial.println("freq update error");
}


void loop() {      //don't add any long delay in loop because GPS data recognition will be crash

  if (gps.available()) Beacon.gpsnmea(gps.read());   //if  char into software serial decode nmea sentence
  if (Beacon.GPGGA.sync ==1) // test if  is time to txing
  {
    Beacon.GPGGA.sync =0;      //txing sequence, so sync done
    /*
     Serial.println( Beacon.GPGGA.hour);          //print some GPS info
     Serial.print( Beacon.GPGGA.Latitude);
     Serial.println( Beacon.GPGGA.NS);
     Serial.print( Beacon.GPGGA.Longitude);
     Serial.println( Beacon.GPGGA.EO);
     Serial.println( Beacon.GPGGA.fix);
     Serial.println( Beacon.GPGGA.altitude);        //altitude meter in char array
     Serial.println( Beacon.GPGGA.altidudeMeters);  //altitude meter in long
     Serial.println( Beacon.GPGGA.altidudeFeet);    //altitude feet in long
     Serial.println( Beacon.GPGGA.feet);            //altitude feet in char array
     */
    lcd.setCursor(0, 0);                      //Print GPS info to lcd
    lcd.print(Beacon.GPGGA.hour);            
    lcd.setCursor(18, 0);
    lcd.print(F("TX"));
    lcd.setCursor(0, 1);
    lcd.print(Beacon.GPGGA.Latitude);            
    lcd.print( Beacon.GPGGA.NS);            
    lcd.setCursor(0, 2);                
    lcd.print(Beacon.GPGGA.Longitude);            
    lcd.print(Beacon.GPGGA.EO);            
    lcd.setCursor(0, 3);              
    lcd.print(Beacon.GPGGA.altitude);            

    memcpy(track +31, Beacon.GPGGA.hour, 6);      //prepare APRS char array track to send
    memcpy(track +38, Beacon.GPGGA.Latitude, 7);     //beware index number from char array
    track[45] = Beacon.GPGGA.NS;
    memcpy(track +47, Beacon.GPGGA.Longitude, 8);
    track[55] = Beacon.GPGGA.EO;


    ptrChaine=0;                                 //prepare RTTY char array track to send                              
    addstring(txString, "     F4GOH ", 11);
    addstring(txString, Beacon.GPGGA.hour, 6);
    addchar(txString, '-');
    addstring(txString, Beacon.GPGGA.Latitude, 7);
    addchar(txString, Beacon.GPGGA.NS);
    addchar(txString, '-');
    addstring(txString, Beacon.GPGGA.Longitude, 8);
    addchar(txString, Beacon.GPGGA.EO);
    addchar(txString, '-');
    addchar(txString, 0);


    //for (int n=0;n<sizeof(track);n++) {Serial.print(track[n],HEX);Serial.print(",");}    //print char array track (for  info)
    //Serial.println();
    Serial.flush();      //purge Serialout before txing*/

    txing();

    // delay(2000);


    lcd.setCursor(18, 0);    
    lcd.print(F("  "));

    Serial.println("TX Done");      //Roger it's done
  }
}

void addstring(char *dest, char *source, int taille)
{
  memcpy(dest+ptrChaine, source, taille); 
  ptrChaine+=taille;
}


void addchar(char *dest, char source)
{
  dest[ptrChaine]=source;
  ptrChaine++;
}



/*
+DMOSETGROUP:0  //command sucessfull 0x30,0xd,0xa
 +DMOSETGROUP:1  //command failed 0x31,0xd,x0a
 so read 3 last bytes into shifted buffer until 0xa found
 then test ack[0] 0x30 or 0x31
 */


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



void txing()
{
  byte save_TIMSK0;
  byte save_PCICR;
  digitalWrite(PwDw, HIGH);              //get up
  delay(2000);  //wait 2000ms to be awake
  digitalWrite(PTT, HIGH);            //ptt on
  delay(500);                         //delay before sending data
  TCCR0B = TCCR0B & 0b11111000 | 1;    //switch to 62500 HZ PWM frequency
  save_TIMSK0=TIMSK0;                  //save Timer 0 register
  TIMSK0 =0;                           //disable Timer 0
  save_PCICR=PCICR;                    //save external pin interrupt register
  PCICR = 0;                           //disable external pin interrupt
  Timer1.attachInterrupt(sinus_irq);   //warp interrupt in library
  Beacon.sendpacket(track, sizeof(track));  //send packet
  Beacon.rttyTx(txString);                 //send rtty
  Beacon.hellTx(txString);                 //send Hellschreiber
  digitalWrite(PTT, LOW);              //PTT off
  digitalWrite(PwDw, LOW);               //Power down
  Timer1.detachInterrupt();            //disable timer1 interrupt
  analogWrite(bfPin, 0);                //PWM at 0
  TCCR0B = TCCR0B & 0b11111000 | 3;    //register return to normal
  TIMSK0 =save_TIMSK0;
  PCICR = save_PCICR;
}


void sinus_irq()    //warp timer1 irq into DRAPRS lib
{
  Beacon.sinus();
}


/*
//DRA818 INFOS
 //Handshake  +crlf
 // AT+DMOCONNECT
 
 // Format˖AT+DMOSETGROUP=BWˈTX_FˈRX_FˈTx_subaudioˈSQˈRx_subaudio
 // AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000
 // SQ 0 à 8
 
 //Scan fréquency
 //S+144.8000
 //Réponse OK -> S=0 0=found 1=no signal
 
 //Volume configuration 1 a 8
 //AT+DMOSETVOLUME=4
 
 //AT+SETFILTER=PRE/DE-EMPH, HIGHPASS, LOWPASS
 //AT+SETFILTER=0,0,0
 
 */

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
//debug dra command and txing with $ car (bridge between 2 serial port)
/*  
 char car;
 if (Serial.available()>0){
 car=Serial.read();
 if (car=='$') {
 txing();
 }
 else
 if (car=='*') digitalWrite(PTT, LOW);
 else                          
 dra_serial.write(car);  
 }
 
 if (dra_serial.available()>0){
 Serial.write(dra_serial.read());  
 }
 */
