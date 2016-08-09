/*
    AFSK APRS with DRA818
 
 Copyright (C) 2016 Anthony LE CREN <f4goh@orange.fr>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 
 */

#define AFFLCD
//#define RTTYHELL
#define SDCARD
#define COMMANDMENU

#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <DRAPRS.h>

#ifdef SDCARD
#include <SdFat.h>
#include <SPI.h>
#endif


#ifdef AFFLCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#endif




#define PTT     3    // PTT pin. This is active low.
#define PwDw    2  // Power Down pin. This need to start low, then be set high before programming.


#define DRA_RXD 7   // The Arduino IO line that is connected to the DRA818's TXD pin. 3.3V only
// We *receive* TTL 3.3V Serial from the DRA818 on this pin.

#define DRA_TXD 6   // The Arduino IO line connected to the DRA818's RXD pin.
// We *send* TTL serial to the DRA818 on this pin.

#define bfPin 5     //afsk output must be PD5 
#define ledPin A2     //led for GPS fix


#define txPin 9      //tx pin into RX GPS connection
#define rxPin 8      //rx pin into TX GPS connection

SoftwareSerial dra_serial(DRA_RXD, DRA_TXD);    //for DRA818

SoftwareSerial gps(rxPin, txPin);  // RX, TX for GPS

#ifdef AFFLCD
LiquidCrystal_I2C lcd(0x27, 20, 4);    //4 lines *20 columns lcd char
#endif

// track char array

unsigned  char     track[72]= {
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, 0x60, //avant APTT4 7 octets (0-6)
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, ('0' + 10) << 1, //F4GOH-11 7 octets (7-13)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '1'<<1, ' '<<1, ('0' + 1) << 1, //WIDE1-1 7 octets (14-20)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '2'<<1, ' '<<1, ('0' + 1) << 1 | 1, //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
  0x03, 0xf0, //ctrl, pid 2 octets (28-29)
  '/', '1', '5', '0', '4', '5', '2', 'h', //heure 8 (30-37)
  '4', '8', '5', '1', '.', '2', '0', 'N', '/', '0', '0', '2', '2', '0', '.', '9', '2', 'E', //lat, long 18 octets (38-55)
  '>', '7', '3', ' ', 'A', 'n', 't', 'h', 'o', 'n', 'y', ' ', ' ', ' ', ' ', ' '
};            //commentaire 15 car octets (56-71)



#define BUFFER_EEPROM 53
#define NB_PARAMS 18
const static int epromPtr[NB_PARAMS] PROGMEM = { 
  0, 6, 8, 23, 24, 28, 36, 37, 40, 41, 43, 44, 45, 48, 49, 50, 51, 52
};
const static int nbCharPtr[NB_PARAMS] PROGMEM = {
  6, 2, 15, 1, 4, 8, 1, 3, 1, 2, 1, 1, 3, 1, 1, 1, 1, 1
};
char buffer[BUFFER_EEPROM];

int ptrChaine;

char gpsDataIn;

byte seqRam=0;
int ptrCarToRam=0;
byte ptrConfig=0;
byte flagEndWriteEeprom=0;

long previousMillisSerial = 0;
long currentMillisSerial;
long EcratMillisSerial;

byte delayMode=0;
int interval=20;
char neomode='N';
int neoDelay=10;
byte aprsOn=1;
byte rttyOn=0;
int  rttyShift=350;
byte hellOn=0;
byte lcdOn=0;
byte sdcardOn=0;
byte debug=1;
byte dumpNMEA=0;
byte ssid=10;
char symbol='>';
//byte trackSize=106;
byte trackSize=72;
int baudrate=9600;
char freq[9];
char call[7];
char message[16];

#ifdef SDCARD
SdFat SD;
File myFile;
#define SD_CS_PIN 10
#endif

void setup() {
  Serial.begin(57600);

  #ifdef AFFLCD
    lcd.begin();  //4 lines *20 columns lcd char
  lcd.setBacklight(HIGH);    
  lcd.clear();
  lcd.print(F("  APRS TRACKER v3.0"));    //intro
  lcd.setCursor(0, 2);
  lcd.print(F("     F4GOH 2016"));
  delay(4000);
  lcd.clear();
  #endif
    pinMode(PwDw, OUTPUT);        //config dra818 ctrl lines
  pinMode(bfPin, OUTPUT);
  digitalWrite(PwDw, LOW);

  pinMode(PTT, OUTPUT);
  digitalWrite(PTT, LOW);

  Timer1.initialize(76);    //µs  fe=13200 hz so TE=76µs 13157.9 mesured

  if  (detectMenu()==1) EepromMenu();
  updateAll();
  #ifdef SDCARD
    if (!SD.begin(SD_CS_PIN)) {      
    Serial.print(F("No SD card detected"));
    sdcardOn=0;
  }
  else
  {
    Serial.print(F("SD card detected"));
  }
  #endif





    Beacon.begin(bfPin, ledPin, 1200, 2200, rttyShift);   //analog pin, led pin, freq1, freq2, shift freq
  //gps.begin(9600);
  gps.begin(baudrate);
  //uncomment if you want to config dra818 frequency
}


void loop() {      //don't add any long delay in loop because GPS data recognition will be crash

  if (gps.available()) {
    gpsDataIn=gps.read();
    if ((gpsDataIn&0x80)!=0x80) Beacon.gpsnmea(gpsDataIn);   //if  char into software serial decode nmea sentence
  }

  if (Beacon.GPGGA.sync ==1) // test if  is time to txing
  {
    Beacon.GPGGA.sync =0;      //txing sequence, so sync done
    Serial.println( Beacon.GPGGA.time);          //print some GPS info

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

    #ifdef AFFLCD
    if (lcdOn==1){
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
    }
    #endif

      memcpy(track +31, Beacon.GPGGA.hour, 6);      //prepare APRS char array track to send
    memcpy(track +38, Beacon.GPGGA.Latitude, 7);     //beware index number from char array
    track[45] = Beacon.GPGGA.NS;
    memcpy(track +47, Beacon.GPGGA.Longitude, 8);
    track[55] = Beacon.GPGGA.EO;


    #ifdef RTTYHELL
      ptrChaine=0;                                 //prepare RTTY char array track to send                              
    addstring(buffer,"   ",3);
    addstring(buffer,call, 5);
    addstring(buffer,"   ",3);
    addstring(buffer, Beacon.GPGGA.hour, 6);
    addchar(buffer, '-');
    addstring(buffer, Beacon.GPGGA.Latitude, 7);
    addchar(buffer, Beacon.GPGGA.NS);
    addchar(buffer, '-');
    addstring(buffer, Beacon.GPGGA.Longitude, 8);
    addchar(buffer, Beacon.GPGGA.EO);
    addchar(buffer, '-');
    addchar(buffer, 0);
    #endif
      #ifdef SDCARD
      writesd();
    #endif

      //for (int n=0;n<sizeof(track);n++) {Serial.print(track[n],HEX);Serial.print(",");}    //print char array track (for  info)
    //Serial.println();
    Serial.flush();      //purge Serialout before txing*/
    gps.flush();      //purge Serialout before txing*/
    gps.stopListening();    
    Serial.println("TX enable");

    txing();

    #ifdef AFFLCD
      lcd.setCursor(18, 0);    
    lcd.print(F("  "));
    #endif
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
  updateAll();
  dra_serial.begin(9600);
  digitalWrite(PwDw, HIGH);
  delay(2000);  //wait 2000ms to be awake
  // dra_serial.println(F("AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000"));

  dra_serial.print(F("AT+DMOSETGROUP=0,"));
  dra_serial.print(freq);
  dra_serial.print(F(","));
  dra_serial.print(freq);
  dra_serial.println(F(",0000,4,0000"));

  ack[2]=0;
  previousMillisSerial = millis();
  do
  {
    if (dra_serial.available()>0) {
      ack[0]=ack[1];
      ack[1]=ack[2];
      ack[2]=dra_serial.read();
    }
    currentMillisSerial = millis();  
    EcratMillisSerial=currentMillisSerial - previousMillisSerial;
  }
  while ( (ack[2]!=0xa)&&(EcratMillisSerial<2000));

  digitalWrite(PwDw, LOW);
  dra_serial.end();
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
  if (aprsOn==1) Beacon.sendpacket(track, trackSize);  //send packet

  #ifdef RTTYHELL
    if (rttyOn==1) Beacon.rttyTx(buffer);                 //send rtty
  if (hellOn==1) Beacon.hellTx(buffer);                 //send Hellschreiber
  #endif

    digitalWrite(PTT, LOW);              //PTT off
  digitalWrite(PwDw, LOW);               //Power down
  Timer1.detachInterrupt();            //disable timer1 interrupt
  analogWrite(bfPin, 0);                //PWM at 0
  TCCR0B = TCCR0B & 0b11111000 | 3;    //register return to normal
  TIMSK0 =save_TIMSK0;
  PCICR = save_PCICR;
  gps.begin(baudrate);
  Beacon.ptrStartNmea=0;
}


void sinus_irq()    //warp timer1 irq into DRAPRS lib
{
  Beacon.sinus();
}


/*
 --------------------------------------- config menu
 */


byte detectMenu()
{
  // lcd.print(F("m for boot menu"));
  Serial.println(F("m for boot menu"));
  // lcd.setCursor(0,1);
  char car=0;
  char countDown=-1;
  digitalWrite(ledPin, HIGH);
  previousMillisSerial = millis();
  do {
    currentMillisSerial = millis();  
    EcratMillisSerial=currentMillisSerial - previousMillisSerial;
    if (Serial.available()>0) {
      if (Serial.read()=='m') {     
        return 1;
      }
    }
    if ((EcratMillisSerial/1000)!=countDown) {
      countDown++;
      //  lcd.write(countDown+0x30);
      Serial.write(countDown+0x30);
    }
  }
  while (EcratMillisSerial<5000);
  digitalWrite(ledPin, LOW);
  Serial.println();
  return 0;
}

#ifndef COMMANDMENU
void EepromMenu()
{
  char carMenu;
  //lcd.clear();
  //lcd.print(F("Boot menu"));
  do {
    carMenu=0;
    Serial.println(F("-----------"));
    Serial.println(F("Eeprom menu"));
    Serial.println(F("1 csv Eeprom Update"));
    Serial.println(F("2 csv Eeprom Read"));
    Serial.println(F("3 Update Freq"));
    Serial.println(F("4 Test GPS"));
    Serial.println(F("0 Quit menu"));
    Serial.println(F("-----------"));
    while (carMenu==0) if (Serial.available()>0) carMenu=Serial.read();
    switch(carMenu) {
    case '1' : 
      updateEeprom();
      break;
    case '2' : 
      generateCsvFromEeprom();
      break;
    case '3' : 
      if (configDra818()==1) Serial.println(F("freq update done")); 
      else Serial.println(F("freq update error"));
      break;
    case '4' : 
      testGps();
      break;
    case '0' :
      Serial.println(F("ok quit"));
      break;
    default : 
      Serial.println(F("error"));
    }
    updateAll();
  } 
  while (carMenu!='0');
}
#endif

#ifdef COMMANDMENU
void EepromMenu()
{
  char car=0;
  int flipCmd=0;
  Serial.println(F("\n\rtype 'help' for commands list"));
  if (EEPROM.read(0)!=0xff) updateAll();
  String cmd;
  String arg;
  byte argNull;
  car=Serial.read();
  do {
    Serial.print(F("\n\r>"));
    flipCmd=0;

    cmd="";
    arg="";
    car=0;
    do {
      if (Serial.available()>0) {
        car=Serial.read();
        if (car!='\n')
        {
          Serial.write(car);
          if (car==' ') {
            flipCmd=1;
          }
          else {
            if (flipCmd==0) cmd=cmd+car; 
            else arg=arg+car;
          }
        }
      }
    }
    while (car!='\n');
    Serial.println();

    cmd.toUpperCase();
    arg.toUpperCase();

    //Serial.println(cmd);
    //Serial.println(arg);
    argNull=arg.equals("");

    if (cmd.equals(F("LOAD"))==1) updateEeprom();
    else
    if (cmd.equals(F("READ"))==1) generateCsvFromEeprom();
    else
    if (cmd.equals(F("FREQ"))==1) if (argNull==1) {
      Serial.println(freq);      
    }
    else
    {                          
      saveEprom('F', arg);
      if (configDra818()==1) Serial.println(F("freq update done")); else Serial.println(F("freq update error"));
    }
    else
    if (cmd.equals(F("CALL"))==1) if (argNull==1) Serial.println(call);

    else
      saveEprom('A', arg);                                                    
    else
    if (cmd.equals(F("SSID"))==1) if (argNull==1)
      Serial.println(ssid);
    else
      saveEprom('B', arg);                          
    else
    if (cmd.equals(F("SYMBOL"))==1) if (argNull==1)
      Serial.println(symbol);
    else
     saveEprom('D', arg);                          
    else
    if (cmd.equals(F("GBAUD"))==1) if (argNull==1)
      Serial.println(baudrate);
    else
      saveEprom('E', arg);                          
     else  
    if (cmd.equals(F("MODE"))==1) if (arg.equals("")==1)
      Serial.println(delayMode);
    else
      saveEprom('G', arg);                          
     else
    if (cmd.equals(F("INTERVAL"))==1) if (argNull==1)
      Serial.println(interval);
    else
      saveEprom('H', arg);                          
     else
    if (cmd.equals(F("NEO"))==1) if (argNull==1)
      Serial.println(neomode);
    else
      saveEprom('I', arg);                          
     else
    if (cmd.equals(F("SECOND"))==1) if (argNull==1)
      Serial.println(neoDelay);
    else
      saveEprom('J', arg);                          
     else
    if (cmd.equals(F("APRS"))==1) if (argNull==1)
      Serial.println(aprsOn);
    else
      saveEprom('K', arg);   
     else
    if (cmd.equals(F("RTTY"))==1) if (argNull==1)
      Serial.println(rttyOn);
    else
      saveEprom('L', arg);   
     else
    if (cmd.equals(F("SHIFT"))==1) if (argNull==1)
      Serial.println(rttyShift);
    else
      saveEprom('M', arg);   
    else
    if (cmd.equals(F("HELL"))==1) if (argNull==1)
      Serial.println(hellOn);
    else
      saveEprom('N', arg);
    else
    if (cmd.equals(F("LCD"))==1) if (argNull==1)
      Serial.println(lcdOn);
    else
      saveEprom('O', arg);  
    else
    if (cmd.equals(F("SDCARD"))==1) if (argNull==1)
      Serial.println(sdcardOn);
    else
      saveEprom('P', arg);  
    else
    if (cmd.equals(F("DEBUG"))==1) if (argNull==1)
      Serial.println(debug);
    else
      saveEprom('Q', arg);   
    else
    if (cmd.equals(F("DUMP"))==1) if (argNull==1)
      Serial.println(dumpNMEA);
    else
      saveEprom('R', arg);   
     else

    if (cmd.equals(F("GPS"))==1)  testGps();
     else
    if (cmd.equals(F("HELP"))==1)  help();
     else
    if (cmd.equals(F("TRACK"))==1)  tracklist();
     else
    
    if (cmd.equals(F("SEND"))==1)  {      
      arg.toCharArray(buffer,arg.length());
       Beacon.begin(bfPin, ledPin, 1200, 2200, rttyShift);   //analog pin, led pin, freq1, freq2, shift freq
       txing();
    }
     else
     if (cmd.equals(F("QUIT"))==0)
       Serial.println(F("syntax error"));


    updateAll();
  } 
  while (cmd.equals (F ("QUIT"))==0);
  Serial.println(F("ok quit"));
}
#endif

void help()
{
  Serial.println(F("COMMANDS LIST"));
  Serial.println(F("LOAD, READ, GPS, HELP, TRACK, QUIT"));  
  Serial.println(F("CALL F4GOH"));
  Serial.println(F("SSID 12"));
  Serial.println(F("SYMBOL >"));
  Serial.println(F("GBAUD 9600"));
  Serial.println(F("FREQ 144.8000"));
  Serial.println(F("INTERVAL 20"));
  Serial.println(F("SECOND 30"));
  Serial.println(F("MODE, APRS, RTTY, HELL, LCD, SDCARD, DEBUG, DUMP 0 or 1"));
  Serial.println(F("NEO N,E or O"));
  Serial.println(F("SHIFT 350"));
  Serial.println(F("SEND HELLO"));
}

void tracklist()
{
  Serial.print(F("Your call is "));
  Serial.print(call); 
  Serial.write('-'); 
  Serial.print((byte)ssid); 
  Serial.print(F(" Symbol is "));
  Serial.println(symbol);
  Serial.print(F("Message is : "));
  Serial.println(message);
  if (delayMode==0) {
    Serial.print(F("Transmit every "));
    Serial.print(interval);  
    Serial.println(F(" seconds"));
  }
  else
  {
    Serial.print(F("Every "));  
    if  (neomode=='E') {
      Serial.print(F("EVEN"));
    }
    if  (neomode=='O') {
      Serial.print(F("ODD"));
    }
    Serial.print(F(" minutes and "));
    Serial.print(neoDelay);
    Serial.println(F(" seconds"));
  }
  Serial.print(F("APRS: "));
  Serial.print(aprsOn);
  Serial.print(F(",RTTY: "));
  Serial.print(rttyOn);
  Serial.print(F(",SHIFT: "));
  Serial.print(rttyShift);
  Serial.print(F(",HELL: "));
  Serial.println(hellOn);
  Serial.print(F("GPS BAUDrate is : "));
  Serial.println(baudrate);
  Serial.print(F("FREQUENCY is : "));
  Serial.println(freq);
  Serial.print(F("Sdcard: "));
  Serial.println(sdcardOn);
  Serial.print(F("debug: "));
  Serial.println(debug);
  Serial.print(F("dumpNMEA: "));
  Serial.println(dumpNMEA);
  Serial.print(F("LCD: "));
  Serial.println(lcdOn);
}

void saveEprom(char car, String arg)
{
  byte n;
  int prtcar=int(pgm_read_word(&epromPtr[car-0x41]));
  int nbcar =int(pgm_read_word(&nbCharPtr[car-0x41]));
  //Serial.print(arg.length());
  for (n=0;n<arg.length();n++)  EEPROM.write(prtcar+n, arg.charAt(n));
  for (n=arg.length();n<nbcar;n++)  EEPROM.write(prtcar+n, 0);
}

void testGps()
{
  updateAll();
  gps.begin(baudrate);
  char s=0;
  Serial.println(F("GPS testing...s to Stop"));
  do
  {  
    if (Serial.available()) {
      s=Serial.read();
    }
    if (gps.available()) {
      gpsDataIn=gps.read();
      Serial.write(gpsDataIn);
    }
  }
  while (s!='s');
  Serial.println(F("Stop GPS test"));
  s=Serial.read();
  gps.end();
}



void updateAll()
{
  //update values

  delayMode=(char) updateValue('G');
  interval= updateValue('H');
  neomode=updateValuechar('I');
  neoDelay=updateValue('J');
  ssid=updateValue('B');
  baudrate=updateValue('E');

  aprsOn=updateValue('K');
  rttyOn=updateValue('L');
  rttyShift=updateValue('M');
  hellOn=updateValue('N');
  lcdOn=updateValue('O');
  sdcardOn=updateValue('P');
  debug=updateValue('Q');
  dumpNMEA=updateValue('R');

  updateValuearray('F', freq);
  //Serial.println(freq);
  updateValuearray('A', call);
  //Serial.println(call);
  updateValuearray('C', message);
  symbol=updateValuechar('D');
  //neoDelay*=10;

  Beacon.GPGGA.pperiod = interval;   //interval between two tx minutes
  Beacon.GPGGA.debug = debug;   //allow debug print char
  Beacon.GPGGA.dumpNmea = dumpNMEA;   //allow print nmea sentence
  Beacon.GPGGA.neo=neomode;
  Beacon.GPGGA.Ndelay=neoDelay;
  Beacon.GPGGA.mode=delayMode;

  updatetrack('A', 7, 1);
  updatetrack('C', 57, 0);
  trackSize=72;

  track[13]= (unsigned char) (('0' + ssid) << 1);
  track[56]=(unsigned char) symbol;


  //lcdheader();
  digitalWrite(ledPin, LOW);
}


int updateValue(char car)
{
  int prtcar=int(pgm_read_word(&epromPtr[car-0x41]));
  int nbcar =int(pgm_read_word(&nbCharPtr[car-0x41]));
  char buffer[nbcar];
  for (byte n=0;n<nbcar;n++) buffer[n]=EEPROM.read(prtcar+n);
  return atoi(buffer);
}

char updateValuechar(char car)
{
  int prtcar;
  prtcar=int(pgm_read_word(&epromPtr[car-0x41]));
  return EEPROM.read(prtcar);
}

void updateValuearray(char car, char *buf)
{
  int prtcar;
  int nbcar;
  prtcar=int(pgm_read_word(&epromPtr[car-0x41]));
  nbcar =int(pgm_read_word(&nbCharPtr[car-0x41]));
  for (byte n=0;n<nbcar;n++) buf[n]=EEPROM.read(prtcar+n);
  buf[nbcar]=0;
  //Serial.print("----");
  //Serial.println(buf);
}


void updatetrack(char car, int ptrTrack, byte shiftEnable)
{
  int nbcar;
  int prtcar;
  int n, m;
  char c;
  n=int(car-0x41);
  prtcar=int(pgm_read_word(&epromPtr[n]));
  nbcar=int(pgm_read_word(&nbCharPtr[n]));  
  for (m=0;m<nbcar;m++){ 
        c=EEPROM.read(prtcar+m);
        if(c==0) c=c+0x20;
     if (shiftEnable==1) track[ptrTrack+m]=c<<1; else track[ptrTrack+m]=c;   
  }
}



void updateEeprom()
{
  Serial.println(F("Wait csv file"));
  flagEndWriteEeprom=0;
  while (flagEndWriteEeprom==0)
  {
    if (Serial.available()>0) {
      writeConfig(Serial.read());
    }
  }
  Serial.println(F("Csv file Updated"));
}

void  writeConfig(char car)
{
  if (seqRam==0) {
    ptrCarToRam=int(pgm_read_word(&epromPtr[int(car-0x41)]));
    if (ptrCarToRam==0) clearRam();
    seqRam=1;
    //Serial.println(ptrCarToRam);
    return;
  }

  if (seqRam==1) {
    if (car==0x0a) {
      seqRam=0;
      ptrConfig++;
      if (ptrConfig==NB_PARAMS) {
        //afficheRam();
        sauveEeprom(); //ok
        flagEndWriteEeprom=1;
      }
      return;
    }
    if ((car!=',') && (car!='"') && (car!=0x0d)) {
      toRam(car);
    }
  }
}

void toRam(char car)
{
  buffer[ptrCarToRam++]=car;
}

void sauveEeprom()
{
  for (int n=0;n<BUFFER_EEPROM;n++) {
    if (EEPROM.read(n)!=buffer[n]) EEPROM.write(n, (byte) buffer[n]);
  }
}

void afficheEeprom()
{
  for (int n=0;n<BUFFER_EEPROM;n++) {
    Serial.write(EEPROM.read(n));
  }
}

void generateCsvFromEeprom()
{
  int nbcar;
  int prtcar;
  int n, m;
  char car;
  for (n=0;n<NB_PARAMS;n++) {
    Serial.write(n+0x41);  
    Serial.write(',');  
    prtcar=int(pgm_read_word(&epromPtr[n]));
    nbcar=int(pgm_read_word(&nbCharPtr[n]));
    for (m=0;m<nbcar;m++) {
      if ((n>=14) && (n<=29) && (m==8)) Serial.write(',');
      if (((n==4) || (n==5)) && (m==0)) Serial.write('"');                           
      car=EEPROM.read(prtcar+m);
      if (car!=0) Serial.write(car);
    }
    if (((n==4) || (n==5)) && (m==nbcar)) Serial.write('"');
    Serial.write(',');
    Serial.println();
  }
}


void afficheRam()
{
  Serial.print("fini");
  for (int n=0;n<BUFFER_EEPROM;n++) {
    Serial.write(buffer[n]);
  }
}

void clearRam()
{
  for (int n=0;n<BUFFER_EEPROM;n++) {
    buffer[n]=0;
  }
}

#ifdef SDCARD
void writesd()
{
  char filename[]="track.txt";
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.print(Beacon.GPGGA.hour);
    myFile.write(',');
    myFile.print( Beacon.GPGGA.Latitude);
    myFile.write(',');
    myFile.write(Beacon.GPGGA.NS);
    myFile.write(',');
    myFile.print(Beacon.GPGGA.Longitude);
    myFile.write(',');
    myFile.print(Beacon.GPGGA.EO);
    myFile.write(',');
    myFile.println(Beacon.GPGGA.altitude);
  }
  myFile.close();
}
#endif



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
