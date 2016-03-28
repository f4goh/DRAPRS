/********************************************************************************************
 * VHF DRA818 APRS beacon Arduino library
 * Created 6/6/2015
 * Anthony LE CREN f4goh@orange.fr 
 * Modified 25/02/2015
 * Use this library freely
 *
 * Instance :
 *
 * Functions :
 *
 *******************************************************************************************/
 
 
#ifndef DRAPRS_H
#define DRAPRS_H
#include <Arduino.h>


//#include <SoftwareSerial.h>

class DRAPRS
{
  public:
    DRAPRS();
   
   
   
   void begin(int p_bf, int p_led, int f1,int f2, int f3);
   void sendpacket(unsigned char buffer[], unsigned char size_array);
   void gpsnmea(char byteGPS);
   void sinus();
   void rttyTx(char * stringRtty);
   	void rttyTxByte (char c);
	void hellTx( char * stringHell);
	
   unsigned long computeDdsWord(double freq);
   
   double freq;
   int sync;
   int led;
   int bf;
   double refclk;
   byte flip_freq;
   
   
   
   volatile unsigned long ddsAccu;   // phase accumulator
   volatile unsigned long ddsWord;
   volatile unsigned long ddsWord0;  // dds tuning word 0
   volatile unsigned long ddsWord1;  // dds tuning word 1
   volatile unsigned long ddsWord2;  // dds tuning word 1
	
   typedef struct  {
  char hour[6+1];
  char Latitude[9+1];
  char NS;
  char Longitude[10+1];
  char EO;
  byte fix;
  char sat[2+1];
  char altitude[7+1];
  byte secondes;     //secondes in byte from hour
  byte pperiod;      //second order
  byte sync;     //flag to send (matching for secondes%pperiod==0)
  boolean debug;
  boolean dumpNmea;
  int nbSat;
  long altidudeMeters;
  long altidudeFeet;
  char feet[15];
  } GGAstruct;
  GGAstruct GPGGA;    //declare la structure
   
  volatile int sinusPtr;
  volatile int countPtr;
  volatile int shift;
 
  private:
  
  
  void send_bit(int tempo);
  unsigned char flip;
  
  
  void sendbyte (unsigned char inbyte);
  void fcsbit(unsigned short tbyte);
  void flipout(void);
 
  
  unsigned char stuff,flag,fcsflag;
 
  unsigned short crc;

  
  int sentence_status;      //0: recherche $, 1:recherche GPxxx, 2:GPGGA trouvé
  char sentenceType[5+1];     //GPxxx
  int ptr;                  //ptr for cahr arrays
  int comma_count;          //count , into sentences 

  
};

extern DRAPRS Beacon;

#endif
