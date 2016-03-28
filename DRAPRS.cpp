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
 

#include <DRAPRS.h>


DRAPRS Beacon;


DRAPRS::DRAPRS(){
 sentence_status=0;      //0: recherche $, 1:recherche GPxxx, 2:GPGGA trouvé
 ptr=0;                  //ptr for char arrays
 comma_count=0;          //count , into sentences 
 GPGGA.nbSat=0;
 GPGGA.debug=false;
 GPGGA.dumpNmea=false;
 sinusPtr=0;
 countPtr=0;
}

void DRAPRS::begin(int p_bf, int p_led, int f1,int f2, int f3) {
 bf=p_bf;
 led=p_led;
 
 pinMode(led, OUTPUT);
 pinMode(bf, OUTPUT);
  
 analogWrite(bf, 0);      //dds off
 digitalWrite(led, LOW);
 refclk=13157.9;      // measured 
 
 ddsWord0=DRAPRS::computeDdsWord(f1);
 ddsWord1=DRAPRS::computeDdsWord(f2);
 ddsWord2=DRAPRS::computeDdsWord(f1+f3);
 
 
}



/********************************************************
 * Send a bit into an FM modulation
 ********************************************************/
 
void DRAPRS::send_bit(int tempo)
{
countPtr=0;
while(countPtr<tempo){
}
digitalWrite(led,digitalRead(led)^1);
}



void DRAPRS::sinus()
{

const static byte sinusTable[512] PROGMEM = {128,129,131,132,134,135,137,138,140,141,143,145,146,148,149,151,152,154,155,157,158,160,161,163,164,166,167,169,170,172,173,175,176,178,179,180,182,183,185,186,
                                             187,189,190,191,193,194,195,197,198,199,201,202,203,204,206,207,208,209,210,212,213,214,215,216,217,218,219,221,222,223,224,225,226,227,228,229,230,230,231,232,
                                             233,234,235,236,236,237,238,239,240,240,241,242,242,243,244,244,245,245,246,247,247,248,248,249,249,249,250,250,251,251,251,252,252,252,253,253,253,253,254,254,
                                             254,254,254,254,254,254,254,254,255,254,254,254,254,254,254,254,254,254,254,253,253,253,253,252,252,252,251,251,251,250,250,249,249,249,248,248,247,247,246,245,
                                             245,244,244,243,242,242,241,240,240,239,238,237,236,236,235,234,233,232,231,230,230,229,228,227,226,225,224,223,222,221,219,218,217,216,215,214,213,212,210,209,
                                             208,207,206,204,203,202,201,199,198,197,195,194,193,191,190,189,187,186,185,183,182,180,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,157,155,154,
                                             152,151,149,148,146,145,143,141,140,138,137,135,134,132,131,129,127,126,124,123,121,120,118,117,115,114,112,110,109,107,106,104,103,101,100,98,97,95,94,92,91,89,
                                             88,86,85,83,82,80,79,77,76,75,73,72,70,69,68,66,65,64,62,61,60,58,57,56,54,53,52,51,49,48,47,46,45,43,42,41,40,39,38,37,36,34,33,32,31,30,29,28,27,26,25,25,24,23,
                                             22,21,20,19,19,18,17,16,15,15,14,13,13,12,11,11,10,10,9,8,8,7,7,6,6,6,5,5,4,4,4,3,3,3,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,4,4,4,5,5,6,
                                             6,6,7,7,8,8,9,10,10,11,11,12,13,13,14,15,15,16,17,18,19,19,20,21,22,23,24,25,25,26,27,28,29,30,31,32,33,34,36,37,38,39,40,41,42,43,45,46,47,48,49,51,52,53,54,56,57,
                                             58,60,61,62,64,65,66,68,69,70,72,73,75,76,77,79,80,82,83,85,86,88,89,91,92,94,95,97,98,100,101,103,104,106,107,109,110,112,114,115,117,118,120,121,123,124,126};
											 

 ddsAccu=ddsAccu+ddsWord; // soft DDS, phase accu with 32 bits
 sinusPtr=ddsAccu >> 23;
analogWrite(bf,pgm_read_byte(&(sinusTable[sinusPtr])));
countPtr++;
}



/********************************************************
 * AX25 routines
 ********************************************************/

void DRAPRS::flipout(void)
 {		       
	stuff = 0;     //since this is a 0, reset the stuff counter
    flip_freq^=1;
	if (flip_freq==0) ddsWord=ddsWord1; else ddsWord=ddsWord0;
	
}

void DRAPRS::fcsbit(unsigned short tbyte)
{
  crc ^= tbyte;
  if (crc & 1)
    crc = (crc >> 1) ^ 0x8408;  // X-modem CRC poly
  else
    crc = crc >> 1;
     
 }
 
 
void DRAPRS::sendbyte (unsigned char inbyte)
{
   unsigned char k, bt;
 
   for (k=0;k<8;k++)
	{                                                         //do the following for each of the 8 bits in the byte
     bt = inbyte & 0x01;                                          //strip off the rightmost bit of the byte to be sent (inbyte)
     if ((fcsflag==0) & (flag==0)) (DRAPRS::fcsbit(bt));                 //do FCS calc, but only if this is not a flag or fcs byte
     if (bt==0) (DRAPRS::flipout());  			                // if this bit is a zero, flip the output state
        else {                          			//otherwise if it is a 1, do the following:
           stuff++;    				                //increment the count of consequtive 1's 
            if ((flag==0) & (stuff==5))
				{   	                        //stuff an extra 0, if 5 1's in a row
                                       		                 //flip the output state to stuff a 0
                               DRAPRS::send_bit(11);
                                DRAPRS::flipout();  
                        }                                      
        }
     inbyte = inbyte>>1;          		                //go to the next bit in the byte
DRAPRS::send_bit(11);
  }//fin pour
}


unsigned long DRAPRS::computeDdsWord(double freq)
{
return pow(2,32)*freq/refclk;
}



void DRAPRS::sendpacket(unsigned char buffer[], unsigned char size_array)
{
	unsigned char i;
    crc=0xffff;
	stuff=0;
   
	shift=6;      //init
	flip_freq=1;
	ddsWord=ddsWord1;
	
    DRAPRS::send_bit(11);
   
   flag = 1;             //The variable flag is true if you are transmitted flags (7E's) false otherwise.
   fcsflag = 0;       //The variable fcsflag is true if you are transmitting FCS bytes, false otherwise.

   for (i=0;i<200;i++) DRAPRS::sendbyte(0x7E);	        //Sends 100 flag bytes.
   flag = 0;          			        //done sending flags
   for(i=0;i<size_array;i++) DRAPRS::sendbyte(buffer[i]);       //send the packet bytes
   fcsflag = 1;       		//about to send the FCS bytes
   DRAPRS::sendbyte((crc ^ 0xff));	// Send the CRC
   crc >>= 8;
   DRAPRS::sendbyte((crc ^ 0xff));
   fcsflag = 0;		//done sending FCS
   flag = 1;  	//about to send flags
  for (i=0;i<100;i++) DRAPRS::sendbyte(0x7E);	        //Sends 100 flag bytes.
 
  }
 
/********************************************
 * RTTY
 ********************************************/ 
void DRAPRS::rttyTx(char * stringRtty)
{
 const static int TableRtty[59] PROGMEM = {4,22,17,5,18,0,11,26,30,9,0,0,6,24,7,23,13,29,25,16,10,1,21,28,12,3,14,15,0,0,0,19,0,24,19,14,18,16,22,11,5,12,26,30,9,7,6,3,13,29,10,20,1,28,15,25,23,21,17};

 int signlett = 1;  // RTTY Baudot signs/letters tables toggle
 char c;
 c = *stringRtty++;

 while ( c != '\0')
 {
  c = toupper(int(c)); // Uppercase
  if(c == 10) // Line Feed
  {
   rttyTxByte(8);
  }
  else if(c == 13) // Carriage Return
  {
   rttyTxByte(2);
  }
  else if(c == 32) // Space
  { 
   rttyTxByte(4);
  }
  else if (c > 32 && c < 91)
  {
   c = c - 32;
   if(c < 33)
   {
    if (signlett == 1)
    {
	 signlett = 0;      // toggle form signs to letters table
     rttyTxByte(27);  //
    }
   }
   else if(signlett == 0)
   {
    signlett = 1;          // toggle form letters to signs table
    rttyTxByte(31);      //
   }
  rttyTxByte(int(pgm_read_word(&TableRtty[int(c)])));  // Send the 5 bits word
  }
  c = *stringRtty++;  // Next character in string
 }
}



void DRAPRS::rttyTxByte(char c){
 int val;
 c = (c << 2)+3; 
 for(int b = 7; b >= 0; b--) // MSB first
 {
  val = bitRead(c,b); // Read 1 bit
 if (val==0) ddsWord=ddsWord0; else ddsWord=ddsWord2; // Let's transmit (bit 1 is shifted)
 DRAPRS::send_bit(292); //baud rate
 }
}

/***********************************************************************************
* Hellschreiber 122.5 bauds => 8,888ms
* http://brainwagon.org/2012/01/11/hellduino-sending-hellschreiber-from-an-arduino
************************************************************************************/

void DRAPRS::hellTx( char * stringHell)
{
const static word GlyphTab[59][8] PROGMEM = {
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x1f9c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0330, 0x0ffc, 0x0330, 0x0ffc, 0x0330, 0x0000, 0x0000},
  {0x078c, 0x0ccc, 0x1ffe, 0x0ccc, 0x0c78, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
  {0x01e0, 0x0738, 0x1c0e, 0x0000, 0x0000, 0x0000, 0x0000},{0x1c0e, 0x0738, 0x01e0, 0x0000, 0x0000, 0x0000, 0x0000},{0x018c, 0x0198, 0x0ff0, 0x0198, 0x018c, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x001c, 0x001c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x001c, 0x0070, 0x01c0, 0x0700, 0x1c00, 0x0000, 0x0000},
  {0x07f8, 0x0c0c, 0x0c0c, 0x0c0c, 0x07f8, 0x0000, 0x0000},{0x0300, 0x0600, 0x0ffc, 0x0000, 0x0000, 0x0000, 0x0000},{0x061c, 0x0c3c, 0x0ccc, 0x078c, 0x000c, 0x0000, 0x0000},{0x0006, 0x1806, 0x198c, 0x1f98, 0x00f0, 0x0000, 0x0000},
  {0x1fe0, 0x0060, 0x0060, 0x0ffc, 0x0060, 0x0000, 0x0000},{0x000c, 0x000c, 0x1f8c, 0x1998, 0x18f0, 0x0000, 0x0000},{0x07fc, 0x0c66, 0x18c6, 0x00c6, 0x007c, 0x0000, 0x0000},{0x181c, 0x1870, 0x19c0, 0x1f00, 0x1c00, 0x0000, 0x0000},
  {0x0f3c, 0x19e6, 0x18c6, 0x19e6, 0x0f3c, 0x0000, 0x0000},{0x0f80, 0x18c6, 0x18cc, 0x18cc, 0x0ff0, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},{0x1800, 0x1800, 0x19ce, 0x1f00, 0x0000, 0x0000, 0x0000},
  {0x01f0, 0x0208, 0x04e4, 0x0514, 0x0514, 0x03e0, 0x0000},{0x07fc, 0x0e60, 0x0c60, 0x0e60, 0x07fc, 0x0000, 0x0000},{0x0c0c, 0x0ffc, 0x0ccc, 0x0ccc, 0x0738, 0x0000, 0x0000},{0x0ffc, 0x0c0c, 0x0c0c, 0x0c0c, 0x0c0c, 0x0000, 0x0000},
  {0x0c0c, 0x0ffc, 0x0c0c, 0x0c0c, 0x07f8, 0x0000, 0x0000},{0x0ffc, 0x0ccc, 0x0ccc, 0x0c0c, 0x0c0c, 0x0000, 0x0000},{0x0ffc, 0x0cc0, 0x0cc0, 0x0c00, 0x0c00, 0x0000, 0x0000},{0x0ffc, 0x0c0c, 0x0c0c, 0x0ccc, 0x0cfc, 0x0000, 0x0000},
  {0x0ffc, 0x00c0, 0x00c0, 0x00c0, 0x0ffc, 0x0000, 0x0000},{0x0c0c, 0x0c0c, 0x0ffc, 0x0c0c, 0x0c0c, 0x0000, 0x0000},{0x003c, 0x000c, 0x000c, 0x000c, 0x0ffc, 0x0000, 0x0000},{0x0ffc, 0x00c0, 0x00e0, 0x0330, 0x0e1c, 0x0000, 0x0000},
  {0x0ffc, 0x000c, 0x000c, 0x000c, 0x000c, 0x0000, 0x0000},{0x0ffc, 0x0600, 0x0300, 0x0600, 0x0ffc, 0x0000, 0x0000},{0x0ffc, 0x0700, 0x01c0, 0x0070, 0x0ffc, 0x0000, 0x0000},{0x0ffc, 0x0c0c, 0x0c0c, 0x0c0c, 0x0ffc, 0x0000, 0x0000},
  {0x0c0c, 0x0ffc, 0x0ccc, 0x0cc0, 0x0780, 0x0000, 0x0000},{0x0ffc, 0x0c0c, 0x0c3c, 0x0ffc, 0x000f, 0x0000, 0x0000},{0x0ffc, 0x0cc0, 0x0cc0, 0x0cf0, 0x079c, 0x0000, 0x0000},{0x078c, 0x0ccc, 0x0ccc, 0x0ccc, 0x0c78, 0x0000, 0x0000},
  {0x0c00, 0x0c00, 0x0ffc, 0x0c00, 0x0c00, 0x0000, 0x0000},{0x0ff8, 0x000c, 0x000c, 0x000c, 0x0ff8, 0x0000, 0x0000},{0x0ffc, 0x0038, 0x00e0, 0x0380, 0x0e00, 0x0000, 0x0000},{0x0ff8, 0x000c, 0x00f8, 0x000c, 0x0ff8, 0x0000, 0x0000},
  {0x0e1c, 0x0330, 0x01e0, 0x0330, 0x0e1c, 0x0000, 0x0000},{0x0e00, 0x0380, 0x00fc, 0x0380, 0x0e00, 0x0000, 0x0000},{0x0c1c, 0x0c7c, 0x0ccc, 0x0f8c, 0x0e0c, 0x0000, 0x0000}
 };
 int val;
 char ch;
 int n;
 word fbits ;
 ch = *stringHell++;
 while (ch != '\0')
 {
  ch = toupper(int(ch)); // Uppercase
  if(ch >= 32 && ch <= 90) // Character is in the range of ASCII space to Z
  {
   ch -= 32;  // Character number starting at 0
   for (int i = 0; i < 7; i++) // Scanning each 7 columns of glyph
   {  
	fbits = int(pgm_read_word(&GlyphTab[int(ch)][i]));  // Get each column of glyph
    for (int b = 0; b < 14; b++) // Scanning each 14 rows
    {
	 val = bitRead(fbits,b);  // Get binary state of pixel
	 if (val==1) {					// Gives the baud rate. 4045µs minus DDS loading time, 4002µs in SPI mode, 3438µs in software serial mode
			ddsWord=ddsWord0;
			DRAPRS::send_bit(53);
	 }
	  else { //pwm à 0
			ddsWord=0;
			DRAPRS::send_bit(53);
	  }
	  delayMicroseconds(34);
    }
   }
  }
  ch = *stringHell++; // Next character in string
 }
}

  
  
 void DRAPRS::gpsnmea(char byteGPS)
{
 if( GPGGA.dumpNmea==true)Serial.print((char)byteGPS);
 switch (byteGPS)
{
  case '$' : if (sentence_status==0) {
                                      sentence_status=1;
                                      ptr=0;
                                      if (GPGGA.debug==true) Serial.print('$');
                                     }
              break;
  case ',' : if (sentence_status==1) {
                                         sentenceType[ptr++]=0;
                                         if (strcmp(sentenceType,"GPGGA")==0) 
                                                  {
                                                   if (GPGGA.debug==true) Serial.println(F("gga found"));
                                                   sentence_status=2;    //can extend sentence_status to 3,4 etc for another GPxxx sentences
                                                  }
                                                  else sentence_status=0;
                                      } 
              if (sentence_status==2) {
                                       switch (comma_count)
                                       {
                                      case 1 :   GPGGA.hour[ptr]=0;
                                                 if (GPGGA.fix==1) {
																	GPGGA.secondes=(GPGGA.hour[4]-'0')*10+GPGGA.hour[5]-'0';
																	if (GPGGA.debug==true) {Serial.print(F("Hour :")); Serial.println(GPGGA.hour);}
																	if ((GPGGA.secondes%GPGGA.pperiod)==0) GPGGA.sync=1; else GPGGA.sync=0;
																	}
												break; 
                                      case 2 :   GPGGA.Latitude[ptr]=0;
												 break; 
                                      case 4 :   GPGGA.Longitude[ptr]=0;
												 break; 
                                      case 6 :  if (GPGGA.fix==1) digitalWrite(led, HIGH); else {
																									digitalWrite(led, digitalRead(led)^1);
																								    if (GPGGA.debug==true)		{
																																	Serial.print(F("Nb sat :"));
																																	Serial.println(GPGGA.sat);
																																	}
																								}
												break; 
									  case 7 : GPGGA.sat[ptr]=0;
											   GPGGA.nbSat=atoi(GPGGA.sat);
                                               break;            
                                      case 9 :   GPGGA.altitude[ptr]=0;
									          GPGGA.altidudeMeters=atol(GPGGA.altitude);
									          GPGGA.altidudeFeet=(long) GPGGA.altidudeMeters*328/100;
											  ltoa(GPGGA.altidudeFeet,GPGGA.feet,10);
											  break; 
                                      }
                                    }
              ptr=0;
              comma_count++;
              break;
  case '*' :  sentence_status=0;
              comma_count=0;
			  break;
  default:
   if (sentence_status==1) sentenceType[ptr++]=byteGPS;
   if (sentence_status==2) {
                             switch (comma_count)
                             {
                              case 1 :   if (ptr<6) GPGGA.hour[ptr++]=byteGPS;
                                         break;
                              case 2 :   GPGGA.Latitude[ptr++]=byteGPS;
                                         break;            
                              case 3 :   GPGGA.NS=byteGPS;
                                         break;            
                              case 4 :   GPGGA. Longitude[ptr++]=byteGPS;
                                         break;            
                              case 5 :   GPGGA.EO=byteGPS;
                                         break;            
                              case 6 :   GPGGA.fix=byteGPS-'0';
                                         break;            
                              case 7 :   GPGGA.sat[ptr++]=byteGPS;
                                         break;            
							  case 9 :   GPGGA.altitude[ptr++]=byteGPS;
                                         break;            
                             }
                       }
 }
}
 
 /*
//sinus table, generate with processing.org
//fe=13200,	13157.9 mesured
//te=1/13200 = 75.7µs (interrupt timer1 = 76)

int sinus[]=new int[512];

void setup()
{
int n;
  for (n=0;n<512;n++){
                    sinus[n]=(int) (128+127*sin(TWO_PI*(float)n/512));
                    print(sinus[n]+","); 
  }
}

void draw()
{
 
}



*/


