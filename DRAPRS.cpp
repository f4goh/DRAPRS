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

void DRAPRS::begin(int p_bf, int p_led) {
 bf=p_bf;
 led=p_led;
 
 pinMode(led, OUTPUT);
 pinMode(bf, OUTPUT);
  
 analogWrite(bf, 0);      //dds off
 digitalWrite(led, LOW);
   
}



/********************************************************
 * Send a bit into an FM modulation
 ********************************************************/
 
void DRAPRS::send_bit()
{
countPtr=0;
while(countPtr<11){
}
digitalWrite(led,digitalRead(led)^1);
}

void DRAPRS::sinus()
{
const static byte sinusTable[66] PROGMEM = {128,140,152,163,175,186,196,206,215,223,231,237,243,248,251,253,254,254,253,251,248,243,237,231,223,215,206,196,186,175,163,
152,140,127,115,103,92,80,69,59,49,40,32,24,18,12,7,4,2,1,1,2,4,7,12,18,24,32,40,49,59,69,80,92,103,115};

			 
analogWrite(bf,pgm_read_byte(&(sinusTable[sinusPtr])));
sinusPtr=(sinusPtr+shift)%66;
countPtr++;
}



/********************************************************
 * AX25 routines
 ********************************************************/

void DRAPRS::flipout(void)
 {		       
	stuff = 0;     //since this is a 0, reset the stuff counter
	if (shift==11) shift=6; else shift=11;
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
                               DRAPRS::send_bit();
                                DRAPRS::flipout();  
                        }                                      
        }
     inbyte = inbyte>>1;          		                //go to the next bit in the byte
DRAPRS::send_bit();
  }//fin pour
}


void DRAPRS::sendpacket(unsigned char buffer[], unsigned char size_array)
{
	unsigned char i;
    crc=0xffff;
	stuff=0;
   
	shift=6;      //init
    DRAPRS::send_bit();
   
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
//fe=13200
//te=1/26400 = 75.7µs (interrupt timer1 = 76)
//13200/1200= 11 ech
//13200/2200= 6 ech
//11*6=66 ech
//add 22 66/11=6 for 2200
//add 12 66/6=11 for 1200

int sinus[]=new int[66];

void setup()
{
int n;
  for (n=0;n<66;n++){
                    sinus[n]=(int) (128+127*sin(TWO_PI*(float)n/66));
                    print(sinus[n]+","); 
  }
}

void draw()
{
 
}



*/


