/*
  written by David Dold, 10Jul22
  Native GPS time sync using NEO-6M via SoftwareSerial on UART1

  SUMMARY:
  NEO-GM Unless throttled is taking a sip from a firehouse.
  as sketch gets busy, checksums fail and sentences discarded.
  tracks warm-start (when ephemerals available) until GPGSA status changes to 2d/3d fix
  resyncs internal time with gps/noGPS every x seconds

  NOTES:
  invariably checksum failing on sentences indicates lost characters and buffer overruns as processor cannot keep up with the volume of data from gps chip.  
  setup mitigates this with sentence pubx,40 suppressing unwanted sentences, and reporting wanted sentences every X cycle.
  from what I can tell, slowest cycle rate is every second; way too fast for platform

  GPS chipset does not report inaccurate data, unless forced.
  
  based on  https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
*/


#include <TimeLib.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define GPS_RX 12
#define GPS_TX 11

SoftwareSerial gpsSerial(GPS_RX,GPS_TX);
time_t lastUpdateGPS=0;  //seconds since January 1, 1970 (epoch)
time_t lastCycleDevices=0;
bool warmStart=true;
bool buildingSentence=false;
char gpsResponse[200]; //no extended ASCII
int packetOffset=0;
char *spamList[]={"GSV","GGA","VTG","GLL"};

void setup() {
  Serial.begin(9600);
  Serial.flush();
  Serial.println("");
  Serial.println("Starting");

  //Using processor eeprom to track the age and internal time decay from the last 2d/3d fix
  //asper spec (pape 29) gps chipset startup hot/warm with no fix and ephemerals available from BBR - battery backup ram (app four hours since fix).
  //HOWEVER, when no fix and continual power, internal clock ia used for time infomation regardless of ephemeral state, having the potential for decay.
  bool bEEConfigured = false;
  //comment out next line to reset memory
  EEPROM.get(0, bEEConfigured);
  if (!bEEConfigured){
      Serial.println("Updating EEProm");
      int iEEAddressOffset = 0;
      EEPROM.put(iEEAddressOffset,true);
      iEEAddressOffset+=sizeof(bool);
      EEPROM.put(iEEAddressOffset, lastUpdateGPS);
      iEEAddressOffset+=sizeof(time_t);
      EEPROM.put(iEEAddressOffset, '\0');
  }

  pinMode(GPS_RX, INPUT);
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_RX,LOW);
  
  gpsSerial.begin(9600);
  gpsSerial.flush();
  buildingSentence=false;


  delay(500);
  Serial.println("Init nmea packets and cycles");
  char pubx40[40];
  memset(pubx40,0,sizeof(pubx40));
  //Throttle Sentences
  //RMCx5
  sprintf(pubx40,"$PUBX,40,RMC,0,5,0,0,0,0");  //page 82
  if (SetCheckSum(pubx40, sizeof(pubx40))) {
    String sPubx=pubx40;
    Serial.print(sPubx);
    gpsSerial.print(sPubx);
  }
  //GSAx5
  sprintf(pubx40,"$PUBX,40,GSA,0,5,0,0,0,0");
  if (SetCheckSum(pubx40, sizeof(pubx40))) {
    String sPubx=pubx40;
    Serial.print(sPubx);
    gpsSerial.print(sPubx);
  }
  SetSpamSentences();
}

void loop() {

    if (timeStatus()==timeSet){
      if (now()-lastCycleDevices>=5){
          time_t t=now();
          char buf[30];
          memset(buf,0,sizeof(buf));
          sprintf(buf, "%02d:%02d:%02dgmt %02d/%02d/%02d ws:%d", hour(t), minute(t), second(t), month(t), day(t), year(t),warmStart);
          Serial.println(buf);
          if (warmStart){
              //when was the last 2d/3d fix?
              EEPROM.get(sizeof(bool),t);
              if (t>0){
                char *fixType;
                EEPROM.get(sizeof(time_t)+sizeof(bool),fixType);
                int iElapsedMinutes=(now()-t)/60;
                if (iElapsedMinutes>10){
                  sprintf(buf, "%d minutes since a %cd fix", iElapsedMinutes,fixType);                
                  Serial.println(buf);
                }
              }
          }
          lastCycleDevices=now();
      }
    }

    if (gpsSerial.available() > 0) 
        ProcessGPS();
}


bool Spam(char *pResponse){
  //correct for spam detected if NEO cycles power
  bool bResult=false;
  int iRowCount=sizeof(spamList)/2;  
  /*
    sizeof decays to pointers in the rowCount * sizeof pointer in second element
    impossible to determing sizeof spamlist without counting the sizeof each element.  
  */
  
  for (int x=0;x<iRowCount;x++){
      char *found=strstr(pResponse,spamList[0,x]);
      if (found){
        char pubx40[40];
        memset(pubx40,0,sizeof(pubx40));
        sprintf(pubx40,"$PUBX,40,%3s,0,0,0,0,0,0",spamList[0,x]);
        if (SetCheckSum(pubx40,sizeof(pubx40))){
          Serial.println("SPAM DETECTED");
          String sTemp=pubx40;
          Serial.println(sTemp);
          gpsSerial.print(sTemp);
        }
      }
  }
  return bResult;  
}

void SetSpamSentences(){
  //correct for spam detected if NEO cycles power
  int iRowCount=sizeof(spamList)/2;  
  /*
    sizeof decays to pointers in the rowCount * sizeof pointer in second element
    impossible to determing sizeof spamlist without counting the sizeof each element.  
  */
  Serial.println(F("BLOCKING:"));
  char pubx40[40];
  for (int x=0;x<iRowCount;x++){
      memset(pubx40,0,sizeof(pubx40));
      sprintf(pubx40,"$PUBX,40,%3s,0,0,0,0,0,0",spamList[0,x]);
      if (SetCheckSum(pubx40,sizeof(pubx40))){
        String sTemp=pubx40;
        Serial.print(sTemp);
        gpsSerial.print(sTemp);
      }
  }
}


bool SyncTimeToGPS(char *pResponse){
    //$GPRMC,184548.00,A,3245.73987,N,11709.12812,W,1.939,,160722,,,A  page 63
    //asper spec device only sends valid data, otherwise field is empty
    bool bResult=false;
    char *found=strstr(pResponse,"$GPRMC");  //this could be anywhere in the buffer -snd- there may be more than one instance
    if (found!=NULL){
      String sParser=pResponse;
      int iLastPacket=sParser.lastIndexOf('$GPRMC');  //buffer may contain multiple RMC packets, use the most recent report (last)
      if (iLastPacket>=5){
        iLastPacket-=5;
        String sRMC=sParser.substring(iLastPacket);
        if (CheckSum(sRMC.c_str())){        
          Serial.print("\r\nRMC Qualified:");
          Serial.print(sRMC);
          int iHour=-1;
          int iMinute=-1;
          int iSecond=-1;
          int iDay=-1;
          int iMonth=-1;
          int iYear=-1;
        
          //$GPRMC,153938.00,V,,,,,,,160722,,,N*78  --VALIDATED
          int iStart=sRMC.indexOf('$GPRMC,')+1;
          if (iStart==7){
            String sTime=sRMC.substring(iStart);
            int iTimeEnd=sTime.indexOf(',');
            iTimeEnd-=3;
            //Serial.print("\r\nTime:");
            sTime=sTime.substring(0,iTimeEnd);
            //Serial.println(sTime);  //HHMMSS
            iHour=sTime.substring(0,2).toInt();
            iMinute=sTime.substring(2,4).toInt();
            iSecond=sTime.substring(4,6).toInt();
          }

          String sDate=sRMC.substring(iStart);
          //date is in ninth position
          for (int iField=0; iField<8;iField++){
            int iOffset=sDate.indexOf(',')+1;
            sDate=sDate.substring(iOffset);
          }
          
          int iDateEnd=sDate.indexOf(',');
          sDate=sDate.substring(0,iDateEnd);
          if (sDate.length()==6){
            iDay=sDate.substring(0,2).toInt();
            iMonth=sDate.substring(2,4).toInt();
            iYear=sDate.substring(4,6).toInt();
            //sprintf(buf, "%2d/%2d/%2d", iMonth,iDay,iYear);
            //Serial.println(buf);

            if ((iHour!=-1) && (iMinute!=-1) && (iSecond!=-1)){
              char buf[30];
              memset(buf,0,sizeof(buf));
              if (warmStart)
                sprintf(buf, "\r\nnoGPS: %02d:%02d:%02d %02d/%02d/%02d", iHour, iMinute, iSecond, iMonth, iDay, iYear);
              else
                sprintf(buf, "\r\nGPS: %02d:%02d:%02d %02d/%02d/%02d", iHour, iMinute, iSecond, iMonth, iDay, iYear);
              Serial.println(buf);
              // set the Time to the latest GPS reading
              setTime(iHour, iMinute, iSecond, iDay, iMonth, iYear);
              lastUpdateGPS=now();
              bResult=true;
            }
          }
        }
      }
    }
    return bResult;
}


bool TimeSource(char *pResponse){
  bool bResult=false;
  //comma counts are variable bassed on fixed satellite list
  //$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30   page 60
  //$GPGSA,A,3,02,11,29,20,31,25,18,129,0.99,1.38*07 
  //datetime from warm start until at least one active satellite in GPGSA  

  char *found=strstr(pResponse,"$GPGSA");  //this could be anywhere in the buffer -snd- there may be more than one instance
  if (found!=NULL){
    String sParser=pResponse;
    int iLastPacket=sParser.indexOf('$GPGSA'); 
    if (iLastPacket>=5){
      iLastPacket-=5;
      String sGSA=sParser.substring(iLastPacket);
      int iCrLf=sGSA.indexOf('\r\n');  //asper packet spec, each "sentence" ends with crlf
  
      if (iCrLf>0){
        String sQualified=sGSA.substring(0,iCrLf);
        if (CheckSum(sQualified.c_str())){ 
          char fixStatus=sGSA[9];
          if (fixStatus=='1')
            warmStart=true;
          else if ((fixStatus=='2') || (fixStatus=='3')){

            if (timeStatus()==timeSet){
              //store the last fix time and type
              EEPROM.put(sizeof(bool), now());
              EEPROM.put(sizeof(time_t)+sizeof(bool),fixStatus);
            }
            warmStart=false;
          }
          //Serial.print("\r\nGSA warmStart:");
          //Serial.println(bWarmStart);
          bResult=true;
        }
      }
    }
  }
  return bResult;
}


bool SetCheckSum(char *pPacket, int bufferSize){
    bool bResult=false;
    //packet sent with five extra bytes -and- without an asterick  0x26 CR_A CR_B 0x0d 0x0a
    int iSize=strlen(pPacket);
    if ((iSize+5)<=bufferSize){
      //do not include $
      int  iChecksum = 0;
      for (int x=1;x<iSize;x++){
        iChecksum = iChecksum ^ (byte)pPacket[x]; //xor
      }
      char Calc[3];
      sprintf(Calc,"%02X",iChecksum);
      pPacket[iSize]='*';
      pPacket[iSize+1]=Calc[0];
      pPacket[iSize+2]=Calc[1];
      pPacket[iSize+3]=0x0D;
      pPacket[iSize+4]=0x0A;
      bResult=true;
    }
    return bResult;
}

bool CheckSum(char *pPacket){
  bool bResult=false;
  int iSize=strlen(pPacket);
  if (iSize>10){
    //find the asterick
    int iAstericks=0;
    for (int x=0;x<iSize;x++){
      if (pPacket[x]=='*'){
        iAstericks=x;
        break;
      }
    }
    if (iAstericks>0){
      char csIN[3];
      memset (csIN,0,sizeof(csIN));
      csIN[0]=pPacket[iAstericks+1];
      csIN[1]=pPacket[iAstericks+2];
      int  iChecksum = 0;
      for (int x=1;x<iAstericks;x++){
        iChecksum = iChecksum ^ (byte)pPacket[x]; //xor
      }
      char Calc[3];
      sprintf(Calc,"%02X",iChecksum);
      bResult=(*csIN==*Calc);
    }
  }
  return bResult;
}

void ProcessGPS(){
    {
      bool bFullSentence=false;
      if (buildingSentence==false){
        memset(gpsResponse, 0, sizeof(gpsResponse));
        packetOffset = 0;
      }
      while ((gpsSerial.available() > 0)) {
        if (packetOffset>=sizeof(gpsResponse)){
          Serial.println("MEMORY OVERFLOW");
          memset(gpsResponse,0,sizeof(gpsResponse));
          packetOffset=0;
        }
        char c=gpsSerial.read();
        if (c=='$'){ 
          //regardless where a start packet comes in, start building a new sentence, tossing buffer
          buildingSentence=true;
          if (packetOffset!=0){
            //Serial.print("Reset mid-sentence:");
            //Serial.println(packetOffset);
            memset(gpsResponse, 0, sizeof(gpsResponse));
          }
          gpsResponse[0]=c;
          packetOffset=1;
        }
        else{
            gpsResponse[packetOffset]=c;
            if (packetOffset>2){
              if ((gpsResponse[packetOffset-1]==0x0D) && (gpsResponse[packetOffset]==0x0A)){
                 packetOffset=0;
                 buildingSentence=false;
                 bFullSentence=true;
                 break;
              }
            }
            packetOffset++;
        }
      }//while
      
      if (!buildingSentence){
        if (bFullSentence){
          if (CheckSum(gpsResponse)){
            Serial.print("CS+:");
            for (int x=0;x<sizeof(gpsResponse);x++)  {
              if (gpsResponse[x]!=0)
                Serial.print(gpsResponse[x]);
            }
            if (!Spam(gpsResponse)){
              if (!TimeSource(gpsResponse)){
                  if ((timeStatus()!=timeSet)||((now()-lastUpdateGPS>60)))
                    SyncTimeToGPS(gpsResponse);
              }
            }
          }
          else{
            Serial.print("CS-:");
            for (int x=0;x<sizeof(gpsResponse);x++)  {
              if (gpsResponse[x]!=0)
                Serial.print(gpsResponse[x]);
            }
          }
        }
      }
    }
}
