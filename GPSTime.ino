/*
  written by David Dold, 10Jul22
  Native GPS time sync using NEO-6M via SoftwareSerial on UART1

  SUMMARY:
  NEO-GM Unless throttled is taking a sip from a firehouse.
  resyncs internal time with gps/noGPS every x seconds

  Without throtling on an ATMEGA328, requires 100% dedicated processing of GPS data, should the processor get busy, sentences become malformed from dropped characters,
  causing checksums to fail and sentences discarded.
  Uses eeprom to track start types (ephemeral availability) until GPGSA status changes to 2d/3d fix 
  Check processor documentation for supported RX ports
  
  NOTES:
  invariably checksum failing on sentences indicates lost characters and buffer overruns as processor cannot keep up with the volume of data from gps chip.  
  setup() mitigates this via sentence $pubx,40 by suppressing unwanted sentences, and reporting desired sentences every X cycle.
  from what I can tell, slowest cycle rate is every second; way too fast for platform.

  GIVEN:
  1.GPS chipset does not report inaccurate data, unless forced; if a field is present, its current and accurate.
  2.GPS LED does not blink on 2d fix, only 2d fix needed for GPS to retrieve time. 
  3.GPS time accuracy may decay without a fix, without a fix gps chipset uses internal time clock.
  4.GPS chipset power cycle & no fix & previous fix: $GPRMC contains time/date information maintained by BBR -page 27

  Tested for 1410 minutes off BBR, 3d sync; no decay (in seconds).


  $GPTXT,01,01,02,u-blox ag - www.u-blox.com*50
  $GPTXT,01,01,02,HW  UBX-G70xx   00070000 FF7FFFFFo*69
  $GPTXT,01,01,02,ROM CORE 1.00 (59842) Jun 27 2012 17:43:52*59
  $GPTXT,01,01,02,PROTVER 14.00*1E
  $GPTXT,01,01,02,ANTSUPERV=AC SD PDoS SR*20
  $GPTXT,01,01,02,ANTSTATUS=DONTKNOW*33
  $GPTXT,01,01,02,LLC FFFFFFFF-FFFFFFFF-FFFFFFFF-FFFFFFFF-FFFFFFFD*2C

  based on  https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
*/


#include <TimeLib.h>
#include <AltSoftSerial.h>//chipset dependent, use this library if you need to rx from two serial devices (ex: gps and bluetooth)
#include <EEPROM.h>

#define GPS_RX 8  //refer to RX designated ports if using ATMEL328 -other-
#define GPS_TX 9
#define NMEA_CYCLE_REPORT 10  //cycles
#define READ_EVERY 5

typedef struct{
    time_t lastGPSFix;
    char gpsFixType;
}GPSSubSettings, *pGPSSubSettings;

typedef enum{
  GMT=0x47,
  EASTERN=0x45,
  MOUNTAIN=0x4D,
  CENTRAL=0x43,
  ARIZONA=0x41,
  PACIFIC=0x50
}TimeZone, *pTimeZone;

AltSoftSerial gpsSerial(GPS_RX,GPS_TX);
short _lastHourDisplayed=0;
bool _buildingSentence=false;
char _gpsResponse[100]; //no extended ASCII
int _packetOffset=0;
time_t _gpsTime=0;
time_t _lastCycleDevices=0;
char _sprintBuf[50];
char _gpsStatus=0;
bool _startingUp=true;

const char *spamList[]={"GSV","GGA","VTG","GLL"};
const char *throttleList[]={"RMC","GSA"};


void setup() {
  Serial.begin(9600);
  Serial.flush();
  Serial.println(F("\r\nStarting"));

  //Using processor eeprom to track the age and internal time decay from the last 2d/3d fix
  //asper spec (pape 29) gps chipset startup hot/warm with no fix and ephemerals available from BBR - battery backup ram (app four hours since fix).
  //HOWEVER, when no fix and continual power, internal clock ia used for time infomation regardless of ephemeral state, having the potential for decay.

  GPSSubSettings settings={0,0};
  bool bEEConfigured = false;
  //comment out next line to reset memory
  EEPROM.get(0, bEEConfigured);
  if (!bEEConfigured){
      Serial.println(F("Updating EEProm"));
      EEPROM.put(0,true);
      EEPROM.put(sizeof(bool), settings);
  }
  else{
      EEPROM.get(sizeof(bool), settings);
  }
  _gpsTime=settings.lastGPSFix;

  pinMode(GPS_RX, INPUT);
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_RX,LOW);

  
  gpsSerial.begin(9600);
  gpsSerial.flush();
  _buildingSentence=false;


  delay(100);
  Serial.println(F("Init nmea packets and cycles"));  //page 82
  delay(100);
  unsigned int iRowCount = sizeof(spamList) / 2;

  SetSentences(spamList, &iRowCount, 0);
  //remove throttle for startup time to sync with GPS Device
  iRowCount = sizeof(throttleList) / 2;
  SetSentences(throttleList, &iRowCount, 1);


}

void loop() {

  if (gpsSerial.available() > 0)
    ProcessGPS();

  if (_startingUp) {
    if (timeStatus() == timeSet) {
      int iRowCount = sizeof(throttleList) / 2;
      SetSentences(throttleList, &iRowCount, NMEA_CYCLE_REPORT);
      _startingUp = false;
   }
   else if ((millis() % 5000) == 0) {
      Serial.println(F("No BBR/GPS time, position device for a 2d/3d fix"));
    }
  }
  else {
    time_t nowT=now();
    if (nowT - _lastCycleDevices >= READ_EVERY) {
      DisplayTime(&nowT);
      _lastCycleDevices = nowT;
    }
  }
}

bool Spam(char *pResponse) {
  //correct for spam detected if NEO cycles power
  bool bResult = false;
  int iRowCount = sizeof(spamList) / 2;
  /*
    sizeof decays to pointers in the rowCount * sizeof pointer in second element
    impossible to determing sizeof spamlist without counting the sizeof each element.
  */

  for (int x = 0; x < iRowCount; x++) {
    char *found = strstr(pResponse, spamList[0, x]);
    if (found) {
      Serial.println(F("SPAM DETECTED"));
      unsigned int iRowCount = sizeof(spamList) / 2;
      SetSentences(spamList, &iRowCount, 0);
      iRowCount = sizeof(throttleList) / 2;
      SetSentences(throttleList, &iRowCount, NMEA_CYCLE_REPORT);
      break;
    }
  }
  return bResult;
}

void SetSentences(char **pList,int *pRowCount, char cycleSetting) {
  if (cycleSetting == 0)
    Serial.println(F("BLOCKING:"));
  else if (cycleSetting == 1)
    Serial.println(F("DEFAULT RATE:"));
  else
    Serial.println(F("THROTTLING:"));
  for (int x = 0; x < *pRowCount; x++) {
    memset(_sprintBuf, 0, sizeof(_sprintBuf));
    sprintf(_sprintBuf, "$PUBX,40,%s,0,%d,0,0,0,0", pList[0, x], cycleSetting);
    if (SetCheckSum(_sprintBuf, sizeof(_sprintBuf))) {
      Serial.print(_sprintBuf);
      gpsSerial.write(_sprintBuf);
    }
  }
}


void ProcessGPS() {
  bool bFullSentence = false;
  if (_buildingSentence == false) {
    memset(_gpsResponse, 0, sizeof(_gpsResponse));
    _packetOffset = 0;
  }


  while ((gpsSerial.available() > 0)) {
    if (_packetOffset >= sizeof(_gpsResponse)) {
      Serial.println(F("GPS MEMORY OVERFLOW"));
      memset(_gpsResponse, 0, sizeof(_gpsResponse));
      _packetOffset = 0;
    }
    char c = gpsSerial.read();
    if (c == '$') {
      //regardless where a start packet comes in, start building a new sentence, tossing buffer
      if (_packetOffset != 0) {
        Serial.print("GPS BUFFER RESET:");
        for (int x = 0; x < sizeof(_gpsResponse); x++)  {
          if (_gpsResponse[x] != 0)
            Serial.print(_gpsResponse[x]);
        }
        Serial.println();
      }
      _buildingSentence = true;
      if (_packetOffset != 0) {
        memset(_gpsResponse, 0, sizeof(_gpsResponse));
      }
      _gpsResponse[0] = c;
      _packetOffset = 1;
    }
    else {
      _gpsResponse[_packetOffset] = c;
      if (_packetOffset > 2) {
        if ((_gpsResponse[_packetOffset - 1] == 0x0D) && (_gpsResponse[_packetOffset] == 0x0A)) {
          _buildingSentence = false;
          bFullSentence = true;
          break;
        }
      }
      _packetOffset++;
    }
  }//while

  if (!_buildingSentence) {
    if (bFullSentence) {
      if (CheckSum(_gpsResponse)) {
        Serial.print(F("CS+:"));
        for (int x = 0; x < sizeof(_gpsResponse); x++)  {
          if (_gpsResponse[x] != 0)
            Serial.print(_gpsResponse[x]);
        }
        if (!Spam(_gpsResponse)) {
          if (!TimeSource(_gpsResponse)) 
              SyncTimeToGPS(_gpsResponse);
        }
      }
      else {
        Serial.print(F("CS-:"));
        for (int x = 0; x < sizeof(_gpsResponse); x++)  {
          if (_gpsResponse[x] != 0)
            Serial.print(_gpsResponse[x]);
        }
      }
      memset(_gpsResponse, 0, sizeof(_gpsResponse));
      _packetOffset = 0;
    }
  }
}

bool SyncTimeToGPS(char *pResponse) {
  //$GPRMC,184548.00,A,3245.73987,N,11709.12812,W,1.939,,160722,,,A  page 63
  //asper spec device only sends valid data, otherwise field is empty
  //String free
  bool bResult = false;
  char *pFound = strstr(pResponse, "$GPRMC,"); 
  if (pFound != NULL) {
    //_gpsStatus &=~(1UL<<0); //set bit 0 to 0 
    tmElements_t gpsElements = {0, 0, 0, 0, 0, 0, 0};

    int iOffset=0;
    for (iOffset = 0; iOffset < 9; iOffset++) {
      //find the first comman
      if (pFound[iOffset]==',')
        break;
    }  
    iOffset++; //advance over the comma
    if (iOffset==7){  //sanity check
      char subString[3];
      memset(subString,0,3);
      subString[0]=pFound[iOffset];
      subString[1]=pFound[iOffset+1];
      gpsElements.Hour=atoi(subString);
      subString[0]=pFound[iOffset+2];
      subString[1]=pFound[iOffset+3];
      gpsElements.Minute=atoi(subString);
      subString[0]=pFound[iOffset+4];
      subString[1]=pFound[iOffset+5];
      gpsElements.Second =atoi(subString);
    
      //advance nine commas (fields) to the date field
      int iCommaCount=0;
      for (iOffset;iOffset<strlen(pFound);iOffset++){
        if (pFound[iOffset]==',')
          iCommaCount++;
        if (iCommaCount==8)
          break;
      }
      if (iCommaCount==8){  //another sanity check
        iOffset++; //advance over the comma
        subString[0]=pFound[iOffset];
        subString[1]=pFound[iOffset+1];
        gpsElements.Day=atoi(subString);
        subString[0]=pFound[iOffset+2];
        subString[1]=pFound[iOffset+3];
        gpsElements.Month=atoi(subString);
        subString[0]=pFound[iOffset+4];
        subString[1]=pFound[iOffset+5];
        gpsElements.Year = atoi(subString) + 30; //based off year=0 1971, year=2 1972  year=30 2000
        if ((gpsElements.Month>0) && (gpsElements.Day>0) && (gpsElements.Year>30))
        {
          //sprintf(_sprintBuf,"Elements: %d %d %d %d %d %d",gpsElements.Month, gpsElements.Day, gpsElements.Year, gpsElements.Hour, gpsElements.Minute, gpsElements.Second);
          //Serial.println(_sprintBuf);
  
          time_t tempTime = makeTime(gpsElements);
          if (tempTime > 0) {
              //_gpsStatus |=1UL<<0;  //set bit 0 to 1 (RMC)
              //bool bbr=(_gpsStatus >> 1) & 1U;
              
              _gpsStatus=bitSet(_gpsStatus,0);
              bool bbr=bitRead(_gpsStatus,1);
              bool fix2=bitRead(_gpsStatus,2);
              bool fix3=bitRead(_gpsStatus,3);

              if (fix2 || fix3)
                _gpsTime=tempTime;
              
              if (bbr || fix2 || fix3){
                if (timeStatus() != timeSet) {
                  if (bbr)
                    Serial.println(F("TimeSet using BBR"));
                  else if (fix2)
                    Serial.println(F("TimeSet using GPS:2d"));
                  else if (fix3)
                    Serial.println(F("TimeSet using GPS:3d"));
                  setTime(tempTime);
                }
                else {
                  int decay = tempTime - now();
                  if (decay != 0) {
                    setTime(tempTime);
                    //memset(_sprintBuf, 0, sizeof(_sprintBuf));
                    Serial.print(F("TIME mismatch: "));
                    sprintf(_sprintBuf, "%d seconds, resync using:", decay);
                    Serial.print(_sprintBuf);
                    if (bbr)
                      Serial.println(F("TimeSet using BBR"));
                    else if (fix2)
                      Serial.println(F("TimeSet using GPS:2d"));
                    else if (fix3)
                      Serial.println(F("TimeSet using GPS:3d"));
                   }
                 }
                 if (fix2 || fix3){
                    //store the last fix time and type
                    GPSSubSettings subSettings={_gpsTime,0};
                    if (fix2)
                      subSettings.gpsFixType=0x32;
                    else
                      subSettings.gpsFixType=0x33;
                    EEPROM.put(sizeof(bool), subSettings);
                  }
                }
                else
                  Serial.println(F("TimeSet pending GPGSA"));
            }
            bResult = true;
          }
          else{
            Serial.println(F("SyncTimeToGPS:  makeTime failed"));
          }
        }
        else{
          Serial.println(F("Malformed or missing RMC date information"));
        }
      }
  }
  return bResult;
}

bool TimeSource(char *pResponse) {
  bool bResult = false;
  //comma counts are variable bassed on fixed satellite list after the ,A,(123)...
  //$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30   page 60
  //$GPGSA,A,3,02,11,29,20,31,25,18,129,0.99,1.38*07
  //datetime from warm start until at 2d/3d fix in GPGSA

  char *found = strstr(pResponse, "$GPGSA"); //this could be anywhere in the buffer -snd- there may be more than one instance
  if (found != NULL) {
      //String sGSA = pResponse;
      char fixStatus = pResponse[9];
      //_gpsStatus &=~(1UL<<1); //set bit 2 to 0   -- so messy, use a macro
      //_gpsStatus &=~(1UL<<2); //set bit 3 to 0 
      //_gpsStatus &=~(1UL<<3); //set bit 3 to 0 
      //_gpsStatus |=1UL<<1;  //set bit 1 to 1

      _gpsStatus=bitClear(_gpsStatus,1);
      _gpsStatus=bitClear(_gpsStatus,2);
      _gpsStatus=bitClear(_gpsStatus,3);


      if (fixStatus == '1')
        _gpsStatus=bitSet(_gpsStatus,1);
      else if (fixStatus == '2')
        _gpsStatus=bitSet(_gpsStatus,2);
      else if (fixStatus=='3')
        _gpsStatus=bitSet(_gpsStatus,3);
      else{
        Serial.print(F("TimeSource, unexpected char in [9]:"));
        Serial.println(fixStatus);
      }
      bResult = true;
  }
  return bResult;
}


bool SetCheckSum(char *pPacket, int bufferSize) {
  bool bResult = false;
  //packet sent with five extra bytes -and- without an asterick  0x26 CR_A CR_B 0x0d 0x0a
  int iSize = strlen(pPacket);
  if ((iSize + 5) <= bufferSize) {
    //do not include $
    int  iChecksum = 0;
    for (int x = 1; x < iSize; x++) {
      iChecksum = iChecksum ^ (char)pPacket[x]; //xor
    }
    char Calc[3];
    sprintf(Calc, "%02X", iChecksum);
    pPacket[iSize] = '*';
    pPacket[iSize + 1] = Calc[0];
    pPacket[iSize + 2] = Calc[1];
    pPacket[iSize + 3] = 0x0D;
    pPacket[iSize + 4] = 0x0A;
    bResult = true;
  }
  return bResult;
}

bool CheckSum(char *pPacket) {
  bool bResult = false;
  int iSize = strlen(pPacket);
  if (iSize > 10) {
    //find the asterick
    int iAstericks = 0;
    for (int x = 0; x < iSize; x++) {
      if (pPacket[x] == '*') {
        iAstericks = x;
        break;
      }
    }
    if (iAstericks > 0) {
      char csIN[3];
      memset (csIN, 0, sizeof(csIN));
      csIN[0] = pPacket[iAstericks + 1];
      csIN[1] = pPacket[iAstericks + 2];
      int  iChecksum = 0;
      for (int x = 1; x < iAstericks; x++) {
        iChecksum = iChecksum ^ (char)pPacket[x]; //xor
      }
      char Calc[3];
      sprintf(Calc, "%02X", iChecksum);
      bResult = (*csIN == *Calc);
    }
  }
  return bResult;
}

void DisplayTime(time_t *pTime) {

  unsigned int iElapsedMinutes=0;
  char fix=0x64;
  char fixType;

  bool rmc=(_gpsStatus >> 0) & 1U;
  bool bbr=(_gpsStatus >> 1) & 1U;
  bool fix2=(_gpsStatus >> 2) & 1U;
  bool fix3=(_gpsStatus >> 3) & 1U;

  if (rmc){
    if (bbr){
      iElapsedMinutes = (*pTime - _gpsTime) / 60;
      fix=0x20;
      fixType=0x42;
    }
    else if (fix2)
      fixType=0x32;
    else if (fix3)
      fixType=0x33;
    sprintf(_sprintBuf, "GMT:%02d/%02d/%02d %02d:%02d:%02d,Decay:%um,FixType:%c%c", month(*pTime), day(*pTime), year(*pTime), hour(*pTime), minute(*pTime), second(*pTime),iElapsedMinutes,fixType,fix);
    Serial.println(_sprintBuf);
    sprintf(_sprintBuf, "GMT:%lu,D:%u,FIX:%c%c",*pTime,iElapsedMinutes,fixType,fix);
  }
  else
    Serial.println(F("GMT:GPRMC MISSING"));
}

time_t LocalTime(TimeZone *pTZ){
  //forget DST for now.
  time_t t = now();
  switch (*pTZ) {
    case PACIFIC:
      t -= (7 * 3600);
      break;
    case EASTERN:
      t -= (4 * 3600);
      break;
    case CENTRAL:
      t -= (5 * 3600);
      break;
    case ARIZONA:
      t -= (7 * 3600);
      break;
    case MOUNTAIN:
      t -= (6 * 3600);
      break;
    case GMT:
      break;
  }
  return t;  
}
