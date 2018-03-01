/*  ================================
 *  Frabrication d'un logger IGC avec stm32, bmp180, module gps et stockage en SPI sur carte SD
 *  GPS sur Serial2, console sur Serial, BPM180 sur I2c1, SD sur SPI1
 *  Serial correspond sur un stm32 à : Pin27-PA10 = com1RX   Pin26-PA9=com1TX     [console et pgm]
 *  Serial1 correspond sur un stm32 à : Pin8-PA3 = com2RX   Pin7-PA2=com2TX       [HC-06 115200]
 *  Serial2 correspond sur un stm32 à : Pin16-PB11 = com3RX   Pin15-PB10=com3TX   [GPS 6900b]
 *  I2C correspond sur un stm32 à : P34 PB6 SCL    P35 PB7 SDA
 *  SPI1 correspond sur un stm32 à : P9 PA4 SS,  P10 PA5 SCK, P11 PA6 MISO, P12 PA7 MOSI 
 *  -------------------------------- OLED SPI -----------------------------------------------------
 *  #define OLED_DC PB15  #define OLED_MOSI PB14   #define OLED_CLK PB13   #define OLED_CS  PB12 
 *  ------------- upload method :serial   generic stm32 -------------------------------------------
 * Serial.println("Monitoring satellite location and signal strength using TinyGPSCustom");
 * Serial.print("Testing TinyGPS++ library v. "); Serial.print(TinyGPSPlus::libraryVersion());
 * Serial.println(" by Mikal Hart");Serial.println();
 * 
 * The resulting B Record becomes (with spaces for clarity in this example):
 * BHH MM SS DD MMMMM N DDD MMMMM E V PPPPP GGGGG AAA SS NNN RRR CR LF"
 * B07 43 22 45 35552 N 005 51724 E A 00000 00322
 * B13 24 54 45 33430 N 005 58763 E A 00000 00285 008 00 000 000
 * B0743224535552N00551724EA0000000322   =>result de ce programme au 03/02/17
 * B1324544533430N00558763EA000000028500800000000 => trame oudie
 * -------------------------------------------------------------------------------------------- 
 *                                          TRAMES GPS
 *  $GPVTG,,T,,M,0.879,N,1.627,K,A*27
 *  $GPGGA,121249.00,4535.55965,N,00551.72597,E,1,07,1.55,332.0,M,47.3,M,,*57
 *  $GPGSA,A,3,13,30,19,24,28,15,17,,,,,,2.60,1.55,2.09*0B
 *  $GPGSV,3,1,12,05,02,185,,10,10,326,,12,19,213,,13,61,135,26*79
 *  $GPGSV,3,2,12,15,76,258,30,17,31,098,32,18,24,301,18,19,23,124,20*7E
 *  $GPGSV,3,3,12,20,25,226,,24,43,280,22,28,30,047,19,30,06,081,16*7B
 *  $GPGLL,4535.55965,N,00551.72597,E,121249.00,A,A*66
 *  $GPRMC,121250.00,A,4535.55919,N,00551.72590,E,0.306,,240317,,,A*7D
 *
 *  The A Record must be the first record in an FR Data File,
 *  A RECORD - FR ID NUMBER
 *  
 *  Voir http://carrier.csi.cam.ac.uk/forsterlewis/soaring/igc_file_format/igc_format_2008.html#link_3.1
 *  3.3.1 Required records. The following H records are required, in the order given below:
 *  HFDTEDDMMYY
 *  HFFXAAAA
 *  HFPLTPILOTINCHARGE:TEXTSTRING
 *  HFCM2CREW2:TEXTSTRING
 *  HFGTYGLIDERTYPE:TEXTSTRING
 *  HFGIDGLIDERID:TEXTSTRING
 *  HFDTMNNNGPSDATUM:TEXTSTRING
 *  HFRFWFIRMWAREVERSION:TEXTSTRING
 *  HFRHWHARDWAREVERSION:TEXTSTRING
 *  HFFTYFRTYPE:MANUFACTURERSNAME,FRMODELNUMBER
 *  HFGPS:MANUFACTURERSNAME,MODEL,CHANNELS,MAXALT(AL3)
 *  HFPRSPRESSALTSENSOR:MANUFACTURERSNAME,MODEL,MAXALT(AL3) *  
 *  
 *  Voir  http://carrier.csi.cam.ac.uk/forsterlewis/soaring/igc_file_format/igc_format_2008.html#link_2.5 
 *  Short file name style: YMDCXXXF.IGC
 *  Y = Year; value 0 to 9, cycling every 10 years
 *  M = Month; value 1 to 9 then A for 10, B=11, C=12.
 *  D = Day; value 1 to 9 then A=10, B=11, C=12, D=13, E=14, F=15, G=16, H=17, I=18, J=19, K=20, L=21,
 *  M=22, N=23, O=24, P=25, Q=26, R=27, S=28, T=29, U=30, V=31.
 *  C = manufacturer's IGC code letter (see table below)
 *  XXX = unique FR Serial Number (S/N); 3 alphanumeric characters
 *  F = Flight number of the day; 1 to 9 then, if needed, A=10 through to Z=35
 *  
 *  to do:
 *     mini prise capteur gps
 *     mini inter valid BT
 *     mémorisation param => init.cfg
 *     liste des planeurs => immat.cfg 
 *     saisie manuelle
 ===========================================================================================*/

#include <TinyGPS++.h>

#include <SD.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LEDB PC13   // on board blue led
#define GNDP PC14   // "GND" du BP du rot
#define PinR PC15   //  push but du codRotat
#define PinA PA0   //  1ere sortie du codeur
//#define GNDB PA12   // "GND" codRotat
#define PinB PA1   //  2e sortie du codeur
/*#define PinA PB14   //  1ere sortie du codeur
#define GNDB PB13   // "GND" codRotat
#define PinB PB12   //  2e sortie du codeur*/

//#define OLED_RESET 4  // ***** FOR I2C ****

// If using software SPI (the default case):
#define OLED_DC    PB15
#define OLED_MOSI  PB14
#define OLED_CLK   PB13
#define OLED_CS    PB12 
#define OLED_RESET PB8

#define TEMPO1 5000
#define TEMPO60 60000 // 1'

String strVersion = "igcTrac_Stm32-bme280-oled-bt-v0.1.4";
String strGrx;
String strNmeaSel;  //$GPVTG, $GPGGA, $GPGSA, $GPGSV, $GPGLL, $GPRMC

volatile boolean mouvement;
volatile boolean up;
volatile boolean ButPushed;
volatile boolean blFileExist;
volatile boolean blSDvalid = false; // carte SD présente
volatile boolean blBmeValid;        // capteur BME280 detecté
volatile boolean blDolDetect;       //  debut de tramme NMEA ( "$")

//static const uint32_t GPSBaud = 9600; 
static const int MAX_SATELLITES = 40;

int intQnh = 101350; //Pa
int intQnhAdj = 50;
int intPalt;              // Altitude pression from BME280
int intVisMod = 0;        // 0 standard, 1 vitesse, 2 auto (if V > xx kmh)
int intMode = 0;          // 0=baro, 1 sto request,
int inByte = 0;           // For incoming serial data
int intRxGps;
int intStoSD = 1;         // Intervalle de stockage sur SD
int intSprf;              // For conv to string
int intAfs;               // AskForSetup
int intBtMod = 1;         // Bluetooth mode on Serial1 => 0:off, 1:console , >1 :out NMEA

long lngTempo1;

char chrGAlt[6] = "GGGGG";
char chrPalt[6] = "PPPPP";
char chrGAltDisp[5] = "0000";
char chrPaltDisp[5] = "0000";
char chrTime[7]; char chrTim2[9];
char chrDate[7]; 
char chrIgcFile[13];
char chrHFDTE[12] = "HFDTE******";
char chrDTE[7] = "******";
char chrLng[10] = "*********";
char chrLat[9] = "********";
char chrV[2] = "*";
char chrRxGps[10];
char chrQnh[7]; // = "101300 Pa";
char chrIgcFileName[13] = "711XJDB0.IGC";

File myFile;

Adafruit_BME280 bme; // I2C
TinyGPSPlus gps;  //The TinyGPS++ object
//Adafruit_SSD1306 display(OLED_RESET);  // ***** FOR I2C ****
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

/*==============================================================================*/

void setup(){    
  Serial.begin(115200); delay(300);
  Serial.println();Serial.println();
  Serial.println(strVersion);
  Serial.println("Init GPIO");  

  if (intBtMod > 0 ){
    Serial1.begin(115200);
    Serial1.println();
    Serial1.println(strVersion);   
  } else Serial1.end();
    
  
  Serial.println("Init Serial2 9600B GPS");
  Serial2.begin(9600); //GPSBaud);
  
  // interruption sur front descendant
  attachInterrupt (PinA, intCodRot, FALLING); 
  attachInterrupt (PinR, intPushBut, FALLING);
    
  //pinMode(GNDB, OUTPUT); digitalWrite (GNDB, LOW);      // butEncod  "Gnd"
  pinMode(GNDP, OUTPUT); digitalWrite (GNDP, LOW);      // Push butEncod  "Gnd"
  pinMode(PinA, INPUT_PULLUP);  
  pinMode(PinB, INPUT_PULLUP);
  pinMode(PinR, INPUT_PULLUP);
  pinMode(LEDB,OUTPUT); // blue led on stm32
  
  Serial.println("Init OLED Display");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE); 
  display.println("Logger Aliantux");
  display.display();
  
  Serial.print("Init SD card...");
  display.print("SD card...");
  if (!SD.begin(4)) {
    Serial.println("FAILED!"); 
    display.println("KO");
  }
  else {
    Serial.println("done.");    
    display.println("OK");
    blSDvalid = 1;
  }
  
  display.display(); 
  
  Serial.print("Init BME280...");  
  display.print("BME280...");
  if (!bme.begin()) {    
    display.println("KO");
    Serial.println("FAILED!"); 
    blBmeValid = false;
  }
  else {
    Serial.println("done");    
    display.println("OK");
    blBmeValid = true;
  }
    
  display.display(); 
  
  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i=0; i<4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }   
  
  //if (gps.location.isValid()){
    if (Serial2.available() > 0){
    Serial.println("Valid GPS datas");
    display.println("Valid GPS datas");
  }
  else {
    Serial.println("NO GPS datas");    
    display.println("NO GPS datas");

  } 
     
  display.display(); 
  delay (3000);
  
fAdjSetup(); // adj qnh, afMode, storing intervall, Bluetooth on/off
}  

/*========================= START LOOP PGM ======================================*/

void loop() {

  // Dispatch incoming characters
  if (Serial2.available() > 0)
  {
    //gps.encode(Serial2.read());    
    intRxGps = Serial2.read();
    gps.encode(intRxGps);
    if (fTraitGprmc(intRxGps)){
      blDolDetect = false;
      if (intBtMod > 1){
        if (strGrx.substring(0,6) == fStrNmea(intBtMod)) Serial1.println(strGrx);
      }
    }

    if (totalGPGSVMessages.isUpdated()) {
      fFillGpsDatas();

      int totalMessages = atoi(totalGPGSVMessages.value());
      int currentMessage = atoi(messageNumber.value());
      if (totalMessages == currentMessage) {  
        digitalWrite(LEDB,LOW);
        String strB="B";
        fCalcTime();        // Time UTC  6 bytes HHMMSS
        strB += chrTime;
        fCalcLay();         // Latitude  8 bytes DDMMmmmN/S
        strB += chrLat;       
        fCalcLng();         // Longitude  9 bytes DDDMMmmmE/W
        strB += chrLng;
        fCalcV();           // validity A=3D  V=2D
        strB += chrV;
        fCalcPh();          // Press Alt. 5 bytes PPPPP
        strB += chrPalt; 
        fCalcGh();          // GNSS Alt.  5 bytes GGGGG 
        strB += chrGAlt;  
        fSerPrint(strB,0);
        fStoSD(strB);       // Test de et stockage sur SD        
        fDispOled();        // Affichage sur OLED
        fReadPushBut();     // test bp appuyé
        fTtestCloseSD();    // vérif si mode pour fermer le fichier en cours   
        if (fTestAFS()) fAdjSetup() ;    // if pushBut > 5sec reInit AskForSetup
        if(intBtMod > 1) fTestModifNMEA();
        //HeartBeat
        digitalWrite(LEDB,HIGH);
      }
      for (int i=0; i<MAX_SATELLITES; ++i){
        sats[i].active = false;
      }  
    }
  }
  if ( gps.charsProcessed() < 10){
    Serial.println("No GPS data received: check wiring");
  }
}
/*========================= END LOOP PGM ======================================*/


/******************************
 * routines d'interruptions   *
 *****************************/

// routine déclanchée quand le signal passe de haut a bas sur A du codeur rotatif
void intCodRot ()  {
  if (digitalRead(PinA))
    up = !digitalRead(PinB);
  else
    up = digitalRead(PinB);
  mouvement = true;
}

// routine déclanchée quand le signal passe de haut a bas sur BP codeur rotatif
void intPushBut ()  {
  ButPushed=true;
}

/******************************************************************\
 *                   Déclaration des fonctions                     *
 ******************************************************************/

/************* Récup Datas des sats *********************/
static void fFillGpsDatas(){
  for (int i=0; i<4; ++i) {
    int no = atoi(satNumber[i].value());
    //Serial.print("SatNumber is "); Serial.println(no);
    if (no >= 1 && no <= MAX_SATELLITES) {
      sats[no-1].elevation = atoi(elevation[i].value());
      sats[no-1].azimuth = atoi(azimuth[i].value());
      sats[no-1].snr = atoi(snr[i].value());
      sats[no-1].active = true;
    }
  }
}

/******************************************************************\
 *                 Print Serial andSerial1 if BT selected          *
 ******************************************************************/
static void fSerPrint(String strSerial,int intLN){
  Serial.print(strSerial); 
  if (intBtMod == 1) Serial1.print(strSerial); 
  if (intLN){
    Serial.println();
    if (intBtMod == 1) Serial1.println(); 
  }
}
 
/*****************************************************************\
* ouverture du fichier passé en paramètre et écriture entete IGC  *
\*****************************************************************/
static void openIgcFic(String strHFDTE){ 
  String strFicIgc = fwFileName() + ".IGC"; 
  fSerPrint("Openning " + strFicIgc +"...",0);
  myFile = SD.open(strFicIgc, FILE_WRITE);
  if (myFile) {   
    myFile.println("AJDB001STM32-Lgr");
    myFile.println(strHFDTE);                         // HFDTEDDMMYY
    myFile.println("HFFXA015");                       //  fix accuracy 15 meters
    myFile.println("HFPLTPILOT:BERTI JEAN-DOMINIQUE");
    myFile.println("HFCM2CREW2:");
    myFile.println("HFGTYGLIDERTYPE:");
    myFile.println("HFGIDGLIDERID:");
    myFile.println("HFDTM100GPSDATUM:WGS-1984");
    myFile.println("HFRFWFIRMWAREVERSION:0.1.2");
    myFile.println("HFRHWHARDWAREVERSION:0.1.");
    myFile.println("HFFTYFRTYPE:Aliantux,STM32");
    myFile.println("HFGPS:Generic GPS");
    myFile.println("HFPRSPRESSALTSENSOR:Bosch, BME280");
    //myFile.println("HFCIDCOMPETITIONID:");
    //myFile.println("HFCCLCOMPETITIONCLASS:");
    //myFile.println("HFTZNTIMEZONE:2.00");
    fSerPrint("done",1);
  }
  else {
    fSerPrint("FAILED " + myFile,1);
    /*Serial.print("FAILED ");Serial.println(myFile);*/
    //Pas pu ouvrir le fichier => RAZ bool SD Valid
    blSDvalid = 0;
    intMode = 0;
  }
}

/****************************\ 
*  Time UTC  6 bytes HHMMSS *
*****************************/
static void fCalcTime(){
  int intYear = gps.date.year()-(int)2000;
  intSprf=sprintf(chrTime, "%02d%02d%02d", gps.time.hour(),gps.time.minute(),gps.time.second());
  intSprf=sprintf(chrTim2, "%02d:%02d:%02d", gps.time.hour(),gps.time.minute(),gps.time.second());
  intSprf=sprintf(chrDate, "%02d%02d%02d", gps.date.month(), gps.date.day(), intYear);
  intSprf=sprintf(chrDTE, "%02d%02d%02d", gps.date.day(), gps.date.month(), intYear);
  intSprf=sprintf(chrHFDTE, "HFDTE%02d%02d%02d", gps.date.day(), gps.date.month(), intYear);
  intSprf=sprintf(chrIgcFile, "%02d%02d%02d%02d", gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute());
}
  
/***********************************\ 
*  Longitude  9 bytes DDDMMmmmE/W  *
************************************/
static void fCalcLng(){
 char strEW [2]; char strLngDeg [4]; char strLngDec [7]; 
 if (gps.location.isValid()) {
    float lnga;
    float lng =gps.location.lng();
    if (lng < 0) { 
      lnga = lng * -1;
      intSprf = sprintf(strEW,"%s","W");
    }
    else {
      lnga = lng;
      intSprf = sprintf(strEW,"%s","E");
    }
    int lngint=(int)lnga;
    intSprf = sprintf (strLngDeg,"%03d",lngint);
    float lngDec=(lnga-lngint)*60000;
    intSprf = sprintf (strLngDec,"%5.0f",lngDec);
    intSprf = sprintf(chrLng,"%s%s%s", strLngDeg,strLngDec,strEW);
  }
}

/********************************\ 
*  Latitude  8 bytes DDMMmmmN/S  *
*********************************/  
static void fCalcLay(){
  char strLatDeg [3]; char strLatDec [7];  
  if (gps.location.isValid()) {
    float lat =gps.location.lat();
    int latint=(int)lat;
    intSprf = sprintf (strLatDeg,"%02d",latint);
    float latDec=(lat-latint)*60000;
    intSprf = sprintf (strLatDec,"%5.0f",latDec);
    intSprf = sprintf(chrLat,"%s%sN", strLatDeg,strLatDec);
  }
}

/************************************************************\ 
* Fix validity  1 byte. A or V                               *
/ Use A for a 3D fix and V for a 2D fix (no GPS altitude)    * 
*************************************************************/
static void fCalcV(){
  if (gps.satellites.value() > 3)   
      {intSprf = sprintf(chrV,"%s", "A");}                
   else
      {intSprf = sprintf(chrV,"%s", "V");}
}

/*****************************\ 
*  Press Alt. 5 bytes PPPPP   *
******************************/  
static void fCalcPh() {        
  if (blBmeValid) {
    //int Palt=(int)(bme.readAltitude(qnh*100));
    intPalt = (int)(bme.readAltitude(intQnh/100));
    intSprf = sprintf (chrPalt,"%05d",intPalt);
    intSprf = sprintf (chrPaltDisp,"%04d",intPalt);
  }
}

/*****************************\ 
*  GNSS Alt.  5 bytes GGGGG   *
******************************/        
static void fCalcGh() {       
  int Alt = (int)gps.altitude.meters(); 
  intSprf = sprintf (chrGAlt, "%.05d",Alt);
  intSprf = sprintf (chrGAltDisp, "%.04d",Alt);
}

/*****************\ 
* test pushButton *
******************/  
static void fReadPushBut() {
  //
  if(digitalRead(PinR)) {
    if (ButPushed){
      ButPushed=false;
      intMode +=1;
      if (intMode >= 3) intMode = 0;
    }
    intAfs=0; 
  } else intAfs +=1;
  ButPushed=false;
  //Serial.print(" intAfs: ");Serial.println(intAfs);
}
/****************\ 
* afficheur OLED *
*****************/  
// display.print("012345678901234567890");// capacité en taille 1: 21 caractères  

static void fDispOled(){
  if (intVisMod == 2 && gps.speed.kmph() > 5 )fDispSpeed();   // commut auto sur vit si >5 kmph
  else if (intVisMod == 1) fDispSpeed();                      // aff vitesse        
  else fFillDisp();                                       // aff standard
  //Serial.println(); 
}

static void fdispYelline() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Ph:");
  display.print(chrPaltDisp);
  display.print("m Gh:");
  display.print(chrGAltDisp); 
  display.print("m ");  
  if (myFile){
    display.print(intStoSD);   // le pas de stockage
    if (gps.time.second() % intStoSD == 0){
       display.print("*");
    } 
  }
  display.setCursor(0,9);
  //display.setTextColor(BLACK, WHITE); // 'inverted' text 
}

static void fDispSpeed() {
  fdispYelline();
  display.setCursor(0,10);  
  display.setTextSize(3);  
  display.print(int(gps.speed.kmph()));
  // 
  display.setTextSize(1);  
  display.setCursor(100,24);
  display.print("kmph");  
  display.display();
}

//------------------
static void fFillDisp() { 
  fdispYelline();
  display.print( "QNH:");
  display.print(fPaToHpa(intQnh));
  display.print(" hPa  BT:");    display.println(intBtMod);
  display.print(chrDTE);display.print(" ");
  display.print(chrTim2);display.print(" ");
  display.print(int(gps.speed.kmph()));display.println("k");
  display.print("Sats="); display.print(gps.satellites.value());
  display.print(" Mod="); display.print(intMode);
  if (blSDvalid){
    display.print(" SD=1");                 // si une SD est présente
    if (gps.location.isValid()){            // si datas valides
      if (intMode == 1){                    // si mode stockage selectionné
        if (gps.time.second() % intStoSD == 0){
          // myFile.println(strB0);                // on rajoute une ligne
          // Serial.print(" stored");
          display.print(" Sto");
        }
      }
    }
  }
  else{
    display.print(" SD=0");
  }   
  // affichage 
  display.display();   
}

/*********************************\ 
* test SD OPEn and sto if mode =1 *
**********************************/  
static void fStoSD(String strB0){
 if (blSDvalid){                            // si une SD est présente
    if (gps.location.isValid()){            // si datas valides
      if (intMode == 1){                    // si mode stockage selectionné
        if (!myFile) {                      // si fichier pas ouvert
          openIgcFic(chrHFDTE); // on ouvre
        }
        if (gps.time.second() % intStoSD == 0){ // sto f(pas selectionné)
          myFile.println(strB0);                // on rajoute une ligne
          fSerPrint(" stored",0);
        }
      }
      
    }
  }
  fSerPrint("",1);
}

/****************\ 
* test modif QNH *
*****************/  
static void ftestAdjQNH(){
  if (mouvement)  {       // on a détecté une rotation du bouton
    if (up) {
      intQnh +=intQnhAdj;
    }
  else {
    intQnh -= intQnhAdj;
  }
  fSerPrint("new QNH :" + fPaToHpa(intQnh) + " hPa, Alt:" + String(bme.readAltitude(intQnh/100)) + " m" ,1); 
  lngTempo1=millis();
  mouvement = false;
  }  
}

/****************\ 
* test modif NMEA *
*****************/  
static void fTestModifNMEA(){
  if (mouvement)  {       // on a détecté une rotation du bouton
    if (up) {
      intBtMod += 1;
      if (intBtMod >7) intBtMod =2;
    }
  else {      
    intBtMod -= 1;
    if (intBtMod <2) intBtMod =7;
  }
  Serial1.println(); Serial1.print("New NMEA Mode : ");Serial1.print(intBtMod);
  Serial1.print(" => OUT "); Serial1.println(fStrNmea(intBtMod));
  mouvement = false;
  }  
}

/****************\ 
* test fermeture *
*****************/  
static void fTtestCloseSD() {              
  if (myFile) {
    if (intMode == 2){
       myFile.close();
       fSerPrint("SD File closed.",1);
    }
  }
}

/**************************\
* Conv intQnh PA to strHpa *
***************************/  
String fPaToHpa(int QNH) {
  char chrQNH[8];
  intSprf = sprintf(chrQNH,"%4d,%1d",QNH/100,((QNH % 100)/10));
  return chrQNH;
}

/*******************************************************\ 
* IGC Naming  creation and test if file existe (1 to 35)*
********************************************************/ 
String fwFileName(){
  File myFile;
  String strIgcTemp;  
  String strIgcDay = "";
  int intItem;    
  int intNumFile = 1;
    
  intItem = ((gps.date.year() % 1000) % 10) + 48;
  strIgcDay += char(intItem);
  intItem = gps.date.month() + 48;
  if (intItem > 57) intItem += 7;
  strIgcDay += char(intItem);
  intItem = gps.date.day() + 48;
  if (intItem > 57) intItem += 7;
  strIgcDay += char(intItem);
  strIgcDay += "XJDB";  
  fSerPrint("",1);
  fSerPrint("StrIgcDay :" + strIgcDay ,1); //strIgcDay="74DXJDB";
  //Serial.print(strIgcDay); 
  Serial.print(" Find FileOrderNum: ");
  blFileExist = false;  
  do{
    intItem = intNumFile + 48;
    if (intItem > 57) intItem +=7;
    strIgcTemp = strIgcDay + char(intItem) +".IGC";
    //Serial.print("Try to open ");Serial.print(strIgcTemp);
    myFile = SD.open(strIgcTemp);  
    if (myFile){
      Serial.print("+");
      //Serial.println(" : Existe");  
      blFileExist=true ;
      myFile.close();      
      intNumFile += 1; 
      if (intNumFile > 35){
        // on reste bloqué à 35 en attendant une solution!
        intNumFile =35;
        blFileExist=false;
      }
    }
    else {
      Serial.println("-"); // : Existe pas!");  
      blFileExist=false; 
    }
    delay (100);
  }
  while (blFileExist);
  blFileExist= false;
  intNumFile += 48;
  if (intNumFile > 57) intNumFile +=7;
  strIgcDay += char(intNumFile);
  myFile.close();  
  return strIgcDay;
}

/*******************************************************\ 
*     Test ask and prepare new init (PushBut >= 5sec)   *
********************************************************/ 
int fTestAFS(){
  if (intAfs >4) {
    intBtMod = 1;    // passage en mode monit pour Bluetooth
    Serial1.begin(115200);
    delay(300);
    Serial1.println();
    Serial1.println(strVersion); 
    fSerPrint(" Waiting for NewInit",1);
    intAfs=0;
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);    
    display.println("NEW SETUP");
    display.println("Push-But Off");
    display.println("For Setup");
      //clear dispaly and write wait to released push but for new init
    if (myFile) {
      intMode = 0;
      myFile.close();
      fSerPrint("SD file Closed",1);
      display.println("SD file Closed"); 
    }
    fSerPrint(" Waiting for release pushBut",1);
    display.display();
    do{
      // Waiting for release pushBut
      delay(100);
    } while (!digitalRead(PinR));            
    intMode = 0;    
    mouvement = false;  
    ButPushed = false;
    fSerPrint(" Pause, RePushBut to Confirm",1);
    lngTempo1=millis(); 
    do {
      // waiting pushBut for 1'      
      if (millis()-lngTempo1 > TEMPO60) ButPushed=true;
    } while (!ButPushed );      
    mouvement = false;  
    ButPushed = false;  
    fSerPrint(" Now let's go to newInit...",1);
    return true;
  } else return false;
}

/*******************************************************************
* =================== Extraction des trames GPS ===================*
********************************************************************/
int fTraitGprmc(int intGrx){
  //Serial1.write(intGrx);Serial1.print ("-");Serial1.println(intGrx);
  if (!blDolDetect) {  
    if (intGrx == 36){
      strGrx = "";
      blDolDetect = true;
      strGrx += char(intGrx);
      //Serial.println(strGrx);
      return false;
    } else return false;
  }
  if (blDolDetect && intGrx != 13){
    strGrx += char(intGrx);
    return false;
  } else {
      strGrx += char(intGrx);
      //Serial1.println(strGrx);
      //while (1);
      return true;
    }
}

/*******************************************************************
* =================== ADJ PARAMS INIT and SETUP ===================*
********************************************************************/
static void fAdjSetup(){

  /*************** ADJ QNH ****************/
  fSerPrint("Waiting adj QNH",1);
  if(blSDvalid) {
    int intByte;    
    myFile = SD.open("init.cfg");
    if (myFile) { 
      //Serial.println("init.cfg:");
      int intC=0;
      while (myFile.available()) {
        intByte=myFile.read();
        if(intByte>13) chrQnh[intC]=intByte;
        intC += 1;
      }
      myFile.close();      
      intQnh=atoi(chrQnh);
      fSerPrint("Read from init.cfg on SD:" + fPaToHpa(intQnh) + " hPa, Alt:" + String(bme.readAltitude(intQnh/100)) + " m" ,1);
     }
  }
  else {
    fSerPrint(" No SD => init to 1013 hPa",1);
    intQnh=101300;
  }
  int intQnhTmp = intQnh;
  lngTempo1=millis();  
  mouvement = false;
  do {     
     ftestAdjQNH();
     display.clearDisplay();
     display.setCursor(0,0);
     display.println("Adj QNH puis pushBut");
     display.setCursor(0,10);
     display.print( "QNH:");
     display.print(fPaToHpa(intQnh));
     display.println(" hPa  "); 
     display.print(bme.readAltitude(intQnh/100));
     display.println(" m"); 
     display.display();    
     delay(500); 
     if (millis()-lngTempo1 > TEMPO1) ButPushed=true;
  } while (!ButPushed );
  ButPushed=false;  
  fSerPrint("new QNH :" + fPaToHpa(intQnh) + " hPa, Alt:" + String(bme.readAltitude(intQnh/100)) + " m" ,1);
  
  /*************** Storing NEW QNH ****************/
  if (!(intQnhTmp == intQnh)){
    if(blSDvalid) {
      SD.remove("init.cfg");
      fSerPrint("Old init.cfg removed",1);
      myFile = SD.open("init.cfg", FILE_WRITE);
      if (myFile) {       
        fSerPrint("Storing new QNH to init.cfg:" + fPaToHpa(intQnh) + " hPa" ,1);
        intSprf = sprintf(chrQnh,"%6d",intQnh);
        myFile.println(chrQnh);
        myFile.close();     
      }
    }
  } else fSerPrint("QNH Inchangé pas de stockage sur SD",1);
  fSerPrint("",1);
  
  /**** ADJ VISMOD: 0=standard, 1=vitesse 2=auto ( si >xx kmph, *****/     
  fSerPrint("Waiting adj VisMod: ",1);       
  fSerPrint(fStrVisMod(intVisMod),1);
  lngTempo1=millis();      
  do {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sel.VisMod->pushBut");
    display.setCursor(0,10);
    display.print( "Mode:");
    if (mouvement)  {
      intVisMod += 1;
      if (intVisMod >2) intVisMod=0;
      fSerPrint(fStrVisMod(intVisMod),1);
      lngTempo1=millis();
      mouvement = false;                        
    }      
    display.print(fStrVisMod(intVisMod));
    display.display();  
    delay(300);
    if (millis()-lngTempo1 > TEMPO1) ButPushed=true;
  } while (!ButPushed);
  ButPushed=false;        
  fSerPrint(fStrVisMod(intVisMod) + " selected",1);
  fSerPrint("",1);
   
  /*************** ADJ Sto Interval ****************/
  fSerPrint("Waiting adj StoInterval: ",1);         
  fSerPrint(String(intStoSD) + " Sec.",1);
  lngTempo1=millis();  
  do {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sel.StIntv->pushBut");
    display.setCursor(0,10);
    display.print( "Interval:");       
    if (mouvement)  {
     intStoSD = intStoSD * 2;
      if (intStoSD > 8) intStoSD = 1;
      lngTempo1=millis();
      mouvement = false;
      fSerPrint(String(intStoSD) + " Sec.",1);
    }
    display.print(intStoSD);  
    display.print( "Sec.");    
    display.display();    
    delay(300);
    if (millis()-lngTempo1 > TEMPO1) ButPushed=true;
  } while (!ButPushed);
  fSerPrint(String(intStoSD) + " Sec. selected",1);
  ButPushed=false;
  fSerPrint("",1);
  
  /*************** Select Bluetooh valid ****************/
  fSerPrint("Waiting select Bluetooth mode: ",1); 
  fSerPrint(fStrBtMod(intBtMod),1);
  int intBtModTmp = intBtMod;
  if (intBtMod > 1) intBtMod = 1;  // pour afficher sur console BT
  lngTempo1=millis();  
  do {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sel. BT mod ->pushBut");
    display.setCursor(0,10);
    if (mouvement) {
      if (up){
        intBtModTmp +=1;
        if (intBtModTmp > 7) intBtModTmp = 0;
      } else {
          if (intBtModTmp == 0) intBtModTmp = 8;
          intBtModTmp -=1;
        }
      lngTempo1=millis();
      mouvement = false; 
      fSerPrint(fStrBtMod(intBtModTmp),1);   
    }
    display.print(fStrBtMod(intBtModTmp));
    display.display();    
    delay(300);
    if (millis()-lngTempo1 > TEMPO1) ButPushed=true;
  } while (!ButPushed);
  intBtMod = intBtModTmp;
  fSerPrint(fStrBtMod(intBtMod) + " selected",1 );
  ButPushed=false;
  if (intBtMod == 0) Serial1.end();
  else {
    Serial1.begin(115200);
    delay(300);
    Serial1.println();
    Serial1.println(strVersion);   
  }
  fSerPrint("",1); 
  fSerPrint("********** END Setup **************",1); 
  delay(200);
  fSerPrint("",1);   
  mouvement = false;  
  ButPushed = false;
}

//********* sous-routines ***********
String fStrVisMod(int intNmod){
  String strToPr = String(intNmod);
  if (intVisMod == 0) strToPr += " => Standard";
  if (intVisMod == 1) strToPr += " => Vitesse"; 
  if (intVisMod == 2) strToPr += " => Auto";
return strToPr;
}

//********* 
String fStrBtMod(int intBtTmp){
  String strToPr = String(intBtTmp);
  if (intBtTmp == 0) strToPr += " => BlTooth OFF";  
  if (intBtTmp == 1) strToPr += " => BlTooth Monit";
  if (intBtTmp == 2) strToPr += " => BlTooth $GPVTG"; 
  if (intBtTmp == 3) strToPr += " => BlTooth $GPGGA"; 
  if (intBtTmp == 4) strToPr += " => BlTooth $GPGSA"; 
  if (intBtTmp == 5) strToPr += " => BlTooth $GPGSV"; 
  if (intBtTmp == 6) strToPr += " => BlTooth $GPGLL"; 
  if (intBtTmp == 7) strToPr += " => BlTooth $GPRMC"; 
  //String strNmeaSel;  //$GPVTG, $GPGGA, $GPGSA, $GPGSV, $GPGLL, $GPRMC
return strToPr; 
}

//**********************
String fStrNmea(int intNmea){
  String strToPr ="$GP" ;
  if (intNmea == 2) strToPr += "VTG"; 
  if (intNmea == 3) strToPr += "GGA"; 
  if (intNmea == 4) strToPr += "GSA"; 
  if (intNmea == 5) strToPr += "GSV"; 
  if (intNmea == 6) strToPr += "GLL"; 
  if (intNmea == 7) strToPr += "RMC"; 
  return strToPr;
}

