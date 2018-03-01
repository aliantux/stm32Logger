IGC Logger for Gliders
---
Frabrication d'un logger IGC avec stm32, bBME280, module gps et stockage en SPI sur carte SD
--
GPS sur Serial2, console sur Serial, BME280 sur I2c1, SD sur SPI1, Bluetooth sur Serial1
-
Serial correspond sur un stm32 à : Pin27-PA10 = com1RX   Pin26-PA9=com1TX     [console et pgm]  
Serial1 correspond sur un stm32 à : Pin8-PA3 = com2RX   Pin7-PA2=com2TX       [HC-06 115200]   
Serial2 correspond sur un stm32 à : Pin16-PB11 = com3RX   Pin15-PB10=com3TX   [GPS 6900b]    
I2C correspond sur un stm32 à : P34 PB6 SCL    P35 PB7 SDA    
PI1 correspond sur un stm32 à : P9 PA4 SS,  P10 PA5 SCK, P11 PA6 MISO, P12 PA7 MOSI 

IF USING OLED SPI
-
'#define OLED_DC PB15  #define OLED_MOSI PB14   #define OLED_CLK PB13   #define OLED_CS  PB12 

Upload method :serial   generic stm32    
--
Serial.println("Monitoring satellite location and signal strength using TinyGPSCustom");     
Serial.print("Testing TinyGPS++ library v. "); Serial.print(TinyGPSPlus::libraryVersion());     
erial.println(" by Mikal Hart");Serial.println();  
--
The resulting B Record becomes (with spaces for clarity in this example):   
BHH MM SS DD MMMMM N DDD MMMMM E V PPPPP GGGGG AAA SS NNN RRR CR LF"   
B07 43 22 45 35552 N 005 51724 E A 00000 00322   
B13 24 54 45 33430 N 005 58763 E A 00000 00285 008 00 000 000   
B0743224535552N00551724EA0000000322   =>result de ce programme au 03/02/17   
B1324544533430N00558763EA000000028500800000000 => trame oudie   

TRAMES GPS
=
$GPVTG,,T,,M,0.879,N,1.627,K,A*27   
$GPGGA,121249.00,4535.55965,N,00551.72597,E,1,07,1.55,332.0,M,47.3,M,,*57   
$GPGSA,A,3,13,30,19,24,28,15,17,,,,,,2.60,1.55,2.09*0B   
$GPGSV,3,1,12,05,02,185,,10,10,326,,12,19,213,,13,61,135,26*79  
$GPGSV,3,2,12,15,76,258,30,17,31,098,32,18,24,301,18,19,23,124,20*7E  
$GPGSV,3,3,12,20,25,226,,24,43,280,22,28,30,047,19,30,06,081,16*7B  
$GPGLL,4535.55965,N,00551.72597,E,121249.00,A,A*66  
$GPRMC,121250.00,A,4535.55919,N,00551.72590,E,0.306,,240317,,,A*7D  
  
  
The A Record must be the first record in an FR Data File,  
A RECORD - FR ID NUMBER
  
Voir http://carrier.csi.cam.ac.uk/forsterlewis/soaring/igc_file_format/igc_format_2008.html#link_3.1  

3.3.1 Required records. The following H records are required, in the order given below:  
-

  HFDTEDDMMYY  
  HFFXAAAA  
  HFPLTPILOTINCHARGE:TEXTSTRING  
  HFCM2CREW2:TEXTSTRING  
  HFGTYGLIDERTYPE:TEXTSTRING  
  HFGIDGLIDERID:TEXTSTRING  
  HFDTMNNNGPSDATUM:TEXTSTRING  
  HFRFWFIRMWAREVERSION:TEXTSTRING  
  HFRHWHARDWAREVERSION:TEXTSTRING  
  HFFTYFRTYPE:MANUFACTURERSNAME,FRMODELNUMBER    
  HFGPS:MANUFACTURERSNAME,MODEL,CHANNELS,MAXALT(AL3)  
  HFPRSPRESSALTSENSOR:MANUFACTURERSNAME,MODEL,MAXALT(AL3)     
  
  
Voir  http://carrier.csi.cam.ac.uk/forsterlewis/soaring/igc_file_format/igc_format_2008.html#link_2.5  
Short file name style: YMDCXXXF.IGC  
-
Y = Year; value 0 to 9, cycling every 10 years  
M = Month; value 1 to 9 then A for 10, B=11, C=12.  
D = Day; value 1 to 9 then A=10, B=11, C=12, D=13, E=14, F=15, G=16, H=17, I=18, J=19, K=20, L=21,  
M=22, N=23, O=24, P=25, Q=26, R=27, S=28, T=29, U=30, V=31.  
C = manufacturer's IGC code letter (see table below)  
XXX = unique FR Serial Number (S/N); 3 alphanumeric characters  
F = Flight number of the day; 1 to 9 then, if needed, A=10 through to Z=35  

