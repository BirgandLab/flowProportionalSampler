#include <EEPROM.h>              //library to write to EEPROM to keep cumulative flow data if power is lost etc
#include "EEPROMAnything.h"      //a data stucture to facilitate writing to EEPROM
//This program writes a 4 letter HEX ID number to the EEProm memory of the Arduino
//Each Arduino only needs to have this written once
//it can then be polled when it is being used, and this information can be used to troubleshoot, or keep track of field sites
//Yay.

//char ID[5] ="1D16"; FOR Arduion Uno R3 A4
//ALSO 1DF5
//char ID[5]="1489";
//char ID[5]="1879";
//char ID[5]="19C6";
//char ID[5]="18D3";
//char ID[5]="16C2";
//char ID[5]="0CBD";
//char ID[5]="05E0";
char ID[5]="CLRU";
//
char query[5];
void setup(){
 Serial.begin(9600);
int i= EEPROM_writeAnything(0, ID);
  //Serial.println(i);
         EEPROM_readAnything(0,query);
 Serial.println(query);

}
void loop(){
  
  
}
