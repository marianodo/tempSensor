#include <EEPROM.h>

void setupEeprom(){
  EEPROM.begin(512);  
}

byte readEeprom(int posc){
  return EEPROM.read(posc);
}

void writeEeprom(int posc, int value){
  EEPROM.write(posc, value);
  delay(500);
  EEPROM.commit();
  delay(500);
}
