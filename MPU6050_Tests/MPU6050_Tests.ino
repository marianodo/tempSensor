#include "sensor.h"

unsigned int sleepTime = 15;                              //15 is defaultSleepTime. Is not in another variable to avoid memory usage
unsigned long int sleepTimeInMicroseconds = 900000000;    //This is 15 * 60000000. 60000000 convert from min to microseconds
                                                          // start reading from the first byte (address 0) of the EEPROM
int addresSleepTime = 0;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Iniciando MPU");
  initMPU();
  thisTime=micros();
  Serial.print("Calibrando Gyro");
  //calibrate_gyro();
  
  Serial.print("Calibrando Accel");
 // calibrate_accel();  

}

void loop() {
  readMPU();
  ESP.deepSleep(5000000);
  //delay(50);
/*
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x40); // Bit SLEEP = 1, pone el MPU6050 en sleep
  Wire.endTransmission(true);
*/
}
