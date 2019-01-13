#include "sensor.h"

void setup() {
  
  Serial.begin(115200);
  Serial.println("Iniciando MPU");
  initMPU();
  thisTime=micros();
  
  calibrate_gyro();
  calibrate_accel();  
}

void loop() {
  readMPU();
 // ESP.deepSleep(5000000);
}
