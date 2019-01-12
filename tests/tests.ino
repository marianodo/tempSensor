#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>         //https://pubsubclient.knolleary.net/ PubSubClient by Nick O'Leary
#include <ArduinoJson.h>
#include "httpServer.h"
#include "sensor.h"
//#include "eeprom.h"

ADC_MODE(ADC_VCC);


int failedCount;


unsigned int sleepTime = 15;                              //15 is defaultSleepTime. Is not in another variable to avoid memory usage
unsigned long int sleepTimeInMicroseconds = 900000000;    //This is 15 * 60000000. 60000000 convert from min to microseconds
                                                          // start reading from the first byte (address 0) of the EEPROM
int addresSleepTime = 0;

void setup() {
  
  Serial.begin(115200);
  //Serial.println("Iniciando MPU");
  initMPU();
  //Serial.print("Calibrando");
  calibrateMPU();  
  
  //Serial.println(ax_cal);
  //Serial.println(ay_cal);


}

void loop() {
  readMPU();
/*
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x40); // Bit SLEEP = 1, pone el MPU6050 en sleep
  Wire.endTransmission(true);
*/
}
