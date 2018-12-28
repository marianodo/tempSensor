#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "httpServer.h"
#include "sensor.h"
#include "eeprom.h"

ADC_MODE(ADC_VCC);

#define TOKEN "1EYJsctk80mhN8Ng1XeQ"
char thingsboardServer[] = "data.senseit.com.ar";
int failedCount;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

//int status = WL_IDLE_STATUS; What is this for?


unsigned int sleepTime = 15; //15 is defaultSleepTime. Is not in another variable to avoid memory usage
unsigned long int sleepTimeInMicroseconds = 900000000; //This is 15 * 60000000. 60000000 convert from min to microseconds
// start reading from the first byte (address 0) of the EEPROM
int addresSleepTime = 0;

void setup() {
  
  //Serial.begin(115200);
  configureWifi();
  readAttributes(); //This might replace sleepTime got it from getSleepTimeFromEeprom
  //Serial.println("Iniciando MPU");
  initMPU();
  //Serial.print("Calibrando");
  calibrateMPU();  
  
  //Serial.println(ax_cal);
  //Serial.println(ay_cal);

  
  client.setServer( thingsboardServer, 1883 );

  if ( !client.connected() ) {
    reconnect();
  } 
}


void configureWifi(){
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("SenseIt", "senseit1234")) {
    ESP.deepSleep(sleepTimeInMicroseconds); 
  }
}

void getSleepTimeFromEeprom(){
  //Read from eeprom
  const int tmpSleepTime = (int)readEeprom(addresSleepTime);
  if (tmpSleepTime != 0){
    sleepTime = tmpSleepTime;
    updateSleepTime();
  }
  //else sleepTime = defaultSleepTime ----> line 14
}

void readAttributes(){  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(getAttribute("sleepTime", TOKEN));
  const char* sensor = root["shared"];
  int sleepTimeFromServer = root["shared"]["sleepTime"];
  if (sleepTimeFromServer != 0 && sleepTime != sleepTimeFromServer ){ //If not 0 and are differents, then update variable and mem eeprom
    sleepTime = sleepTimeFromServer;
    //writeEeprom(addresSleepTime,sleepTime);
    updateSleepTime();
    //Serial.println("Update SleepTime to");
    //Serial.println(sleepTime);
  }
}

void loop() {
  readMPU();
/*
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x40); // Bit SLEEP = 1, pone el MPU6050 en sleep
  Wire.endTransmission(true);
*/
 
  sendData();
  client.loop();
  
  ESP.deepSleep(sleepTimeInMicroseconds); // 20e6 is 20 microseconds
}


void sendData()
{
  
  /*Serial.print("Angulo: ");
  Serial.print(angle);
  Serial.print(" cm\t ");
  Serial.print("Battery: ");
  Serial.print(valBat/1065,1);
  Serial.println(" V ");
*/
  const String angle = String(angle_roll,1);
  const String battery = String(ESP.getVcc()/1065,1);

  // Just debug messages
  /*Serial.print( "Sending angle and battery level : [" );
  Serial.print( angle ); Serial.print( "," );
  Serial.print( battery );  
  Serial.print( "]   -> " );
  */
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"angle\":"; payload += angle; payload += ",";
  payload += "\"temperature\":"; payload += Temp; payload += ",";
  payload += "\"battery\":"; payload += battery;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish( "v1/devices/me/telemetry", attributes );
  //Serial.println( attributes );

}

void updateSleepTime(){
  sleepTimeInMicroseconds = sleepTime * 60000000;
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {

//    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
//      Serial.println( "[DONE]" );
      failedCount = 0;
    } else {
      failedCount ++;
 //     Serial.print( "[FAILED] [ rc = " );
 //     Serial.print( client.state() );
 //     Serial.println( " : retrying in 5 seconds]" );
      if (failedCount == 5) ESP.deepSleep(3600e6);//Only in this case, the device will sleep for 1 hs.
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
} 
