#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "httpServer.h"
#include "sensor.h"
//#include "eeprom.h"


#define TOKEN "vjZ5jfmwzwsFsMr9gUGO"

ADC_MODE(ADC_VCC);

unsigned int failedCount;
//int status = WL_IDLE_STATUS; What is this for?

unsigned int sleepTime = 15; //15 is defaultSleepTime. Is not in another variable to avoid memory usage
unsigned long int sleepTimeInMicroseconds = 900000000; //This is 15 * 60000000. 60000000 convert from min to microseconds
// start reading from the first byte (address 0) of the EEPROM
int addresSleepTime = 0;

char thingsboardServer[] = "data.senseit.com.ar"; //Why this is global??

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup()
{
  Serial.begin(115200);
  configureWifi();
  //setupEeprom();
  //getSleepTimeFromEeprom();
  readAttributes(); //This might replace sleepTime got it from getSleepTimeFromEeprom
  SetupDS18B20();
  
  client.setServer( thingsboardServer, 1883 );
  if (!client.connected()) 
    reconnect();
  
  //Read temp and send
  TempLoop();
  getAndSendTemperatureAndHumidityData();
  client.loop();
  ////////////////////////
  
  ESP.deepSleep(sleepTimeInMicroseconds); // 20e6 is 20 microseconds
}

void configureWifi(){
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("SenseIt", "senseit1234")) {
    ESP.deepSleep(sleepTimeInMicroseconds); // 900e6 son 900 segundos
  }
}

/*void getSleepTimeFromEeprom(){
  //Read from eeprom
  const byte read = readEeprom(addresSleepTime);
  const int tmpSleepTime = (int)read;
  if (tmpSleepTime != 0){
    sleepTime = tmpSleepTime;
    updateSleepTime();
  }
  //else sleepTime = defaultSleepTime ----> line 14
}*/

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

void updateSleepTime(){
  sleepTimeInMicroseconds = sleepTime * 60000000;
}

void loop()
{
}

void getAndSendTemperatureAndHumidityData()
{
  
  // Read temperature as Celsius (the default)
    
  // Check if any reads failed and exit early (to try again).
    if(isnan(tempDev)){
      //Serial.println("FALLO");
    return;
  }

  String temperature = String(tempDev);
  String battery = String((float)ESP.getVcc()/1065,1);
  //Serial.println(battery);
  // Just debug messages

  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"temperature\":"; payload += temperature; payload += ",";
  payload += "\"battery\":"; payload += battery;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish( "v1/devices/me/telemetry", attributes );
  //Serial.println(attributes);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
      failedCount = 0;
    } 
    else{
      failedCount ++;
      if (failedCount == 5) ESP.deepSleep(3600e6);//Only in this case, the device will sleep for 1 hs.
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
