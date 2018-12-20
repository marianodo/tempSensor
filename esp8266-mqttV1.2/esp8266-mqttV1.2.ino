#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
//#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
ADC_MODE(ADC_VCC);

//------------------------------------------
//DS18B20
#define ONE_WIRE_BUS 5 //Pin to which is attached a temperature sensor
#define ONE_WIRE_MAX_DEV 15 //The maximum number of devices

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
int numberOfDevices; //Number of temperature devices found
DeviceAddress devAddr[ONE_WIRE_MAX_DEV];  //An array device temperature sensors
float tempDev[ONE_WIRE_MAX_DEV]; //Saving the last measurement of temperature
float tempDevLast[ONE_WIRE_MAX_DEV]; //Previous temperature measurement
long lastTemp; //The last measurement
//const int durationTemp = 5000; //The frequency of temperature measurement
float valBat;
int failedCount;

//#define WIFI_AP "Fiberteeel WiFi280 2.4GHz"
//#define WIFI_PASSWORD "01434560492"

#define TOKEN "0KhCbxkJiT61alGgIMhD"

char thingsboardServer[] = "www.senseit.com.ar";

WiFiClient wifiClient;

PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

//Convert device id to String
String GetAddressToString(DeviceAddress deviceAddress){
  String str = "";
  for (uint8_t i = 0; i < 8; i++){
    if( deviceAddress[i] < 16 ) str += String(0, HEX);
    str += String(deviceAddress[i], HEX);
  }
  return str;
}

//Setting the temperature sensor
void SetupDS18B20(){
  DS18B20.begin();

//  Serial.print("Parasite power is: "); 
  if( DS18B20.isParasitePowerMode() ){ 
//    Serial.println("ON");
  }else{
//    Serial.println("OFF");
  }
  
  numberOfDevices = DS18B20.getDeviceCount();
//  Serial.print( "Device count: " );
//  Serial.println( numberOfDevices );

  lastTemp = millis();
  DS18B20.requestTemperatures();

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if( DS18B20.getAddress(devAddr[i], i) ){
      //devAddr[i] = tempDeviceAddress;
//      Serial.print("Found device ");
//      Serial.print(i, DEC);
//      Serial.print(" with address: " + GetAddressToString(devAddr[i]));
//      Serial.println();
    }else{
//      Serial.print("Found ghost device at ");
//      Serial.print(i, DEC);
//      Serial.print(" but could not detect address. Check power and cabling");
    }

    //Get resolution of DS18b20
//    Serial.print("Resolution: ");
//    Serial.print(DS18B20.getResolution( devAddr[i] ));
//    Serial.println();

    //Read temperature from DS18b20
    float tempC = DS18B20.getTempC( devAddr[i] );
//    Serial.print("Temp C: ");
//    Serial.println(tempC);
  }
}

//Loop measuring the temperature
void TempLoop(void){
    for(int i=0; i<numberOfDevices; i++){
      float tempC = DS18B20.getTempC( devAddr[i] ); //Measuring temperature in Celsius
      tempDev[i] = tempC; //Save the measured value to the array
    }
    DS18B20.setWaitForConversion(false); //No waiting for measurement
    DS18B20.requestTemperatures(); //Initiate the temperature measurement  
}

void setup()
{
//  Serial.begin(115200);
//  delay(10);
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPNET", "pablincho")) {
//    Serial.println("No se conecta, yendo a dormir...");
//    delay(2000);
    ESP.deepSleep(900e6); // 900e6 son 900 segundos
  }
  client.setServer( thingsboardServer, 1883 );
  lastSend = 0;
  //Setup DS18b20 temperature sensor
  SetupDS18B20();
  if ( !client.connected() ) {
    reconnect();
  } 
  TempLoop();
  getAndSendTemperatureAndHumidityData();
  client.loop();
  

//  Serial.println("Entrando en modo deep sleep");
  ESP.deepSleep(900e6); // 20e6 is 20 microseconds
}

void loop()
{
 
}

void getAndSendTemperatureAndHumidityData()
{
//  Serial.println("Collecting temperature data.");
  valBat = ESP.getVcc();
  // Reading temperature or humidity takes about 250 milliseconds!

  // Read temperature as Celsius (the default)
    float t = tempDev[0];
  // Check if any reads failed and exit early (to try again).
    if(isnan(t)){
 //   Serial.println("Failed to read from DHT sensor!");
    return;
  }
/*
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C\t ");
  Serial.print("Battery: ");
  Serial.print(valBat/1065,1);
  Serial.println(" V ");
*/  
  String temperature = String(t);
  String battery = String(valBat/1065,1);

  // Just debug messages
/*  Serial.print( "Sending temperature and battery level : [" );
  Serial.print( temperature ); Serial.print( "," );
  Serial.print( battery );  
  Serial.print( "]   -> " );
*/
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"temperature\":"; payload += temperature; payload += ",";
  payload += "\"battery\":"; payload += battery;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish( "v1/devices/me/telemetry", attributes );
//  Serial.println( attributes );

}
/*
void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}
*/

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
/*    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
*/    
//    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
//      Serial.println( "[DONE]" );
      failedCount = 0;
    } else {
      failedCount ++;
//      Serial.print( "[FAILED] [ rc = " );
//      Serial.print( client.state() );
//      Serial.println( " : retrying in 5 seconds]" );
//      Serial.println(failedCount);
      if (failedCount == 5) ESP.deepSleep(900e6);
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}




