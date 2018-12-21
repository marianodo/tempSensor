#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5 //Pin to which is attached a temperature sensor


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress devAddr;

int numberOfDevices; //Number of temperature devices found
float tempDev; //Saving the last measurement of temperature

//Setting the temperature sensor
void SetupDS18B20(){
  DS18B20.begin();
  DS18B20.isParasitePowerMode();
  numberOfDevices = DS18B20.getDeviceCount();
  DS18B20.requestTemperatures();
  DS18B20.getAddress(devAddr, 0);
  //Read temperature from DS18b20
  float tempC = DS18B20.getTempC( devAddr );
  
}

//Loop measuring the temperature
void TempLoop(void){
    float tempC = DS18B20.getTempC(devAddr); //Measuring temperature in Celsius
    tempDev = tempC; //Save the measured value to the array
    DS18B20.setWaitForConversion(false); //No waiting for measurement
    DS18B20.requestTemperatures(); //Initiate the temperature measurement  
}
