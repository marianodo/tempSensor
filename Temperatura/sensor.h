#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 5 //Pin to which is attached a temperature sensor

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress devAddr;

float tempDev; //Saving the last measurement of temperature

//Setting the temperature sensor
void SetupDS18B20(){
  DS18B20.begin();
  DS18B20.isParasitePowerMode();
  DS18B20.requestTemperatures();
  DS18B20.getAddress(devAddr, 0);  
}

//Loop measuring the temperature
void TempLoop(void){
    tempDev = DS18B20.getTempC(devAddr); //Measuring temperature in Celsius
    DS18B20.setWaitForConversion(false); //No waiting for measurement
    DS18B20.requestTemperatures(); //Initiate the temperature measurement  
}
