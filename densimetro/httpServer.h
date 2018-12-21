#include <ESP8266HTTPClient.h>
String attributesApi = "/attributes?sharedKeys=";

String getAttribute(String attribute, String token){
#include <ESP8266HTTPClient.h>
  String payload = "0";
  HTTPClient http;  //Declare an object of class HTTPClient
  String url = "http://data.senseit.com.ar/api/v1/" + token + attributesApi + attribute;
  http.begin(url);  //Specify request destination
  int httpCode = http.GET();                                                                  //Send the request
  if (httpCode > 0) { //Check the returning code
    payload = http.getString();   //Get the request response payload
  }
  
  http.end();   //Close connection
  return payload;
}
