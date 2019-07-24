#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// WiFi Parameters
const char* ssid = "Yetri";
const char* password = "hayreniq";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
}

void loop() {
  // Check WiFi Status
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;  // Object of class HTTPClient
    http.begin("http://jsonplaceholder.typicode.com/users/1");
    int httpCode = http.GET();
    
    //Check the returning code                                                                  
    if (httpCode > 0) {
      // Parsing
      const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(8) + 370;
      DynamicJsonDocument doc(bufferSize);
      // JsonObject& root = doc.parseObject(http.getString());
      deserializeJson(doc,http.getString());
      
      // Parameters
      const char* username = doc["username"]; // "Bret"
      JsonObject address = doc["address"];
      const char* address_street = address["street"]; // "Kulas Light"
      const char* address_geo_lat = address["geo"]["lat"]; // "-37.3159"
      
      // Output to serial monitor
      
      Serial.print("Username: ");
      Serial.println(username);
      
      Serial.print("Street: "); 
      Serial.println(address_street);

      Serial.print("Latitude: ");
      Serial.println(address_geo_lat);
    }
    http.end();   //Close connection
  }
  // Delay
  delay(60000);
}
