
#include <WiFi.h>
//#include "SocketWrapper.h"
#include "MbedSSLClient.h"
const char* ssid = "";
const char* password = "";


void setup(){
// Set up wifi connection
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
delay(1000);
Serial.println("Connecting to WiFi..");
}

Serial.println("Connected to the WiFi network");

// Set up websocket connection
MBEDSSLClient client;
client.connect("ws://echo.websocket.org", 443);

if (client.connected()) {
Serial.println("Connected to websocket server");
} else {
Serial.println("Error connecting to websocket server");
}



// Send a message to the server
client.print("Hello from Arduino Portenta!");
}

void loop(){
// Read response from server

while(clinet.connected){
  string message = client.readString();
  Serial.println("Received: " + message);
}




// Close the connection
client.stop();
Serial.println("Websocket connection closed");
}


