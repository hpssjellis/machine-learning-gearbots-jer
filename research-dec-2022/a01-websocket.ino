/*
  Simple WebSocket client for ArduinoHttpClient library
  Connects to the WebSocket server, and sends a hello
  message every 5 seconds

  created 28 Jun 2016
  by Sandeep Mistry
  modified 22 Jan 2019
  by Tom Igoe

  this example is in the public domain
*/
#include <ArduinoHttpClient.h>
//#include <WiFi101.h>
//#include "arduino_secrets.h"

#include <WiFi.h>
#include <WiFiSSLClient.h>

//#include "arduino_secrets.h"     // more safe
#define SECRET_SSID ""   // your network
#define SECRET_PASS "" // your password





///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char serverAddress[] = "8080-hpssjellis-gitpodporten-k56rqynrp6i.ws-us79.gitpod.io";  // server address
int port = 443;

//WiFiClient wifi;
//WiFiClient wifi;
WiFiSSLClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;
int count = 0;

void setup() {
  Serial.begin(115200);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println("starting WebSocket client");
  client.begin();

  
   if (client.connected()) {
  //if (client.connect(server, 443)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET / HTTP/1.1");
   // client.println("GET /chat");
    client.println("Host: "+String(serverAddress));
    client.println("Upgrade: websocket");
    client.println("Connection: Upgrade");
    client.println("Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==");
    client.println("Sec-WebSocket-Version: 13");
    client.println();   
  } 
}

void loop() {


  while (client.connected()) {
    Serial.print("Sending hello ");
    Serial.println(count);

    // send a hello #
    client.beginMessage(TYPE_TEXT);
    client.print("hello ");
    client.print(count);
    client.endMessage();

    // increment count for next message
    count++;

    // check if a message is available to be received
    int messageSize = client.parseMessage();

    if (messageSize > 0) {
      Serial.println("Received a message:");
      Serial.println(client.readString());
    }

    // wait 5 seconds
    delay(1000);
  }

  Serial.println("disconnected");
}
