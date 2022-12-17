
const char* ssid = "";
const char* password = "";


char server[] = "wss://8080-hpssjellis-gitpodporten-k56rqynrp6i.ws-us79.gitpod.io/";  // server address
//char server[] = "8080-hpssjellis-gitpodporten-k56rqynrp6i.ws-us79.gitpod.io";  // server address
int port = 443;




#include <WiFi.h>
#include <WiFiSSLClient.h>



WiFiSSLClient client;

// Variables to measure the speed
unsigned long beginMicros, endMicros;
unsigned long byteCount = 0;
bool printWebData = true; // set to false for better speed measurement

void setup()
{

    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // start the WiFi connection:
    Serial.println("Initialize WiFi:");
    if (WiFi.begin(ssid, password) == 0) {

    }
    // give the WiFi shield a second to initialize:
    delay(1000);
    
    Serial.print("  WiFi assigned IP ");
    Serial.println(WiFi.localIP());
    Serial.print("connecting to ");
    Serial.print(server);
    Serial.println();

    // if you get a connection, report back via serial:
   // if (client.connect(server, port)) {   // 8080
    if (client.connectSSL(server, port)) {  // 443
        Serial.print("connected to ");
        Serial.println(client.remoteIP());
        // Make a HTTP request:
    client.println("GET / HTTP/1.1");
    //client.println("GET /chat");
    client.println("Host: "+String(server));
    client.println("Upgrade: websocket");
    client.println("Connection: Upgrade");
    client.println("Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==");
    client.println("Sec-WebSocket-Version: 13");
    client.println();
    } else {
        // if you didn't get a connection to the server:
        Serial.println("connection failed");
    }
    beginMicros = micros();
}

void loop()
{
    // if there are incoming bytes available
    // from the server, read them and print them:
    int len = client.available();
    if (len > 0) {
        byte buffer[80];
        if (len > 80)
            len = 80;
        client.read(buffer, len);
        if (printWebData) {
            Serial.write(buffer, len); // show in the serial monitor (slows some boards)
        }
        byteCount = byteCount + len;
    }

    // if the server's disconnected, stop the client:
    if (!client.connected()) {
        endMicros = micros();
        Serial.println();
        Serial.println("disconnecting.");
        client.stop();
        Serial.print("Received ");
        Serial.print(byteCount);
        Serial.print(" bytes in ");
        float seconds = (float)(endMicros - beginMicros) / 1000000.0;
        Serial.print(seconds, 4);
        float rate = (float)byteCount / seconds / 1000.0;
        Serial.print(", rate = ");
        Serial.print(rate);
        Serial.print(" kbytes/second");
        Serial.println();

        // do nothing forevermore:
        while (true) {
            delay(1);
        }
    }
}
