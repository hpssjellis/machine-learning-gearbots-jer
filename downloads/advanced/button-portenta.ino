char rx_buf[512];
char tx_buf[512];

int baud = 115200;
uint8_t parity = 0;

int rx = 0;
int tx = 0;

bool mySent = false;


void setup() {
   SerialLoRa.begin(9600); // for LoRa must be 9600
   Serial.begin(115200);
    pinMode(D5, INPUT_PULLDOWN);
     
    pinMode(LEDR, OUTPUT); 
    pinMode(LEDG, OUTPUT);   
    pinMode(LEDB, OUTPUT);  
    
    digitalWrite(LEDR, HIGH); 
    digitalWrite(LEDG, HIGH); 
    digitalWrite(LEDB, HIGH); 
  // put your setup code here, to run once:

}

void loop() {
  //myProgramMurataIfNeeded();
  delay(10);
  // put your main code here, to run repeatedly:
  int myD5 = digitalRead(D5);
  if (myD5 > 0){
        digitalWrite(LEDB, LOW);  // Internal LED HIGH is on
        Serial.println("Send: Stop the Car: " + String(myD5));
       // tx_buf[0] = 'G';
       // tx = 1;
      //  mySent = true;
      
       SerialLoRa.write('G');
    } else {   
        digitalWrite(LEDB, HIGH);  
       // mySent = false;
  }


  while (Serial.available()) {      // If anything comes in Serial (USB),
    tx_buf[tx++] = Serial.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (tx > 0 ) {
    SerialLoRa.write(tx_buf, tx);
    tx = 0;
  }

  while (SerialLoRa.available()) {      // If anything comes in Serial (USB),
    rx_buf[rx++] = SerialLoRa.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (rx > 0) {
    Serial.write(rx_buf, rx);
    rx = 0;
  }



}


void myProgramMurataIfNeeded() {
  if (Serial.baud() != baud) {
    baud = Serial.baud();
    if (baud == 2400) {
      digitalWrite(LORA_BOOT0, HIGH);
      pinMode(LORA_BOOT0, OUTPUT);
      digitalWrite(LORA_BOOT0, HIGH);
      pinMode(LORA_RESET, OUTPUT);
      digitalWrite(LORA_RESET, HIGH);
      delay(100);
      digitalWrite(LORA_RESET, LOW);
      delay(100);
      digitalWrite(LORA_RESET, HIGH);
      SerialLoRa.begin(115200, SERIAL_8E1);
      while (SerialLoRa.available()) {
        SerialLoRa.read();
      }
      digitalWrite(LEDG, LOW);
    } else {
      SerialLoRa.begin(baud, SERIAL_8N1);
      pinMode(LORA_BOOT0, OUTPUT);
      digitalWrite(LORA_BOOT0, LOW);
      pinMode(LORA_RESET, OUTPUT);
      digitalWrite(LORA_RESET, HIGH);
      delay(100);
      digitalWrite(LORA_RESET, LOW);
      delay(100);
      digitalWrite(LORA_RESET, HIGH);
      digitalWrite(LEDG, HIGH);
    }
  }
}
