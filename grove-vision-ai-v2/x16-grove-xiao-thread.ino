/*
 * Connections XIAO-esp32s3 to Grove Vision AI V2 
 * GND to GND
 * 3V3 to 3V3
 * SDA (D4) to SDA grove
 * SCL (D5) to SCL Grove
 * 
 * 
 * Coonections XIAO-esp32S3 to Servo
 * D2 to Orange wire
 * 
 * Connections XIAO-esp32S3 to Big Motor Driver Pololu 1451 VNH5019
 * https://github.com/hpssjellis/maker100-eco/blob/main/media/b-b-g-big-dc-motor-driver.png
 * GND to top left 1   Digital turn
 * D0 to top left 3    PWM motor speed
 * 3V3 to top left 6   Digital turn
 * 3V3 to top left 7
 * GND to top left 8
 * 
 * 
 * 
 * 
 */


#include <Arduino.h>
#include <Seeed_Arduino_SSCMA.h>
#include <ESP32Servo.h>   // for XIAO-ESP32S3-Sense


TaskHandle_t myTaskHandle = NULL;
//int myPWMSpeed = 0; // Global variable to store PWM speed


// Global Variables
SSCMA AI;
Servo myServo_D2;
int myLowestSpeed = 58;        //  was 60;  37;   // slowest speed that the car moves on a charged battery. Typically between about 20 and 50 depending on your DC motor  (It will "hum" if not enough)
int myChosenSpeed = 0;         // speed the car should be going
int myDrivePwmPin = D0;        // PWM pin for the big motor driver.
int myMultipleObjects = 0;     // zero the object count
int myObjectMax = 5;           // maximum number of objects to increase speed
int mySpeedMultiplier =  2;   // object speed mulitplier
int myMotorDelay  =   40;     // was 30 was 150;  delay makes motor respond longer, but try to match classification

void myTask(void *parameter) {
  while (true) {
    if (myChosenSpeed > 255){myChosenSpeed = 255;}
    if (myChosenSpeed < 0){myChosenSpeed = 0;}
    if (myMultipleObjects > myObjectMax){  myMultipleObjects = myObjectMax; }
    if (myMultipleObjects > 1){ myChosenSpeed = myChosenSpeed + ( myMultipleObjects * mySpeedMultiplier );  }  // make it go faster if it sees multiple objects
    
    analogWrite(myDrivePwmPin, myChosenSpeed);
    Serial.print(" Speed:");
    Serial.print(myChosenSpeed);
   // if (myChosenSpeed == myLowestSpeed){delay(700);}   // extra delay when nothing found
   // if (myChosenSpeed == 0){delay(700);}               // extra delay when nothing found
    delay(myMotorDelay);  // just to give the motor a bit of time to react
  }
}


void setup(){
    Serial.begin(115200);
  
    pinMode(myDrivePwmPin, OUTPUT);   // PWM 0 to 255
    
    myServo_D2.attach(D2); // D2 should do PWM on XIOA
    // note the two drive pins on the big motor driver are just connected to GND and 3V3 respectively.



    // PWM research on XIAO-ESP32S3 pins with Grove Vision AI V2 board:
    // Note on the grove board we only have PINS 3V3, GND, D4, D5 set

    // D0  PWM works no startup issue
    // D1  PWM works no startup issue
    // D2  PWM works no startup issue
    // D3  JUST don't use! When both boards connected, D3 is the reset pin.    
    // D4 (SDA) which is used for I2C communication with the Grove board
    // D5 (SCL) which is used for I2C communication with the Grove board
    // D6  TX UART pin, PWM works after a brief HIGH, shorter HIGH duration if pins set before AI.begin();
    
    // D7  RX UART pin  PWM works no startup issue
    // D8  PWM works no startup issue,  SCK SPI connection for Image data if needed from Grove Board
    // D9  PWM works no startup issue,  MISO SPI connection for Image data if needed from Grove Board 
    // D10 PWM works no startup issue,  MOSI SPI connection for Image data if needed from Grove Board

    



    // Grove Vision AI V2 and zero at least one value
    
    AI.begin();
    xTaskCreatePinnedToCore(
      myTask,         // Function to be executed
      "MyTask",       // Name of the task
      2048,           // Stack size in words
      NULL,           // Task input parameter
      1,              // Priority of the task
      &myTaskHandle,  // Task handle
      0               // Core where the task should run
  );

}

void loop(){
    if (!AI.invoke() ){
  
     AI.perf().prepocess;
     AI.perf().inference;
     AI.perf().postprocess;


 
     Serial.println();    
     if (AI.boxes()[0].score > 85 ){
        myMultipleObjects = 0;
        Serial.print(" Score[0]:"); 
        Serial.print(String(AI.boxes()[0].score)+", ");
       // analogWrite(myDrivePwmPin, myLowestSpeed);   // go medium  
        myChosenSpeed = myLowestSpeed;
        myMultipleObjects = AI.boxes().size();

      // 320 x 320 width and height 
      if( AI.boxes()[0].x < 120){
            Serial.println(" Right ");
            myServo_D2.write(120); // turn Right
      }
      else if( AI.boxes()[0].x >= 120 && AI.boxes()[0].x < 150){
            Serial.println("Little Right ");
            myServo_D2.write(105); // turn Little Right
      }
      else if(AI.boxes()[0].x >= 150 && AI.boxes()[0].x <= 170 ){

            Serial.print(" Center ");
            myServo_D2.write(90); // turn center
      }
      else if (AI.boxes()[0].x > 170 && AI.boxes()[0].x <= 200) {
            Serial.print("Little Left ");
            myServo_D2.write(75); // turn Little left
      }
      else if (AI.boxes()[0].x > 200) {
            Serial.print(" Left ");
            myServo_D2.write(60); // turn left
      }
      if (myMultipleObjects > 1){
         Serial.print(" Multiple objects: " + String(myMultipleObjects));
      }
     }
      else {
        Serial.print(" Score[0]:");        
        Serial.print(AI.boxes()[0].score);
        Serial.print(",  None ");
      //  analogWrite(myDrivePwmPin, 0);   // No objects detected so stop
        myChosenSpeed = 0;   // stop the car!
      // I don't like the delay here so I tried to run the motor in it's own thread
       // delay(500);   //  hmmmm  does not feel like the correct way to deal with this.
      }

     AI.boxes()[0].score = 0;  // pre-zero the score for next loop
     AI.boxes().clear();


    }

}
