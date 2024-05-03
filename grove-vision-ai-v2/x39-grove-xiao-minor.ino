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
 * Connections XIAO-esp32S3 to Big Motor Driver
 * D0 to top left 1   Digital turn
 * D6 to top left 3    PWM motor speed
 * D1 to top left 6     digital turn
 * 3V3 to top left 7
 * GND to top left 8
 * 
 * 
 * DON'T USE PINS D3 AND D6 ON THE XIAO AS THEY ARE PART OF THE COM WITH THE GROVE.
 * 
 */



#include <Seeed_Arduino_SSCMA.h>
#include <ESP32Servo.h>   // for XIAO-ESP32S3-Sense


SSCMA AI;
Servo myServo_D2;
int myDrivePwmPin = D0;
int mySlowestSpeed = 53;   // slowest speed that the car moves on a charged battery
int myMainSpeed = 0;  
int myOldSpeed = 0;   

int myMappedAngle = 0;
int myOldMappedAngle = 0;
int myAngleChange = 9;  // was 5 the degrees the servfo will move each time it moves
int myMaxAngle = 127;    // max right turn on my car
int myMinAngle = 53;     // max left turn on my car
int myRealMinX = 10;     // Should be 0 but is the lowest x reading your vision model gets
int myRealMaxX = 210;    // should be 320 but is the real max reading your vision model gets



void setup(){
    Serial.begin(115200);
    pinMode(myDrivePwmPin, OUTPUT);   // PWM 0 to 255
    myServo_D2.attach(D2); // D2 should do PWM on XIOA
    // note the two drive pins on the big motor driver are just connected to GND and 3V3 respectively.
    
   // test motors
   Serial.println("motor testing");
    delay(500);
    analogWrite(myDrivePwmPin, mySlowestSpeed); // slowest speed motor test
    myServo_D2.write(myMinAngle);
    delay(500);                                // wait a bit
    analogWrite(myDrivePwmPin, 0);             // stop motor
    myServo_D2.write(90);
    delay(500);       
    myServo_D2.write(myMaxAngle);
    delay(500);       
        
    // Grove Vision AI V2 and zero at least one value
    AI.begin();

}

void loop(){
    if (!AI.invoke() ){

     AI.perf().prepocess;
     AI.perf().inference;
     AI.perf().postprocess;

     Serial.println();
     if (AI.boxes()[0].score > 85 ){
      
      myMappedAngle = map(AI.boxes()[0].x, myRealMinX, myRealMaxX, myMaxAngle, myMinAngle); // x location to angle note reverse numbers 
          
      if (myOldMappedAngle < myMappedAngle - myAngleChange ){
           myOldMappedAngle += myAngleChange;    
           if (myOldMappedAngle > myMaxAngle){myOldMappedAngle = myMaxAngle;}  // protect from maxTurn
           myServo_D2.write(myOldMappedAngle);                   // turn towards new bigger angle
          Serial.print(" >");   
      } else if (myOldMappedAngle > myMappedAngle + myAngleChange ){
         myOldMappedAngle -= myAngleChange;
         if (myOldMappedAngle < myMinAngle){myOldMappedAngle = myMinAngle;}      // protect from minimum turn
         myServo_D2.write(myOldMappedAngle - 5);                 // turn towards new smaller angle
         Serial.print(" <");  
      }  // note: angles  myAngleChange < oldmapped < myAngleChange should not change servo at all


      Serial.print(" score:"+String(AI.boxes()[0].score) + ", x:" + String(AI.boxes()[0].x) + ", map:" + String(myMappedAngle) );
      myMainSpeed = mySlowestSpeed;
      
     } else {
        Serial.print(String( AI.boxes()[0].score)+", None");
        analogWrite(myDrivePwmPin, 0);   // No objects detected so stop
        myMainSpeed = 0;
     }

      if ( myOldSpeed != myMainSpeed){     // test if speed is different
         analogWrite(myDrivePwmPin, myMainSpeed);   
         Serial.print(" ."); 
         myOldSpeed = myMainSpeed;
      }

      AI.boxes()[0].score = 0;  // pre-zero for next loop
      AI.boxes().clear();
    }

   
}