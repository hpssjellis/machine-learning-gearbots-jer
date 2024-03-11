// RC Car servo and big motor driver
// By Jeremy Ellis
// MIT license

// for now reference at https://github.com/hpssjellis/particle.io-photon-high-school-robotics/tree/master/a11-dc-motor-drivers
// although pin names wrong

// You are suppossed to get it working using the web-app
// Draw your circuit diagram first
// This program will just tell you if  the connections are working
// See https://www.pololu.com/product/1451 for assistance

// On motor driver board LED goes red for one direction and green for the other








#include <Seeed_Arduino_SSCMA.h>
#include <Arduino.h> // Only needed by https://platformio.org/
//#include <Servo.h>  .. for Portenta or XIAO-SAMD
#include <ESP32Servo.h>   // for XIAO-ESP32S3-Sense

SSCMA AI;
Servo myServo_D2;
int myMainSpeed = 30;   // slowest speed that the car moves on a charged battery


void setup()
{
    AI.begin();
    Serial.begin(9600);
    myServo_D2.attach(D2); // D2 should do PWM on Portenta
    // for big motor driver
    pinMode(D3, OUTPUT);   // digital 0 to 1
    pinMode(D5, OUTPUT);   // PWM 0 to 255
    pinMode(D6, OUTPUT);   // digital 0 to 1
    
                            // both off = glide, both on = brake (if motor can do that) 
    digitalWrite(D6, 0);    // not needing to be attached
    digitalWrite(D3, 1);    // set one direction 
    
}

void loop(){




  
    if (!AI.invoke() ){

   
     AI.perf().prepocess; 
     AI.perf().inference;
     AI.perf().postprocess;


     if (AI.boxes()[0].score > 90 ){
      Serial.print(String(AI.boxes()[0].score)+", ");
      analogWrite(D5, myMainSpeed);   // go medium  

      if( AI.boxes()[0].x < 100){
            Serial.println("Left");   // turn left
            myServo_D2.write(70); // turn
      }
      else if(AI.boxes()[0].x >= 100 && AI.boxes()[0].x <= 150 ){

            Serial.println("Center");
            myServo_D2.write(90); // turn center

      }

      else if (AI.boxes()[0].x > 150) {
            Serial.println("Right");
            myServo_D2.write(110); // turn right
      }
     }
      else {
        Serial.println(String(AI.boxes()[0].score)+", None");
        analogWrite(D5, 0);   // No objects detected so stop 
      }




    }

    
}
