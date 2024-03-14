
#include <Seeed_Arduino_SSCMA.h>
#include <ESP32Servo.h>   // for XIAO-ESP32S3-Sense


SSCMA AI;
Servo myServo_D2;
int myMainSpeed = 37;   // slowest speed that the car moves on a charged battery



void setup(){
    AI.begin();
    Serial.begin(115200);
    
    // Stay away from pins D4 (SDA) and D5 (SCL) whiuch are used for I2C communication with the Grove board
    // D3 when both are connected is the reset pin so that messes things up.

    myServo_D2.attach(D2); // D2 should do PWM on XIOA
    pinMode(D6, OUTPUT);   // PWM 0 to 255
    pinMode(D1, OUTPUT);   // digital 0 to 1
    pinMode(D0, OUTPUT);   // digital 0 to 1

    analogWrite(D6, 0);      // have car stopped at beginning
                                // both off = glide, both on = brake (if motor can do that)
    digitalWrite(D0, 0);    // not needing to be attached
    digitalWrite(D1, 1);    // set one direction
}

void loop()
{
    if (!AI.invoke() ){

   
     AI.perf().prepocess;
     AI.perf().inference;
     AI.perf().postprocess;


     if (AI.boxes()[0].score > 85 ){
      Serial.print(String(AI.boxes()[0].score)+", ");
      analogWrite(D6, myMainSpeed);   // go medium  

      if( AI.boxes()[0].x < 100){
            Serial.println("Left");
            myServo_D2.write(110); // turn left
      }
      else if(AI.boxes()[0].x >= 100 && AI.boxes()[0].x <= 150 ){

            Serial.println("Center");
            myServo_D2.write(90); // turn center
      }

      else if (AI.boxes()[0].x > 150) {
            Serial.println("Right");
            myServo_D2.write(90); // turn right
      }
     }
      else {
        Serial.println(String(AI.boxes()[0].score)+", None");
        analogWrite(D6, 0);   // No objects detected so stop
      }



    }

   
}
