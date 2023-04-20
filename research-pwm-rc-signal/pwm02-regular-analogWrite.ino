// PWM in to measure using pulseIn() on a Nano33BleSense
// oops lets try true pwm and not servo PWM

//pin 2 set for PWM out say as a servo
//pin 3 set for PWM in using pulseIn() to measeure the duration

#include <Arduino.h>  // only needed for https://platformio.org/
#include <Servo.h>

//Servo myServo_D2;

int myDelayBig = 3000;   // non-block delay in milliseconds
unsigned long myStartBig;

int myDelaySmall = 1000;   // non-block delay in milliseconds
unsigned long myStartSmall;

// this constant won't change. It's the pin number of the sensor's output:
const int pwmPin = 2;   // D2 near GND on right side 
const int pingPin = 3;  // D3
int myPWM = 0;   //0-255 
unsigned long myPulseDuration = 0;

void setup() {
  Serial.begin(115200);
  pinMode(2,OUTPUT);        // do you have to declare this????
  //myServo_D2.attach(2);   // D2 should do PWM on Nano33BleSense
  myStartBig = millis();    // set delay   
  myStartSmall = millis();  
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
    
  long myDuration;

  if ( (millis() - myStartBig ) >= myDelayBig) {       
    myStartBig = millis();           //  reset the delay time
    myPWM = random(0, 255);       // 0-255 
    if (myPWM <= 50){myPWM = 0;}     // test zero
    if (myPWM >= 230){myPWM = 255;}  // test full on
    //myServo_D2.write(myPWM);
    analogWrite(2, myPWM);
    Serial.println();
    Serial.println("The PWM has been set to: "+ String(myPWM) + " for "+ String(myDelayBig) + " milliseconds.");

  }
  if ( (millis() - myStartSmall ) >= myDelaySmall) {       
    myStartSmall = millis();      
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
    myDuration = pulseIn(pingPin, HIGH);

    Serial.println("The PWM is measured at: "+ String(myDuration) + ", a reading every "+ String(myDelaySmall) + " milliseconds.");
   
  }


}
