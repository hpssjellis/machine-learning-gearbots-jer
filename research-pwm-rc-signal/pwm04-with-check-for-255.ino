// PWM in to measure using pulseIn() on a Nano33BleSense
// oops lets try true pwm and not servo PWM

//pin 2 set for PWM out say as a servo
//pin 3 set for PWM in using pulseIn() to measeure the duration

#include <Arduino.h>  // only needed for https://platformio.org/
//#include <Servo.h>

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
    //if (myPWM >= 150  && myPWM < 230){myPWM = 254;}  // almost full PWM gives 1992 reading

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

    // safety for 0 and 255 which both show a duraton of zero
    if (myDuration == 0){
      pinMode(2,INPUT);  
      int myRead = digitalRead(2);  // test if the pin is getting a full signal.
      //Serial.println("Since myDuration is zero then analogRead(2) is: "+ String(myRead) );
      if (myRead == 1) {myDuration = 9999;}   // special case obviously full speed could use 2000 for a more linear situation
      pinMode(2,OUTPUT);  
    }



    Serial.println("The PWM is measured at: "+ String(myDuration) + ", a reading every "+ String(myDelaySmall) + " milliseconds.");
   
  }


}
