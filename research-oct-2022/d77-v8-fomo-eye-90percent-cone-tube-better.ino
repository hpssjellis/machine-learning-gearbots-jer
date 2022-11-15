/*
 * Base RC car FOMO 96x96 with DC motor, servo and OLED all on the M7 core
 * 
 * 
 * NOTE: THE MEMORY IS CRASHING IF YOU USE THE SERIAL PRINTf FUNCTION VERY MUCH
 * MUCH BETTER TO USE THE OLED FOR SHOWING DATA WHILE TESTING 
 * 
 * Note 1:  To get more than 10 detections
 * 
 * go into the edgeimpulse library you installed for this project find the file
 * \src\model-parameters\model_metadata.h
 * 
 * and change the line from
 * #define EI_CLASSIFIER_OBJECT_DETECTION_COUNT       10
 * to
 * #define EI_CLASSIFIER_OBJECT_DETECTION_COUNT       30
 * another reference here 
 * https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives
 *
 *
 *
 *
 * Note 2: Should work with any Edge Impulse model just change the below include to your model name
 * 
 * 
 * 
 * 
 * Note 3: I you are getting the error  "TFLITE arena allocation"  
 * AppData\Local\Arduino15\packages\arduino\hardware\mbed_portenta\3.0.1
 * copy the boards.txt file, rename to boards.local.txt delete everything in this file except the followoing line.
 * 
         envie_m7.build.extra_flags=-DEI_CLASSIFIER_ALLOCATION_STATIC
 *
 * Note 4:(advanced!) In edge impulse training: choose ... expert mode and object_weight=100 (less for fewer more accuarte objects) 
 * and cut_point for layer reduction       cut_point = mobile_net_v2.get_layer('block_6_expand_relu')
 * 
 * Note 5: FOMO has been set to onlshow the top value. 
 * See Note 1   so it was set to 
 * 
 *  #define EI_CLASSIFIER_OBJECT_DETECTION_COUNT       1
 * 
 * 
 * 
 * Use at your own risk!
 * by Jeremy Ellis Twitter @rocksetta
 * 
 * 
 *  FOR the GRAYSCALE Waveshare OLED 128 x 128 using library Adafruit_SSD1327.h
 *   
 *   blue  DIN (mosi) D8
 *   yellow (sck) D9                                                                                                                                                             
 *   orange (cs) D7
 *   green (dc)  D6
 *   white (reset) not needed but D14 if you did
 *
 * another reference here 
 * https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives
 *
 */




 
/* Includes ---------------------------------------------------------------- */

//#include <e-nov-2022-v2-fomo-eye-tangent_inferencing.h>
#include <ei-fomo-v5-nov-2022-eye-tube-cone_inferencing.h>
//#include <ei-fomo-v6-reduced-nov-2022-eye-tube-cone_inferencing.h>


//#include <ei-fomo-nov-2022-eye-tube-cone_inferencing.h>



//#include <ei-v67-1eye-only-for-testing-fomo_inferencing.h>


//#include <ei-v66-1eye-2tube-3cone-4dog_inferencing.h>



//#include <ei-v64-1eye-2tp-3pyramid-4dog_inferencing.h>


//#include <ei-v63_inferencing.h>

//#include <ei-v01-oct-2022-fomo-3-shapes_inferencing.h>


//#include <ei-sept-dots-fomo-v002_inferencing.h>



//#include <ei-sept-2022-fomo-lines-1turnLeft-2-turnRight-3fast_inferencing.h>


//#include <ei-v1-1-5-rc-car-1-lines-fomo-objectweight-60-a_inferencing.h>

//#include <ei-v1-1-3-rocksetta-rc-car-1-road-lines-fomo-96x96_inferencing.h>

//#include <ei-v6-0-1-fomo-2.8.0mbed-96x96-vision-1pop_inferencing.h>


#include "edge-impulse-advanced-v2.h"
#include <Servo.h>
#include "mbed.h"
#include "rtos.h"
//using namespace mbed;  // sometimes needed
using namespace rtos;

#include <Adafruit_SSD1327.h>

// Used for software SPI
#define OLED_CLK D9
#define OLED_MOSI D8

// Used for software or hardware SPI
#define OLED_CS D7
#define OLED_DC D6

// Used for I2C or SPI
#define OLED_RESET -1

// hardware SPI
Adafruit_SSD1327 display(128, 128, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Global Variables
int myDelay = 0;  // delay between readings, can be zero, default 2000 = 2 seconds
int x1Map, x2Map, y1Map, y2Map;
//int myClassCount[EI_CLASSIFIER_LABEL_COUNT]; // not yet used

// the OLED uses these
#define CUTOUT_COLS                 EI_CLASSIFIER_INPUT_WIDTH
#define CUTOUT_ROWS                 EI_CLASSIFIER_INPUT_HEIGHT
const int cutout_row_start = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS - CUTOUT_ROWS) / 2;
const int cutout_col_start = (EI_CAMERA_RAW_FRAME_BUFFER_COLS - CUTOUT_COLS) / 2;



// main thread for the servo and dc motor to keep their operation away from the analysis
Thread myThread01;

int myGlobalD5 = 0; // the Big Motor PWM speed

Servo myServo_D2;

bool myLoRaStop = false;
int myTurning = 0;
int myServoAngle = 0;
int myVertical = 0;
int myTurning2 = 0;
int myValue = 0;
          

                                  // if greater than 20 basically don't use slope X
const float SLOPE_CUTOFF = 10;   //  5; default degrees when slope of line becomes important 
const float HEIGHT_WIDTH_SLOPE  =   10;  // multiplier for wide boxes efffect on the servo through mySlopeX;
const float SHIFT_PIXEL_TO_DEGREE = 0.6; 
const float SLOPE_PIXEL_TO_DEGREE = 0.6;    // can be adjusted. closer to 1 car turns faster
const int  MY_MAXHEIGHT_GOOD = 20;    // y and height
const int  MY_THREAD_SLEEP = 10;      // default 10, ms the motor thread sleeps, lower the number faster the response
//const int  MY_INTENT_COUNT = 2000;   // >= 2 how many threads until the intent kicks in. Higher more mellow, lower more intense

const float MY_FOMO_CUTOFF = 0.85;    // default 0.85;
const int MY_MIDDLE_X = 42; //38;  //42;    //  (98 pixels / 2) - 4
//const int MY_SERVO_STRAIGHT_RANGE = 20;  // middle location allowable error
//const int  MY_SERVO_JUMP = 10;   // 3 if increasing but +-10 if fixed for INTENT matching

const int MY_SERVO_MIN = 65;        // degrees
const int MY_SERVO_SLIGHT_LOW = 82;
const int MY_SERVO_STRAIGHT = 90;
const int MY_SERVO_SLIGHT_HIGH = 98;
const int MY_SERVO_MAX = 115;
//const int MY_LOST_COUNT_MAX = 30;  // how many loops do you wait when lost to start slowly turning to the right


int myServoNow = MY_SERVO_STRAIGHT;    // start going straight
//int myServoOld = MY_SERVO_STRAIGHT;    // start going straight


const int  HEIGHT_WIDTH_PWM = 25;     //30   multiplier for box wide = slow, tall = fast

const int MY_PWM_MIN = 29;      // 25 or 30
const int MY_PWM_MAX = 45;     //50;  // careful this is the max speed for the car and can be up to 255!
int myPwmNow = 0;                 // start with it stopped!
//int myPwmOld = 0;      

int myGlobalCount; // countof all relevant objects
//int myIntentGoal = 0; // 0=stop, 1=go right, 2=go left, 3=straight, 4=lost look for objects
//int myIntentServo = MY_SERVO_STRAIGHT; // go straight  
//int myIntentPWM = 0;   // cars intent to go a certain speed 0=stop slow=MY_PWM_MIN  fast=MY_PWM_MAX
int myThreadCount = 0; // how many times the thread has ran since the last intent
//int myLostCount = 0;






// should probably be a struct array, but this was easier

int myMaxHeight = 0;
//int myMaxHeightOld = 0;    // not yet using
int myConnectedX = 0;
int myConnectedY = 0;
int myConnectedWidth = 0;
int myConnectedLabelNumber = 1; // always 1 if only one object
int myConnectedValueAsIntPercent = 0;  // are these needed


int myMaxHeight2nd = 0;
//int myMaxHeightOld2nd = 0;
int myConnectedX2nd = 0;
int myConnectedY2nd = 0;
int myConnectedWidth2nd = 0;
int myConnectedLabelNumber2nd = 1;        // always 1 if only one object
int myConnectedValueAsIntPercent2nd = 0;  // are these needed


// Thread to change the motor and servo away from the other code
// Testing myIntentGoal a concept that the intent of the car is to 0=stop, 1=go right, 2=go left, 3=straight, 4=lost look for objects
// myIntentServo should be globablly set to match the global intent 90 = straight 
// Testing later myIntentPWM should be globablly set to match the global intent 0 = stop, min and max  255  = way to fast!
// The problem is, how to change the intent based on the incoming data 

void myLedBlue_myThread01(){
  // don't do display or ei_printf commands here as it goes to fast

// following was for an intent concept that the car wanted to do something unless changed by 
// continuous data
  /*
   while (true) {
      myThreadCount += 1;
      if (myThreadCount >= MY_INTENT_COUNT ) {
        myThreadCount = 0;   // reset threadCount
          analogWrite(D5, myIntentPWM); 
          myServo_D2.write(myIntentServo);   
      } else {
        analogWrite(D5, myGlobalD5); 
        myServo_D2.write(myServoNow);  
      }
      ThisThread::sleep_for(MY_THREAD_SLEEP);   
   }
*/
   while (true) {

      // PROTECT PWM FROM GOING TOO FAST
      if (myGlobalD5 > MY_PWM_MAX) {myGlobalD5 = MY_PWM_MAX; }    
      analogWrite(D5, myGlobalD5); 
      // PROTECT THE SERVO FROM WEIRD VALUES
      if (myServoNow > MY_SERVO_MAX) {myServoNow = MY_SERVO_MAX;} 
      if (myServoNow < MY_SERVO_MIN) {myServoNow = MY_SERVO_MIN;} 
      myServo_D2.write(myServoNow);  
      
      ThisThread::sleep_for(MY_THREAD_SLEEP);   
   }
   
}





/**
* @brief      Arduino setup function
*/
void setup(){
   
     myThread01.start(myLedBlue_myThread01);
    // put your setup code here, to run once:
    Serial.begin(115200);
   // Serial.println("Edge Impulse Inferencing Demo");

    pinMode(LEDR, OUTPUT); 
    pinMode(LEDG, OUTPUT);   // this is LED_BUILTIN
    pinMode(LEDB, OUTPUT); 

    myServo_D2.attach(D2);   // D2 should do PWM on Portenta
    

    pinMode(D3, OUTPUT);   // digital 0 to 1 
    pinMode(D10, OUTPUT);   // digital 0 to 1 
    pinMode(D5, OUTPUT);   // PWM 0 to 255

    digitalWrite(D10, 0);    // set one direction   
    digitalWrite(D3, 1);       // set one direction 





    
#ifdef EI_CAMERA_FRAME_BUFFER_SDRAM
    // initialise the SDRAM
    SDRAM.begin(SDRAM_START_ADDRESS);
#endif

    if (ei_camera_init()) {
        Serial.println("Failed to initialize Camera!");
    }
    else {
        Serial.println("Camera initialized");
    }

    for (size_t ix = 0; ix < ei_dsp_blocks_size; ix++) {
        ei_model_dsp_t block = ei_dsp_blocks[ix];
        if (block.extract_fn == &extract_image_features) {
            ei_dsp_config_image_t config = *((ei_dsp_config_image_t*)block.config);
            int16_t channel_count = strcmp(config.channels, "Grayscale") == 0 ? 1 : 3;
            if (channel_count == 3) {
                Serial.println("WARN: You've deployed a color model, but the Arduino Portenta H7 only has a monochrome image sensor. Set your DSP block to 'Grayscale' for best performance.");
                break; // only print this once
            }
        }
    }
    
    // Following for the Grayscale OLED

   if ( ! display.begin(0x3D) ) {   // start Grayscale OLED
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }    
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);

    display.setRotation(0);
    display.setCursor(0,0);

    //   map cutout of the 320 x 320   // 240 model to OLED 128 x 64 screen
    x1Map = map((int)cutout_col_start, 0, 320, 0, 127);  
    x2Map = map((int)CUTOUT_COLS, 0, 320, 0, 127);
    y1Map = map((int)cutout_row_start, 0, 320, 0, 127);
    y2Map = map((int)CUTOUT_ROWS, 0, 320, 0, 127);

    
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop(){
  

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(myDelay) != EI_IMPULSE_OK) {
        return;
    }
    
   // Serial.println("Starting inferencing in "+String(myDelay)+" microseconds...");

    // Put the image on the OLED
    display.clearDisplay();                 // clear the internal memory for OLED
    for (int x=0; x < EI_CAMERA_RAW_FRAME_BUFFER_COLS; x++){     // EI_CAMERA_RAW_FRAME_BUFFER_COLS = 320
      for (int y=0; y < EI_CAMERA_RAW_FRAME_BUFFER_ROWS; y++){       //EI_CAMERA_RAW_FRAME_BUFFER_ROWS = 320   //240
      
        uint8_t myGRAY = ei_camera_frame_buffer[(y * (int)EI_CAMERA_RAW_FRAME_BUFFER_COLS) + x];  

          int myGrayMap = map(myGRAY, 0, 255, 0, 15);  
          int xMap = map(x, 0, 320, 0, 127);
          int yMap = map(y, 0, 320, 0, 127);
          display.drawPixel(xMap, yMap, myGrayMap );   // grayscale 0-255, 128x128  //128 x 64
      } 
    }

    display.drawRect(0,0,128,128, SSD1327_WHITE );  // rectangle around outside of OLED




    myGlobalCount = 0;  // how many acceptable objects
   // myPwmOld = myPwmNow; 
   // myPwmNow = 0;   //  iF no objects we want the car to stay stopped // not always
    //myServoOld = myServoNow; 
    //myServoNow = 0;    // global so don't touch here

    //myMaxHeightOld = myMaxHeight;
    myMaxHeight = 0;
    myConnectedX = 0;
    myConnectedY = 0;
    myConnectedWidth = 0;
    myConnectedValueAsIntPercent = 0;  

    
    myMaxHeight2nd = 0;
    myConnectedX2nd = 0;
    myConnectedY2nd = 0;
    myConnectedWidth2nd = 0;
    myConnectedValueAsIntPercent2nd = 0;  

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_cutout_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, NULL) == false) {
        Serial.println("Failed to capture image\r\n");
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.println("ERR: Failed to run classifier, error number: " + String(err));
        return;
    }


    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
        myValue = 0;  // reset main value for later on
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            digitalWrite(D10, 0);   // zero forward, both break
            digitalWrite(D3, 0);    // 1 forward,    neither glide
            display.setCursor(3, 30);
            display.println("0: I am lost");    
            //myPwmNow = MY_PWM_MIN;   // slowly go in a right turning circle when lost
            //myServoNow = MY_SERVO_MAX;   
            myPwmNow = 0;   // stop if nothing
            myServoNow = MY_SERVO_STRAIGHT;    // change direction to straight if nothing found.
            digitalWrite(LEDB, HIGH);   //on board LED's are turned off by HIGH    
            digitalWrite(LEDG, HIGH);   
            digitalWrite(LEDR, HIGH); 
            continue;
        }



       Serial.print(F("Label:"));
       Serial.print(bb.label);
       Serial.print(F(", value:"));
       Serial.print(bb.value);
       
       Serial.print(F(", X:"));
       Serial.print(bb.x);
       Serial.print(F(", Y:"));
       Serial.print(bb.y);
       Serial.print(F(", W:"));
       Serial.print(bb.width);
       Serial.print(F(", H:"));
       Serial.print(bb.height);

       
     /*   
        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
       // ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
     */
       // good but uses too much memory 
       // ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]", bb.x, bb.y, bb.width, bb.height);


        
        // fake mapping
       // display.drawRect(bb.x/2, bb.y/2, bb.width*3, bb.height*3, SSD1327_WHITE ); 


        int xMap = map(bb.x, 0,96, 0,127);
        int yMap = map(bb.y, 0,96, 0,127);
        int widthMap = map(bb.width, 0,96, 0,127);
        int heightMap = map(bb.height, 0,96, 0,127);
        display.setCursor(xMap+2, yMap);
        display.println(bb.label);

        // draw the box where the FOMO was found
        display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_WHITE ); // good value`               

        display.setCursor(3, 30);  // write them near the left midway down the page


         myValue = bb.value * 100;

         if ( myValue  > 95 && String(bb.label).substring(0, 1) == "4"){    // looking for the coding numbers
          display.println("4 Dog Stop");   // 4DogStop 
          
          Serial.print(F(", DOG"));
          digitalWrite(D10, 1);   // zero forward, both break
          digitalWrite(D3, 1);    // 1 forward,    neither glide       
          digitalWrite(LEDR, LOW);       // red
          digitalWrite(LEDG, HIGH);    
          digitalWrite(LEDB, HIGH);  
          myPwmNow = 0;                // stop
          myServoNow = MY_SERVO_STRAIGHT;  // servo set to straight;     
        }    


           // check value to ditch false positives       
        if (myValue  > 90 && String(bb.label).substring(0, 1) == "3"){    // 3 looking for the coding numbers
          display.println("3 cone Turn Left");   // cone 
          Serial.print(F(", CONE"));
          digitalWrite(D10, 0);   // zero forward, both break
          digitalWrite(D3, 1);    // 1 forward,    neither glide       
          digitalWrite(LEDR, HIGH);    
          digitalWrite(LEDG, LOW);      // green 
          digitalWrite(LEDB, HIGH);  
          myPwmNow = MY_PWM_MIN;   // slowly turn left
          myServoNow = MY_SERVO_MAX;  // MY_SERVO_SLIGHT_HIGH;     
        }

        if (myValue  > 90 && String(bb.label).substring(0, 1) == "2"){ //  2 tube
          display.println("2 tube Turn Right ");
          Serial.print(F(", TUBE"));       
          digitalWrite(D10, 0);   // zero forward, both break
          digitalWrite(D3, 1);    // 1 forward,    neither glide    
          digitalWrite(LEDR, HIGH);    
          digitalWrite(LEDG, HIGH);    
          digitalWrite(LEDB, LOW);      // blue
          myPwmNow = MY_PWM_MIN;        // slowly turn right
          myServoNow = MY_SERVO_MIN ;   //MY_SERVO_SLIGHT_LOW;         
        }



        
        if (String(bb.label).substring(0, 1) == "1"){  // 1bigeyeFast
          display.println("1 bigeye Fast");   
          Serial.print(F(", EYE"));
          digitalWrite(D10, 0);   // zero forward, both break
          digitalWrite(D3, 1);    // 1 forward,    neither glide  
          digitalWrite(LEDR, LOW);    
          digitalWrite(LEDG, HIGH);     // purple ish
          digitalWrite(LEDB, LOW);  
         // myTurning = ((int)bb.x)-48;   // from -48 to + 48, 0 = straight   
         // myTurning = (int)bb.x;   // from -48 to + 48, 0 = straight 
           // int myTurningEnter = (int)bb.x;
           // myTurning = myTurningEnter - 48; 
          // myTurning = (int)bb.x - 48;   // what if I subtract a larger or smaller number?
           myTurning = (int)bb.x - 44;   // what if I subtract a larger or smaller number?
          // myTurning = (int)bb.x - 38;   // -38 to shift the middle to the left!!
            


          myVertical = 96-(int)bb.y;   // from 0-96 --> 96 to 0 
          myTurning2 = round( atan2 (myVertical, abs(myTurning)) * 180/3.14159265 ); // radians to degrees and rounding        
 
          
          // object not reaching the edges of the screen 
         // myServoAngle = map(myTurning2, -48,48, MY_SERVO_MIN, MY_SERVO_MAX); // raw position to car turn angle

         // myServoAngle = map(myTurning, -40,40, MY_SERVO_MIN, MY_SERVO_MAX); // raw position to car turn angle
           
          //myServoAngle = map(myTurning, -48,48, MY_SERVO_MIN,MY_SERVO_MAX); // raw position to car turn angle
          //  int myServoAngle = map(myTurning, -48,48, 80,100); // raw position to car turn angle
         //int myServoAngle=90;
         //int myTest = Math.atan(0.6);
        
         
         
         if (myTurning < 0) {  // means needs to turn left angle between  min and 90
           // myServoAngle = map(myTurning2, 0,90, MY_SERVO_MIN, MY_SERVO_STRAIGHT); // raw position to car turn angle
            myServoAngle = map(myTurning2, 44, MY_SERVO_STRAIGHT, MY_SERVO_MIN, MY_SERVO_STRAIGHT); // raw position to car turn angle
            myServoAngle = constrain(myServoAngle, MY_SERVO_MIN, MY_SERVO_STRAIGHT);
            //myPwmNow = MY_PWM_MIN + myServoAngle - MY_SERVO_MIN;  // faster if near middle
         } else {
            //myServoAngle = map(myTurning2, 90, 0, MY_SERVO_STRAIGHT, MY_SERVO_MAX); // raw position to car turn angle
            myServoAngle = map(myTurning2, MY_SERVO_STRAIGHT, 44, MY_SERVO_STRAIGHT, MY_SERVO_MAX); // raw position to car turn angle
            myServoAngle = constrain(myServoAngle,  MY_SERVO_STRAIGHT, MY_SERVO_MAX);
          //myPwmNow = MY_PWM_MIN + MY_SERVO_MAX - myServoAngle;  // faster if near middle
         }
          myPwmNow = map(myTurning2, 0,90, MY_PWM_MIN, MY_PWM_MAX);  // faster if near middle faster closer to 90 degrees
        
         /* 
          // Math.atan(myY/myX)*180/Math.PI
          // better use tan to change angle based on nearnest
          if (myTurning > -5 && myTurning < 5){ // means it is near the middle
            myPwmNow = MY_PWM_MAX;  // go fast
          } else {
            myPwmNow = MY_PWM_MIN;  // go slow
          }  

       */

               
          //myPwmNow = MY_PWM_MAX;   // straight and fast - not too fast for testing
          myServoNow = myServoAngle;        
        }       

 


        


        
    }        // END of analysis loop!

    if (!bb_found) {
        ei_printf("    No objects found\n");
    }



    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    Serial.println("    anomaly score: " + String(result.anomaly, 5));
#endif
   
/*

   if (bb_found) {    // if objects are found lets load some lights

    
   //  Leave full white to show LORA stop if we implement that.
   //  if (myGlobalCount  > 14) { digitalWrite(LEDR, LOW);  digitalWrite(LEDG, LOW);   digitalWrite(LEDB, LOW); }

   }
*/

    // ei_printf(", myTurning: %i, myServoAngle: %u", myTurning, myServoAngle);
    // ei_printf(", Servo: %u, PWM: %u\n", myServoNow, myPwmNow);

       Serial.print(F(", myTurn:"));
       Serial.print(myTurning);
       Serial.print(F(", servo:"));
       Serial.print(myServoNow);
       Serial.print(F(", PWM:"));
       Serial.println(myPwmNow);




      display.setCursor(20,10);
      display.println("Rocksetta-Drive");  



  //     myValue = 0;  

  
      display.setCursor(20,20);
      display.println("Value"); 
      display.setCursor(60,20);
      display.println(myValue); 
      
      display.setCursor(80,20);
      display.println("%");  
  
      display.setCursor(10,100);
      display.println(String(myTurning));
      display.setCursor(50,100);
      display.println(String(myTurning2));

      display.setCursor(70,100);
      display.println(String(myVertical));
      
      display.setCursor(10,120);
      display.println("Servo:");
      display.setCursor(50, 120);
      display.println(String(myServoNow));
      display.setCursor(70, 120);
      display.println("PWM:");
      display.setCursor(100, 120);
      display.println(String(myPwmNow));

      myGlobalD5 = myPwmNow;  // activate the motor in it's own thread
         
     // Last thing is to show the 128x128 GRAYSCALE OLED
     display.display();  // OLED 4 bit 16 color GRAYSCALE update
}




   
