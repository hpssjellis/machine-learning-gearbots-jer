/*
 * Base RC car FOMO 96x96 with DC motor, servo and OLED all on the M7 core
 * 
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
#include <ei-v1-1-5-rc-car-1-lines-fomo-objectweight-60-a_inferencing.h>

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

const int MY_SERVO_MIN = 70;        // degrees
const int MY_SERVO_STRAIGHT = 90;
const int MY_SERVO_MAX = 110;
//const int MY_LOST_COUNT_MAX = 30;  // how many loops do you wait when lost to start slowly turning to the right


int myServoNow = MY_SERVO_STRAIGHT;    // start going straight
//int myServoOld = MY_SERVO_STRAIGHT;    // start going straight


const int  HEIGHT_WIDTH_PWM = 25;     //30   multiplier for box wide = slow, tall = fast

const int MY_PWM_MIN = 25;      // 25
const int MY_PWM_MAX = 35;     //50;  // careful this is the max speed for the car and can be up to 255!
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

      analogWrite(D5, myGlobalD5); 
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
    pinMode(D5, OUTPUT);   // PWM 0 to 255

  //  digitalWrite(D10, 0);    // set one direction   // best just to not use this one as it seems to overload the portenta.
    digitalWrite(D3, 1);    // set one direction 





    
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
    myPwmNow = 0;   //  iF no objects we want the car to stay stopped
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
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }


        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
        ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);

        
        // fake mapping
       // display.drawRect(bb.x/2, bb.y/2, bb.width*3, bb.height*3, SSD1327_WHITE ); 


        int xMap = map(bb.x, 0,96, 0,127);
        int yMap = map(bb.y, 0,96, 0,127);
        int widthMap = map(bb.width, 0,96, 0,127);
        int heightMap = map(bb.height, 0,96, 0,127);
        display.setCursor(xMap+2, yMap);
        display.println(bb.label);

        
        if ((float)bb.value > MY_FOMO_CUTOFF){

           myGlobalCount += 1;
                  
           // Determine the largest Height and the connected x,y,width
           if (myMaxHeight < (int)bb.height + (int)bb.y){ 
            
            // store the old values
             myMaxHeight2nd  = myMaxHeight;     
             myConnectedX2nd = myConnectedX;
             myConnectedY2nd = myConnectedY;
             myConnectedWidth2nd  = myConnectedWidth;
             myConnectedValueAsIntPercent2nd = myConnectedValueAsIntPercent;
             
             // save the new values
             // myMaxHeight =  (int)bb.height;   
             myMaxHeight = (int)bb.height + (int)bb.y;   
             myConnectedX = (int)bb.x; 
             myConnectedY = (int)bb.y; 
             myConnectedWidth = (int)bb.width; 
             myConnectedValueAsIntPercent = round(bb.value * 100);
           }   // end maxHeight if
           ei_printf("Good, ");
           display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_WHITE ); // good value
        } else {
          ei_printf("Bad,  ");
           display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_BLACK ); // bad value        
        }



        
    }        // END of analysis loop!

    if (!bb_found) {
        ei_printf("    No objects found\n");
    }



    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    Serial.println("    anomaly score: " + String(result.anomaly, 5));
#endif
   
    digitalWrite(LEDB, HIGH);   //on board LED's are turned off by HIGH    
    digitalWrite(LEDG, HIGH);   
    digitalWrite(LEDR, HIGH); 

   if (bb_found) {    // if objects are found lets load some l

    
   //  Leave full white to show LORA stop if we implement that.
   //  if (myGlobalCount  > 14) { digitalWrite(LEDR, LOW);  digitalWrite(LEDG, LOW);   digitalWrite(LEDB, LOW); }

   }

       // LET'S TRY SOME FUZZY LOGIC
        
        // RC car speed!

            
         if (myMaxHeight == 0 ) {  // stop if no obects detected
             myPwmNow = 0;  // MY_PWM_MIN;  // 0;  // stop or go slow
             //myIntentGoal = 0;
             myServoNow = MY_SERVO_STRAIGHT;  // MY_SERVO_MAX;   // so turn   MY_SERVO_STRAIGHT;   // wheels straight
             display.setCursor(50, 50);
             display.println("STOP!");
          } else {  // got objects so speed depends on how skinny best box is

               myPwmNow = (myMaxHeight/myConnectedWidth) * HEIGHT_WIDTH_PWM  ;   // taller the bounding box faster the car
 
              
              // change later
              //myPwmNow =   MY_PWM_MIN;                                          // wider the bounding box slower the car
   
              // check max and mins for PWM (SPEED)
              if (myPwmNow >  MY_PWM_MAX )  {
                 myPwmNow = MY_PWM_MAX; 
              }
              if (myPwmNow < MY_PWM_MIN  )  {
                 myPwmNow = MY_PWM_MIN;   
              }
        int mySlopeX = 0;
        int myShiftX = 0;
        //not really it should never really go straight

        
        if (myGlobalCount == 1) {  // && myMaxHeight > MY_MAXHEIGHT_GOOD) {   // one object large maxHeight so go straight

             // myServoNow = MY_SERVO_STRAIGHT;
            //  digitalWrite(LEDB, LOW);      // Blue 
             // digitalWrite(LEDG, LOW);      // green  
             
              display.setCursor(10, 40);
              display.println("One Object");
              display.setTextColor(SSD1327_BLACK);
              
              display.setCursor(50, 40);
              display.println("One Object");
              display.setTextColor(SSD1327_WHITE);  
              ei_printf("one Object: ");
            
        } 
        

            myShiftX = round( SHIFT_PIXEL_TO_DEGREE * ( MY_MIDDLE_X - (myConnectedX + (myConnectedWidth/2)))  );
            
            if (myGlobalCount <= 1 ) {   // one object large maxHeight so go straight

              
              // mySlopeX = 0;  // possibly something here if wide box.
               
               mySlopeX = round(SLOPE_PIXEL_TO_DEGREE * (myConnectedWidth/myMaxHeight) * HEIGHT_WIDTH_SLOPE );

               //    myPwmNow = (myMaxHeight/myConnectedWidth) * HEIGHT_WIDTH_PWM  ;   // taller the bounding box faster the car
 
            } else {            
                 myPwmNow = MY_PWM_MIN;  // 2 or more objects so lets go slow
              
                 // 2nd x location - tallest x location = x pixels of the slope
                // Argggh which one is it. Oh well just try both and see which one works
                // mySlopeX = round( ( myConnectedX2nd + (myConnectedWidth2nd/2))  - ( myConnectedX    + (myConnectedWidth/2))    );
                mySlopeX = round( SLOPE_PIXEL_TO_DEGREE * ( ( myConnectedX    + (myConnectedWidth/2))     - ( myConnectedX2nd + (myConnectedWidth2nd/2)) ) );
  
            }


                    
          display.setCursor(20, 20);
          display.println(String(myShiftX)); 
          ei_printf(", myShiftX: %d, ", myShiftX );  
                  
          display.setCursor(60, 20);
          display.println(String(mySlopeX)); 
          ei_printf(", mySlopeX: %d, ", mySlopeX );

          display.setTextColor(SSD1327_BLACK);     // no show black so visible
          display.setCursor(20, 30);
          display.println(String(myShiftX)); 
           
                  
          display.setCursor(60, 30);
          display.println(String(mySlopeX)); 
    

         display.setTextColor(SSD1327_WHITE);

          // Lets print but ignore the slopeX in pixels
         // mySlopeX = 0;



       // new algorithm always this equation
       // myServoNow = MY_SERVO_STRAIGHT - round((myShiftX + mySlopeX) * PIXEL_TO_DEGREE );   //* mySlopeX);         

                                    // these values cancel each other out if
        //myServoNow = MY_SERVO_STRAIGHT    - myShiftX  - mySlopeX  ;      

      // if ( abs(mySlopeX) > SLOPE_CUTOFF) {  // Should we use shiftX or slope X or both 
         if (myGlobalCount > 1 && abs(mySlopeX) > SLOPE_CUTOFF){
          myServoNow = MY_SERVO_STRAIGHT - mySlopeX  ; 
          ei_printf(" using slope ");     
        } else {
          myServoNow = MY_SERVO_STRAIGHT - myShiftX   ;  
          ei_printf(" using shift ");       
        }



       
     //  if (myShiftX > 0 ) { // green go left
       if (myServoNow < MY_SERVO_STRAIGHT ) { //servoNow less than middle degrees, green go left
         // myServoNow = MY_SERVO_STRAIGHT + MY_SERVO_JUMP;   
        //  myServoNow = MY_SERVO_STRAIGHT + myShiftX;   
          digitalWrite(LEDG, LOW);      // green right 
          ei_printf("Green go left: %d", myServoNow );

          display.setCursor(60, 60);
          display.println("Green Left"); 
          display.setTextColor(SSD1327_BLACK);  

          display.setCursor(60, 70);
          display.println("Green Left"); 
          display.setTextColor(SSD1327_WHITE);  

          
       } else {                         // go right
           // myServoNow = MY_SERVO_STRAIGHT - MY_SERVO_JUMP; //   -= for increasing but worse for intent
           // myServoNow = MY_SERVO_STRAIGHT + myShiftX; //   -= for increasing but worse for intent
            digitalWrite(LEDB, LOW);      // Blue left 


             
            display.setCursor(10, 60);
            display.println("blue right");

            display.setTextColor(SSD1327_BLACK);  

            display.setCursor(10, 70);
            display.println("blue right");

            display.setTextColor(SSD1327_WHITE);  
            
            ei_printf("Blue go right: %d", myServoNow );
          
      //} // end mySlopeX > 0
     } // end more than one object
    //} // end big else maxHeight== 0
     




      
       



        
        
                
                // check max and mins for the servo
                if (myServoNow >  MY_SERVO_MAX )  {myServoNow = MY_SERVO_MAX;}
                if (myServoNow < MY_SERVO_MIN  )  {myServoNow = MY_SERVO_MIN;}











        
               // use this to show a variable in the middle of the screen 
               // display.setCursor(50, 50);
               // display.println(String(myConnectedX));



          }  // End STOP

  

                display.setCursor(20,10);
                display.println("Rocksetta-Drive");  

 
                
                display.setCursor(10,120);
                display.println("Servo:");
                display.setCursor(50, 120);
                display.println(String(myServoNow));
                display.setCursor(70, 120);
                display.println("PWM:");
                display.setCursor(100, 120);
                display.println(String(myPwmNow));







         // myPwmOld = myPwmNow;   // reset the old value for next loop
         // myServoOld = myServoNow;   // reset the old value for next loop
          
         // ACTIVATE THE Main MOTOR
         myGlobalD5 = myPwmNow;     // this is updated in the thread   
         ei_printf("\n" );




     // Last thing is to show the 128x128 GRAYSCALE OLED
     display.display();  // OLED 4 bit 16 color GRAYSCALE update
}
