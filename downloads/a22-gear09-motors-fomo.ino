/*
 * Base RC car FOMO 96x96 with DC motor and servo.
 * 
 * NOTE: NO OLED, NO SERIAL PRINTING, JUST LED AND MOTOR RESPONSES
 * 
 *  To get more than 10 detections
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
 * Note1: Should work with any Edge Impulse model just change the below include to your model name
 * 
 * IF getting TFLITE arena allocation errors 
 * AppData\Local\Arduino15\packages\arduino\hardware\mbed_portenta\3.0.1
 * copy the boards.txt file, rename to boards.local.txt with just the followoing line.
envie_m7.build.extra_flags=-DEI_CLASSIFIER_ALLOCATION_STATIC
 * 
*/

/* Includes ---------------------------------------------------------------- */

// this is the Arduino library downloaded from edge Impulse
//#include <ei-v8-1-1-jeremy-rc-car-1red-2white-cup-fomo-96x96_inferencing.h>
//#include <ei-v1-1-1-rocksetta-rc-car-1-road-lines-fomo-996_inferencing.h>
#include <ei-v1-1-2-rocksetta-rc-car-1-road-lines-fomo-96x96_inferencing.h>


#include "edge-impulse-advanced-v2.h"
#include <Servo.h>
#include "mbed.h"
#include "rtos.h"
//using namespace mbed;  // sometimes needed
using namespace rtos;

// Global Variables
int myDelay = 0;  // delay between readings, can be zero, default 2000 = 2 seconds
//int mySlowSpeed = 30;

Thread myThread01;

int myGlobalD5 = 0; // the Big Motor PWM speed

Servo myServo_D2;

bool myLoRaStop = false;

int myGlobalCount;

const float MY_FOMO_CUTOFF = 0.80;
const int MY_MIDDLE_X = 44;    // 44;   (98 pixels / 2) - 4
const int MY_SERVO_STRAIGHT_RANGE = 3;  // middle location allowable error


const int MY_SERVO_MIN = 70;
const int MY_SERVO_STRAIGHT = 90;
const int MY_SERVO_MAX = 110;

int myServoNow = MY_SERVO_STRAIGHT;    // start going straight
int myServoOld = MY_SERVO_STRAIGHT;    // start going straight

const int MY_PWM_MIN = 30;  // 30
const int MY_PWM_MAX = 40; //50;  // careful this is the max speed for the car can be up to 255!
int myPwmNow = 0;            // start with it stopped!
int myPwmOld = 0;      


int myMaxHeight = 0;
int myMaxHeightOld = 0;
int myConnectedX = 0;
int myConnectedY = 0;
int myConnectedWidth = 0;

 int myYmax = 0;
 int myYmaxXcenter = 0;
 int myYmin = 0;
 int myYminXcenter = 0;

// Big Motor controlling thread
void myLedBlue_myThread01(){
   while (true) {
      analogWrite(D5, myGlobalD5);  
     // ei_printf("D5 at: %3d\n", myGlobalD5);
      ThisThread::sleep_for(10);
   }
}





/**
* @brief      Arduino setup function
*/
void setup(){
   
     myThread01.start(myLedBlue_myThread01);
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(LEDR, OUTPUT); 
    pinMode(LEDG, OUTPUT);   // this is LED_BUILTIN
    pinMode(LEDB, OUTPUT); 

    myServo_D2.attach(D2);   // D2 should do PWM on Portenta
    
    pinMode(LEDR, OUTPUT); 
    pinMode(LEDG, OUTPUT);   // this is LED_BUILTIN
    pinMode(LEDB, OUTPUT); 
    pinMode(D3, OUTPUT);   // digital 0 to 1 
    pinMode(D5, OUTPUT);   // PWM 0 to 255

  //  digitalWrite(D10, 0);    // set one direction   // best just to not use this one as it seems to overload the portenta.
    digitalWrite(D3, 1);    // set one direction 


#ifdef EI_CAMERA_FRAME_BUFFER_SDRAM
    // initialise the SDRAM
    SDRAM.begin(SDRAM_START_ADDRESS);
#endif

    if (ei_camera_init()) {
       // Serial.println("Failed to initialize Camera!");
    }
    else {
       // Serial.println("Camera initialized");
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
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
   
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(myDelay) != EI_IMPULSE_OK) {
        return;
    }

  
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_cutout_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, NULL) == false) {
        Serial.println("Failed to capture image\r\n");
        return;
    }


   // set values before the loop

    myGlobalCount = 0;
    myPwmOld = myPwmNow; 
    myPwmNow = 0;   //  iF no objects we want the car to stay stopped
    //myServoOld = myServoNow; 
    //myServoNow = 0;    // global so don't touch here

    myMaxHeightOld = myMaxHeight;
    myMaxHeight = 0;
    myConnectedX = 0;
    myConnectedY = 0;
    myConnectedWidth = 0;
    myYmax = 0;
    myYmaxXcenter = 0;
    myYmin = 0;
    myYminXcenter = 0;

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.println("ERR: Failed to run classifier, error number: " + String(err));
        return;
    }
    // print the predictions

    // For complex prints best to run Edge Impulse ei_printf
   // ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
   //     result.timing.dsp, result.timing.classification, result.timing.anomaly);
   // ei_printf(": \n");




    // new code for FOMO bounding box edge impulse models
    // if no value in the first box then nothing in any of the boxes
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }

      // do some detection logic here
/*
       // sepecific to RC car Library and label  1popgoright 2watergoleft  might have to change these
       if (bb.label == "1"){  // upside down red cup was pop
        ei_printf("1,");
          if (myMax1Y < (int)bb.y){ myMax1Y = (int)bb.y;}  
        // ei_printf("-%u-",myMax1Y); 
       }
       

       if (bb.label == "2"){   // right side up white cup / toilet paper roll was water bottle
        ei_printf("2,");
          if (myMax2Y < (int)bb.y){ myMax2Y = (int)bb.y;} 
         //ei_printf("-%u-",myMax2Y);  
       }   
*/
     // Serial.print("Label: "+String(bb.label) + ", bb.y: " +String(bb.y) + ":::");




        if ((float)bb.value > MY_FOMO_CUTOFF){
           myGlobalCount += 1;

           
           // Determine the largest Height and the connected x,y,width
           if (myMaxHeight < (int)bb.height){ 
             myMaxHeight =  (int)bb.height;      
             myConnectedX = (int)bb.x; 
             myConnectedY = (int)bb.y; 
             myConnectedWidth = (int)bb.width; 
           } 



           if (myYmax < (int)bb.y){ 
             myYmax   = (int)bb.y;      
             myYmaxXcenter  = (int)bb.x + ((int)bb.width)/2; 

           }
           if (myYmin > (int)bb.y){ 
             myYmin   = (int)bb.y;      
             myYminXcenter  = (int)bb.x + ((int)bb.width)/2; 
           }

           
           ei_printf("Good, ");
           //display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_WHITE ); // good value
        } else {
           ei_printf("Bad,  ");
          // display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_BLACK ); // bad value        
        } 

        // Serial print the data
        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
        //ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
        ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);

    
    
    }  // END THE BIG MAIN CLASSIFIER LOOP

    if (!bb_found) {
       // ei_printf("    No objects found\n");
    }



    

     //   how the above is done for classifying exported edge impulse models 
     //   for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
     //      ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    // }


    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    Serial.println("    anomaly score: " + String(result.anomaly, 5));
#endif

    digitalWrite(LEDB, HIGH);   //on board LED's are turned off by HIGH    
    digitalWrite(LEDG, HIGH);   
    digitalWrite(LEDR, HIGH); 


     // Serial.print(", myMax1Y: "+String(myMax1Y) + ", myMax2Y: " +String(myMax2Y) + "+++");
   //   Serial.print("\t 1Y: "+String(myMax1Y) + ",\t2Y: " +String(myMax2Y) + "\t");



 // LET'S TRY SOME FUZZY LOGIC
        
        // RC car speed!
       // if (myMaxHeight > myMaxHeightOld ) {myPwmNow += 1;}  //If bounding box taller them make car go faster  
       // else { myPwmNow -= 1; }
       
      //  myPwmNow = (myMaxHeight * 5) - (myConnectedWidth * 3)  ;   // taller the bounding box faster the car

      if (myMaxHeight == 0) {myPwmNow = 0;}
      else {
      
        myPwmNow = (myMaxHeight/myConnectedWidth) * 30  ;   // taller the bounding box faster the car
                                                            // wider the bounding box slower the car
   
        // check max and mins
        if (myPwmNow >  MY_PWM_MAX )  {myPwmNow = MY_PWM_MAX;}
        if (myPwmNow < MY_PWM_MIN  )  {myPwmNow = MY_PWM_MIN;}


       if (myGlobalCount == 1) {



        // RC car Direction! Based on x location of tallest bounding box
        if (myConnectedX + (myConnectedWidth/2) > MY_MIDDLE_X + MY_SERVO_STRAIGHT_RANGE) {
           myServoNow += MY_SERVO_STRAIGHT_RANGE;
           digitalWrite(LEDG, LOW);      // green right 
           ei_printf("Green go Right: %d", myServoNow );
          }  //If bounding box taller them make car go faster  
        else if (myConnectedX + (myConnectedWidth/2) < MY_MIDDLE_X - MY_SERVO_STRAIGHT_RANGE) {
           myServoNow -= MY_SERVO_STRAIGHT_RANGE; 
           digitalWrite(LEDB, LOW);      // Blue left  
           ei_printf("Blue go Left: %d", myServoNow );
         } else {
          myServoNow = MY_SERVO_STRAIGHT;
          digitalWrite(LEDB, LOW);      // Blue 
          digitalWrite(LEDG, LOW);      // green  
          ei_printf("Cyan go Straight: %d", myServoNow );
          }  
       }   else

       {

        int myXchange = myYmaxXcenter - myYminXcenter;
        if ( myXchange > 0){
          myServoNow = MY_SERVO_STRAIGHT - (myXchange/2);
          digitalWrite(LEDG, LOW);      // green right 
          ei_printf("**Green go Right: %d", myServoNow );
        } else {
          myServoNow = MY_SERVO_STRAIGHT + (myXchange/2);
          digitalWrite(LEDB, LOW);      // green right 
          ei_printf("**Blue go Left: %d", myServoNow );
        }
       }


          
        // check max and mins
        if (myServoNow >  MY_SERVO_MAX )  {myServoNow = MY_SERVO_MAX;}
        if (myServoNow < MY_SERVO_MIN  )  {myServoNow = MY_SERVO_MIN;}

      }  // end testing if myMaxHeight > 0

        
       if (myPwmNow == 0 ){
        
          digitalWrite(LEDR, LOW);      // red stop  
          ei_printf(", STOP");
        }


      ei_printf(", PWM: %d", myPwmNow);
      ei_printf(", x: %d, y:%d, w:%d, h:%d", myConnectedX, myConnectedY, myConnectedWidth, myMaxHeight);

    // ACTIVATE THE MOTORS
      myGlobalD5 = myPwmNow;     // this is updated in the thread      
      myServo_D2.write(myServoNow); 


/*


   // more fuzzy logic here
   if (myMax1Y < 0 && myMax2Y < 0){myObjectCode = 0;}   // nothing
   if (myMax1Y > 0 && myMax2Y < 0){myObjectCode = 1;}   // red cup
   if (myMax1Y < 0 && myMax2Y > 0){myObjectCode = 2;}   // white cup / toilet paper roll
   if (myMax1Y > 0 && myMax2Y > 0){myObjectCode = 3;}   // both cups



   // no buffer code written yet
   // rc car responds to every classification
   
      ei_printf("\tCode: %u \t", myObjectCode);

   if (myObjectCode == 0){    // 0 unknown do nothing
        digitalWrite(LEDR, LOW);      // red stop   
        myServo_D2.write(90);          // wheels straight 
        myGlobalD5 = 0;                // stop the car
     // ei_printf("0: Unknown Stop: %u\n", myObjectCode);
      ei_printf("stop");
    }


    if (myObjectCode == 1){       // red cup was pop: Go Right
      digitalWrite(LEDB, LOW);   // Blue LED on
      myGlobalD5 = mySlowSpeed;           // car slow
      myServo_D2.write(110);     // go right
     // ei_printf("1: Red Cup Go right: %u\n", myObjectCode);
      ei_printf("right");
    }


    if (myObjectCode == 2){      // white cup or toilet paper was water bottle go left
      digitalWrite(LEDG, LOW);   // Green LED on
      myGlobalD5 = mySlowSpeed;           // car slow
      myServo_D2.write(70);      // go left
     // ei_printf("2: white cup go left: %u\n", myObjectCode);
      ei_printf("left");
    }

    
    if (myObjectCode == 3 ){             // both detected
      digitalWrite(LEDB, LOW);          // blue and green = cyan 
      digitalWrite(LEDG, LOW);              
      myGlobalD5 = mySlowSpeed;                  // slow
      myServo_D2.write(90);             // go straight
      //ei_printf("3: Both: %u\n", myObjectCode);
      ei_printf("straight");
    }





*/
      ei_printf("\n");
      

}   // end main loop
