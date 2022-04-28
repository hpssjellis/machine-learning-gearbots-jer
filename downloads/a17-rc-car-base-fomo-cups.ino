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
#include <ei-rc-car-v7-1-1-cups-fomo-96x96_inferencing.h>
#include "edge-impulse-advanced-v2.h"
#include <Servo.h>
#include "mbed.h"
#include "rtos.h"
//using namespace mbed;  // sometimes needed
using namespace rtos;

// Global Variables
int myDelay = 0;  // delay between readings, can be zero, default 2000 = 2 seconds
int mySlowSpeed = 30;

Thread myThread01;

int myGlobalD5 = 0; // the Big Motor PWM speed

Servo myServo_D2;

bool myLoRaStop = false;

int myObectCode = 0;   // 0=unknown,   1= pop,  2= water,     ,  3  both pop and water  now 1 red cup upside down and 2 right side up white cup
int myMaxPopY = -1;       
int myMaxWaterY = -1;     
int myObjectCount = 0;     
int myObjectCountOld = 0;     



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



    myObectCode = 0;   // 0=unknown,   1= pop,  2= water,  3=cone   ,  4  both pop and cone or pop and water 
    myMaxPopY = -1;       
    myMaxWaterY = -1;     
    myObjectCount = 0;     
    myObjectCountOld = 0;   



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

       // sepecific to RC car Library and label  1popgoright 2watergoleft  might have to change these
       if (bb.label == "1"){  // upside down red cup was pop
        ei_printf("upside down Red Cup label 1");
          if (bb.y > myMaxPopY){ myMaxPopY = bb.y;}  
       }
       

       if (bb.label == "2"){   // right side up white cup / toilet paper roll was water bottle
        ei_printf("right side up White Cup label 2");
          if (bb.y > myMaxWaterY){ myMaxWaterY = bb.y;}  
       }   


       // ei_printf("    %s (", bb.label);
       // ei_printf_float(bb.value);
       // ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
    }

    if (!bb_found) {
       // ei_printf("    No objects found\n");
    }



    

   // how the above is done for classifying exported edge impulse models 
   // for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    //    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
   // }


    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    Serial.println("    anomaly score: " + String(result.anomaly, 5));
#endif

    digitalWrite(LEDB, HIGH);   //on board LED's are turned off by HIGH    
    digitalWrite(LEDG, HIGH);   
    digitalWrite(LEDR, HIGH); 


   // more fuzzy logic here
   if (myMaxPopY < 0 && myMaxWaterY < 0){myObjectCount = 0;}   // nothing
   if (myMaxPopY > 0 && myMaxWaterY < 0){myObjectCount = 1;}   // red cup
   if (myMaxPopY < 0 && myMaxWaterY > 0){myObjectCount = 2;}   // white cup / toilet paper roll
   if (myMaxPopY > 0 && myMaxWaterY > 0){myObjectCount = 3;}   // both cups



   // no buffer code written yet
   // rc car responds to every classification


   if (myObectCode == 0){    // 0 unknown do nothing
        digitalWrite(LEDR, LOW);      // green and red on 
        digitalWrite(LEDG, LOW);    
        myServo_D2.write(90);          // wheels straight 
        myGlobalD5 = 0;                // stop the car
     // ei_printf("0: Unknown Stop: %u\n", myObectCode);
    }


    if (myObectCode == 1){       // red cup was pop: Go Right
      digitalWrite(LEDB, LOW);   // Blue LED on
      myGlobalD5 = mySlowSpeed;           // car slow
      myServo_D2.write(110);     // go right
     // ei_printf("1: Pop Go right: %u\n", myObectCode);
    }


    if (myObectCode == 2){      // white cup or toilet paper was water bottle go left
      digitalWrite(LEDG, LOW);   // Green LED on
      myGlobalD5 = mySlowSpeed;           // car slow
      myServo_D2.write(70);      // go left
     // ei_printf("1: Pop Go right: %u\n", myObectCode);
    }

    
    if (myObectCode == 3 ){             // both detected
      digitalWrite(LEDR, LOW);          // Red LED on     
      myGlobalD5 = mySlowSpeed;                  // slow
      myServo_D2.write(90);             // go straight
      //ei_printf("2: Water Go Left: %u\n", myObectCode);
    }



}   // end main loop
