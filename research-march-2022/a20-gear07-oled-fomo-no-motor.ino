/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
 * Note1: Should work with any Edge Impulse model just change the below include to your model name
 */




 
/* Includes ---------------------------------------------------------------- */
//#include <ei-v1-5-0-minst-96x96-f180_inferencing.h>
#include <ei-v1-1-1-rocksetta-rc-car-1-road-lines-fomo-996_inferencing.h>


//#include <ei-v1-2-minst-96x96_inferencing.h>
//#include <ei-v6-0-1-fomo-2.8.0mbed-96x96-vision-1pop_inferencing.h>


#include "edge-impulse-advanced-v2.h"
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


int myGlobalCount;

const float MY_FOMO_CUTOFF = 0.85;
const int MY_MIDDLE_X = 44;    //  (98 pixels / 2) - 4
const int MY_SERVO_STRAIGHT_RANGE = 5;  // middle location allowable error


const int MY_SERVO_MIN = 70;
const int MY_SERVO_STRAIGHT = 90;
const int MY_SERVO_MAX = 110;

int myServoNow = MY_SERVO_STRAIGHT;    // start going straight
int myServoOld = MY_SERVO_STRAIGHT;    // start going straight

const int MY_PWM_MIN = 30;  // 30
const int MY_PWM_MAX = 50; //50;  // careful this is the max speed for the car can be up to 255!
int myPwmNow = 0;            // start with it stopped!
int myPwmOld = 0;      


int myMaxHeight = 0;
int myMaxHeightOld = 0;
int myConnectedX = 0;
int myConnectedY = 0;
int myConnectedWidth = 0;


/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");

    pinMode(LEDR, OUTPUT); 
    pinMode(LEDG, OUTPUT);   // this is LED_BUILTIN
    pinMode(LEDB, OUTPUT); 
    
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
    
    Serial.println("Starting inferencing in "+String(myDelay)+" microseconds...");

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

    display.setCursor(20,10);
    display.println("Rocksetta");


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

        myGlobalCount += 1;
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
           // Determine the largest Height and the connected x,y,width
           if (myMaxHeight < (int)bb.height){ 
             myMaxHeight =  (int)bb.height;      
             myConnectedX = (int)bb.x; 
             myConnectedY = (int)bb.y; 
             myConnectedWidth = (int)bb.width; 
           } 
           display.drawRect(xMap, yMap, widthMap, heightMap, SSD1327_WHITE ); // good value
        } else {
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

     // Show some color on the onboard LED
/*
     if (myGlobalCount  == 1) { digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH);  digitalWrite(LEDB, LOW); }
     if (myGlobalCount  == 2) { digitalWrite(LEDR, HIGH); digitalWrite(LEDG, LOW);   digitalWrite(LEDB, HIGH);}
     if (myGlobalCount  > 2 && myGlobalCount  <= 5) {   digitalWrite(LEDR, HIGH); digitalWrite(LEDG, LOW);   digitalWrite(LEDB, LOW); }
     if (myGlobalCount  > 5 && myGlobalCount  <= 8) {   digitalWrite(LEDR, LOW);  digitalWrite(LEDG, HIGH);  digitalWrite(LEDB, HIGH);}
     if (myGlobalCount  > 8 && myGlobalCount  <= 11) {  digitalWrite(LEDR, LOW);  digitalWrite(LEDG, HIGH);  digitalWrite(LEDB, LOW); }
     if (myGlobalCount  > 11 && myGlobalCount  <= 14) { digitalWrite(LEDR, LOW);  digitalWrite(LEDG, LOW);   digitalWrite(LEDB, HIGH);}
*/
   //  Leave full white to show LORA stop if we implement that.
   //  if (myGlobalCount  > 14) { digitalWrite(LEDR, LOW);  digitalWrite(LEDG, LOW);   digitalWrite(LEDB, LOW); }

   }

       // LET'S TRY SOME FUZZY LOGIC
        
        // RC car speed!
       // if (myMaxHeight > myMaxHeightOld ) {myPwmNow += 1;}  //If bounding box taller them make car go faster  
       // else { myPwmNow -= 1; }
       
      //  myPwmNow = (myMaxHeight * 5) - (myConnectedWidth * 3)  ;   // taller the bounding box faster the car
      
        myPwmNow = (myMaxHeight/myConnectedWidth) * 30  ;   // taller the bounding box faster the car
                                                            // wider the bounding box slower the car
   
        // check max and mins
        if (myPwmNow >  MY_PWM_MAX )  {
           myPwmNow = MY_PWM_MAX; 
        }
        if (myPwmNow < MY_PWM_MIN  )  {
           myPwmNow = MY_PWM_MIN;   
        }

        // RC car Direction! Based on x location of tallest bounding box
        if (myConnectedX > MY_MIDDLE_X + MY_SERVO_STRAIGHT_RANGE) {
          myServoNow += 3;   
          digitalWrite(LEDG, LOW);      // green right 
          display.setCursor(50, 50);
          display.println("Green go Right");
        }  //If bounding box taller them make car go faster  
        else if (myConnectedX < MY_MIDDLE_X - MY_SERVO_STRAIGHT_RANGE) { 
          myServoNow -= 3; 
          digitalWrite(LEDB, LOW);      // Blue left            
          display.setCursor(50, 50);
          display.println("Blue go Left");
        }
        else {
          myServoNow = MY_SERVO_STRAIGHT;
          digitalWrite(LEDB, LOW);      // Blue 
          digitalWrite(LEDG, LOW);      // green  
          display.setCursor(50, 50);
          display.println("Straight");
          }  
        // check max and mins
        if (myServoNow >  MY_SERVO_MAX )  {myServoNow = MY_SERVO_MAX;}
        if (myServoNow < MY_SERVO_MIN  )  {myServoNow = MY_SERVO_MIN;}
        
        
        display.setCursor(10,120);
        display.println("Servo:");
        display.setCursor(50, 120);
        display.println(String(myServoNow));
        display.setCursor(70, 120);
        display.println("PWM:");
        display.setCursor(100, 120);
        display.println(String(myPwmNow));

       // use this to show a variable in the middle of the screen 
       // display.setCursor(50, 50);
       // display.println(String(myConnectedX));






     // Last thing is to show the 128x128 GRAYSCALE OLED
     display.display();  // OLED 4 bit 16 color GRAYSCALE update
}
