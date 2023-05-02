/// @file    FirstLight.ino
/// @brief   Animate a white dot moving along a strip of LEDs
/// @example FirstLight.ino

// Use if you want to force the software SPI subsystem to be used for some reason (generally, you don't)
// #define FASTLED_FORCE_SOFTWARE_SPI
// Use if you want to force non-accelerated pin access (hint: you really don't, it breaks lots of things)
// #define FASTLED_FORCE_SOFTWARE_SPI
// #define FASTLED_FORCE_SOFTWARE_PINS
#include <FastLED.h>

///////////////////////////////////////////////////////////////////////////////////////////
//
// Move a white dot along the strip of leds.  This program simply shows how to configure the leds,
// and then how to turn a single pixel white and then off, moving down the line of pixels.
// 

#define NUM_LEDS 16

#define DATA_PIN D3

#define BRIGHTNESS 5


// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

//speed variable is the delay in the lighting loop. smaller speed value will be faster
//spinning
int speed1 = 600;
int speed2 = 450;
int speed3 = 300;
int speed4 = 150;
int speed5 = 50;
int speed6 = 10;

//distance variable is distance from obj, calculated by gps function
int distance = 200;

int y = 0;
int i = 0;

// This function sets up the ledsand tells the controller about them
void setup() {
	// sanity check delay - allows reprogramming if accidently blowing power w/leds
   	delay(500);
    
    

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(BRIGHTNESS);  
  Serial.begin(9600);


  //set all leds blue, this will be the "background"
  for(int x = 0; x < NUM_LEDS; x++) {
      // set all leds to blue
      leds[x] = CRGB::Blue;
   }

   FastLED.show();
}


void loop() {

  //distance from objective
  //distance == 10;

  
  
  Serial.println(y);
  Serial.println(i);

  int speedz[6] = {speed1, speed2, speed3, speed4, speed5, speed6};


  if(i == 6){
    i = 0;
  }

  //speed is how fast the indicator led rings around, 1 is slow, 2 is faster etc.
  if(distance > 100){
    //blink all leds red every idk 300ms
  } else if(distance >= 75 && distance <= 100 ){
    //send the one led around at speed 1
  } else if(distance >= 50 && distance <= 75){
    //send the one led around at speed 2
  } else if(distance >= 20 && distance <= 50){
    //send the one led around at speed 3
  } else if(distance >= 10 && distance <= 20){
    //send the one led around at speed 4
  } else if(distance >= 5 && distance <= 10){
    //send the one led around at speed 5
  } else if(distance < 5){
    //send the one led around at speed 6
  }

   // Move a single white led 
   for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
      // Turn our current led on to white, then show the leds
      leds[whiteLed] = CRGB::White;

      // Show the leds (only one of which is set to white, from above)
      FastLED.show();

      // Wait a little bit
      delay(speedz[i]);

      // Turn our current led back to blue for the next loop around
      leds[whiteLed] = CRGB::Blue;
   }

  for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
      // Turn our current led on to white, then show the leds
      leds[whiteLed] = CRGB::White;

      // Show the leds (only one of which is set to white, from above)
      FastLED.show();

      // Wait a little bit
      delay(speedz[i]);

      // Turn our current led back to blue for the next loop around
      leds[whiteLed] = CRGB::Blue;
   }
i ++;
   delay(200);
}