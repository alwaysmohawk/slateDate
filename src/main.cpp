//slateDate

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <FastLED.h>

#include <math.h>
#include "objective.h"

#define NUM_LEDS 16

#define DATA_PIN 4

#define BRIGHTNESS 200

#define maxBrightness 220
#define minBrightness 5

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

#define red 0
#define green 1

//gonna try a different technique for closenesses
#define far 2 //breathing brightness steps = .1 * 10
#define close 5
#define veryClose 10
#define urThere 20
#define noObj 30
   
 



  bool statusSet = 0;

  float distance = 0;
  int intDistance = 0;

  int ledMover = 0;

  int buttonPressed = 0;

  int goodEnough = 15;

  int currentObj = 5;
  
  bool breathingDirection = 1;

  int currentBrightness = 0;

  int brightnessStep = 10;

  int closeness = 999;

  int brightness = 50;

  int upOrDown = 1;


  int arrivalTimer = 60;

// what's the name of the hardware serial port?
//#define GPSSerial Serial1
HardwareSerial GPSSerial(2);

// Connect to the GPS on the hardware port
//Adafruit_GPS GPS(&GPSSerial);
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();


// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];


// Function to convert ddmm.ssss to decimal degrees
// float convertToDecimal(float value) {
//   int degrees = int(value / 100);      // Extracting degrees
//   float minutes = value - degrees * 100;  // Extracting minutes
//   float decimalDegrees = degrees + (minutes / 60);  // Conversion to decimal degrees
//   return decimalDegrees;
// }


void indicateObjective(){
  if(currentObj < 5){
  for(int i = 0; i < 16; i++){
    //set all pixels to green, less pixel 16 (that one's special)
    leds[i] = CRGB::Green;
  }
  leds[objLedArray[currentObj]] = CRGB::Blue;
  } else {
    for(int i = 0; i < 16; i++){
    //set all pixels to blue, less pixel 16 (that one's special)
    leds[i] = CRGB::Blue;
  }
  }

}

void indicateCompleted(){
  for(int x=0; x < 5; x++){
    if(completionArray[x]) leds[objLedArray[x]] = CRGB::Purple;
  }
}


void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
}

int whichObj(){
  int objToCheck = 0;
  for(int i = 0; i < 5; i++){
    Serial.print("gonna send this to distance func: lat1: "); Serial.print(objLatArray[objToCheck], 7); Serial.print(" , "); Serial.println(objLonArray[objToCheck], 7);
    distance = calculateDistance(objLatArray[objToCheck], objLonArray[objToCheck], GPS.latitudeDegrees, GPS.longitudeDegrees);
    Serial.print("distance to obj "); Serial.print(objToCheck); Serial.print(": "); Serial.println(distance);
    if(distance < 350){
      Serial.print("whichObj determined currentObj to be:"); Serial.println(objToCheck);
      return objToCheck;
    }
    objToCheck++;
  }

  //currently handling this in main loop
  // if(distance < 350) closeness = far;
  // else if(distance < 150) closeness = close;
  // else if(distance < 75) closeness = veryClose;
  // else if(distance < 25) closeness = urThere;
  Serial.print("end of whichObj determined currentObj to be:"); Serial.println(objToCheck);
  return objToCheck;
}

  //jeremy's place in decimal degrees 
  //43.66351364124962, -79.41824485327953

// void breathe(int closeness){
//   if(currentBrightness > 250) brightnessStep = brightnessStep * -1;
//   if(currentBrightness < 10) brightnessStep = brightnessStep * -1;
//   currentBrightness = currentBrightness + brightnessStep;
//   FastLED.setBrightness(currentBrightness);
//   delay(35);
// }

//let's try breathing again
void breathe(int closeness){
  int brightness = 0;
  for(int i=0; i < 20; i++){
    brightness +=10;
    FastLED.setBrightness(brightness);
    delay(1200);
    FastLED.show();
  }
}


void setup()
{
  //**********led setup*****************/

  Objective Objectives[5] = {
    Objective(43.6636272, -79.4172969, 0),
    Objective(43.666558417, -79.4195833, 4),
    Objective(43.66245890, -79.42328144, 8),
    Objective(43.65922697, -79.4207747, 12),
    Objective(43.66198487, -79.41385822, 16),
  };





  //add leds and set brightness
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(BRIGHTNESS);

  //************end led setup************//

  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  
  /* lets comment out the debugging

  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);

  */

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // lat2 = GPS.latitudeDegrees;
  // lon2 = GPS.longitudeDegrees;
  // approximately every 700ms or so, print out the current stats
  if (millis() - timer > 50) {
    timer = millis(); // reset the timer

    Serial.print("Fix: "); Serial.println((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      //Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      Serial.print("currentObj = "); Serial.println(currentObj);
      
      currentObj = whichObj();

      //FastLED.clear();
      
      if(currentObj < 5){
            if (distance <= 300 && distance > 150) closeness = far;
            else if (distance <= 150 && distance > 100) closeness = close;
            else if (distance <= 100 && distance > 50) closeness = veryClose;
            else if (distance <50){
                    closeness = urThere;
                    completionArray[currentObj] = 1;
                    arrivalTimer --;
                    if(arrivalTimer <0){
                      objLatArray[currentObj] = 0;
                      objLonArray[currentObj] = 0;
                      arrivalTimer = 60;
                    }
                  }
        indicateObjective();
        indicateCompleted();
        if (brightness >= maxBrightness) upOrDown = upOrDown * -1;
        if (brightness <= minBrightness) upOrDown = upOrDown * -1;
        brightness = brightness + (closeness * upOrDown);
        Serial.print("brightness: "); Serial.println(brightness);
        Serial.print("upOrDown: "); Serial.println(upOrDown);
        if(completionArray[0] == 1){
          if(completionArray[1] == 1){
            if(completionArray[2] == 1){
              if(completionArray[3] == 1){
                if(completionArray[4] == 1){
                  pride();
                }
              }
            }
          }
        }
        FastLED.setBrightness(abs(brightness));
        FastLED.show();
      } 
      
      

    } else {
      //run a lil animation while waiting for fix
      if(ledMover == 17) ledMover = 0;
      leds[ledMover] = CRGB::Red;
      leds[ledMover-1] = CRGB::Blue;
      ledMover++;
      FastLED.show();
      }
  }

  // Convert GPS coordinates to decimal degrees


  //lat2 = GPS.latitude_fixed*multiplier;
  
