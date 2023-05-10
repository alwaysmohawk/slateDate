/*
//slateDate

#include <FastLED.h>

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
  //add leds and set brightness
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(BRIGHTNESS);  

  //start Serial for debugging
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

   // Move a single green led 
   for(int greenLed = 0; greenLed < NUM_LEDS; greenLed++) {
      // Turn our current led on to green, then show the leds
      leds[greenLed] = CRGB::Green;
      FastLED.show();

      // Wait a little bit
      delay(speedz[i]);

      // Turn our current led back to blue for the next loop around
      leds[greenLed] = CRGB::Blue;
   }

  for(int greenLed = 0; greenLed < NUM_LEDS; greenLed++) {
      // Turn our current led on to green, then show the leds
      leds[greenLed] = CRGB::Green;
      FastLED.show();

      // Wait a little bit
      delay(speedz[i]);

      // Turn our current led back to blue for the next loop around
      leds[greenLed] = CRGB::Blue;
   }
i ++;
   delay(200);
}

*/


// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <FastLED.h>

#include <math.h>

#define NUM_LEDS 16

#define DATA_PIN D3

#define BRIGHTNESS 5

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

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
float convertToDecimal(float value) {
  int degrees = int(value / 100);      // Extracting degrees
  float minutes = value - degrees * 100;  // Extracting minutes
  float decimalDegrees = degrees + (minutes / 60);  // Conversion to decimal degrees
  return decimalDegrees;
}

// Function to calculate distance between two GPS points
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float EarthRadiusKm = 6371.0;  // Radius of the Earth in kilometers
  const float MetersPerKm = 1000.0;    // Conversion factor from kilometers to meters

  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);

  float dlat = lat2Rad - lat1Rad;  // Difference in latitude
  float dlon = lon2Rad - lon1Rad;  // Difference in longitude

  float a = pow(sin(dlat / 2), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = EarthRadiusKm * c * MetersPerKm;  // Calculating distance in meters

  return distance;
}

  //jeremy's place in decimal degrees 
  //43.66351364124962, -79.41824485327953


  float lat1 = 43.66351364124962;
  float lon1 = -79.41824485327953;
  float lat2 = 4030.750;
  float lon2 = -7515.125;

  float multiplier = 0.0000001000;


void setup()
{

  //**********led setup*****************//

  //add leds and set brightness
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(BRIGHTNESS);
  //set all leds blue, this will be the "background"
  for(int x = 0; x < NUM_LEDS; x++) {
      // set all leds to blue
      leds[x] = CRGB::Blue;
   }
   FastLED.show();

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

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

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

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

  /* comment out the time/date stuff

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);

   end commenting out time/date stuff*/

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
    }

  // Convert GPS coordinates to decimal degrees

  //lat2 = convertToDecimal(GPS.latitude);
  //lon2 = convertToDecimal(GPS.longitude);
  //lon2 = lon2 * -1.00000;

  //lat2 = GPS.latitude_fixed*multiplier;
   lat2 = GPS.latitudeDegrees;
  lon2 = GPS.longitudeDegrees;

  Serial.print("lat1,lon1:");
  Serial.print(lat1, 7);
  Serial.print(" , ");
  Serial.println(lon1, 7);

  Serial.print("lat2,lon2:");
  Serial.print(lat2, 7);
  Serial.print(" , ");
  Serial.println(lon2, 7);

  // Calculate distance between the two GPS points
  float distance = calculateDistance(lat1, lon1, lat2, lon2);

  Serial.print("Distance between the two points: ");
  Serial.print(distance, 5);  // Print distance with 2 decimal places
  Serial.println(" meters");
  }
}

