#include "Adafruit_WS2801.h"
#include "SPI.h"

/*****************************************************************************
  Lamp controller code by MJ
  
  Uses utility functions from Adafruit industries:
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

  Pin read interrupt and rotary encoder code adapted from Simon Merrett 

*****************************************************************************/

#define PIXELCOUNT 23

// GLOBALS

// Globals for rotary controller
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncoderPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Globals for LED neopixels
uint8_t dataPin  = 11;    // Yellow wire on Adafruit Pixels
uint8_t clockPin = 12;    // Green wire on Adafruit Pixels
int currentLED = 0; // LED currently lit
int nextLED = 0;    // LED to be lit

// construct global neopixel object "strip"
Adafruit_WS2801 strip = Adafruit_WS2801(PIXELCOUNT, dataPin, clockPin);

void setup() {

  // setup for rotary encoder
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  // initialized LED strip. 
  strip.begin();

  // Update LED contents, to start they are all 'off'
  strip.show();

  Serial.begin(9600);
  Serial.print("start\n"); 
  
}


void loop() {

  nextLED = currentLED;
  if( encoderPos > oldEncoderPos ) {
    nextLED++;
    if( nextLED >= PIXELCOUNT ) { nextLED = 0; }
    oldEncoderPos = encoderPos;
  }
    
  if( encoderPos < oldEncoderPos ) {
    nextLED--;
    oldEncoderPos = encoderPos;
   if( nextLED < 0 ) { nextLED = PIXELCOUNT; }
  }

  strip.setPixelColor(currentLED, 0);
  strip.setPixelColor(nextLED, 100); 

  currentLED = nextLED;

  Serial.println(currentLED);

  strip.show();

}


/* strip API reference
 *  strip.setPixelColor(n, r, g, b) (or can be a 32 bit int)
 *  uint32_t c = strip.getPixelColor( n )
 *  uint16_t n = strip.numPixels();
 */


/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
