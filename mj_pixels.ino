#include "Adafruit_WS2801.h"
#include "SPI.h"

/*****************************************************************************
  Pixel manipulation code by MJ
  
  Uses utility functions from Adafruit industries:

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

*****************************************************************************/

#define PIXELCOUNT 23

// GLOBALS

// Pins for the rotary controller
#define encoder0PinA  2
#define encoder0PinB  4

// Encoder position. Must be 'volatile' because it's used as a hardware interrupt
volatile unsigned int encoder0Pos = 0;
unsigned int lastEncoderPos = 0; // will be used to detect movement
int currentLED = 0; // LED currently lit
int nextLED = 0;    // LED to be lit

// Pins for the pixels
uint8_t dataPin  = 11;    // Yellow wire on Adafruit Pixels
uint8_t clockPin = 12;    // Green wire on Adafruit Pixels

// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
Adafruit_WS2801 strip = Adafruit_WS2801(PIXELCOUNT, dataPin, clockPin);

void setup() {

/*flag for removal
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif
*/
  // setup for rotary encoder
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  // set up interrupt; now encoder changes do not need to be part of loop
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2

  strip.begin();
//  Serial.begin(9600);
  Serial.print("start\n"); // just to look at the serial output

  // Update LED contents, to start they are all 'off'
  strip.show();
}


void loop() {

  nextLED = currentLED;
  if( encoder0Pos > lastEncoderPos ) {
    nextLED++;
    if( currentLED >= PIXELCOUNT ) { nextLED = 0; }
    lastEncoderPos = encoder0Pos;
  }
    
  if( encoder0Pos < lastEncoderPos ) {
    nextLED--;
    lastEncoderPos = encoder0Pos;
   if( currentLED < 0 ) { nextLED = PIXELCOUNT; }
  }

  strip.setPixelColor(currentLED, 0);
  strip.setPixelColor(nextLED, 100); 

  currentLED = nextLED;

  Serial.print(currentLED);

  strip.show();

}

// 
void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }

  // debug
  Serial.println (encoder0Pos, DEC);
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
