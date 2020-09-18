// 'roboface' example sketch for Adafruit I2C 8x8 LED backpacks:
//
//  www.adafruit.com/products/870   www.adafruit.com/products/1049
//  www.adafruit.com/products/871   www.adafruit.com/products/1050
//  www.adafruit.com/products/872   www.adafruit.com/products/1051
//  www.adafruit.com/products/959   www.adafruit.com/products/1052
//
// Requires Adafruit_LEDBackpack and Adafruit_GFX libraries.
// For a simpler introduction, see the 'matrix8x8' example.
//
// This sketch demonstrates a couple of useful techniques:
// 1) Addressing multiple matrices (using the 'A0' and 'A1' solder
//    pads on the back to select unique I2C addresses for each).
// 2) Displaying the same data on multiple matrices by sharing the
//    same I2C address.
//
// This example uses 5 matrices at 4 addresses (two share an address)
// to animate a face:
//
//     0     0
//
//      1 2 3
//
// The 'eyes' both display the same image (always looking the same
// direction -- can't go cross-eyed) and thus share the same address
// (0x70).  The three matrices forming the mouth have unique addresses
// (0x71, 0x72 and 0x73).
//
// The face animation as written is here semi-random; this neither
// generates nor responds to actual sound, it's simply a visual effect
// Consider this a stepping off point for your own project.  Maybe you
// could 'puppet' the face using joysticks, or synchronize the lips to
// audio from a Wave Shield (see wavface example).  Currently there are
// only six images for the mouth.  This is often sufficient for simple
// animation, as explained here:
// http://www.idleworm.com/how/anm/03t/talk1.shtml
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing
// products from Adafruit
//
// Written by P. Burgess for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// Because the two eye matrices share the same address, only four
// matrix objects are needed for the five displays:
#define MATRIX_EYES         0
#define MATRIX_MOUTH_LEFT   1
#define MATRIX_MOUTH_MIDDLE 2
#define MATRIX_MOUTH_RIGHT  3
Adafruit_8x8matrix matrix[4] = { // Array of Adafruit_8x8matrix objects
  Adafruit_8x8matrix(), Adafruit_8x8matrix(),
  Adafruit_8x8matrix(), Adafruit_8x8matrix() };
#define EYE_MIN              1
#define EYE_MAX              5

// Rather than assigning matrix addresses sequentially in a loop, each
// has a spot in this array.  This makes it easier if you inadvertently
// install one or more matrices in the wrong physical position --
// re-order the addresses in this table and you can still refer to
// matrices by index above, no other code or wiring needs to change.
static const uint8_t matrixAddr[] = { 0x70, 0x71, 0x72, 0x73 };

static const uint8_t PROGMEM // Bitmaps are stored in program memory
  blinkImg[][8] = {    // Eye animation frames
  { B00111100,         // Fully open eye
    B01111110,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B01111110,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000000,
    B00111100,
    B11111111,
    B11111111,
    B11111111,
    B00111100,
    B00000000 },
  { B00000000,
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B01111110,
    B00011000,
    B00000000 },
  { B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000 } },

    relaxImg[][8] = {    // Eye animation frames
  { B00000000,         // Fully open eye
    B00000000,
    B00000000,
    B01111110,
    B11111111,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B01111110,
    B11111111,
    B00111100,
    B00000000 },
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B11111111,
    B01111110,
    B00011000,
    B00000000 },
  { B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000 } },
    
    closedImg[][8] = {    // Eye animation frames
  { B00000000,         // slightly open eye
    B00000000,
    B00000000,
    B00000000,
    B11000011,
    B01111110,
    B00111100,
    B00000000 } },

    sad1Img[][8] = {    // Eye animation frames
  { B00000001,         // Fully open eye
    B00000011,
    B00000111,
    B00011111,
    B01111111,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000011,
    B00000111,
    B00011111,
    B01111111,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000000,
    B00000111,
    B00011111,
    B01111111,
    B11111111,
    B00111100,
    B00000000 },
  { B00000000,
    B00000000,
    B00000000,
    B00011111,
    B01111111,
    B11111111,
    B00111000,
    B00000000 },
  { B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000 } },
    sad2Img[][8] = {    // Eye animation frames
  { B10000000,         // Fully open eye
    B11000000,
    B11100000,
    B11111000,
    B11111110,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B11000011,
    B11100000,
    B11111000,
    B11111110,
    B11111111,
    B01111110,
    B00111100 },
  { B00000000,
    B00000000,
    B11100000,
    B11111000,
    B11111110,
    B11111111,
    B00111100,
    B00000000 },
  { B00000000,
    B00000000,
    B00000000,
    B11111000,
    B11111110,
    B11111111,
    B00011100,
    B00000000 },
  { B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000 } },
  // mouthImg[][24] = {                 // Mouth animation frames
  // { B00000000, B00000000, B00000000, // Mouth position A
  //   B00000000, B00000000, B00000000,
  //   B01111111, B11111111, B11111110,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000 },
  // { B00000000, B00000000, B00000000, // Mouth position B
  //   B00000000, B00000000, B00000000,
  //   B00111111, B11111111, B11111100,
  //   B00000111, B00000000, B11100000,
  //   B00000000, B11111111, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000 },
  // { B00000000, B00000000, B00000000, // Mouth position C
  //   B00000000, B00000000, B00000000,
  //   B00111111, B11111111, B11111100,
  //   B00001000, B00000000, B00010000,
  //   B00000110, B00000000, B01100000,
  //   B00000001, B11000011, B10000000,
  //   B00000000, B00111100, B00000000,
  //   B00000000, B00000000, B00000000 },
  // { B00000000, B00000000, B00000000, // Mouth position D
  //   B00000000, B00000000, B00000000,
  //   B00111111, B11111111, B11111100,
  //   B00100000, B00000000, B00000100,
  //   B00010000, B00000000, B00001000,
  //   B00001100, B00000000, B00110000,
  //   B00000011, B10000001, B11000000,
  //   B00000000, B01111110, B00000000 },
  // { B00000000, B00000000, B00000000, // Mouth position E
  //   B00000000, B00111100, B00000000,
  //   B00011111, B11000011, B11111000,
  //   B00000011, B10000001, B11000000,
  //   B00000000, B01111110, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000 },
  // { B00000000, B00111100, B00000000, // Mouth position F
  //   B00000000, B11000011, B00000000,
  //   B00001111, B00000000, B11110000,
  //   B00000001, B00000000, B10000000,
  //   B00000000, B11000011, B00000000,
  //   B00000000, B00111100, B00000000,
  //   B00000000, B00000000, B00000000,
  //   B00000000, B00000000, B00000000 } };
  // mouthImg[][24] = {                 // Mouth animation frames
  // { B01011011, B11011011, B11011010, // Mouth position A
  //   B11011011, B11011011, B11011011,
  //   B00000011, B11011011, B11000000,
  //   B11011000, B00000000, B00011011,
  //   B11011011, B11011011, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B01011011, B11011011, B11011010 },
  // { B01011011, B11011011, B11011010, // Mouth position B
  //   B11011011, B11011011, B11011011,
  //   B00000000, B00000000, B00000000,
  //   B11011000, B11011011, B00011011,
  //   B11011011, B00000000, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B01011011, B11011011, B11011010 },
  // { B01011011, B11011011, B11011010, // Mouth position C
  //   B11011011, B11011011, B11011011,
  //   B00000000, B00000000, B00000000,
  //   B11010011, B11011011, B11001011,
  //   B11011001, B11011011, B10011011,
  //   B11011010, B00111100, B01011011,
  //   B11011011, B11000011, B11011011,
  //   B01011011, B11011011, B11011010 },
  // { B01011011, B11011011, B11011010, // Mouth position D
  //   B11011011, B11011011, B11011011,
  //   B00000000, B00000000, B00000000,
  //   B11011011, B11011011, B11011011,
  //   B11001011, B11011011, B11010011,
  //   B11010011, B11011011, B11001011,
  //   B11011000, B11011011, B00011011,
  //   B01011011, B10000001, B11011010 },
  // { B01011011, B11011011, B11011010, // Mouth position E
  //   B11011011, B11000011, B11011011,
  //   B00000000, B00111100, B00000000,
  //   B11011000, B01011010, B00011011,
  //   B11011011, B10000001, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B01011011, B11011011, B11011010 },
  // { B01011011, B11000011, B11011010, // Mouth position F
  //   B11011011, B00111100, B11011011,
  //   B00000000, B11011011, B00000000,
  //   B11011010, B11011011, B01011011,
  //   B11011011, B00111100, B11011011,
  //   B11011011, B11000011, B11011011,
  //   B11011011, B11011011, B11011011,
  //   B01011011, B11011011, B11011010 } };

  //   mouthImg[][24] = {                 // Mouth animation frames
  // { B11101111, B11101111, B01011110, // Mouth position A
  //   B11101111, B11101111, B11011111,
  //   B11001111, B11101111, B11011111,
  //   B11011111, B11101111, B11011111,
  //   B11011111, B11101111, B11011111,
  //   B11011111, B11101111, B11001111,
  //   B11011111, B11101111, B11101111,
  //   B01011110, B11101111, B11101111 },
  // { B11001111, B11010111, B01011110, // Mouth position B
  //   B11001111, B11010111, B11011111,
  //   B11001111, B11010111, B11011111,
  //   B11011111, B11010111, B11011111,
  //   B11011111, B11010111, B11011111,
  //   B11011111, B11010111, B11001111,
  //   B11011111, B11010111, B11001111,
  //   B01011110, B11010111, B11001111 },
  // { B11011011, B11011011, B01011110, // Mouth position C
  //   B11010111, B11011011, B11011111,
  //   B11000111, B11011101, B11011111,
  //   B11001111, B11011101, B11011111,
  //   B11011111, B11011101, B11001111,
  //   B11011111, B11011101, B11000111,
  //   B11011111, B11011011, B11010111,
  //   B01011110, B11011011, B11011011 },
  // { B11011101, B11011101, B01011110, // Mouth position D
  //   B11011101, B11011110, B11011111,
  //   B11011011, B11011110, B11001111,
  //   B11011011, B11011110, B11010111,
  //   B11010111, B11011110, B11011011,
  //   B11001111, B11011110, B11011011,
  //   B11011111, B11011110, B11011101,
  //   B01011110, B11011101, B11011101 },
  // { B11011111, B11001111, B01011110, // Mouth position E
  //   B11011111, B11010111, B11011111,
  //   B11011111, B10110111, B11011111,
  //   B11011111, B10110111, B11011111,
  //   B11011111, B10110111, B11011111,
  //   B11011111, B10110111, B11011111,
  //   B11011111, B11010111, B11011111,
  //   B01011110, B11001111, B11011111 },
  // { B11001111, B10110111, B01011110, // Mouth position F
  //   B11011111, B10110111, B11011111,
  //   B11011111, B01111011, B11011111,
  //   B11011111, B01111011, B11011111,
  //   B11011111, B01111011, B11011111,
  //   B11011111, B01111011, B11011111,
  //   B11011111, B10110111, B11011111,
  //   B01011110, B10110111, B11001111 } };
   mouthImg_s[][24] = {                 // Mouth animation frames
  { B00010000, B00100000, B00000000, // Mouth position A
    B00010000, B01100000, B01111110,
    B00000000, B01000000, B11001001,
    B00000000, B01000000, B00011000,
    B00000000, B01000000, B00010000,
    B00000000, B01100000, B00110000,
    B00000000, B00100000, B00100000,
    B00000000, B00110000, B00100000 },
  { B00110000, B00101000, B00000000, // Mouth position B
    B00110000, B00101000, B01111110,
    B00110000, B00101000, B11001001,
    B00000000, B00101000, B00011000,
    B00000000, B00101000, B00010000,
    B00000000, B00101000, B00110000,
    B00000000, B00101000, B00110000,
    B00000000, B00101000, B00110000 },
  { B00100100, B00100100, B00000000, // Mouth position C
    B00010100, B01000100, B01111110,
    B00010100, B01000100, B11001101,
    B00001000, B01000100, B00010100,
    B00000100, B01000100, B00010100,
    B00000000, B01000100, B00100100,
    B00000000, B01000100, B00100100,
    B00000000, B00100100, B00100100 },
  { B00100010, B00100010, B00000000, // Mouth position D
    B00100010, B00100001, B01111110,
    B00100010, B01000001, B11001011,
    B00010100, B01000001, B00010010,
    B00001100, B01000001, B00010010,
    B00000000, B01000001, B00110010,
    B00000000, B00100001, B00100010,
    B00000000, B00100010, B00100010 },
  { B00100000, B00110000, B00000000, // Mouth position E
    B00100000, B00101000, B01111110,
    B00100000, B01001000, B11001001,
    B00000000, B01001000, B00011000,
    B00000000, B01001000, B00010000,
    B00000000, B01001000, B00110000,
    B00000000, B00101000, B00100000,
    B00000000, B00110000, B00100000 },
  { B00110000, B01001000, B00000000, // Mouth position F
    B00100000, B01001000, B01111110,
    B00100000, B10000100, B11001001,
    B00000000, B10000100, B00001000,
    B00000000, B10000100, B00011000,
    B00000000, B10000100, B00010000,
    B00000000, B01001000, B00110000,
    B00000000, B01001000, B00110000 } },
    mouthImg_a[][24] = {                 // Mouth animation frames
  { B00010000, B00010000, B00000000, // Mouth position A
    B00010000, B00010000, B00000000,
    B00110000, B00010000, B00000000,
    B00100000, B00010000, B00100000,
    B00100000, B00010000, B00100000,
    B00000000, B00010000, B00110000,
    B00000000, B00010000, B00010000,
    B00000000, B00010000, B00010000 },
  { B00110000, B00101000, B00000000, // Mouth position B
    B00110000, B00101000, B00000000,
    B00110000, B00101000, B00000000,
    B00100000, B00101000, B00100000,
    B00100000, B00101000, B00100000,
    B00000000, B00101000, B00110000,
    B00000000, B00101000, B00110000,
    B00000000, B00101000, B00110000 },
  { B00100100, B00100100, B00000000, // Mouth position C
    B00101000, B00100100, B00000000,
    B00111000, B00100010, B00000000,
    B00110000, B00100010, B00100000,
    B00100000, B00100010, B00110000,
    B00000000, B00100010, B00111000,
    B00000000, B00100100, B00101000,
    B00000000, B00100100, B00100100 },
  { B00100010, B00100010, B00000000, // Mouth position D
    B00100010, B00100001, B00000000,
    B00100100, B00100001, B00000000,
    B00100100, B00100001, B00111000,
    B00111000, B00100001, B00100100,
    B00000000, B00100001, B00100100,
    B00000000, B00100001, B00100010,
    B00000000, B00100010, B00100010 },
  { B00100000, B00110000, B00000000, // Mouth position E
    B00100000, B00101000, B00000000,
    B00100000, B01001000, B00000000,
    B00100000, B01001000, B00100000,
    B00100000, B01001000, B00100000,
    B00000000, B01001000, B00100000,
    B00000000, B00101000, B00100000,
    B00000000, B00110000, B00100000 },
  { B00110000, B01001000, B00000000, // Mouth position F
    B00100000, B01001000, B00000000,
    B00100000, B10000100, B00000000,
    B00100000, B10000100, B00100000,
    B00100000, B10000100, B00100000,
    B00000000, B10000100, B00100000,
    B00000000, B01001000, B00100000,
    B00000000, B01001000, B00110000 } },
    mouthImg[][24] = {                 // Mouth animation frames
  { B00010000, B00010000, B00100000, // Mouth position A
    B00010000, B00010000, B00100000,
    B00110000, B00010000, B00100000,
    B00100000, B00010000, B00100000,
    B00100000, B00010000, B00100000,
    B00100000, B00010000, B00110000,
    B00100000, B00010000, B00010000,
    B00100000, B00010000, B00010000 },
  { B00110000, B00101000, B00100000, // Mouth position B
    B00110000, B00101000, B00100000,
    B00110000, B00101000, B00100000,
    B00100000, B00101000, B00100000,
    B00100000, B00101000, B00100000,
    B00100000, B00101000, B00110000,
    B00100000, B00101000, B00110000,
    B00100000, B00101000, B00110000 },
  { B00100100, B00100100, B00100000, // Mouth position C
    B00101000, B00100100, B00100000,
    B00111000, B00100010, B00100000,
    B00110000, B00100010, B00100000,
    B00100000, B00100010, B00110000,
    B00100000, B00100010, B00111000,
    B00100000, B00100100, B00101000,
    B00100000, B00100100, B00100100 },
  { B00100010, B00100010, B00100000, // Mouth position D
    B00100010, B00100001, B00100000,
    B00100100, B00100001, B00110000,
    B00100100, B00100001, B00101000,
    B00101000, B00100001, B00100100,
    B00110000, B00100001, B00100100,
    B00100000, B00100001, B00100010,
    B00100000, B00100010, B00100010 },
  { B00100000, B00110000, B00100000, // Mouth position E
    B00100000, B00101000, B00100000,
    B00100000, B01001000, B00100000,
    B00100000, B01001000, B00100000,
    B00100000, B01001000, B00100000,
    B00100000, B01001000, B00100000,
    B00100000, B00101000, B00100000,
    B00100000, B00101000, B00100000 },
  { B01010000, B01000100, B00100000, // Mouth position F
    B00100000, B01000010, B00100000,
    B00100000, B10000001, B00100000,
    B00100000, B10000001, B00100000,
    B00100000, B10000001, B00100000,
    B00100000, B10000001, B00100000,
    B00100000, B01000010, B00100000,
    B00100000, B01000100, B01010000 } };

uint8_t
  blinkIndex[] = { 1, 2, 3, 4, 3, 2, 1 }, // Blink bitmap sequence
  blinkCountdown = 100, // Countdown to next blink (in frames)
  relaxIndex[] = { 1, 2, 3, 4, 3, 2, 1 }, // Blink bitmap sequence
  gazeCountdown  =  75, // Countdown to next eye movement
  gazeJitterCountdown = 30,
  defaultGazeFrames     =  50, // Duration of eye movement (smaller = faster)
  gazeFrames = defaultGazeFrames,
  mouthPos       =   0, // Current image number for mouth
  mouthCountdown =  10, // Countdown to next mouth change
  mouthState = 255,
  faceState = 0,
  eyeState = 0,
  eyeClose = 0,
  pupilSize = 2;
int8_t
  eyeX = 3, eyeY = 3,   // Current eye position
  newX = 3, newY = 3,   // Next eye position
  dX   = 0, dY   = 0,   // Distance from prior to new position
  tX   = 0, tY   = 0,
  inX  = 3, inY  = 0;   // user-defined positions

int inputChar = 0;
boolean stringComplete = false;
boolean wanderingEye = true;
boolean changeGaze = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
//  inputString.reserve(200);

  // Seed random number generator from an unused analog input:
  randomSeed(analogRead(A0));

  // Initialize each matrix object:
  for(uint8_t i=0; i<4; i++) {
    matrix[i].begin(matrixAddr[i]);
    // If using 'small' (1.2") displays vs. 'mini' (0.8"), enable this:
    // matrix[i].setRotation(3);
  }
}

void loop() {
  delay(20);
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(500);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(500);     
  serialEvent();
  // if (stringComplete) {
  //   changeFaceState();  // depends on inputString
  // }
  // Draw eyeball in current state of blinkyness (no pupil).  Note that
  // only one eye needs to be drawn.  Because the two eye matrices share
  // the same address, the same data will be received by both.
  matrix[MATRIX_EYES].clear();
  // When counting down to the next blink, show the eye in the fully-
  // open state.  On the last few counts (during the blink), look up
  // the corresponding bitmap index.
  if (eyeClose == 0) {
    //Serial.println(blinkCountdown);
    matrix[MATRIX_EYES].drawBitmap(0, 0,
      blinkImg[
        (blinkCountdown < sizeof(blinkIndex)) ? // Currently blinking?
        blinkIndex[blinkCountdown] :            // Yes, look up bitmap #
        0                                       // No, show bitmap 0
      ], 8, 8, LED_ON);
//      if (blinkCountdown < sizeof(blinkIndex)) {
//        Serial.print("blinkIndex: ");
//        Serial.println(blinkIndex[blinkCountdown]);
//      }
  } else if (eyeClose == 1) {
    matrix[MATRIX_EYES].drawBitmap(0, 0,
      relaxImg[
        (blinkCountdown < sizeof(relaxIndex)) ? // Currently blinking?
        relaxIndex[blinkCountdown] :            // Yes, look up bitmap #
        0                                       // No, show bitmap 0
      ], 8, 8, LED_ON);
  } else if (eyeClose == 2) {
    matrix[MATRIX_EYES].drawBitmap(0, 0,
      closedImg[0], 8, 8, LED_ON);  // There's only one frame
  }

  else if (eyeClose == 3) {
    matrix[MATRIX_EYES].drawBitmap(0, 0,
      sad1Img[
        (blinkCountdown < sizeof(relaxIndex)) ? // Currently blinking?
        relaxIndex[blinkCountdown] :            // Yes, look up bitmap #
        0                                       // No, show bitmap 0
      ], 8, 8, LED_ON);
  }

  else if (eyeClose == 4) {
    matrix[MATRIX_EYES].drawBitmap(0, 0,
      sad2Img[
        (blinkCountdown < sizeof(relaxIndex)) ? // Currently blinking?
        relaxIndex[blinkCountdown] :            // Yes, look up bitmap #
        0                                       // No, show bitmap 0
      ], 8, 8, LED_ON);
  }
  // Decrement blink counter.  At end, set random time for next blink.
  if(--blinkCountdown == 0) blinkCountdown = random(5, 180);

  // Add a pupil (2x2 black square) atop the blinky eyeball bitmap.
  // Periodically, the pupil moves to a new position...
  normalEyes();
//   if(--gazeCountdown <= gazeFrames) {
// //    Serial.print(gazeCountdown);
// //    Serial.print(" | ");
// //    Serial.println(gazeFrames);
//     // Eyes are in motion - draw pupil at interim position
// //    matrix[MATRIX_EYES].fillRect(
// //      newX - (dX * gazeCountdown / gazeFrames),
// //      newY - (dY * gazeCountdown / gazeFrames),
// //      pupilSize, pupilSize, LED_OFF);
//     tX = dX * gazeCountdown / gazeFrames;
//     tY = dY * gazeCountdown / gazeFrames;
//     matrix[MATRIX_EYES].fillCircle(
//       newX - (dX * gazeCountdown / gazeFrames),
//       newY - (dY * gazeCountdown / gazeFrames),
//       pupilSize, LED_OFF);
// //    Serial.print("eye dx/dy: ");
// //    Serial.print(dX);
// //    Serial.print("/");
// //    Serial.print(dY);
// //    Serial.print(" | tx/ty: ");
// //    Serial.print(tX);
// //    Serial.print("/");
// //    Serial.println(tY);
//     if(gazeCountdown == 0) {    // Last frame?
//       eyeX = newX; eyeY = newY; // Yes.  What's new is old, then...
//       do { // Pick random positions until one is within the eye circle
         
//         if (eyeClose == 0) {
//           newY = random(7);
//           newX = random(7);
//         } else if (eyeClose == 1) {          
//           newX = random(7);
//           newY = random(4, 7);
//         } 
//         if (eyeState == 2) {           
//           newY = inY;
//           newX = inX;
//         }
//         dX   = newX - 3;  dY   = newY - 3;
//       } while((dX * dX + dY * dY) >= 10);      // Thank you Pythagoras
//       dX            = newX - eyeX;             // Horizontal distance to move
//       dY            = newY - eyeY;             // Vertical distance to move
//       gazeFrames    = random(3, 15);           // Duration of eye movement
//       if (eyeState != 2) gazeCountdown = random(gazeFrames, 120); // Count to end of next movement
//     }
//   } else {
// //    Serial.println("not in motion");
// //    Serial.print(gazeCountdown);
// //    Serial.print(" / ");
// //    Serial.println(gazeFrames);
//     // Not in motion yet -- draw pupil at current static position
//     //matrix[MATRIX_EYES].fillRect(eyeX, eyeY, pupilSize, pupilSize, LED_OFF);
//     matrix[MATRIX_EYES].fillCircle(eyeX, eyeY, pupilSize, LED_OFF);
//   }

  // Draw mouth, switch to new random image periodically
  if (faceState == 7) {
    drawMouth(mouthImg_s[mouthPos]);
  } else if (faceState == 8) {
    drawMouth(mouthImg_a[mouthPos]);
  } else {
    drawMouth(mouthImg[mouthPos]);
  }
  if (mouthState==255) {
    if(--mouthCountdown == 0) {
      mouthPos = random(6); // Random image
      // If the 'neutral' position was chosen, there's a 1-in-5 chance we'll
      // select a longer hold time.  This gives the appearance of periodic
      // pauses in speech (e.g. between sentences, etc.).
      mouthCountdown = ((mouthPos == 0) && (random(5) == 0)) ?
        random(10, 40) : // Longer random duration
        random(2, 8);    // Shorter random duration
    }
  }
  else {
    mouthPos = mouthState-1;
  }

  // Refresh all of the matrices in one quick pass
  for(uint8_t i=0; i<4; i++) matrix[i].writeDisplay();

  delay(20); // ~50 FPS
  
}

void normalEyes() {
//  Serial.print(gazeCountdown);
//  Serial.print(" | ");
//  Serial.println(gazeFrames);
  if(--gazeCountdown <= gazeFrames) {
//    Serial.print("gazeCountdown <= gazeFrames : ");
    
    // Eyes are in motion - draw pupil at interim position

//    tX = dX * gazeCountdown / gazeFrames;
//    tY = dY * gazeCountdown / gazeFrames;
    matrix[MATRIX_EYES].fillCircle(
      newX - (dX * gazeCountdown / gazeFrames),
      newY - (dY * gazeCountdown / gazeFrames),
      pupilSize, LED_OFF);
//    Serial.print("eye dx/dy: ");
//    Serial.print(dX);
//    Serial.print("/");
//    Serial.print(dY);
//    Serial.print(" | tx/ty: ");
//    Serial.print(tX);
//    Serial.print("/");
//    Serial.println(tY);
    
    if(gazeCountdown == 0) {    // Last frame?
//      Serial.println("Woot");
      
      eyeX = newX; eyeY = newY; // Yes.  What's new is old, then...    
      do { // Pick random positions until one is within the eye circle
//        Serial.print(".");
        if (eyeClose == 0) {
          newY = random(7);
          newX = random(7);
        } else if (eyeClose == 1) {          
          newX = random(7);
          newY = random(4, 7);
        }
        if (eyeState == 2) {
          newY = inY;
          newX = inX;
          break;
        }
        dX   = newX - 3;  dY   = newY - 3;
      } while((dX * dX + dY * dY) >= 10);      // Thank you Pythagoras
//      Serial.println();      
      dX            = newX - eyeX;             // Horizontal distance to move
      dY            = newY - eyeY;             // Vertical distance to move
      gazeFrames    = random(3, 15);           // Duration of eye movement
      gazeCountdown = random(gazeFrames, 120); // Count to end of next movement      
    }
  } else {
    // Not in motion yet -- draw pupil at current static position
    //matrix[MATRIX_EYES].fillRect(eyeX, eyeY, pupilSize, pupilSize, LED_OFF);
    if(--gazeJitterCountdown == 0) {
//      Serial.println("---");
//      Serial.print(eyeX);
//      Serial.print("/");
//      Serial.print(eyeY);
//      Serial.print(" --> ");
      eyeX = eyeX + random(2) - random(2);
      eyeY = eyeY + random(2) - random(2);
      constrain(eyeX, 2, 6);
      constrain(eyeY, 2, 6);
//      Serial.print(eyeX);
//      Serial.print("/");
//      Serial.println(eyeY);
      gazeJitterCountdown = random(50, 100);
    }
//    Serial.println("Drawing pupil");
    if(changeGaze) {
      gazeCountdown = gazeFrames+1;
      changeGaze=false;
    }
    
    matrix[MATRIX_EYES].fillCircle(eyeX, eyeY, pupilSize, LED_OFF);
  }
}

// Draw mouth image across three adjacent displays
void drawMouth(const uint8_t *img) {
  for(uint8_t i=0; i<3; i++) {
    matrix[MATRIX_MOUTH_LEFT + i].clear();
    matrix[MATRIX_MOUTH_LEFT + i].drawBitmap(i * -8, 0, img, 24, 8, LED_ON);
  }
}

void serialEvent() { 
  int input_size = 3;
  if(Serial.available() == 1) {
    
    char inChar = (char)Serial.read();
    inputChar = inChar;
    changeFaceState();
//    Serial.println(inputChar);

    // if (inChar == '\n') {
    //   stringComplete = true;
    // }
  } else if (Serial.available() >= input_size) {
//    int input_size = 3;
    Serial.println("Change gaze");
    eyeState = 2;
    changeGaze = true;
    gazeCountdown = 1;  // immediately change to this state (must be 1 because it'll be decremented)
    
    char input[input_size + 1];
    Serial.readBytes(input, input_size);
    input[input_size] = 0;
    char* command = strtok(input, "&");
    while(command != 0)
    {
      char* separator = strchr(command, ':');
      if (separator != 0) {
        *separator = 0;
        int xpos = atoi(command);         
        inX = constrain(xpos, EYE_MIN, EYE_MAX);
        ++separator;
        int ypos = atoi(separator);
        inY = constrain(ypos, EYE_MIN, EYE_MAX);
        Serial.print("xpos/ypos: ");
        Serial.print(inX);
        Serial.print("/");
        Serial.println(inY);
      }
      command = strtok(0, "&");
//      Serial.println("tick");
    }
    blinkCountdown = 8;
  }
}


void changeFaceState() {
//  Serial.print("Current face state: ");
//  Serial.println(faceState);
  switch (inputChar) {
    case 'q': // fully open eyes
      eyeClose = 0;
      Serial.println("Fully open eyes");
      break;
    case 'w': // slightly closed eyes
      Serial.println("Slightly closed eyes");
      eyeClose = 1;
      break;
    case 'e': // fully closed eyes
      Serial.println("Fully closed eyes");
      eyeClose = 2;
      break;
    case 'z': // sad eyes
      Serial.println("Sad eyes");
      eyeClose = 3;
      break;
    case 'x': // sad eyes 2
      Serial.println("Sad eyes 2");
      eyeClose = 4;
      break;
    case 'a':
      Serial.println("Random pupil movements");
      eyeState = 0;  // random pupil movement
      break;
    case 's':
      Serial.println("Manual pupil control");
      eyeState = 2;  // manual pupil control
      break;
    case 'r': // default mouth/smile
      Serial.println("Default mouth shape");
      faceState = 6;
      break;
    case 'f': // frown mouth
      Serial.println("Frowning mouth shape");
      faceState = 7;
      break;
    case 'g': // small mouth
      Serial.println("Small mouth shape");
      faceState = 8;
      break;
    case 'm':
      Serial.println("Random rambling");
      mouthState = 255;  // random mouth mode
      break;
    case 'i':
      Serial.println("Mouth pose A");
      mouthState = 1;   // A
      break;
    case 'o':
      Serial.println("Mouth pose B");
      mouthState = 2;   // B
      break;
    case 'p':
      Serial.println("Mouth pose C");
      mouthState = 3;   // C
      break;
    case 'j':
      Serial.println("Mouth pose D");
      mouthState = 4;   // D
      break;
    case 'k':
      Serial.println("Mouth pose E");
      mouthState = 5;   // E
      break;
    case 'l':
      Serial.println("Mouth pose F");
      mouthState = 6;   // F
      break;
    case 'b':
      Serial.println("Blink");
      blinkCountdown = sizeof(blinkIndex) + 1;
      break;
    default:
      Serial.println("(Default) Random eye and mouth movements");
      faceState = 0;
      eyeClose = 0;
      eyeState = 0;      
      mouthState = 255;
      break;
  }
  // stringComplete = false;
//  Serial.print("New face state: ");
//  Serial.println(faceState); 
}
