/*****************************************************************************

  Arduino Vaporizer (Mini)
  Adafruit OLED Feather Wing
  :: Alpha Version ::

******************************************************************************/

static const unsigned char PROGMEM  batt[]   =
{
  0b00001111, 0b00000000, //     ####    
  0b00001111, 0b00000000, //     ####    
  0b11111111, 0b11110000, // ############
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b10000000, 0b00010000, // #          #
  0b11111111, 0b11110000, // ############
};

static const unsigned char PROGMEM  temp[]   =
{
  0b00011111, 0b10000000, //    ######   
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b10110000, //    #    # ##
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b11100000, //    #    ### 
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b10110000, //    #    # ##
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b11100000, //    #    ### 
  0b00010000, 0b10000000, //    #    #   
  0b00010000, 0b10110000, //    #    # ##
  0b00010000, 0b10000000, //    #    #   
  0b00110000, 0b11000000, //   ##    ##  
  0b01100000, 0b01100000, //  ##      ## 
  0b11001111, 0b00110000, // ##  ####  ##
  0b10011111, 0b10010000, // #  ######  #
  0b10110011, 0b11010000, // # ##  #### #
  0b10110011, 0b11010000, // # ##  #### #
  0b10111111, 0b11010000, // # ######## #
  0b10111111, 0b11010000, // # ######## #
  0b10011111, 0b10010000, // #  ######  #
  0b11001111, 0b00110000, // ##  ####  ##
  0b01100000, 0b01100000, //  ##      ## 
  0b00111111, 0b11000000, //   ########  
};

/* Includes */
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Filters.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Hardware */
Adafruit_SSD1306 display = Adafruit_SSD1306();

/* Pin Mapping */
#define rdboardLED     13       // On-board LED
#define tempButtonA    9        // A. Sets Temperature
#define hitsButtonB    6        // B. Resets Hits
#define scrnButtonC    5        // C. Screen Power
#define fireButtonE    00       // E. Fire Button
#define vibeMotorPin   12       // Vibration motor
#define mosfetPin      10       // Output to Coil Pin
#define battPin        A1       // Monitors Voltage

/* Enable / Disable */
#define VBAT_ENABLED  1         // Enable / Disable integrated batt mgmt

/* Draws Battery Level in Icon */
#define battStartX    4
#define battStartY    25
#define battWidth     8

/* Draws Temp Level in Icon */
#define tempStartX    53
#define tempStartY    6
#define tempWidth     2
#define tempMaxHeight 13

/* Debounce / Button Hold */
#define debounce      10        // prevent button noise

/* Temperature Variables (A) */
int tButtonCount = 1;           // press counter
int tButtonState = 0;           // current state
int tButtonPrev = 0;            // previous state
int firePower = 0;              // power output to mosfet

/* Hit Counter Variables (B) */
int hButtonCount = 0;
int hButtonState = 0;
int hButtonPrev = 0;

/* Screen Variables      (C) */
int sButtonCount = 0;
int sButtonState = 0;
int sButtonPrev = 0;
int screenOff = 0;

/* Fire Button Variables (E) */
int fButtonCount = 25;
int fButtonState = 0;
int fButtonPrev = 0;

/* Battery Calculations */
int battRead;                 // Mapped Voltage for LED
int battSingleRead;           // Actual Voltage x1000
int avgVoltRead = 0;          // the average used
const int numReadings = 85;   // smooths voltage readings


/************  SETUP  ************/

void setup()
{ 
  /* Set Pin Input/Output & Pullups */
  pinMode(battPin,     INPUT);
  pinMode(tempButtonA, INPUT);
  pinMode(hitsButtonB, INPUT_PULLUP);
  pinMode(scrnButtonC, INPUT_PULLUP);
  pinMode(fireButtonE, INPUT_PULLUP);  
  pinMode(rdboardLED,  OUTPUT);
  pinMode(mosfetPin,   OUTPUT);
  pinMode(vibeMotorPin,OUTPUT);

  /* Resistors & References */
  // Nothing Here
  
  /* Setup Display */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);     // Initialize I2C addr 0x3C (for 128x32)
  
  // display.invertDisplay(true);                // White screen logo
  // display.setRotation(2);                     // Rotate Display: 0, 90, 180 or 270* (0,1,2,3)
  
  display.display();                             // Display splashscreen
  delay(1000);                                   // Time to display splash screen
  display.clearDisplay();                        // Clear buffer
}


/************  PROGRAM  ************/

void loop()
{
  Serial.begin(9600);  

  /* Battery Functions */
  battSingleRead = getBatteryVoltage();         // Reads Voltage
  Smooth();                                     // Removes Jitter from Voltage Reads
  LowBattery();                                 // Haptic notification to recharge
  BattAdjust();                                 // Set Screen Readout
  Serial.println(avgVoltRead);

  /* Button Functions */
  FireCoil();                                   // Heats Coil
  ButtonReader();                               // Check button presses
  ResetCount();                                 // Reset button counters

  /* Conditionally Display Main Menu */
  if (sButtonCount <= 1) {
    display.invertDisplay(false);
    MainMenu();               // Render screen icons & text
    TempAdjust();
  }

  if (sButtonCount == 2) {
    display.invertDisplay(true);
    MainMenu();
    TempAdjust();
  }

  if (sButtonCount == 3 && screenOff == 0 ) {
    screenOff++;
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(18, 13);
    display.println("Off To Sleep...");
    display.display();
    delay(2000);
    display.clearDisplay();
    display.invertDisplay(false);
    display.setCursor(18, 13);
    display.println("Press to Wake...");
    display.display();
    delay(1500);
    yield();
  }

  if (sButtonCount >= 3 && screenOff == 1 ) {
    display.clearDisplay();
    display.display();
    delay(1);
  }
}

/************  FUNCTIONS  ************/

/* Creates Main Interface  */
void MainMenu()
{
  // Draw Icons & Line
  display.drawBitmap(2, 4,  batt, 12, 24, 1);
  display.drawBitmap(48, 4,  temp, 12, 25, 1);
  display.drawFastVLine(95, 4, 26, WHITE);

  // Battery Text
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(19, 4);

  if (battRead >= 95) {
    display.println("FL");
  }
  else {
    display.println(battRead);
  }

  // Battery Text Cont.
  display.setTextSize(1);
  display.setCursor(19, 21);
  display.println("BATT");

  // Temp Text
  display.setTextSize(1);
  display.setCursor(66, 21);
  display.println("TEMP");

  // Puffs Text
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(101, 4);
  display.println(fButtonCount);
  display.setTextSize(1);
  display.setCursor(101, 21);
  display.println("HITS");

  display.display();
  display.clearDisplay();
}

/* Adjusts Screen OF/HI/MD/LO (via Button A) */
void TempAdjust()
{ 
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(67, 4);

  int tempHeight = map(tButtonCount, 3, 0, 0, 13); 
  display.fillRect(tempStartX, tempStartY + tempHeight, tempWidth, tempMaxHeight - tempHeight, 1);   

  if (tButtonCount == 0)  {
    display.println("OF");
  }
  if (tButtonCount == 1)  {
    display.println("LO");      
  }
  if (tButtonCount == 2)  {
    display.println("MD");
  }
  if (tButtonCount == 3)  {
    display.println("HI");    
  }
  delay(1);
}

/* Read Temperature Button + Haptic Feedback (Add to All buttons later) */
void ButtonReader()
{
  tButtonState = analogRead(tempButtonA);
  hButtonState = digitalRead(hitsButtonB);
  sButtonState = digitalRead(scrnButtonC);
  fButtonState = digitalRead(fireButtonE);

  if (tButtonState == LOW)  {
    tButtonCount++;
  }
  if (hButtonState == LOW)  {
    hButtonCount++;
  }
  if (sButtonState == LOW)  {
    sButtonCount++;
  }
  if (fButtonState == LOW)  {   
    fButtonCount++;
  }
  if (fButtonState == LOW){     // Needs to hold
    display.clearDisplay();
    display.invertDisplay(true);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(18, 11);
    display.println("FIRE UP");
    display.display();
  }
  delay(1);
}

/* Reset Button Counts */
void ResetCount()
{
  if (fButtonCount >= 99 || hButtonCount >= 3)  {
    fButtonCount = 0;
  }
  if (tButtonState == LOW && tButtonCount > 3)  {
    tButtonCount = 0;
  }
  if (hButtonState == LOW && hButtonCount >= 3)  {
    hButtonCount = 0;
  }
  if (sButtonState == LOW && sButtonCount >= 5)  {
    sButtonCount = 0;
    screenOff = 0;
  }
}

/* Read 1.1V reference against VCC for approx battery level */
int getBatteryVoltage()
{
    float measuredvbat = analogRead(battPin);

    measuredvbat *= 2;        // we divided by 2, so multiply back
    measuredvbat *= 3.228;    // 3.3V - Internal reference voltage
    // measuredvbat *= 2.048; // External Reference LM4040 Breakout
    measuredvbat /= 1024;     // convert to voltage
    measuredvbat *= 1000;     // multiply by 1000 for range values

    return measuredvbat;
}

/* Smooth Readings */
void Smooth()
{  
  for (int i = 0; i < numReadings; i++)  {
    avgVoltRead = avgVoltRead + (battSingleRead - avgVoltRead) / numReadings;
  }
  delay(1);
}

/* Updates Battery Icon & Sets #*/
void BattAdjust()
{
  battRead = map(avgVoltRead, 2750, 4200, 0, 99);
  int battLines = map(battRead, 0, 99, 1, 17);
  
  for (int i = 0; i <= battLines; i++)
  display.drawFastHLine(battStartX, battStartY - i, battWidth, 1);  
}

/* Haptic Feedback as Low Battery Warning */
void LowBattery()
{
  if (avgVoltRead < 2850)
  {
    for (int i = 0; i <= 5; i++)
    Vibrate();
  }
}

/* Haptic Feedback Function *** Will use a piezo for now *** */
void Vibrate()
{
    digitalWrite(vibeMotorPin, 2500); // Runs vibe motor
    delay(20);
    digitalWrite(vibeMotorPin, 0);
    yield();
}

/* Check Fire Button and Turns on Heat */
void FireCoil()
{
  fButtonState = digitalRead(fireButtonE);

  if (fButtonState == LOW && fButtonCount >= 1)  {
    firePower = map(tButtonCount, 1, 3, 100, 255);
    analogWrite(mosfetPin, firePower);
  }
  else  {
    analogWrite(mosfetPin, 0);
  }
  delay(10);
}

/* End Functions */
