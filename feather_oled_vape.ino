/* ===========================================================================================*/
/*                                                                                            */
/*      Arduino Vaporizer (Mini) by Jack D. Ciallella                                         */
/*      Adafruit OLED Feather Wing & Adafruit Feather Proto 32u                               */
/*      :: Alpha Version ::                                                                   */
/*      Last Updated 5/2017                                                                   */
/*                                                                                            */
/* ===========================================================================================*/

/* Icons in Binary for OLED Display */
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

/* ===========================================================================================*/
/*                                                                                            */
/*     INCLUDES / DEFINITIONS                                                                 */
/*                                                                                            */
/* ===========================================================================================*/

/* Includes */
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Hardware */
Adafruit_SSD1306 display = Adafruit_SSD1306();

/* Pin Mapping */
#define ruptPin        2        // Interrupt Pin
#define scrnButtonC    5        // C. Screen Power
#define hitsButtonB    6        // B. Resets Hits       [ 6 = Pin A7]
#define tempButtonA    9        // A. Sets Temperature  [ 9 = Pin A9]
#define mosfetPin      10       // Output to Mosfet
#define vibeMotorPin   11       // Vibration motor
#define fireButtonE    12       // E. Fire Button
#define rdboardLED     13       // On-board LED
#define fireButtonLED  13       // LED Inside power button
#define motionPin      A0       // Piezo As Motion Sensor
#define battPin        A1       // Monitors Voltage
#define thermoPin      A2       // TMP36 Temp Sensor
#define fireRpin       A3       // -->      Red
#define fireGpin       A4       // Fire LED Green
#define fireBpin       A5       // -->      Blue

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
#define debounce   75           // prevent button noise

/* Temperature Variables   (A) */
int tButtonCount = 1;           // press counter
int tButtonState = 0;           // current state
int tButtonPrev =  0;           // previous state
int firePower =    0;           // power output to mosfet

/* Hit Counter Variables   (B) */
int hButtonCount = 0;
int hButtonState = 0;
int hButtonPrev =  0;

/* Color Counter Variables (C2) */
int chButtonCount;
int chButtonState = 0;
int chButtonPrev =  0;

/* Screen Variables        (C) */
int sButtonCount = 0;
int sButtonState = 0;
int sButtonPrev =  0;
int screenOff =    0;
int battScreen =   0;

/* Fire Button Variables   (E) */
int fButtonCount = 0;
int fButtonState = 0;
int fButtonPrev =  0;

/* Motion Sensor */
int mSensor;                       // Motion Sensor Value
int gSeconds = 0;                  // Sleep Counter
int timeSleep = 5000;              // Fall Asleep at # count
int prevMSensor;                   // Last State
int mSensorChange;                 // Piezo Sensor Readings Change

/* Battery Calculations */
int battRead;                       // Mapped Voltage for LED
int battSingleRead;                 // Actual Voltage x1000
int lowBattWarn;                    // Counter stops vibrations after 3x
int avgVoltRead = 0;                // the average used
const int numReadings = 85;         // smooths voltage readings
const float calcVolt = 3.3;         // Tested Board Voltage [3.228]
long oneMinTime = 60000;              // Time between low batt vibration warnings
unsigned long previousMillis = 0;   // Low Batt timer

/* Temperature Variables */
float temperatureF;                 // Temperature Reading
int avgTempRead;                    // Average Temperature

/* LED Variables */
int epAddress = 0;                  // Current EEPROM Address / byte
int colorR;                         // (R) -->
int colorG;                         // (G) LED Color Values
int colorB;                         // (G) -->

/* Held Button Variables */
int current;                         // Current state of button (LOW = Pressed for pullup resistors)
long millis_held;                    // How long the button was held (milliseconds)
long secs_held;                      // How long the button was held (seconds)
long prev_secs_held;                 // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime;             // how long since the button was first pressed


/* ===========================================================================================*/
/*                                                                                            */
/*     SETUP PROGRAM                                                                          */
/*                                                                                            */
/* ===========================================================================================*/

void setup()
{
  /* Set Pin Input/Output & Pullups */
  pinMode(hitsButtonB,   INPUT_PULLUP);
  pinMode(scrnButtonC,   INPUT_PULLUP);
  pinMode(battPin,       INPUT);
  pinMode(tempButtonA,   INPUT);
  pinMode(thermoPin,     INPUT);
  pinMode(motionPin,     INPUT);
  pinMode(fireButtonE,   INPUT);
  pinMode(ruptPin,       INPUT);
  pinMode(rdboardLED,    OUTPUT);
  pinMode(mosfetPin,     OUTPUT);
  pinMode(vibeMotorPin,  OUTPUT);
  pinMode(fireRpin,      OUTPUT);
  pinMode(fireGpin,      OUTPUT);
  pinMode(fireBpin,      OUTPUT);
  pinMode(fireButtonLED, OUTPUT);

  /* Pin States */
  digitalWrite(fireButtonE,   HIGH);
  digitalWrite(fireButtonLED, LOW);
  digitalWrite(ruptPin,       HIGH);

  /* Setup Display */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);     // Initialize I2C addr 0x3C (for 128x32)
  display.setRotation(2);                        // Rotate Display: 0, 90, 180 or 270* (0,1,2,3)
  // display.invertDisplay(true);                // White screen logo
  display.display();                             // Display splashscreen
  delay(750);                                    // Time to display splash screen
  display.clearDisplay();                        // Clear buffer

  /* Read from Memory */
  chButtonCount = EEPROM.read(epAddress);        // Reads EEPROM for LED Color
  colorR =        EEPROM.read(epAddress);        // -->              Red LED 
  colorG =        EEPROM.read(epAddress);        // -->              Green LED
  colorB =        EEPROM.read(epAddress);        // -->              Blue LED

  /* Functions */
  ledOff();                                      // Turn off LED (Clear Color)
}

/* ===========================================================================================*/
/*                                                                                            */
/*    END SETUP                                                                                */
/*                                                                                            */
/* ===========================================================================================*/
/*                                                                                            */
/*    BEGIN LOOP                                                                            */
/*                                                                                            */
/* ===========================================================================================*/

void loop()
{
  /* Start PC Communications */
  Serial.begin(9600);

  /* Test Readings */
  Serial.print("Volts...");
  Serial.println(avgVoltRead);
  Serial.print("TempF:...");
  Serial.println(avgTempRead);
  Serial.print("EEPROM Value(s)...");
  Serial.println(chButtonCount);
  Serial.print("Motion Sense Level...");
  Serial.print(mSensor);
  Serial.print(" / ");
  Serial.println(mSensorChange);
  Serial.print("Time to Sleep...");
  Serial.println(gSeconds);

/* ===========================================================================================*/
/*                                                                                            */
/*    Execute Functions                                                                       */
/*                                                                                            */
/* ===========================================================================================*/

  /* Internal */
  AutoSleepMode();                              // For physical handling of device
  ReadTemp();                                   // Reads Ambient Temp

  /* Battery */
  battSingleRead = getBatteryVoltage();         // Reads Voltage
  Smooth();                                     // Removes Jitter from Voltage Reads
  BattAdjust();                                 // Set Screen Readout
  LowBattery();                                 // Haptic notification to recharge

  /* Buttons */
  ButtonReader();                               // Check button presses
  HoldButton();                                 // De-bounces buttons
  ResetCount();                                 // Reset button counters
  
  /* Memory */
  WriteEEPROM();                                // Saves Information


/* ===========================================================================================*/
/*                                                                                            */
/*    Display Menus                                                                           */
/*                                                                                            */
/* ===========================================================================================*/

  /* Conditionally Display Main Menu */
  if (fButtonState != LOW)
  {
    if (sButtonCount <= 1) {
      display.invertDisplay(false);
      MainMenu();                               // Render screen icons & text
      TempAdjust();                             // Render "Mercury" in thermometer icon
      ledOff();
    }
    if (sButtonCount == 2) {
      display.invertDisplay(true);
      MainMenu();
      TempAdjust();
      ledOff();
    }

    /* Notify Screen Going Off */
    if (sButtonCount == 4 && screenOff == 0 ) {
      screenOff++;
      ledOff();
      StealthMode();
    }
    /* Turn Screen Off */
    if (sButtonCount >= 4 && screenOff == 1 ) {
      display.clearDisplay();
      display.display();
      delay(1);
    }

    /* Conditionally Display Second & Third Menus */
    if (sButtonCount == 3 && screenOff == 0 && battScreen == 0) {
      ThirdMenuBatt();
      ledOff();

    }
    if (sButtonCount == 3 && battScreen == 1) {
      SecondMenuLED();
    }

  } /* Close If */

  /* Press B & C Buttons together to send to sleep */
  if (sButtonState == LOW && tButtonState == LOW ) {
    EnterSleep();
  }

} /* Close Program */

/* ===========================================================================================*/
/*                                                                                            */
/*    END LOOP                                                                                */
/*                                                                                            */
/* ===========================================================================================*/
/*                                                                                            */
/*    Main Menus   [ OLED ]                                                                   */
/*                                                                                            */
/* ===========================================================================================*/

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
  if (battRead >= 1 && battRead <= 15) {
    display.println("LO");
  }
  if (battRead < 0) {
    display.println("NO");
  }
  else if (battRead > 15 && battRead < 95) {
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

  if (fButtonCount == 0) {
    display.println("NO");
  }
  else {
    display.println(fButtonCount);
  }

  display.setTextSize(1);
  display.setCursor(101, 21);
  display.println("HITS");

  display.display();
  display.clearDisplay();
}

/* Creates Secondary Interface  */
void SecondMenuLED()
{
  // LED Color
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(3, 4);
  display.println("LED COLOR:");

  SetLEDColor();              // Sets fire LED Color
  display.display();
}

/* Creates Tertiary Interface  */
void ThirdMenuBatt()
{
  // Internal Temperature
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(7, 13);
  display.println("BATTERY TEMP:");
  display.setTextSize(2);
  display.setCursor(86, 8);
  display.print(avgTempRead);
  display.display();
}

/* Turns of Screen / LED's */
void StealthMode()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(18, 13);
  display.println("Stealth Mode: ON");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.invertDisplay(false);
  display.setCursor(17, 13);
  display.println("Press to Return");
  display.display();
  delay(1000);
  yield();  
}

/* ===========================================================================================*/
/*                                                                                            */
/*    Buttons - Read & Count                                                                  */
/*                                                                                            */
/* ===========================================================================================*/

/* Read Temperature Button + Haptic Feedback (Add to All buttons later) */
void ButtonReader()
{
  tButtonState = analogRead(tempButtonA);
  hButtonState = digitalRead(hitsButtonB);
  sButtonState = digitalRead(scrnButtonC);
  fButtonState = digitalRead(fireButtonE);

  if (tButtonState == LOW && sButtonCount <= 2)  {
    tButtonCount++;
  }
  if (hButtonState == LOW && sButtonCount != 3)  {
    hButtonCount++;
  }
  if (hButtonState == LOW && sButtonCount == 3)  {
    chButtonCount++;
  }
  if (sButtonState == LOW)  {
    sButtonCount++;
  }
  if (tButtonState == LOW && sButtonCount == 3) {
    battScreen++;
  }
  if (fButtonState == LOW) {
    FireCoil();
  }
  else {
    analogWrite(mosfetPin, 0);
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
  if (hButtonState == LOW && hButtonCount >= 3 && sButtonState != 3)  {
    hButtonCount = 0;
  }
  if (sButtonCount == 3 && chButtonCount >= 8)  {
    chButtonCount = 0;
  }
  if (battScreen > 1) {
    battScreen = 0;
  }
  if (sButtonState == LOW && sButtonCount >= 5)  {
    sButtonCount = 0;
    screenOff = 0;
  }
}

void HoldButton()
{
  int minHoldTime = 50;
  current = digitalRead(fireButtonE);

  // if the button state changes to pressed, remember the start time
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  /* Basic debouncing tool. Button must be pushed for a certain time to be considered a push. */
  if (millis_held > minHoldTime) {

    if (current == HIGH && previous == LOW) {  // check if button was released since last check

      if (millis_held >= 350) {                // Button held for more than 3 seconds
        fButtonCount++;
      }
    }
  }
  previous = current;
  prev_secs_held = secs_held;
}


/* ===========================================================================================*/
/*                                                                                            */
/*    LED / OLED                                                                              */
/*                                                                                            */
/* ===========================================================================================*/

/* Turns off external "Fire" LED */
void ledOff()
{
  analogWrite(fireRpin, 255);
  analogWrite(fireGpin, 255);
  analogWrite(fireBpin, 255);
  digitalWrite(fireButtonLED, LOW);
}

void SetLEDColor()
{
  display.setTextSize(2);
  display.setCursor(3, 14);

  colorR = 0;
  colorG = 0;
  colorB = 0;

  if (chButtonCount == 0) {
    setColor(0, 255, 0);                // Purple
    display.println("PURP HAZE");
    colorG = 255;
  }
  if (chButtonCount == 1) {
    setColor(0, 0, 255);                // Green
    display.println("MOJITOS");
    colorB = 0;
  }
  if (chButtonCount == 2) {
    setColor(0, 255, 255);              // Red
    display.println("WATERMELON");
    colorG = 255;
    colorB = 255;
  }
  if (chButtonCount == 3) {
    setColor(255, 255, 0);              // Mid-Blue [ DEEP WATER ]
    display.println("OCEANIC");
    colorR = 255;
    colorG = 255;
  }
  if (chButtonCount == 4) {
    setColor(255, 0, 255);              // Green [ NEW GRASS, FOREST ]
    display.println("CANOPY");
    colorR = 255;
    colorB = 255;
  }
  if (chButtonCount == 5) {
    setColor(255, 0, 0);                // Light Blue [ICE]
    display.println("GLACIER");
    colorR = 255;
  }
  if (chButtonCount == 6) {
    setColor(80, 0, 80);                // White [GHOST, SEAGLASS, CLOUD, MINT]
    display.println("SEAGLASS");
    colorR = 255;
    colorB = 255;
    colorB = 255;                       // Not sure if I need to store a color or not here
  }
  if (chButtonCount == 7) {
    ledOff();            // Off
    display.println("OFF");
  }

  display.display();

  delay(1);
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
    display.println("NO");
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

/* ===========================================================================================*/
/*                                                                                            */
/*   Battery / Vibrate                                                                        */
/*                                                                                            */
/* ===========================================================================================*/

/* Haptic Feedback as Low Battery Warning */
void LowBattery()
{  
  unsigned long currentMillis = millis();

  if (avgVoltRead < 2950) {

    if (currentMillis - previousMillis > oneMinTime) {
        previousMillis = currentMillis;  
      
      if (lowBattWarn <= 9) {
        for (int i = 0; i <= 3; i++)
          Vibrate();
          lowBattWarn++;                        
        }
      }
   }  
  else if (avgVoltRead > 3500) {
    lowBattWarn = 0;
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

/* ===========================================================================================*/
/*                                                                                            */
/*    Sleep Functions                                                                         */
/*                                                                                            */
/* ===========================================================================================*/

/* The interrupt is handled after wakeup || Show wake screen while functions turn on */
void WakeUpNow()
{
  // WakeScreen();   // Not sure why this isn't working
}

/* Placeholder Screen while buttons come back online */
void WakeScreen()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(14, 9);
  display.println("Waking Back Up...");
  display.display();
  delay (3000);
}

/* Setup ruptPin as an interrupt and attach handler. */
void EnterSleep()
{
  display.clearDisplay();
  display.invertDisplay(true);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(16, 13);
  display.println("Going to Sleep...");
  display.display();
  delay(750);
  
  display.setCursor(19, 13);
  display.clearDisplay();
  display.println(" ~ Goodnight ~ ");
  display.display();
  delay(1500);
  
  display.invertDisplay(false);
  display.clearDisplay();
  display.display();

  sButtonCount = 0;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  attachInterrupt(0, WakeUpNow, LOW);

  sleep_mode();               // The program will continue from here.
  sleep_disable();
  detachInterrupt(0);
}

/* Piezeo sensor checks for motion - Replace function with actual seconds */
void AutoSleepMode()
{
  mSensor = analogRead(motionPin);
  delay (1);
  prevMSensor = analogRead(motionPin);

  mSensorChange = mSensor - prevMSensor;

  if (mSensorChange > 175 || mSensorChange < - 175) {
    gSeconds = 0;
  }
  else {
    gSeconds++;
  }

  if (gSeconds > 10000)  {
    display.clearDisplay();
    display.display();
    sButtonCount = 0;
//    sleep = true;
  }
  
  if (gSeconds > timeSleep)  {
    EnterSleep();
    gSeconds = 0;
  }
}

/* ===========================================================================================*/
/*                                                                                            */
/*    Fire Vape / Coil                                                                        */  
/*                                                                                            */
/* ===========================================================================================*/

/* Check Fire Button and Turns on Heat */
void FireCoil()
{
  if (fButtonState == LOW)  {
    firePower = map(tButtonCount, 1, 3, 100, 255);
    analogWrite(mosfetPin, firePower);

    if (sButtonCount !=4) {
    display.clearDisplay();
    display.invertDisplay(true);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(18, 9);
    display.println("FIRE UP!");
    display.display();
    display.clearDisplay();

    digitalWrite(fireButtonLED, HIGH);
    
    analogWrite(fireRpin, colorR);
    analogWrite(fireGpin, colorG);
    analogWrite(fireBpin, colorB);
    }
  }

  else  {
    analogWrite(mosfetPin, 0);
    analogWrite(fireBpin, 0);
    ledOff();
  }

  delay(1);
}

/* ===========================================================================================*/
/*                                                                                            */
/*    Internal Functions                                                                      */  
/*                                                                                            */
/* ===========================================================================================*/

/* Read 1.1V reference against VCC for approx battery level */
int getBatteryVoltage()
{
  float measuredvbat = analogRead(battPin);

  measuredvbat *= 2;          // we divided by 2, so multiply back
  measuredvbat *= calcVolt;   // 3.3V - Internal reference voltage
  // measuredvbat *= 2.048;   // External Reference LM4040 Breakout
  measuredvbat /= 1024;       // convert to voltage
  measuredvbat *= 1000;       // multiply by 1000 for range values

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

/* Reads Temp Sensor & Shuts Down Functions if Too Hot*/
void ReadTemp()
{
  int reading = analogRead(thermoPin);

  float voltage = reading * calcVolt;                  // converting that reading to voltage
  voltage /= 1024.0;

  //converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)
  float temperatureC = (voltage - 0.5) * 100;

  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;    // Change to F

  avgTempRead = temperatureF;

  delay(1);

  if (temperatureF >= 180) {

    analogWrite(mosfetPin, 0);

    display.clearDisplay();
    display.invertDisplay(true);
    display.setCursor(13, 13);
    display.println("TOO HOT!!! WAIT...");
    display.display();
    setColor(255, 255, 0);
    delay(10000);
    setColor(0, 0, 0);
  }
}

/* Writes settings to static ram */
void WriteEEPROM()
{
  // int val = analogRead(0) / 4;            // Divide by 4 to convert to analog [if digital]
  EEPROM.write(epAddress, chButtonCount);    // Write value to the appropriate byte of the EEPROM
  EEPROM.write(epAddress, colorR);
  EEPROM.write(epAddress, colorG);
  EEPROM.write(epAddress, colorB);

  // Advance to the next address, when at the end restart at the beginning *For Compatibility*
  epAddress = epAddress + 1;
  if (epAddress == EEPROM.length()) {
    epAddress = 0;
  }
  delay(1);
}

/* Sets Pins to Colors */
void setColor(int red, int green, int blue)
{
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  analogWrite(fireRpin, red);
  analogWrite(fireGpin, green);
  analogWrite(fireBpin, blue);
}

