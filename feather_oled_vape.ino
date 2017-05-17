/*****************************************************************************

  Arduino Vaporizer (Mini)
  Adafruit OLED Feather Wing & Adafruit Feather Proto 32u
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
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/* Hardware */
Adafruit_SSD1306 display = Adafruit_SSD1306();

/* Pin Mapping */
#define fireButtonE    12       // E. Fire Button
#define scrnButtonC    5        // C. Screen Power
#define hitsButtonB    6        // B. Resets Hits       [ 6 = Pin A7]
#define tempButtonA    9        // A. Sets Temperature  [ 9 = Pin A9]
#define vibeMotorPin   11       // Vibration motor
#define rdboardLED     13       // On-board LED
#define motionPin      A0       //
#define battPin        A1       // Monitors Voltage
#define thermoPin      A2       // TMP36 Temp Sensor
#define fireRpin       A3       //           Red
#define fireGpin       A4       // Fire LED  Green
#define fireBpin       A5       //           Blue
#define mosfetPin      A10      // Output to Mosfet
#define fireButtonLED  13       // LED Inside power button

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
#define debounce      20        // prevent button noise

/* Temperature Variables   (A) */
int tButtonCount = 1;           // press counter
int tButtonState = 0;           // current state
int tButtonPrev = 0;            // previous state
int firePower = 0;              // power output to mosfet

/* Hit Counter Variables   (B) */
int hButtonCount = 0;
int hButtonState = 0;
int hButtonPrev = 0;

/* Color Counter Variables (C2) */
int chButtonCount;
int chButtonState = 0;
int chButtonPrev = 0;

/* Screen Variables        (C) */
int sButtonCount = 0;
int sButtonState = 0;
int sButtonPrev = 0;
int screenOff = 0;

/* Fire Button Variables   (E) */
int fButtonCount = 0;
int fButtonState = 0;
int fButtonPrev = 0;

/* Battery Calculations */
int battRead;                       // Mapped Voltage for LED
int battSingleRead;                 // Actual Voltage x1000
int avgVoltRead = 0;                // the average used
const int numReadings = 85;         // smooths voltage readings
const float calcVolt = 3.228;       // Tested Board Voltage

/* Temperature Variables */
float temperatureF;                 // Temperature Reading
int avgTempRead;                    // Average Temperature

/* LED Variables */
int epAddress = 0;
int colorR;
int colorG;
int colorB;

/* Held Button Variables */
int current;                         // Current state of the button (LOW is pressed b/c i'm using pullup resistors)
long millis_held;                    // How long the button was held (milliseconds)
long secs_held;                      // How long the button was held (seconds)
long prev_secs_held;                 // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime;             // how long since the button was first pressed

/* Motion Sensor */
int mSensor;


/************  SETUP  ************/

void setup()
{
  /* Set Pin Input/Output & Pullups */
  pinMode(hitsButtonB,  INPUT_PULLUP);
  pinMode(scrnButtonC,  INPUT_PULLUP);
  pinMode(battPin,      INPUT);
  pinMode(tempButtonA,  INPUT);
  pinMode(thermoPin,    INPUT);
  pinMode(motionPin,    INPUT);
  pinMode(fireButtonE,  INPUT);
  pinMode(rdboardLED,   OUTPUT);                  //  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(mosfetPin,    OUTPUT);
  pinMode(vibeMotorPin, OUTPUT);
  pinMode(fireRpin,     OUTPUT);
  pinMode(fireGpin,     OUTPUT);
  pinMode(fireBpin,     OUTPUT);
  pinMode(fireButtonLED, OUTPUT);

  digitalWrite(fireButtonE,   HIGH);
  digitalWrite(fireButtonLED, LOW);

  /* Setup Display */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);     // Initialize I2C addr 0x3C (for 128x32)

  // display.invertDisplay(true);                // White screen logo
  display.setRotation(2);                        // Rotate Display: 0, 90, 180 or 270* (0,1,2,3)

  display.display();                             // Display splashscreen
  delay(1000);                                   // Time to display splash screen
  display.clearDisplay();                        // Clear buffer

  chButtonCount = EEPROM.read(epAddress);        // Reads EEPROM for LED Color

  ledOff();                                      // Turn off LED (Clear Color)
}


/************  PROGRAM  ************/

void loop()
{
  /* Test Readings 
  Serial.begin(9600);
  Serial.print("Volts...");
  Serial.println(avgVoltRead);
  Serial.print("TempF:...");
  Serial.println(avgTempRead);
  Serial.print("EEPROM Value:");
  Serial.println(chButtonCount);

  /* Internal Functions */
  ReadTemp();                                   // Reads Ambient Temp
  MotionPower();

  /* Battery Functions */
  battSingleRead = getBatteryVoltage();         // Reads Voltage
  Smooth();                                     // Removes Jitter from Voltage Reads
  BattAdjust();                                 // Set Screen Readout
  LowBattery();                                 // Haptic notification to recharge

  /* Button Functions */
  ButtonReader();                               // Check button presses
  HoldButton();
  ResetCount();                                 // Reset button counters
  WriteEEPROM();                                // Saves Information
  
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
    }
    if (sButtonCount == 3) {
      SecondMenu();
    }
    if (sButtonCount == 4) {
      ThirdMenu();
      ledOff();
    }
    if (sButtonCount == 5 && screenOff == 0 ) {
      screenOff++;
      ledOff();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(18, 13);
      display.println("Off To Sleep...");
      display.display();
      delay(800);
      display.clearDisplay();
      display.invertDisplay(false);
      display.setCursor(18, 13);
      display.println("Press to Wake...");
      display.display();
      delay(700);
      yield();
    }
    if (sButtonCount >= 5 && screenOff == 1 ) {
      display.clearDisplay();
      display.display();
      delay(1);      
    }
  }

} /* End Program */

/************  FUNCTIONS  ************/

/* Creates Tertiary Interface  */
void ThirdMenu()
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

/* Creates Secondary Interface  */
void SecondMenu()
{
  // LED Color
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(3, 4);
  display.println("LED COLOR:");

  SetLEDColor();              // Sets fire LED Color
  display.display();
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
  /*  else {
      setColor(80, 0, 80);          // White
      display.println("WHITE");
    } */

  display.display();

  delay(1);
}

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
  if (hButtonState == LOW && sButtonCount != 3)  {
    hButtonCount++;   // This needs debounce
  }
  if (hButtonState == LOW && sButtonCount == 3)  {
    chButtonCount++;
  }
  if (sButtonState == LOW)  {
    sButtonCount++;
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
  if (sButtonState == LOW && sButtonCount >= 6)  {
    sButtonCount = 0;
    screenOff = 0;
  }
}

/* Read 1.1V reference against VCC for approx battery level */
int getBatteryVoltage()
{
  float measuredvbat = analogRead(battPin);

  measuredvbat *= 2;        // we divided by 2, so multiply back
  measuredvbat *= calcVolt;    // 3.3V - Internal reference voltage
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
void FireCoil() // Fix the pin #'s
{
  if (fButtonCount >= 1)  {
    display.clearDisplay();
    display.invertDisplay(true);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(18, 9);
    display.println("FIRE UP!");
    display.display();

    digitalWrite(fireButtonLED, HIGH);

    firePower = map(tButtonCount, 1, 3, 100, 255);
    analogWrite(mosfetPin, firePower);
    analogWrite(fireRpin, colorR);       // Temporary LED Color
    analogWrite(fireGpin, colorG);
    analogWrite(fireBpin, colorB);
    // setColor (color, color, color)     // Turn on LED
  }

  else  {
    analogWrite(mosfetPin, 0);
    analogWrite(fireBpin, 0);
    ledOff();                    // Turn off LED
  }

  delay(1);
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
    setColor(0, 0, 0);            // Off
  }
}

/* this */
void ledOff()
{
  analogWrite(fireRpin, 255);
  analogWrite(fireGpin, 255);
  analogWrite(fireBpin, 255);
  digitalWrite(fireButtonLED, LOW);
}

/* this */
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

  delay(10);

  /*** Note:
    As the EEPROM sizes are powers of two, wrapping (preventing overflow) of an
    EEPROM address is also doable by a bitwise and of the length - 1.
    ++addr &= EEPROM.length() - 1;
  ***/
}

void HoldButton()
{
  current = digitalRead(fireButtonE);

  // if the button state changes to pressed, remember the start time
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  // This if statement is a basic debouncing tool, the button must be pushed for at least
  // 100 milliseconds in a row for it to be considered as a push.
  if (millis_held > 50) {

    if (current == HIGH && previous == LOW) {       // check if button was released since last check

      if (secs_held >= .5) {               // Button held for more than 3 seconds
        fButtonCount++;
      }
    }
  }
  previous = current;
  prev_secs_held = secs_held;
}

void MotionPower()
{
  mSensor = analogRead(motionPin);
  
  if (mSensor<1022){                       // While sensor is not moving, analog pin receive 1023~1024 value
    Serial.print("Sensor Value: ");
    Serial.println(mSensor);
  }
  
  else{ 
    Serial.print("Sensor Value: ");
    Serial.println("NOWLOW");
  }  
delay(1);
}

/* End Functions */
