/* ===========================================================================================*/
/*                                                                                            */
/*      Arduino Vaporizer (Mini) by Jack D. Ciallella                                         */
/*      Adafruit OLED Feather Wing & Adafruit Feather Proto 32u                               */
/*      :: Alpha Version ::                                                                   */
/*      Last Updated 5/2017                                                                   */
/*                                                                                            */
/* ===========================================================================================*/

/* Icons & Logo in Binary for OLED Display */
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

static const unsigned char PROGMEM  feather[] =
{
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000011, 0b11111110, 0b01111111, 0b11000000, 0b00111000, 0b11111111, 0b11110011, 0b00000000, 0b11001111, 0b11111001, 0b11111110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //       #########  #########        ###   ############  ##        ##  #########  ########
  0b00000011, 0b11111110, 0b01111111, 0b11000000, 0b00111000, 0b11111111, 0b11110011, 0b00000000, 0b11001111, 0b11111001, 0b11111111, 0b10000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //       #########  #########        ###   ############  ##        ##  #########  ##########
  0b00000011, 0b00000000, 0b01100000, 0b00000000, 0b01101000, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000001, 0b10000000, 0b11111111, 0b10001100, 0b00011111, 0b00000000, //       ##         ##              ## #        ##       ##        ##  ##         ##      ##       #########   ##     #####
  0b00000011, 0b00000000, 0b01100000, 0b00000000, 0b01101100, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000000, 0b11000000, 0b11111111, 0b10001100, 0b00111111, 0b10000000, //       ##         ##              ## ##       ##       ##        ##  ##         ##       ##      #########   ##    #######
  0b00000011, 0b00000000, 0b01100000, 0b00000000, 0b01101100, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000000, 0b11000000, 0b00000011, 0b00001100, 0b01110001, 0b11000000, //       ##         ##              ## ##       ##       ##        ##  ##         ##       ##            ##    ##   ###   ###
  0b00000011, 0b00000000, 0b01100000, 0b00000000, 0b11000110, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000000, 0b11000000, 0b00000111, 0b00001100, 0b01100000, 0b11100000, //       ##         ##             ##   ##      ##       ##        ##  ##         ##       ##           ###    ##   ##     ###
  0b00000011, 0b00000000, 0b01100000, 0b00000000, 0b11000110, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000000, 0b11000000, 0b00000110, 0b00001100, 0b01100000, 0b01100000, //       ##         ##             ##   ##      ##       ##        ##  ##         ##       ##           ##     ##   ##      ##
  0b00000011, 0b00000000, 0b01111111, 0b10000000, 0b11000110, 0b00000110, 0b00000011, 0b11111111, 0b11001111, 0b11110001, 0b10000001, 0b10000000, 0b00001110, 0b00001100, 0b01100000, 0b01100000, //       ##         ########       ##   ##      ##       ############  ########   ##      ##           ###     ##   ##      ##
  0b00000011, 0b11111100, 0b01111111, 0b10000001, 0b10000011, 0b00000110, 0b00000011, 0b11111111, 0b11001111, 0b11110001, 0b11111111, 0b10000000, 0b00001100, 0b00001100, 0b01100000, 0b11100000, //       ########   ########      ##     ##     ##       ############  ########   ##########           ##      ##   ##     ###
  0b00000011, 0b11111100, 0b01100000, 0b00000001, 0b10000011, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b11111110, 0b00000000, 0b00011100, 0b00001100, 0b01110000, 0b11000000, //       ########   ##            ##     ##     ##       ##        ##  ##         ########            ###      ##   ###    ##
  0b00000011, 0b00000000, 0b01100000, 0b00000011, 0b10000011, 0b00000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10001100, 0b00000000, 0b00011000, 0b00001100, 0b00111111, 0b11000000, //       ##         ##           ###     ##     ##       ##        ##  ##         ##   ##             ##       ##    ########
  0b00000011, 0b00000000, 0b01100000, 0b00000011, 0b11111111, 0b10000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10001110, 0b00000000, 0b00111000, 0b00001100, 0b00011111, 0b10000000, //       ##         ##           ###########    ##       ##        ##  ##         ##   ###           ###       ##     ######
  0b00000011, 0b00000000, 0b01100000, 0b00000011, 0b11111111, 0b10000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000110, 0b00000000, 0b00110000, 0b00000000, 0b00000000, 0b00000000, //       ##         ##           ###########    ##       ##        ##  ##         ##    ##           ##
  0b00000011, 0b00000000, 0b01100000, 0b00000111, 0b00000001, 0b10000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000011, 0b00000000, 0b01110000, 0b00000000, 0b00000000, 0b00000000, //       ##         ##          ###       ##    ##       ##        ##  ##         ##     ##         ###
  0b00000011, 0b00000000, 0b01100000, 0b00000110, 0b00000000, 0b11000110, 0b00000011, 0b00000000, 0b11001100, 0b00000001, 0b10000011, 0b10000000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, //       ##         ##          ##         ##   ##       ##        ##  ##         ##     ###        ##
  0b00000011, 0b00000000, 0b01111111, 0b11100110, 0b00000000, 0b11000110, 0b00000011, 0b00000000, 0b11001111, 0b11111101, 0b10000001, 0b10000000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, //       ##         ##########  ##         ##   ##       ##        ##  ########## ##      ##        ##
  0b00000011, 0b00000000, 0b01111111, 0b11101100, 0b00000000, 0b11100110, 0b00000011, 0b00000000, 0b11001111, 0b11111101, 0b10000000, 0b11000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //       ##         ########## ##          ###  ##       ##        ##  ########## ##       ##
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //
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
#define VBAT_ENABLED   1        // Enable / Disable integrated batt mgmt

/* Draws Battery Level in Icon */
#define battStartX     4
#define battStartY     25
#define battWidth      8

/* Draws Temp Level in Icon */
#define tempStartX     53
#define tempStartY     6
#define tempWidth      2
#define tempMaxHeight  13

/* Debounce / Button Hold */
#define debounce       199       // prevent button noise

/* Temperature Variables   (A) */
int tButtonCount =  1;           // press counter
int tButtonState =  0;           // current state
int tButtonPrev =   0;           // previous state
int firePower =     0;           // power output to mosfet

/* Hit Counter Variables   (B) */
int hButtonCount =  0;
int hButtonState =  0;
int hButtonPrev =   0;

/* Color Counter Variables (C2) */
int chButtonCount;
int chButtonState = 0;
int chButtonPrev =  0;

/* Screen Variables        (C) */
int sButtonCount =  0;
int sButtonState =  0;
int sButtonPrev =   0;
int screenOff =     0;
int battScreen =    0;
int heatCount =     0;
int showLogoTime =  1200;          // How long to show Product Logo on screen
boolean screenFlip = true;           // Screen Inversion State (Light or Dark)

/* Fire Button Variables   (E) */
int fButtonCount =  0;
int fButtonState =  0;
int fButtonPrev =   0;
int preHeatStage =  0;             // 0, 1, 2, 3 :: OFF, Countdown, Animation, Static
int preHeatOn =     0;             // 0, 1 :: OFF, ON

/* Motion Sensor */
int mSensor;                       // Motion Sensor Value
int gSeconds =      0;             // Sleep Counter
int screenSleep =   2500;          // Turns off Screen at #
int deepSleep =     5000;          // Fall Asleep at # count
int prevMSensor;                   // Last State
int mSensorChange;                 // Piezo Sensor Readings Change

/* Temperature Variables */
float temperatureF;                 // Temperature Reading
int avgTempRead =   0;              // Average Temperature

/* LED Variables */
int epAddress =     0;              // Current EEPROM Address / byte
int colorR;                         // (R) -->
int colorG;                         // (G) LED Color Values
int colorB;                         // (G) -->

/* Battery Calculations */
int battRead;                       // Mapped Voltage for LED
int battSingleRead;                 // Actual Voltage x1000
int lowBattWarn;                    // Counter stops vibrations after 3x
int avgVoltRead = 0;                // the average used
const int numReadings = 85;         // smooths voltage readings
const float calcVolt = 3.3;         // Tested Board Voltage [3.228]
long oneMinTime = 60000;            // Time between low batt vibration warnings
unsigned long previousMillis = 0;   // Low Batt timer

/* Held Button Variables */
int current;                         // Current state of button (LOW = Pressed for pullup resistors)
long millis_held;                    // How long the button was held (milliseconds)
long secs_held;                      // How long the button was held (seconds)
long prev_secs_held;                 // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime;             // How long since the button was first pressed

int iCount = 0;

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
  delay(1500);                                   // Time to display splash screen
  display.clearDisplay();                        // Clear buffer

  /* Setup Timer */

  /* Read from Memory */
  chButtonCount = EEPROM.read(epAddress);        // Reads EEPROM for LED Color
  colorR =        EEPROM.read(epAddress);        // -->              Red LED
  colorG =        EEPROM.read(epAddress);        // -->              Green LED
  colorB =        EEPROM.read(epAddress);        // -->              Blue LED

  /* Functions */
  ledOff();                                      // Turn off LED (Clear Color)
  ShowLogo();                                    // Quickly show 710 Logo
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
  Serial.print(colorR);
  Serial.print(", ");
  Serial.print(colorG);
  Serial.print(", ");
  Serial.println(colorB);
  Serial.print("Motion Sense Level...");
  Serial.print(mSensor);
  Serial.print(" / ");
  Serial.println(mSensorChange);
  Serial.print("Time to Sleep...");
  Serial.println(gSeconds);
  Serial.print("Pre-Heater State...");
  Serial.println(preHeatOn);

  /* ===========================================================================================*/
  /*                                                                                            */
  /*    Execute Functions                                                                       */
  /*                                                                                            */
  /* ===========================================================================================*/

  /* Internal */
  AutoSleepMode();                              // For physical handling of device
  ReadTemp();                                   // Reads Ambient Temp
  battSingleRead = getBatteryVoltage();         // Reads Voltage
  Smooth();                                     // Removes Jitter from Voltage Reads
  PreHeatDisplay();                             // Shows Pre-Heating Stages
  BattAdjust();                                 // Set Screen Readout
  LowBattery();                                 // Haptic notification to recharge

  /* Buttons */
  ButtonReader();                               // Check button presses
  ResetCount();                                 // Reset button counters
  HoldButton();                                 // De-bounces buttons

  /* Memory */
  WriteEEPROM();                                // Saves Information Last


  /* ===========================================================================================*/
  /*                                                                                            */
  /*    Display Menus                                                                           */
  /*                                                                                            */
  /* ===========================================================================================*/

  /* Conditionally Display Main Menu */
  if (fButtonState != LOW && screenFlip == true)
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

    /* Conditionally Display Second & Alternate Menus */
    if (sButtonCount == 3 && screenOff == 0 && battScreen == 0) {
      ThirdMenuBatt();
      ledOff();

    }
    if (sButtonCount == 3 && battScreen == 1) {
      SecondMenuLED();
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
  if (battRead <= 0) {
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

  // Hits Count (Conditional)                                                                       // **** BOOKMARK *** //
  if (preHeatOn == 0) {
    display.drawFastVLine(95, 4, 26, WHITE);
    if (fButtonCount == 0) {
      display.println("NO");
    }
    else {
      display.println(fButtonCount);
    }
    display.setTextSize(1);
    display.setCursor(101, 21);
    display.println("HITS");
  }
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
  int gfxChar = 0xf7;        // Uses character map to generate "degrees" symbol

  // Internal Temperature
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(7, 14);
  display.println("FEATHER TEMP:");

  display.setTextSize(2);
  display.setCursor(87, 10);
  display.print(avgTempRead);
  if (avgTempRead < 100) {
    display.setTextSize(1);
    display.write(gfxChar);
  }
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
  display.clearDisplay();
  yield();
}

/* ===========================================================================================*/
/*                                                                                            */
/*    Buttons - Read & Count & Reset                                                          */
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
    gSeconds = 0;
  }
  if (hButtonState == LOW && sButtonCount != 3)  {
    hButtonCount++;
    gSeconds = 0;
  }
  if (hButtonState == LOW && sButtonCount == 3)  {
    chButtonCount++;
    gSeconds = 0;
  }
  if (sButtonState == LOW)  {
    sButtonCount++;
    gSeconds = 0;
  }
  if (tButtonState == LOW && sButtonCount == 3) {
    battScreen++;
  }
  if (fButtonState == LOW) {
    FireCoil();
    gSeconds = 0;
  }
  else {
    analogWrite(mosfetPin, 0);
  }
  delay(1);
}

/* Reset Button Counts */
void ResetCount()
{
  if (hButtonCount == 3) {
    preHeatOn = 1;
  }
  if (fButtonCount >= 99 || hButtonCount >= 3)  {
    fButtonCount = 0;
  }
  if (tButtonState == LOW && tButtonCount > 3)  {
    tButtonCount = 0;
  }
  if (hButtonState == LOW && hButtonCount >= 4 && sButtonState != 3)  {
    hButtonCount = 0;
    preHeatStage = 0;
    preHeatOn = 0;
  }
  if (preHeatStage == 1)  {
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

/* Checks if Buttons are Held */
void HoldButton()
{
  int minHoldTime = 50;

  /* For Fire Button */
  if (fButtonState == LOW) {
    current = digitalRead(fireButtonE);

    /* If the button state changes to pressed, remember the start time */
    if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
      firstTime = millis();
    }

    millis_held = (millis() - firstTime);       /* Basic debouncing tool. */
    secs_held = millis_held / 1000;             /* Button must be pushed for a certain time to be considered a push. */

    if (millis_held > minHoldTime) {

      if (current == HIGH && previous == LOW) {

        if (millis_held >= 350) {               /*Button held for more than x time*/
          fButtonCount++;
        }
      }
    }
  }

  /* For Hits Button */
  if (hButtonState == LOW) {
    current = digitalRead(hitsButtonB);

    if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
      firstTime = millis();
    }

    millis_held = (millis() - firstTime);
    secs_held = millis_held / 1000;

    if (millis_held > minHoldTime) {

      if (current == HIGH && previous == LOW) {

        if (millis_held >= 1000) {
          fButtonCount = 0;
          Serial.println("**************** B WAS HELD FOR TIME ********************");                    // **** BOOKMARK *** //
          // This function needs work above. Not sure why its hard to achieve.
        }
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

/* Turns off external "Fire" LED & Button LED */
void ledOff()
{
  analogWrite(fireRpin, 255);
  analogWrite(fireGpin, 255);
  analogWrite(fireBpin, 255);
  digitalWrite(fireButtonLED, LOW);
}

/* Counts Secondary Button Presses on 'C' Button & Sets LED Color */
void SetLEDColor()
{
  display.setTextSize(2);
  display.setCursor(3, 14);

  colorR = 0;
  colorG = 0;
  colorB = 0;

  /* Colors */
  switch (chButtonCount) {
    case 0:
      setColor(0, 255, 0);                // Purple
      display.println("PURP HAZE");
      colorG = 255;
      break;
    case 1:
      setColor(0, 0, 255);                // Green
      display.println("MOJITOS");
      colorB = 255;
      break;
    case 2:
      setColor(0, 255, 255);              // Red
      display.println("WATERMELON");
      colorG = 255;
      colorB = 255;
      break;
    case 3:
      setColor(255, 255, 0);              // Mid-Blue [ DEEP WATER ]
      display.println("OCEANIC");
      colorR = 255;
      colorG = 255;
      break;
    case 4:
      setColor(255, 0, 255);              // Green [ NEW GRASS, FOREST ]
      display.println("CANOPY");
      colorR = 255;
      colorB = 255;
      break;
    case 5:
      setColor(255, 0, 0);                // Light Blue [ICE]
      display.println("GLACIER");
      colorR = 255;
      break;
    case 6:
      setColor(80, 0, 80);                // White [GHOST, SEAGLASS, CLOUD, MINT]
      display.println("SEAGLASS");
      colorR = 255;
      colorB = 255;
      colorB = 255;
      break;
    case 7:
      ledOff();
      display.println("OFF");
      break;
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

  switch (tButtonCount) {
    case 0:
      display.println("NO");
      break;
    case 1:
      display.println("LO");
      break;
    case 2:
      display.println("MD");
      break;
    case 3:
      display.println("HI");
      break;
  }
  delay(1);
}

/* ===========================================================================================*/
/*                                                                                            */
/*    PRE-HEATING                                                                             */
/*                                                                                            */
/* ===========================================================================================*/

/* Lightly Heat Coil */
void HeatCoil()
{
  if (preHeatStage = true) {
    analogWrite(mosfetPin, 50);                   // Figure out a good number for this
  }
}

/* Displays Simple Text about coil */
void WarmCoilText()
{
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 13);
  display.println("WARMING COIL");
}

/* Full Animation */
void PreHeat()
{
  for (int i = 0; i <= 50; i++) {

    // GFX Placement
    int baseCursorY = 7;
    int baseCursorX = 101;

    unsigned long TimerA;
    TimerA = millis();

    unsigned long timerCount = 1000000UL;

    // Create Animation
    switch (heatCount) {
      case 0:
        heatCount++;
        StaticHeatDisplay();
        WarmCoilText();
        display.setCursor(baseCursorX, baseCursorY - 2);
        display.write(0x7E);
        display.setCursor(baseCursorX + 7, baseCursorY - 3);
        display.write(0x7E);
        display.setCursor(baseCursorX + 15, baseCursorY - 2);
        display.write(0x7E);
        display.display();
        if (millis() - TimerA > 0 && TimerA < timerCount) {
          display.clearDisplay();
          break;
        }

      case 1:
        heatCount++;
        StaticHeatDisplay();
        WarmCoilText();
        display.setCursor(baseCursorX, baseCursorY - 3);
        display.write(0x7E);
        display.setCursor(baseCursorX + 7, baseCursorY - 5);
        display.write(0x7E);
        display.setCursor(baseCursorX + 15, baseCursorY - 3);
        display.write(0x7E);
        display.display();
        if (millis() - TimerA > timerCount && TimerA < timerCount * 2) {
          display.clearDisplay();
          break;
        }

      case 2:
        heatCount++;
        StaticHeatDisplay();
        WarmCoilText();
        display.setCursor(baseCursorX, baseCursorY - 4);
        display.write(0x7E);
        display.setCursor(baseCursorX + 7, 0);
        display.write(0x7E);
        display.setCursor(baseCursorX + 15, baseCursorY - 4);
        display.write(0x7E);
        display.display();
        if (millis() - TimerA > timerCount * 2 && TimerA < timerCount * 3) {
          display.clearDisplay();
          break;
        }

      case 3:
        heatCount = 0;
        StaticHeatDisplay();
        WarmCoilText();
        display.setCursor(baseCursorX, 0);
        display.write(0x7E);
        display.setCursor(baseCursorX + 15, 0);
        display.write(0x7E);
        display.display();
        if (millis() - TimerA > timerCount * 3 && TimerA < timerCount * 4) {
          display.clearDisplay();
          break;
        }
    }
  }

  display.clearDisplay();
  display.display();

  delay (1);
}

/* Adds pre-heat text and animation to main screen (replaces hits when active) */
void StaticHeatDisplay()
{
  if (sButtonCount <= 2) {
    int baseCursorY = 7;
    int baseCursorX = 101;

    // Display Text
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(baseCursorX + 1, baseCursorY + 8);
    display.println("PRE");
    display.setCursor(baseCursorX - 1, baseCursorY + 17);
    display.println("HEAT");

    // Display Icons
    display.setCursor(baseCursorX, baseCursorY);
    display.write(0xA9);
    display.setCursor(baseCursorX + 2, baseCursorY);
    display.write(0x1D);
    display.setCursor(baseCursorX + 7, baseCursorY);
    display.write(0x1D);
    display.setCursor(baseCursorX + 12, baseCursorY);
    display.write(0x1D);
    display.setCursor(baseCursorX + 14, baseCursorY);
    display.write(0xAA);
  }
}

/* Pre-Heating Function */
void PreHeatDisplay()
{
  /* Pre-Heating Stage:  [0] 1 2 */
  while (preHeatOn == 1 && preHeatStage == 0) {
    HeatCoil();           // Begin Heating Actual Coil
    preHeatStage = 1;
  }

  /* Pre-Heating Stage:  0 [1] 2 */
  if (preHeatOn == 1 && preHeatStage == 1) {
    PreHeat();
    preHeatStage = 2;
  }

  /* Pre-Heating Stage: 0 1 [2] */
  if (preHeatOn == 1 && preHeatStage == 2 ) {     /* && sButtonCount <= 3 if you dont want this on stealth screen */
    int baseCursorY = 5;
    int baseCursorX = 101;

    /* Displays Bottom Coil (Not Heat) */
    StaticHeatDisplay();

    /* Animate Heat from Coils in Pre-Heat Section */
    if (iCount <= 5) {
      display.setCursor(baseCursorX, baseCursorY - 1);
      display.write(0x7E);
      display.setCursor(baseCursorX + 7, baseCursorY - 3);
      display.write(0x7E);
      display.setCursor(baseCursorX + 15, baseCursorY - 1);
      display.write(0x7E);
      display.display();
      iCount++;
    }

    if (iCount > 5 && iCount <= 20) {
      display.setCursor(baseCursorX, baseCursorY - 3);
      display.write(0x7E);
      display.setCursor(baseCursorX + 7, baseCursorY - 4);
      display.write(0x7E);
      display.setCursor(baseCursorX + 15, baseCursorY - 3);
      display.write(0x7E);
      display.display();
      iCount++;
    }

    if (iCount > 20 && iCount <= 30) {
      for (int i = 0; i <= 10; i++) {
        display.setCursor(baseCursorX + 7, baseCursorY - 4);
        display.write(0x7E);
        display.display();
      }
      iCount = 0;
    }
  }
  display.display();
  display.clearDisplay();
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
        display.clearDisplay();
        display.invertDisplay(true);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(15, 9);
        display.println("LOW BATTERY");
        display.display();
        delay (1500);
        display.clearDisplay();
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
  // WakeScreen(); This function is not working so...
  // Do Nothing
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
  // START SLEEP
  display.clearDisplay();
  display.invertDisplay(true);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(16, 13);
  display.println("Going to Sleep...");
  display.display();
  delay(750);

  // GOODNIGHT
  display.setCursor(15, 13);
  display.clearDisplay();
  display.write(0xF6);
  display.write(0xF6);
  display.write(0xF6);
  display.print(" Goodnight ");
  display.write(0xF6);
  display.write(0xF6);
  display.write(0xF6);
  display.display();
  delay(1500);

  // CLEAN DISPLAY
  display.invertDisplay(false);
  display.clearDisplay();
  display.display();

  sButtonCount = 0;
  ledOff();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  attachInterrupt(0, WakeUpNow, LOW);

  sleep_mode();               // The program will continue from here.
  sleep_disable();
  detachInterrupt(0);
}

/* Piezeo sensor checks for motion */
void AutoSleepMode()
{
  /* Corrects Screen Inversion */
  if (sButtonCount >= 1) {
    screenFlip = true;
  }

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

  /* Sleep Screen Only */
  if (gSeconds == screenSleep )  {
    display.invertDisplay(false);
    display.clearDisplay();
    display.display();
    sButtonCount = 0;
    screenFlip = false;
  }

  /* Sleep Entire Device */
  if (gSeconds > deepSleep)  {
    EnterSleep();
    gSeconds = 0;
  }
}

/* ===========================================================================================*/
/*                                                                                            */
/*    Fire Vape Coil & LED's                                                                  */
/*                                                                                            */
/* ===========================================================================================*/

/* Check Fire Button and Turns on Heat */
void FireCoil()
{
  if (fButtonState == LOW)  {
    firePower = map(tButtonCount, 1, 3, 100, 255);
    analogWrite(mosfetPin, firePower);

    if (sButtonCount != 4) {
      display.clearDisplay();
      display.invertDisplay(true);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(18, 9);
      display.println("FIRE UP!");
      display.display();

      digitalWrite(fireButtonLED, HIGH);

      /* Set RGB LED Color */
      analogWrite(fireRpin, colorR);
      analogWrite(fireGpin, colorG);
      analogWrite(fireBpin, colorB);
    }
  }

  else  {
    analogWrite(mosfetPin, 0);
    analogWrite(fireBpin, 0);
    display.clearDisplay();
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

  for (int i = 0; i < 175; i++)  {
    avgTempRead = avgTempRead + (temperatureF - avgTempRead) / numReadings;
  }

  delay(1);
}

/* Updates Battery Icon & Sets #*/
void BattAdjust()
{
  if (sButtonCount != 4) {

    battRead = map(avgVoltRead, 2750, 4200, 0, 99);
    int battLines = map(battRead, 0, 99, 1, 17);

    for (int i = 0; i <= battLines; i++)
      display.drawFastHLine(battStartX, battStartY - i, battWidth, 1);
  }
}

/* Reads Temp Sensor & Shuts Down Functions if Too Hot*/
void ReadTemp()
{
  int reading = analogRead(thermoPin);

  float voltage = reading * calcVolt;                  // converting that reading to voltage
  voltage /= 1024.0;

  //converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)
  float temperatureC = (voltage - 0.5) * 100;

  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;    // Change to F (Line431)

  avgTempRead = (int) temperatureF;                    // Truncate digits after decimal

  delay(1);

  if (avgTempRead >= 180) {

    analogWrite(mosfetPin, 0);

    display.clearDisplay();
    display.invertDisplay(true);
    display.setCursor(13, 13);
    display.println("TOO HOT!!! WAIT...");
    display.display();
    setColor(255, 255, 0);
    delay(3500);
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

/* Shows the Product Logo Splash Screen */
void ShowLogo() {
  display.drawBitmap(0, 0, feather, 128, 32, 1);
  display.display();
  delay (showLogoTime);
  display.clearDisplay();
}

// **** BOOKMARK *** //
