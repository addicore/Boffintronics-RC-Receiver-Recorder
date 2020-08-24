//**********************************************************************************************************************
//
//                                    Steve Koci's DIY Remote Receiver Recorder
//                                        by Addicore.com & Boffintronics
//
//**********************************************************************************************************************
// Revision History:
//
//       06-16-2020   ARM     Version 1_0  Initial production Release
//       08-24-2020   ARM     Version 2_1  Added multi track recording
//                                         Fixed retrigger after lockout bug when using digital I/O as trigger
//                                         Fixed digital output polarity bug
//                                         Added flash code version by pressing PGM & REC on power up
//
//
//**********************************************************************************************************************
//
//      Arduino IDE Notes:
//      1. When uploading to the Receiver Recorder, Set Tools>Board: to "Arduino Leonardo"
//
//**********************************************************************************************************************
//
// Copyright (C) 2020  Boffintronics, LLC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
//**********************************************************************************************************************

#include <EEPROM.h>
#include <Servo.h>
#include <nRF24L01.h>    // NRF24L01 library by TMRh20 https://github.com/TMRh20/RF24
#include <RF24.h>
#include <SPI.h>
#include <SdFat.h>      // Can be found in the Arduino Library Manager
#include <MCP23S17.h>   // MCP23S17 Library by Majenko https://github.com/MajenkoLibraries/MCP23S17


const byte VERSION = 2;
const byte REVSION = 1;

//**********************************************************************************************************************
// SDcard

const int SD_CS = 12; // SD card chip select pin

SdFat SD;
File file;
File file2;

//------------------------------------------------------------------------------
#define errorHalt(msg) {Serial.println(F(msg)); SysCall::halt();}
//------------------------------------------------------------------------------

//**********************************************************************************************************************
// IO Expander

const byte IOX_CS = 9;   // IO Expander chip select (low true)
const byte IOX_RST = 8;  // IO Expander reset  (low true)

// Outputs
const byte EXA0 = 0;       //
const byte EXA1 = 1;       //
const byte PLAY_LED = 3;   // Play mode status LED (high true)
const byte REC_LED = 4;    // Record mode status LED (high true)

const byte EXB0 = 8;       // Outer left and shoulder button (Controller S4 & S8)
const byte EXB1 = 9;       // Left Toggle Switch (Controller S2)
const byte EXB2 = 10;      // Left Joystick button (Controller A1)
const byte EXB3 = 11;      // Center left button (Controller S5)
const byte EXB4 = 12;      // Center right button (Controller S6)
const byte EXB5 = 13;      // Right Joystick button (Controller A2)
const byte EXB6 = 14;      // Right Toggle Switch (Controller S3)
const byte EXB7 = 15;      // Outer right and shoulder button (Controller S7 & S9)

// Inputs
const byte SC_CP = 2;      // SD card present (low true)
const byte REC_BTN = 5;    // Record button (low true)
const byte PLAY_BTN = 6;   // Play button (low true)
const byte PROG_BTN = 7;   // Program mode button (low true)

MCP23S17 IOX(&SPI, IOX_CS, 0);

//**********************************************************************************************************************
// Misc Processor I/O

const int PROG_LED = 13;  // Program mode status LED
const int TRIG_HIGH = 7;  // Play trigger (high true)
const int LED_ON = 1;
const int LED_OFF = 0;

//**********************************************************************************************************************
// Servo output assignments

Servo myServo1;           // Left Slide pot (Controller R1)
Servo myServo2;           // Left Joystick X pot (Controller A1)
Servo myServo3;           // Left Joystick Y pot (Controller A1)
Servo myServo4;           // Right Joystick X pot (Controller A2)
Servo myServo5;           // Right Joystick Y pot (Controller A2)
Servo myServo6;           // Right Slide pot (Controller R2)

//**********************************************************************************************************************
// NRF24L01 Radio

const int NRF_CE = 10;
const int NRF_CSN = 11;

RF24 radio(NRF_CE, NRF_CSN);

struct Data_Package {     // NRF24L01 Data payload package from controller
  int LsPot;
  int LjPotX;
  int LjPotY;
  int RjPotX;
  int RjPotY;
  int RsPot;
  byte Switches;
};
Data_Package data;

int Program = false;

//**********************************************************************************************************************
// Setup
//**********************************************************************************************************************

void setup() {

  delay(2000);

  //**********************************************************************************************************************
  // IO setup

  pinMode(SD_CS, OUTPUT);    digitalWrite(SD_CS, 1);
  pinMode(NRF_CE, OUTPUT);   digitalWrite(NRF_CE, 1);
  pinMode(NRF_CSN, OUTPUT);  digitalWrite(NRF_CSN, 1);
  pinMode(IOX_CS, OUTPUT);   digitalWrite(IOX_CS, 1);
  pinMode(IOX_RST, OUTPUT);  digitalWrite(IOX_RST, 1);
  pinMode(PROG_LED, OUTPUT); digitalWrite(PROG_LED, 0);

  pinMode(TRIG_HIGH, INPUT);

  myServo1.attach(A5);
  myServo2.attach(A4);
  myServo3.attach(A3);
  myServo4.attach(A2);
  myServo5.attach(A1);
  myServo6.attach(A0);

  //**********************************************************************************************************************
  // IO Expander setup

  IOX.begin();

  // MCP23S17 bank A
  IOX.pinMode(0, INPUT_PULLUP);     // EXA0, play trigger (low true)
  IOX.pinMode(1, INPUT);            // EXA1, spare (currently unuse
  IOX.pinMode(2, INPUT_PULLUP);     // SD_CP, SD card present (low true)
  IOX.pinMode(3, OUTPUT);           // play LED (green)
  IOX.pinMode(4, OUTPUT);           // record LED (red)
  IOX.pinMode(5, INPUT_PULLUP);     // record button
  IOX.pinMode(6, INPUT_PULLUP);     // play button
  IOX.pinMode(7, INPUT_PULLUP);     // prog button

  // MCP23S17 bank B
  IOX.pinMode(8, OUTPUT);
  IOX.pinMode(9, OUTPUT);
  IOX.pinMode(10, OUTPUT);
  IOX.pinMode(11, OUTPUT);
  IOX.pinMode(12, OUTPUT);
  IOX.pinMode(13, OUTPUT);
  IOX.pinMode(14, OUTPUT);
  IOX.pinMode(15, OUTPUT);

  IOX.writePort(0, 0b00000000); // init bank A

  byte mask = EEPROM.read(1); //  init bank B using polarity mask from EEPROM to set inactive state
  IOX.digitalWrite(EXB0, !bitRead(mask, 7));
  IOX.digitalWrite(EXB1, !bitRead(mask, 6));
  IOX.digitalWrite(EXB2, !bitRead(mask, 5));
  IOX.digitalWrite(EXB3, !bitRead(mask, 4));
  IOX.digitalWrite(EXB4, !bitRead(mask, 3));
  IOX.digitalWrite(EXB5, !bitRead(mask, 2));
  IOX.digitalWrite(EXB6, !bitRead(mask, 1));
  IOX.digitalWrite(EXB7, !bitRead(mask, 0));


  //**********************************************************************************************************************
  //Serial.begin(19200);  // Initialize serial communications with PC  // uncomment for serial debug, must have IDE serial monitor
  //while (!Serial);      // wait for serial                           //  connected or code will hang
  Serial.println(F(" Steve Koci's DIY Remote Controller"));
  Serial.println(F("         by Addicore.com"));
  Serial.println(F("Receiver Recorder Release Version 2_1"));
  Serial.println(F("------------------------------------"));


  if (!IOX.digitalRead(PROG_BTN) && !IOX.digitalRead(REC_BTN)) { // if PGM & REC are down, flash code version
    BlinkCodeVersion();
  }

  if (IOX.digitalRead(SC_CP)) { // Flash error if no SD card inserted
    CardNotPresentError();
  }

  ReadSdConfig(); // load config file from SD card

  SetUpRadio();  // Setup Radio for receive

}

//**********************************************************************************************************************
// end of setup
//**********************************************************************************************************************


//**********************************************************************************************************************
// main loop
//**********************************************************************************************************************

// operational modes
const byte REALTIME = 1;
const byte PLAY = 2;
const byte REPLAY_DELAY = 3;
const byte TRIGGER_PLAY = 4;
const byte TRIGGER_LOCKOUT = 5;
const byte START_RECORD = 6;
const byte RECORD = 7;
const byte READY_TRACK_EDIT = 8;
const byte START_TRACK_RECORD = 9;
const byte TRACK_1_RECORD = 10;
const byte TRACK_RECORD = 11;
const byte TRACK_EDIT_PLAY = 12;
//

const int PLAY_REPLAY_DELAY_FLASH = 200;
const int TRIGGER_LOCKOUT_FLASH = 100;
const int TRACK_FLASH = 200;
const int TRACK_FLASH_DELAY = 600;

const int PLAY_ROW_DIM = 1; // 1 X 7 play array to read from card
const int PLAY_COL_DIM = 7; //
int plArray[PLAY_ROW_DIM][PLAY_COL_DIM];

const int SERVO_PARAM_ROW_DIM = 6; // 6 X 5 servo parameter array to read from card
const int SERVO_PARAM_COL_DIM = 5; //
int spArray[SERVO_PARAM_ROW_DIM][SERVO_PARAM_COL_DIM];

// parameters read from SD card
bool recordProtect;
byte radioFrequency;
byte digitalOutputPolarity;
bool triggerEnable;
unsigned long triggerPowerUpDelay;
unsigned long triggerLockOutTime;
unsigned long triggerDelay;
bool replayEnable;
unsigned long replayDelayTime;
unsigned long replayDelay;
//

byte mode = REALTIME;
unsigned long loopTime = millis();
unsigned long playLedFlashTime = millis();
unsigned long triggerPowerUpDelayTime = millis();
byte track;
byte trackBuf;
byte trackFlashCount;

//**********************************************************************************************************************
void loop() {

  CheckButtons();  // check play and record buttons

  switch (mode) {
    //---------------------------------------------------------------------------------------------
    case REALTIME:
      digitalWrite(PROG_LED, LED_OFF);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      IOX.digitalWrite(REC_LED, LED_OFF);


      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        //Serial.print(millis());
        //Serial.println(" ");
        WriteOutputs();
      }

      break;
    //---------------------------------------------------------------------------------------------
    case PLAY:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_OFF);
      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }
      loopTime = millis();

      if (readPlayArray() == 1) {
        Serial.println("End of file");
        mode = REALTIME;
        if (replayEnable) {
          mode = REPLAY_DELAY;
          replayDelayTime = millis();
          Serial.println("Replay Delay");
        }
        break;
      }

      data.LsPot = plArray[0][0];
      data.LjPotX = plArray[0][1];
      data.LjPotY = plArray[0][2];
      data.RjPotX = plArray[0][3];
      data.RjPotY = plArray[0][4];
      data.RsPot = plArray[0][5];
      data.Switches = plArray[0][6];

      WriteOutputs();
      break;

    //---------------------------------------------------------------------------------------------
    case REPLAY_DELAY:
      if (millis() > (playLedFlashTime + PLAY_REPLAY_DELAY_FLASH)) {
        IOX.digitalWrite (PLAY_LED, !(IOX.digitalRead(PLAY_LED)));
        playLedFlashTime = millis();
      }

      if (millis() > (replayDelayTime + replayDelay)) {
        mode = PLAY;
        file.rewind();
        Serial.println("End of Replay Delay - Play it Again");
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRIGGER_PLAY:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_OFF);
      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }
      loopTime = millis();

      if (readPlayArray() == 1) {
        Serial.println("End of file");
        mode = REALTIME;
        if (triggerEnable) {
          mode = TRIGGER_LOCKOUT;
          triggerLockOutTime = millis();
          Serial.println("Trigger Lockout");
        }
        break;
      }

      data.LsPot = plArray[0][0];
      data.LjPotX = plArray[0][1];
      data.LjPotY = plArray[0][2];
      data.RjPotX = plArray[0][3];
      data.RjPotY = plArray[0][4];
      data.RsPot = plArray[0][5];
      data.Switches = plArray[0][6];

      WriteOutputs();
      break;

    //---------------------------------------------------------------------------------------------
    case TRIGGER_LOCKOUT:
      if (millis() > (playLedFlashTime + TRIGGER_LOCKOUT_FLASH)) {
        IOX.digitalWrite (PLAY_LED, !(IOX.digitalRead(PLAY_LED)));
        playLedFlashTime = millis();
      }

      if (millis() > (triggerLockOutTime + triggerDelay)) {
        mode = REALTIME;
        Serial.println("End of Lockout");
        SetUpRadio(); // flush radio RX buffer
      }

      break;
    //---------------------------------------------------------------------------------------------
    case START_RECORD: // flash record led, recording starts on the third flash
      if (recordProtect) {
        RecordProtectError();
        mode = REALTIME;
        break;
      }

      IOX.digitalWrite(REC_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(1000);
      mode = RECORD;
      break;
    //---------------------------------------------------------------------------------------------
    case RECORD:
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      IOX.digitalWrite(REC_LED, LED_ON);

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        WriteSDcard();
        WriteOutputs();
      }
      break;

    //---------------------------------------------------------------------------------------------
    case READY_TRACK_EDIT: // flash next track number to play/record on prog LED

      IOX.digitalWrite(PLAY_LED, LED_OFF);
      IOX.digitalWrite(REC_LED, LED_OFF);

      if ((trackFlashCount != 0) && (millis() > (playLedFlashTime + TRACK_FLASH))) {
        digitalWrite (PROG_LED, !(digitalRead(PROG_LED)));
        if (!(digitalRead(PROG_LED))) {
          trackFlashCount--;
        }
        playLedFlashTime = millis();
      }

      if ((trackFlashCount == 0) && (millis() > (playLedFlashTime + TRACK_FLASH_DELAY))) {
        trackFlashCount = track;
        playLedFlashTime = millis();
      }
      break;

    //---------------------------------------------------------------------------------------------
    case START_TRACK_RECORD: // flash record & play leds, recording starts on the third flash

      if (recordProtect) {
        RecordProtectError();
        mode = REALTIME;
        break;
      }

      digitalWrite(PROG_LED, LED_ON);

      IOX.digitalWrite(REC_LED, LED_ON);
      IOX.digitalWrite(PLAY_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_ON);
      IOX.digitalWrite(PLAY_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      delay(1000);
      if (track == 1) {
        mode = TRACK_1_RECORD;
      } else {
        mode = TRACK_RECORD;
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRACK_1_RECORD:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_ON);

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        WriteSDcard();
        WriteOutputs();
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRACK_RECORD:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_ON);

      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }
      loopTime = millis();

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
      }

      if (readPlayArray() == 1) {
        Serial.println("End of track file");
        digitalWrite(PROG_LED, LED_OFF);
        mode = READY_TRACK_EDIT;
        file.close();
        file2.println("EOF");
        file2.close();
        if (SD.remove("TRACK_1.TXT")) {
          Serial.println("File removed");
        } else {
          Serial.println("Remove Error");
          SDerrorFlash();
        }
        if (SD.rename("TRACK_C.TXT", "TRACK_1.TXT")) {
          Serial.println("File renamed");
        } else {
          Serial.println("Rename Error");
          SDerrorFlash();
        }
        break;
      }

      data.LjPotX = plArray[0][1];
      data.LjPotY = plArray[0][2];

      if ((track == 3) || (track == 4)) {
        data.RjPotX = plArray[0][3];
        data.RjPotY = plArray[0][4];
      }
      if ((track == 2) || (track == 4)) {
        data.LsPot = plArray[0][0];
        data.RsPot = plArray[0][5];
      }
      if ((track == 2) || (track == 3)) {
        data.Switches = plArray[0][6];
      }

      WriteSDcard2();
      WriteOutputs();

      break;
    //---------------------------------------------------------------------------------------------
    case TRACK_EDIT_PLAY:
      digitalWrite(PROG_LED, LED_ON);
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_OFF);
      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }
      loopTime = millis();

      if (readPlayArray() == 1) {
        Serial.println("End of file");
        mode = READY_TRACK_EDIT;
        digitalWrite(PROG_LED, LED_OFF);
        break;
      }

      data.LsPot = plArray[0][0];
      data.LjPotX = plArray[0][1];
      data.LjPotY = plArray[0][2];
      data.RjPotX = plArray[0][3];
      data.RjPotY = plArray[0][4];
      data.RsPot = plArray[0][5];
      data.Switches = plArray[0][6];

      WriteOutputs();
      break;

    //---------------------------------------------------------------------------------------------
    default:
      break;
  }
}

//**********************************************************************************************************************
// end of main loop
//**********************************************************************************************************************

//--------------------------------------------------------------------------------------------------------------------
// Check play, record, prog buttons and play trigger pins
//--------------------------------------------------------------------------------------------------------------------

byte playButtonNow;
byte playButtonPrev = 0;
unsigned long playButtonTime = 0;
byte recButtonNow;
byte recButtonPrev = 0;
unsigned long recButtonTime = 0;
byte progButtonNow;
byte progButtonPrev = 1;
unsigned long progButtonTime;
unsigned long progButtonDownTime;
bool waitForProgUp = false;
const int RP_DEBOUNCE = 100;   // button debounce time (ms)
const int SHORT_PGM_PRESS = 150;   // PGM button short press (ms)
const int LONG_PGM_PRESS = 2000;   // PGM button long press (ms)


void CheckButtons(void) {

  //--------------------------------------------------------------------------------------------------------------------
  // Check Play Button

  playButtonNow = IOX.digitalRead(PLAY_BTN);
  if (playButtonNow == 0 && playButtonPrev == 1 && (millis() - playButtonTime) > RP_DEBOUNCE) {
    switch (mode) {

      case REALTIME:
        mode = PLAY;
        Serial.println("Play");
        //InitSDcard();
        file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
        file.rewind();
        break;

      case PLAY:
        mode = REALTIME;
        file.close();
        break;

      case REPLAY_DELAY:
        mode = REALTIME;
        file.close();
        break;

      case TRIGGER_PLAY:
        mode = REALTIME;
        file.close();
        break;

      case TRIGGER_LOCKOUT:
        mode = REALTIME;
        file.close();
        break;

      case START_RECORD:
        mode = START_RECORD;
        break;

      case RECORD:
        mode = RECORD;
        break;

      case READY_TRACK_EDIT:
        mode = TRACK_EDIT_PLAY;
        Serial.println("Track Edit Play");
        //InitSDcard();
        file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
        file.rewind();
        break;

      case TRACK_EDIT_PLAY:
        mode = READY_TRACK_EDIT;
        file.close();
        break;

      default:
        break;
    }
    playButtonTime = millis();
  }

  playButtonPrev = playButtonNow;

  //--------------------------------------------------------------------------------------------------------------------
  // Check Record Button

  recButtonNow = IOX.digitalRead(REC_BTN);
  if (recButtonNow == 0 && recButtonPrev == 1 && (millis() - recButtonTime) > RP_DEBOUNCE) {
    switch (mode) {

      case REALTIME:
        if (IOX.digitalRead(PROG_BTN)) {
          mode = START_RECORD;    // regular record mode
          Serial.println("Record");
          //InitSDcard();
          file = SD.open("TRACK_1.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
          if (!file) {
            Serial.println("file open failed");
            SDerrorFlash();
          }
        } else {    // track edit mode
          digitalWrite(PROG_LED, LED_OFF);
          mode = READY_TRACK_EDIT;
          waitForProgUp = true;
          Serial.println("Ready Track EDIT");
          track = 1;
          trackFlashCount = track;
          playLedFlashTime = millis();
        }
        break;

      case RECORD:
        mode = REALTIME;
        file.println("EOF");
        file.close();
        Serial.println("End Record");
        break;

      case PLAY:
        mode = PLAY;
        break;

      case READY_TRACK_EDIT:
        mode = START_TRACK_RECORD;
        Serial.print("Track ");
        Serial.print(track);
        Serial.println(" Record");

        if (track == 1) {
          file = SD.open("TRACK_1.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
          if (!file) {
            Serial.println("file 1 open failed");
            SDerrorFlash();
          }
          break;

        } else {
          file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
          if (!file) {
            Serial.println("file open failed");
            SDerrorFlash();
          }
          file.rewind();
        }

        file2 = SD.open("TRACK_C.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
        if (!file2) {
          Serial.println("file 2 open failed");
          SDerrorFlash();
        }
        break;

      case TRACK_1_RECORD:
        digitalWrite(PROG_LED, LED_OFF);
        mode = READY_TRACK_EDIT;
        file.println("EOF");
        file.close();
        Serial.println("End Record");
        break;

      /*
        case TRACK_RECORD:
        mode = READY_TRACK_EDIT;
        file.println("EOF");
        file.close();
        file2.close();
        Serial.println("End Record");
        break;
      */


      default:
        break;
    }
    recButtonTime = millis();
  }
  recButtonPrev = recButtonNow;

  //--------------------------------------------------------------------------------------------------------------------
  // Check prog Button

  progButtonNow = IOX.digitalRead(PROG_BTN);

  if ((waitForProgUp) && (IOX.digitalRead(PROG_BTN))) {
    waitForProgUp = 0;
  } else {
    if (progButtonNow == 1 && progButtonPrev == 1 && (millis() - progButtonTime) > RP_DEBOUNCE) {
      progButtonDownTime = millis();
    }

    if (progButtonNow == 1 && progButtonPrev == 0 && (millis() - progButtonTime) > RP_DEBOUNCE) {
      if (((millis() - progButtonDownTime ) >= SHORT_PGM_PRESS)
          && ((millis() - progButtonDownTime ) < LONG_PGM_PRESS)
          && ( mode == READY_TRACK_EDIT)) {
        track++;
        if (track > 4) {
          track = 1;
        }
      }
      if (((millis() - progButtonDownTime ) > LONG_PGM_PRESS)
          && ( mode == READY_TRACK_EDIT)) {
        digitalWrite(PROG_LED, LED_OFF);
        mode = REALTIME;
      }
      progButtonTime = millis();
    }
  }

  progButtonPrev = progButtonNow;

  //--------------------------------------------------------------------------------------------------------------------
  // Check Trigger inputs
  if (millis() > (triggerPowerUpDelayTime + triggerPowerUpDelay)) {        // don't check trigger inputs while in trigger power up delay
    if ((IOX.digitalRead(EXA0) == 0) || (digitalRead(TRIG_HIGH) == 1)) {   // check if either are active

      switch (mode) {

        case REALTIME:
          mode = TRIGGER_PLAY;
          Serial.println("Trigger Play");
          //InitSDcard();
          file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
          file.rewind();
          break;

        default:
          break;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------------------------

void InitSDcard(void) {

  Serial.println("Init SD Card");

  if (!SD.begin(SD_CS, SPI_HALF_SPEED)) {
    Serial.println("Init failed");
    SDerrorFlash();
  }
  Serial.println("Init success");
}

//--------------------------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------------------------

void WriteSDcard(void) {
  file.print(data.LsPot); file.print(",");
  file.print(data.LjPotX); file.print(",");
  file.print(data.LjPotY); file.print(",");
  file.print(data.RjPotX); file.print(",");
  file.print(data.RjPotY); file.print(",");
  file.print(data.RsPot); file.print(",");
  for (int i = 7; i >= 0; i--)
    file.print(bitRead(data.Switches, i));
  file.println();
}

void WriteSDcard2(void) {
  file2.print(data.LsPot); file2.print(",");
  file2.print(data.LjPotX); file2.print(",");
  file2.print(data.LjPotY); file2.print(",");
  file2.print(data.RjPotX); file2.print(",");
  file2.print(data.RjPotY); file2.print(",");
  file2.print(data.RsPot); file2.print(",");
  for (int i = 7; i >= 0; i--)
    file2.print(bitRead(data.Switches, i));
  file2.println();
}

//--------------------------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------------------------

void WriteOutputs(void) {

  // Write to servos with adjustments from config.txt
  myServo1.write(AdjustServoData(data.LsPot, spArray[0][1], spArray[0][2], spArray[0][3], spArray[0][4])); // left slide pot
  myServo2.write(AdjustServoData(data.LjPotX, spArray[1][1], spArray[1][2], spArray[1][3], spArray[1][4])); // left joystick x pot
  myServo3.write(AdjustServoData(data.LjPotY, spArray[2][1], spArray[2][2], spArray[2][3], spArray[2][4])); // left joystick y pot
  myServo4.write(AdjustServoData(data.RjPotX, spArray[3][1], spArray[3][2], spArray[3][3], spArray[3][4])); // right joystick x pot
  myServo5.write(AdjustServoData(data.RjPotY, spArray[4][1], spArray[4][2], spArray[4][3], spArray[4][4])); // right joystick y pot
  myServo6.write(AdjustServoData(data.RsPot, spArray[5][1], spArray[5][2], spArray[5][3], spArray[5][4])); // right slide pot

  // Write digital data (buttons) to outputs, flip bit polarity per the output polarity mask from config.txt
  IOX.digitalWrite(EXB0, bitRead(digitalOutputPolarity, 0) ^ bitRead(data.Switches, 7));
  IOX.digitalWrite(EXB1, bitRead(digitalOutputPolarity, 1) ^ bitRead(data.Switches, 6));
  IOX.digitalWrite(EXB2, bitRead(digitalOutputPolarity, 2) ^ bitRead(data.Switches, 5));
  IOX.digitalWrite(EXB3, bitRead(digitalOutputPolarity, 3) ^ bitRead(data.Switches, 4));
  IOX.digitalWrite(EXB4, bitRead(digitalOutputPolarity, 4) ^ bitRead(data.Switches, 3));
  IOX.digitalWrite(EXB5, bitRead(digitalOutputPolarity, 5) ^ bitRead(data.Switches, 2));
  IOX.digitalWrite(EXB6, bitRead(digitalOutputPolarity, 6) ^ bitRead(data.Switches, 1));
  IOX.digitalWrite(EXB7, bitRead(digitalOutputPolarity, 7) ^ bitRead(data.Switches, 0));

}

//--------------------------------------------------------------------------------------------------------------------
// Adjust servo data for end points and deadzone
//--------------------------------------------------------------------------------------------------------------------

int AdjustServoData (int receivedData, int end1, int end2, int center, int deadZone) {
  int adjustedData;
  int scaledData;
  int deadZoneHigh;
  int deadZoneLow;

  scaledData = map(receivedData, 0, 1023, end1, end2);
  deadZoneHigh = center + (deadZone / 2);
  deadZoneLow = center - (deadZone / 2);

  if ((scaledData < deadZoneHigh) && (scaledData > deadZoneLow)) {
    adjustedData = center;
  }

  if (end1 < end2) {

    if (scaledData >= (deadZoneHigh)) {
      adjustedData = map(scaledData, deadZoneHigh, end2, center, end2);
    }

    if (scaledData <= (deadZoneLow)) {
      adjustedData = map(scaledData, deadZoneLow, end1, center, end1);
    }

  } else {

    if (scaledData >= (deadZoneHigh)) {
      adjustedData = map(scaledData, deadZoneHigh, end1, center, end1);
    }

    if (scaledData <= (deadZoneLow)) {
      adjustedData = map(scaledData, deadZoneLow, end2, center, end2);
    }
  }

  return adjustedData;
}

// -------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------
//   NRF24L01 radio setup
//--------------------------------------------------------------------------------------------------------------------


// NRF24L01 frequency table
// this table contains 11 selected frequencies out of the 125 possible for the NRF24L01
// [0] = the master frequency used only in program mode to sync with the receiver
// [1] through [10] are the available operating frequencies that can be set in program mode
// valid NRF24 frequencies are numbered 0 to 124, frequencies 100-124 are reccomended as are they above wifi
const int NRFfrequencyTable[12] = {100, 102, 104, 106, 108, 110, 112, 114, 116, 120, 124};

// NRF24L01 pipe address table
const byte pipes[][6] = {"Pipe0", "Pipe1", "Pipe2", "Pipe3", "Pipe4", "Pipe5"};

void SetUpRadio(void) {
  byte NRFfrequencyIndex = 0;
  byte NRFpipeIndex = 1;

  NRFfrequencyIndex = radioFrequency; // radio frequency from SD card
  NRFpipeIndex = 1;                   // always pipe1 for receiver recorder

  radio.begin();
  Serial.println("Init radio:");
  radio.setChannel(NRFfrequencyTable[NRFfrequencyIndex]);  // use the index to look up the frequency and set the NRF24l01
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);   // Set data rate to 250kbps
  radio.startListening();
  radio.openReadingPipe(1, pipes[NRFpipeIndex]);

  Serial.print ("  Frequency = ");
  Serial.print (NRFfrequencyIndex);
  Serial.print ("  ");
  Serial.println (NRFfrequencyTable[NRFfrequencyIndex]);
  Serial.print ("  Pipe = ");
  Serial.print (NRFpipeIndex);
  Serial.print ("  ");
  String pipe = pipes[NRFpipeIndex];
  Serial.println (pipe);
  return;
}

//--------------------------------------------------------------------------------------------------------------------

int readPlayArray(void) {

  // Array for play data.
  int i = 0;     // First array index.
  int j = 0;     // Second array index
  size_t n;      // Length of returned field with delimiter.
  char str[20];  // Must hold longest field with delimiter and zero byte.
  char *ptr;     // Test for valid field.
  int error = 0;

  // Read the file and store the data.

  for (i = 0; i < PLAY_ROW_DIM; i++) {
    for (j = 0; j < PLAY_COL_DIM; j++) {
      n = readField(&file, str, sizeof(str), ",\n");

      if (str[0] == 'E') {
        Serial.println("EOF");
        error = 1;
        return error;
      }

      if (n == 0) {
        errorHalt("Too few lines");
      }
      if (j == 6) {
        plArray[i][j] = strtol(str, &ptr, 2);
      } else {
        plArray[i][j] = strtol(str, &ptr, 10);
      }
      if (ptr == str) {
        errorHalt("bad number");
      }
      while (*ptr == ' ') {
        ptr++;
      }
      if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
        errorHalt("extra characters in field");
      }
      if (j < (PLAY_COL_DIM - 1) && str[n - 1] != ',') {
        errorHalt("line with too few fields");
      }
    }
    // Allow missing endl at eof.
    if (str[n - 1] != '\n' && file.available()) {
      errorHalt("missing endl");
    }
  }

  return error;
}

//--------------------------------------------------------------------------------------------------------------------

int readServoParamArray(void) {

  // Array for servo parameter data.
  int i = 0;     // First array index.
  int j = 0;     // Second array index
  size_t n;      // Length of returned field with delimiter.
  char str[20];  // Must hold longest field with delimiter and zero byte.
  char *ptr;     // Test for valid field.
  int error = 0;

  // Read the file and store the data.

  for (i = 0; i < SERVO_PARAM_ROW_DIM; i++) {
    for (j = 0; j < SERVO_PARAM_COL_DIM; j++) {
      n = readField(&file, str, sizeof(str), ",\n");

      if (str[0] == 'E') {
        Serial.println("EOF");
        error = 1;
        return error;
      }

      if (n == 0) {
        errorHalt("Too few lines");
      }

      spArray[i][j] = strtol(str, &ptr, 10);
      if (ptr == str) {
        errorHalt("bad number");
      }
      while (*ptr == ' ') {
        ptr++;
      }
      if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
        errorHalt("extra characters in field");
      }
      if (j < (SERVO_PARAM_COL_DIM - 1) && str[n - 1] != ',') {
        errorHalt("line with too few fields");
      }
    }
    // Allow missing endl at eof.
    if (str[n - 1] != '\n' && file.available()) {
      errorHalt("missing endl");
    }
  }

  return error;
}

//--------------------------------------------------------------------------------------------------------------------
//

size_t readField(File * file, char* str, size_t size, const char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
      break;
    }
  }
  str[n] = '\0';
  return n;
}

//--------------------------------------------------------------------------------------------------------------------
void printArray(void) {

  int i = 0;     // First array index.
  int j = 0;     // Second array index

  // Print the array.
  for (i = 0; i < SERVO_PARAM_ROW_DIM; i++) {
    Serial.print("   S");
    for (j = 0; j < SERVO_PARAM_COL_DIM; j++) {
      if (j) {
        Serial.print(" ");
      }
      Serial.print(spArray[i][j]);
    }
    Serial.println();
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Read config information from SD card

size_t n;      // Length of returned field with delimiter.
char str[20];  // Must hold longest field with delimiter and zero byte.
char *ptr;     // Test for valid field.

void ReadSdConfig(void) {

  // init the card
  InitSDcard();

  // open the file for reading:
  file = SD.open("config.txt", O_READ );
  if (file) {
    Serial.println("Read config.txt:");

    // read from the file until there's nothing else in it:
    while (file.available()) {
      int readData = file.read();
      if (readData == '*') { // only process lines that begin with '*'
        readData = file.read();

        switch (readData) {

          case 'S':
            switch (file.read()) {
              case 'P':
                readData = file.read();   // burn the 'cr' char
                readData = file.read();   // burn the 'lf' char
                readServoParamArray();    // read the servo parameter block
                Serial.println("  Servo Parameters =");
                printArray();
                break;

              default:
                break;
            }

          case 'O':
            switch (file.read()) {
              case 'P':
                readData = file.read();    // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                digitalOutputPolarity = (strtol(str, &ptr, 2));
                if (EEPROM.read(1) != digitalOutputPolarity) {  // save in eeprom if value changed
                  EEPROM.write(1, digitalOutputPolarity);
                }
                Serial.print("  Digital Output Polarity Mask = ");
                Serial.print(str);
                break;

              default:
                break;
            }

          case 'T':
            switch (file.read()) {
              case 'E':
                readData = file.read();           // burn the 'space' char
                triggerEnable = false;
                if (file.read() == 'E') {         //
                  triggerEnable = true;
                }
                Serial.print("  Trigger Enable = ");
                Serial.println(triggerEnable);
                break;

              case 'P':
                readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                triggerPowerUpDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                Serial.print("  Trigger Power Up Delay = ");
                Serial.print(triggerPowerUpDelay / 1000);
                Serial.println(" secs");
                break;

              case 'D':
                readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                triggerDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                Serial.print("  Trigger Delay = ");
                Serial.print(triggerDelay / 1000);
                Serial.println(" secs");
                break;

              default:
                break;
            }
            break;

          case 'R':
            switch (file.read()) {

              case 'D':
                readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                replayDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                Serial.print("  Replay Delay = ");
                Serial.print(replayDelay / 1000);
                Serial.println(" secs");
                break;

              case 'E':
                readData = file.read();             // burn the 'space' char
                replayEnable = false;
                if (file.read() == 'E') {           //
                  replayEnable = true;
                }
                Serial.print("  Replay Enable = ");
                Serial.println(replayEnable);
                break;

              case 'F':
                readData = file.read();             // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                radioFrequency = (strtol(str, &ptr, 10));
                Serial.print("  Radio Frequency = ");
                Serial.println(radioFrequency);
                break;

              case 'P':
                readData = file.read();             // burn the 'space' char
                recordProtect = false;
                if (file.read() == 'P') {           //
                  recordProtect = true;
                }
                Serial.print("  Record Protect = ");
                Serial.println(recordProtect);
                break;

              default:
                break;
            }

          default:
            break;
        }
      }
    }
    // close the file:
    file.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening config.txt");
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Record Protect error flash

void RecordProtectError(void) {
  Serial.println("Record Protected");
  IOX.digitalWrite(REC_LED, LED_ON);
  IOX.digitalWrite(PLAY_LED, LED_OFF);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_OFF);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_ON);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_OFF);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_ON);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_OFF);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_ON);
  delay(200);
  IOX.digitalWrite(REC_LED, LED_OFF);
}

//--------------------------------------------------------------------------------------------------------------------
// Card Not Present Error

void CardNotPresentError(void) {
  Serial.println("NO SD CARD");

  while (1) {
    IOX.digitalWrite(REC_LED, LED_OFF);
    IOX.digitalWrite(PLAY_LED, LED_ON);
    delay(200);
    IOX.digitalWrite(REC_LED, LED_ON);
    IOX.digitalWrite(PLAY_LED, LED_OFF);
    delay(200);
  }
}

//--------------------------------------------------------------------------------------------------------------------
// SD Card Error

void SDerrorFlash(void) {
  Serial.println("SD error");

  while (1) {
    IOX.digitalWrite(PROG_LED, LED_OFF);
    delay(200);
    IOX.digitalWrite(PROG_LED, LED_ON);
    delay(200);
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Blink Code Version

void BlinkCodeVersion(void) {
  while (1) {
    for (int i = 1; i <= VERSION; i++) {
      IOX.digitalWrite(PLAY_LED, LED_ON);
      delay(200);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      delay(200);
    }

    for (int i = 1; i <= REVSION; i++) {
      IOX.digitalWrite(REC_LED, LED_ON);
      delay(200);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(200);
    }
  }
}

//**********************************************************************************************************************
// end of file
//**********************************************************************************************************************
