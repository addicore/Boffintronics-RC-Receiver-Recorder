
//**********************************************************************************************************************
//
//                                    Steve Koci's DIY Remote Receiver Recorder
//                                        by Addicore.com & Boffintronics
//
//**********************************************************************************************************************
// Revision History:
//
//       06-16-2020   ARM     Version 1_0   Initial production Release
//
//       08-24-2020   ARM     Version 2_1   Added multi track recording
//                                          Fixed retrigger after lockout bug when using digital I/O as trigger
//                                          Fixed digital output polarty bug
//                                          Added flash code version by pressing PGM & REC on power up
//
//       08-13-2021   ARM     Version 3_1   Added sound board support
//                                          Added servos 7 and 8

//**********************************************************************************************************************
//
//      Arduino IDE Notes:
//      1. When uploading to the Receiver Recorder, Set Tools>Board: to "Arduino Leonardo"
//      2. This sketch has been verified with the library versions noted below. Versions other than those
//         may cause eratic operation
//
//**********************************************************************************************************************
//
// Copyright (C) 2021  Boffintronics, LLC
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

#include <Servo.h>              // Servo Library v1.1.6
#include <nRF24L01.h>           // RF24 library by TMRh20 v1.3.6 https://github.com/TMRh20/RF24
#include <RF24.h>               // RF24 library by TMRh20 v1.3.6 https://github.com/TMRh20/RF24
#include <SPI.h>                // Standard Arduino Library
#include <SdFat.h>              // SDFat Library v1.1.1
#include <MCP23S17.h>           // MCP23S17 Library by Majenko v1.1.3 https://github.com/MajenkoLibraries/MCP23S17
#include <DFPlayerMini_Fast.h>  // DFPlayerMini_Fast Library by PowerBroker2 v1.1.12
#include <EEPROM.h>             // Standard Arduino Library

const byte VERSION = 3;
const byte REVSION = 1;
const byte BETA = 0;

//**********************************************************************************************************************
// SDcard

const int SD_CS = 12; // SD card chip select pin

SdFat SD;
File file;
File file2;


//**********************************************************************************************************************
// IO Expander

const byte IOX_CS = 9;   // IO Expander chip select (low true)
const byte IOX_RST = 8;  // IO Expander reset  (low true)

// Outputs
const byte EXA0 = 0;       //
const byte EXA1 = 1;       //
const byte PLAY_LED = 3;   // Play mode status Green LED (high true)
const byte REC_LED = 4;    // Record mode status Red LED (high true)

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

const byte PROG_LED = 13;  // Program mode status Blue LED (high true)
const byte TRIG_HIGH = 7;  // Play trigger (high true)
const byte LED_ON = 1;
const byte LED_OFF = 0;

//**********************************************************************************************************************
// Servo output assignments

const byte servoPins[] = {A5, A4, A3, A2, A1, A0, 4, 5};
Servo myServo[8];

//myServo[0] S1 on PCB, Track 3 Left Slide pot (Controller R1)
//myServo[1] S2 on PCB, Track 1 Left Joystick X pot (Controller A1)
//myServo[2] S3 on PCB, Track 1 Left Joystick Y pot (Controller A1)
//myServo[3] S4 on PCB, Track 2 Right Joystick X pot (Controller A2)
//myServo[4] S5 on PCB, Track 2 Right Joystick Y pot (Controller A2)
//myServo[5] S6 on PCB, Track 3 Right Slide pot (Controller R2)
//myServo[6] S7 on PCB, Track 5 Left Slide pot (Controller R1)
//myServo[7] S8 on PCB, Track 5 Right Slide pot (Controller R2) /Jaw servo

//**********************************************************************************************************************
// NRF24L01 Radio

const byte NRF_CE = 10;
const byte NRF_CSN = 11;

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


//**********************************************************************************************************************
// Sound board

const byte MSGEQ7_RESET = 1;       // reset (io expander)
const byte MSGEQ7_STROBE = 3;      // strobe
const byte MP3_PLAYER_nBUSY = 2;   // mp3 player busy / playing (low true)
const byte MSGEQ7_AUDIO_OUT = A7;  // audio out to ADC in

DFPlayerMini_Fast myMP3;

//**********************************************************************************************************************
// globals
const byte PLAY_ROW_DIM = 1; // 1 X 10 play array to read from card
const byte PLAY_COL_DIM = 10; //
//int plArray[PLAY_ROW_DIM][PLAY_COL_DIM];
int plArray[PLAY_COL_DIM];
const byte SERVO_PARAM_ROW_DIM = 8; // 8 X 5 servo parameter array to read from config file
const byte SERVO_PARAM_COL_DIM = 5; //
int spArray[SERVO_PARAM_ROW_DIM][SERVO_PARAM_COL_DIM];
byte digitalOutputPolarity;
int servoData[8];

//**********************************************************************************************************************
// Setup
//**********************************************************************************************************************

void setup() {


  //**********************************************************************************************************************
  // IO setup

  pinMode(SD_CS, OUTPUT);    digitalWrite(SD_CS, 1);
  pinMode(NRF_CE, OUTPUT);   digitalWrite(NRF_CE, 1);
  pinMode(NRF_CSN, OUTPUT);  digitalWrite(NRF_CSN, 1);
  pinMode(IOX_CS, OUTPUT);   digitalWrite(IOX_CS, 1);
  pinMode(IOX_RST, OUTPUT);  digitalWrite(IOX_RST, 1);
  pinMode(PROG_LED, OUTPUT); digitalWrite(PROG_LED, 0);

  pinMode(TRIG_HIGH, INPUT);

  //**********************************************************************************************************************
  // IO Expander setup

  IOX.begin();

  // MCP23S17 bank A
  IOX.pinMode(0, INPUT_PULLUP);     // EXA0, play trigger (low true)
  //IOX.pinMode(1, INPUT);            // EXA1, spare (currently unused)
  IOX.pinMode(2, INPUT_PULLUP);     // SD_CP, SD card present (low true)
  IOX.pinMode(3, OUTPUT);           // play LED (green)
  IOX.pinMode(4, OUTPUT);           // record LED (red)
  IOX.pinMode(5, INPUT_PULLUP);     // record button
  IOX.pinMode(6, INPUT_PULLUP);     // play button
  IOX.pinMode(7, INPUT_PULLUP);     // prog button

  //**********************************************************************************************************************
  //

  //Serial.begin(115200); // Initialize serial communications with PC  // uncomment for serial debug, must have IDE serial monitor
  //while (!Serial);      // wait for serial                           //  connected or code will hang
  /*
    Serial.println(F("Steve Koci's DIY Remote Controller"));
    Serial.println(F("      by Addicore.com"));
    Serial.println(F("Receiver Recorder Version 3.1"));
  */

  // if PGM & REC buttons are down at reset, flash code version
  if (!IOX.digitalRead(PROG_BTN) && !IOX.digitalRead(REC_BTN)) {
    BlinkCodeVersion();
  }

  // Flash error if no SD card inserted
  if (IOX.digitalRead(SC_CP)) {
    CardNotPresentError();
  }

  if (EEPROM.read(1023) == 0xAA) { // check if EEPROM is initialzed
    for (byte i = 0; i <= 7; i++) { // attach servos and set to rest position from EEPROM
      myServo[i].write(EEPROM.read(i));
      myServo[i].attach(servoPins[i]);
    }
  }

  ReadSdConfig();       // load config file from SD card

  if (EEPROM.read(1023) != 0xAA) { // check if EEPROM is initialzed
    for (byte i = 0; i <= 7; i++) { // attach servos and set to rest position from config file
      myServo[i].write(spArray[i][1]);
      myServo[i].attach(servoPins[i]);
    }
  }

  // digital outputs (MCP23S17 bank B) set up from config file
  for (byte i = 8; i <= 15; i++) {
    IOX.pinMode(i, OUTPUT);
    IOX.digitalWrite(i, !bitRead(digitalOutputPolarity, 15 - i));
  }

  // Setup Radio with parameters from config file
  SetUpRadio();

  // Setup audio board I/O and with parameters from config file
  SetUpAudioBoard();
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

byte AudioClip;

// parameters read from SD card
bool recordProtect;
byte radioFrequency;
//byte digitalOutputPolarity;
bool triggerEnable;
unsigned long triggerPowerUpDelay;
unsigned long triggerLockOutTime;
unsigned long triggerDelay;
bool replayEnable;
unsigned long replayDelayTime;
unsigned long replayDelay;
bool audioEnable;
byte audioVolume1;
byte audioVolume2;
byte jawEnable;
byte jawFrequencyBands;
bool playAmbiantAtPowerUpEnable;
bool playClipTwoAtStartEnable;
bool JawGainPotEnable;
bool clip1Playing = false;
bool clip2Playing = false;
//

byte mode = REALTIME;
unsigned long loopTime = millis();
unsigned long playLedFlashTime = millis();
unsigned long triggerPowerUpDelayTime = millis();
byte track;
byte trackBuf;
byte trackFlashCount;
int MSGEQ7bands[8];

//**********************************************************************************************************************
void loop() {

  CheckButtons();  // check play and record buttons
  ReadMSGEQ7();    // read the mp3 audio levels

  switch (mode) {
    //---------------------------------------------------------------------------------------------
    case REALTIME:
      digitalWrite(PROG_LED, LED_OFF);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      IOX.digitalWrite(REC_LED, LED_OFF);

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        CopyArray();
        WriteOutputs();
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRACK_EDIT_PLAY:
      digitalWrite(PROG_LED, LED_ON);
    case PLAY:
    case TRIGGER_PLAY:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_OFF);
      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }
      loopTime = millis();

      if (audioEnable && digitalRead(MP3_PLAYER_nBUSY)) { // start clip1 (ambiant) if clip2 is finished and track is still running
        StopAudioClip2(); // stop mp3 player & start clip 1
      }

      if (readPlayArray() == 1) {
        //Serial.println("EF");

        if (clip2Playing) {
          StopAudioClip2(); // stop clip 2 & start clip 1 if clip1 isn't already playing
        }

        if (mode == PLAY) {
          mode = REALTIME;
          if (replayEnable) {
            mode = REPLAY_DELAY;
            replayDelayTime = millis();
            //Serial.println("RD");
          }
        }

        if (mode == TRIGGER_PLAY) {
          if (triggerEnable) {
            mode = TRIGGER_LOCKOUT;
            triggerLockOutTime = millis();
            //Serial.println("Tg Lk");
          }
        }

        if (mode == TRACK_EDIT_PLAY) {
          mode = READY_TRACK_EDIT;
          digitalWrite(PROG_LED, LED_OFF);
        }

        break;
      }

      for (byte i = 0; i <= 5; i++) {
        servoData[i] = plArray[i];
      }

      data.Switches = plArray[6];
      servoData[6] = plArray[7];

      if ((audioEnable == false) || (jawEnable == false)) {
        servoData[7] = plArray[8];
      } else {
        if (clip2Playing == true) {
          servoData[7] = MSGEQ7bands[0];
        }
      }
      AudioClip = plArray[9];

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
        //Serial.println("ERD");
        StartAudioClip2();
      }
      break;


    //---------------------------------------------------------------------------------------------
    case TRIGGER_LOCKOUT:
      if (millis() > (playLedFlashTime + TRIGGER_LOCKOUT_FLASH)) {
        IOX.digitalWrite (PLAY_LED, !(IOX.digitalRead(PLAY_LED)));
        playLedFlashTime = millis();
      }

      if (millis() > (triggerLockOutTime + triggerDelay)) {
        mode = REALTIME;
        Serial.println("ELO");
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
      IOX.digitalWrite(PLAY_LED, LED_OFF);

      IOX.digitalWrite(REC_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_ON);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(1000);
      IOX.digitalWrite(REC_LED, LED_ON);

      StartAudioClip2();

      mode = RECORD;
      break;

    //---------------------------------------------------------------------------------------------
    case RECORD:

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        CopyArray();
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
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_ON);

      StartAudioClip2();

      if (track == 1) {
        mode = TRACK_1_RECORD;
      } else {
        mode = TRACK_RECORD;
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRACK_1_RECORD:

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        CopyArray();
        WriteSDcard();
        WriteOutputs();
      }
      break;

    //---------------------------------------------------------------------------------------------
    case TRACK_RECORD:

      while (millis() < (loopTime + 33)) {  // sync to 33 ms
      }

      loopTime = millis();

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
      }

      if (readPlayArray() == 1) {
        //Serial.println("End track file");
        digitalWrite(PROG_LED, LED_OFF);
        mode = READY_TRACK_EDIT;
        StopAudioClip2();
        SaveServoRest();
        file.close();
        file2.println("EF");
        file2.close();
        if (SD.remove("TRACK_1.TXT")) {
          //Serial.println("FD");
        } else {
          //Serial.println("Remove Error");
          SDerrorFlash(5); // file delete error
        }
        if (SD.rename("TRACK_C.TXT", "TRACK_1.TXT")) {
          //Serial.println("FR");
        } else {
          //Serial.println("Rename Error");
          SDerrorFlash(4); // file rename error
        }
        break;
      }

      switch (track) {

        //       case 1:
        //        plArray[0][1] = data.LjPotX;
        //        plArray[0][2] = data.LjPotY;
        //        break;

        case 2:
          plArray[3] = data.RjPotX;
          plArray[4] = data.RjPotY;
          break;

        case 3:
          plArray[0] = data.LsPot;
          plArray[5] = data.RsPot;
          break;

        case 4:
          plArray[6] = data.Switches;
          break;

        case 5:
          plArray[7] = data.LsPot;

          if ((audioEnable == true) && (jawEnable == true)) {
            plArray[8] = 0;
          } else {
            plArray[8] = data.RsPot;
          }
          break;

        case 6:
          plArray[9] = data.Switches;
          break;

        default:
          break;
      }


      if ((audioEnable == true)
          && (jawEnable == true)
          && (clip2Playing == true)) {
        plArray[8] = MSGEQ7bands[0];
      }

      AudioClip = plArray[9];

      for (int i = 0; i <= 5; i++) {
        file2.print(plArray[i]); file2.print(",");
        servoData[i] = plArray[i];
      }

      for (int i = 7; i >= 0; i--) {
        file2.print(bitRead(plArray[6], i));
      }

      file2.print(",");
      file2.print(plArray[7]); file2.print(",");
      servoData[6] = plArray[7];
      file2.print(plArray[8]); file2.print(",");
      servoData[7] = plArray[8];

      for (int i = 7; i >= 0; i--) {
        file2.print(bitRead(AudioClip, i));
      }
      file2.println();
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
const byte LAST_TRACK = 6;     // last valid track number

void CheckButtons(void) {

  //--------------------------------------------------------------------------------------------------------------------
  // Check Play Button

  playButtonNow = IOX.digitalRead(PLAY_BTN);
  if (playButtonNow == 0 && playButtonPrev == 1 && (millis() - playButtonTime) > RP_DEBOUNCE) {
    switch (mode) {

      case REALTIME:
        mode = PLAY;
        //Serial.println("P");
        file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
        file.rewind();
        StartAudioClip2();
        break;


      case PLAY:
      case REPLAY_DELAY:
      case TRIGGER_PLAY:
        StopAudioClip2(); // stop mp3 player
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
        //Serial.println("TEP");
        file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
        file.rewind();
        StartAudioClip2();
        break;

      case TRACK_EDIT_PLAY:
        mode = READY_TRACK_EDIT;
        StopAudioClip2();
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
          //Serial.println("R");
          file = SD.open("TRACK_1.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
          if (!file) {
            //Serial.println("file 1 open fail");
            SDerrorFlash(2); // file 1 open fail
          }
        } else {    // track edit mode
          digitalWrite(PROG_LED, LED_OFF);
          mode = READY_TRACK_EDIT;
          waitForProgUp = true;
          //Serial.println("TE");
          track = 1;
          trackFlashCount = track;
          playLedFlashTime = millis();
        }
        break;

      case RECORD:
        mode = REALTIME;
        file.println("EF");
        file.close();
        //Serial.println("ER");
        SaveServoRest();
        break;

      case PLAY:
        mode = PLAY;
        break;

      case READY_TRACK_EDIT:
        mode = START_TRACK_RECORD;
        //Serial.print("Trk ");
        //Serial.print(track);
        //Serial.println(" R");

        if (track == 1) {
          file = SD.open("TRACK_1.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
          if (!file) {
            //Serial.println("file 1 open fail");
            SDerrorFlash(2); //file 1 open fail
          }
          break;

        } else {
          file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
          if (!file) {
            //Serial.println("file 1 open fail");
            SDerrorFlash(2); // file 1 open fail
          }
          //Serial.println("FO");
          file.rewind();
        }

        file2 = SD.open("TRACK_C.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
        if (!file2) {
          //Serial.println("file 2 open fail");
          SDerrorFlash(3); // file 2 open fail
        }
        //Serial.println("F2O");
        break;

      case TRACK_1_RECORD:
        digitalWrite(PROG_LED, LED_OFF);
        mode = READY_TRACK_EDIT;
        file.println("EF");
        file.close();
        //Serial.println("ER");
        StopAudioClip2();
        SaveServoRest();
        break;

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
        if (track > LAST_TRACK) {
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
          //Serial.println("TP");
          file = SD.open("TRACK_1.TXT", O_READ ); // open file for reading
          file.rewind();
          StartAudioClip2();
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

void CopyArray(void) {
  servoData[0] = data.LsPot;
  servoData[1] = data.LjPotX;
  servoData[2] = data.LjPotY;
  servoData[3] = data.RjPotX;
  servoData[4] = data.RjPotY;
  servoData[5] = data.RsPot;
  servoData[6] = spArray[6][1]; // default servo 7 & 8 to endpoint 1 from config
  servoData[7] = spArray[7][1]; //

  // replace servo 8 with jaw data if in simple record
  // or track 1 record and audio and jaw are enabled
  if ((audioEnable == true)
      && (jawEnable == true)
      && (clip2Playing == true)
      && ((mode == RECORD) || (mode == TRACK_1_RECORD))) {
    servoData[7] = MSGEQ7bands[0];
  }
}

//--------------------------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------------------------

// simple record tracks 1 to 4 with controller data, tracks 5 & 6 dummy data
void WriteSDcard(void) {

  for (int i = 0; i <= 5; i++) {
    file.print(servoData[i]); file.print(",");
  }

  for (int i = 7; i >= 0; i--) {
    file.print(bitRead(data.Switches, i));
  }
  file.print(",");
  file.print(servoData[6]);
  file.print(",");
  file.print(servoData[7]);
  file.println(",11111111");
}


//--------------------------------------------------------------------------------------------------------------------
// Send servo data to servos
//--------------------------------------------------------------------------------------------------------------------

byte AudioClipPrevious;

void WriteOutputs(void) {

  // Write to servos with adjustments from config.txt
  for (byte i = 0; i <= 7; i++) {
    myServo[i].write(AdjustServoData(servoData[i], spArray[i][1], spArray[i][2], spArray[i][3], spArray[i][4]));
  }

  // Write digital data (buttons) to outputs, flip bit polarity per the output polarity mask from config.txt
  for (byte i = 8; i <= 15; i++) {
    IOX.digitalWrite(i, bitRead(digitalOutputPolarity, i - 8) ^ bitRead(data.Switches, 15 - i));
  }

  //------------------------------------------------------------------------------------------------------------
  // sound board - play clip

  if (audioEnable == true) {
    if ((mode == TRACK_EDIT_PLAY)
        || (mode == TRACK_RECORD)
        || (mode == PLAY)) {
      for (byte i = 1; i <= 8; i++) {
        if ((!bitRead(AudioClip, i - 1)) && (bitRead(AudioClipPrevious, i - 1))) { // play on 1 to 0 transition
          //Serial.print (mode);
          //Serial.print (" ");
          //Serial.print (AudioClip, 2);
          //Serial.print ("  clip");
          //Serial.println (i);

          if ((i == 1) && (playClipTwoAtStartEnable == false)) {
            StartAudioClip2();
          }
        }
      }
    }
    AudioClipPrevious = AudioClip;
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Adjust servo data for end points and deadzone using parameters from config file
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
const byte NRFfrequencyTable[12] = {100, 102, 104, 106, 108, 110, 112, 114, 116, 120, 124};

// NRF24L01 pipe address table
const byte pipes[][6] =  {"Pipe1"};

void SetUpRadio(void) {

  radio.begin();
  //Serial.println("NRF");
  radio.setChannel(NRFfrequencyTable[radioFrequency]);  // use the index to look up the frequency and set the NRF24l01
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);   // Set data rate to 250kbps
  radio.startListening();
  //const uint64_t pipe = 0x5069706531LL; // address = "Pipe1" to match controller (this desn't work - fix it later)
  radio.openReadingPipe(1, pipes[0]);
  //Serial.print ("F=");
  //Serial.println (NRFfrequencyTable[radioFrequency]);

  return;
}

//--------------------------------------------------------------------------------------------------------------------

int readPlayArray(void) {

  // Array for play data.
  byte j = 0;     // array index
  size_t n;       // Length of returned field with delimiter.
  char str[20];   // Must hold longest field with delimiter and zero byte.
  char *ptr;      // Test for valid field.
  byte error = 0;

  // Read the file and store the data.

  //for (i = 0; i < PLAY_ROW_DIM; i++) {
  for (j = 0; j < PLAY_COL_DIM; j++) {
    n = readField(&file, str, sizeof(str), ",\n");

    if (str[0] == 'E') {
      //Serial.println("EF");
      error = 1;
      return error;
    }

    if (n == 0) {
      SDerrorFlash(6); // too few lines
    }
    if ((j == 6) || (j == 9)) { // ascii binary fields
      plArray[j] = strtol(str, &ptr, 2);
    } else {
      plArray[j] = strtol(str, &ptr, 10);
    }
    if (ptr == str) {
      SDerrorFlash(7); // bad number
    }
    while (*ptr == ' ') {
      ptr++;
    }
    if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
      SDerrorFlash(8); // extra characters in field
    }
    if (j < (PLAY_COL_DIM - 1) && str[n - 1] != ',') {
      SDerrorFlash(9); // line with too few fields
    }
  }
  // Allow missing endl at eof.
  if (str[n - 1] != '\n' && file.available()) {
    SDerrorFlash(10); // no endl
  }
  //}

  return error;
}

//--------------------------------------------------------------------------------------------------------------------

int readServoParamArray(void) {

  // Array for servo parameter data.
  byte i = 0;     // First array index.
  byte j = 0;     // Second array index
  size_t n;      // Length of returned field with delimiter.
  char str[20];  // Must hold longest field with delimiter and zero byte.
  char *ptr;     // Test for valid field.
  byte error = 0;

  // Read the file and store the data.

  for (i = 0; i < SERVO_PARAM_ROW_DIM; i++) {
    for (j = 0; j < SERVO_PARAM_COL_DIM; j++) {
      n = readField(&file, str, sizeof(str), ",\n");

      if (str[0] == 'E') {
        //Serial.println("EF");
        error = 1;
        return error;
      }

      if (n == 0) {
        SDerrorFlash(6); // too few lines
      }

      spArray[i][j] = strtol(str, &ptr, 10);
      if (ptr == str) {
        SDerrorFlash(7); // bad number
      }
      while (*ptr == ' ') {
        ptr++;
      }
      if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
        SDerrorFlash(8); // extra characters in field
      }
      if (j < (SERVO_PARAM_COL_DIM - 1) && str[n - 1] != ',') {
        SDerrorFlash(9); // line with too few fields
      }
    }
    // Allow missing endl at eof.
    if (str[n - 1] != '\n' && file.available()) {
      SDerrorFlash(10); // no endl
    }
  }

  return error;
}

//--------------------------------------------------------------------------------------------------------------------
// read field

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
// Read config information from SD card

size_t n;      // Length of returned field with delimiter.
char str[20];  // Must hold longest field with delimiter and zero byte.
char *ptr;     // Test for valid field.

void ReadSdConfig(void) {

  // init the card
  if (!SD.begin(SD_CS, SPI_HALF_SPEED)) {
    //Serial.println("Init fail");
    SDerrorFlash(1); // SD init fail
  }

  // open the file for reading:
  file = SD.open("config.txt", O_READ );
  if (file) {
    //Serial.println("CFG:");

    // read from the file until there's nothing else in it:
    while (file.available()) {
      int readData = file.read();
      if (readData == '*') { // only parse lines that begin with '*'
        readData = file.read();

        switch (readData) {

          case 'S':
            switch (file.read()) {
              case 'P':
                readData = file.read();   // burn the 'cr' char
                readData = file.read();   // burn the 'lf' char
                readServoParamArray();    // read the servo parameter block
                //Serial.println("SP");
                //printArray();
                break;

              default:
                break;
            }

          case 'O':
            switch (file.read()) {
              case 'P':
                //readData = file.read();    // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                digitalOutputPolarity = (strtol(str, &ptr, 2));
                //Serial.print("PM=");
                //Serial.print(str);
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
                //Serial.print("TE=");
                //Serial.println(triggerEnable);
                break;

              case 'P':
                //readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                triggerPowerUpDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                //Serial.print("PUD=");
                //Serial.print(triggerPowerUpDelay);
                //Serial.println("mS");
                break;

              case 'D':
                //readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                triggerDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                //Serial.print("TD=");
                //Serial.print(triggerDelay);
                //Serial.println("mS");
                break;

              default:
                break;
            }
            break;

          case 'R':
            switch (file.read()) {

              case 'D':
                //readData = file.read();                       // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                replayDelay = (strtol(str, &ptr, 10)) * 1000; // read and convert char to seconds delay in milliseconds
                //Serial.print("RD=");
                //Serial.print(replayDelay);
                //Serial.println("mS");
                break;

              case 'E':
                readData = file.read();             // burn the 'space' char
                replayEnable = false;
                if (file.read() == 'E') {           //
                  replayEnable = true;
                }
                //Serial.print("RE=");
                //Serial.println(replayEnable);
                break;

              case 'F':
                //readData = file.read();             // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                radioFrequency = (strtol(str, &ptr, 10));
                //Serial.print("NRF=");
                //Serial.println(radioFrequency);
                break;

              case 'P':
                readData = file.read();             // burn the 'space' char
                recordProtect = false;
                if (file.read() == 'P') {           //
                  recordProtect = true;
                }
                //Serial.print("RP");
                //Serial.println(recordProtect);
                break;

              default:
                break;
            }
          case 'A':
            switch (file.read()) {
              case 'E':
                readData = file.read();           // burn the 'space' char
                audioEnable = false;
                if (file.read() == 'E') {         //
                  audioEnable = true;
                }
                //Serial.print("AE=");
                //Serial.println(audioEnable);
                break;

              case '1':
                readField(&file, str, sizeof(str), ",\n");
                audioVolume1 = (strtol(str, &ptr, 10));
                //Serial.print("V1=");
                //Serial.println(audioVolume1);
                break;

              case '2':
                readField(&file, str, sizeof(str), ",\n");
                audioVolume2 = (strtol(str, &ptr, 10));
                //Serial.print("V2=");
                //Serial.println(audioVolume2);
                break;

              case 'J':
                readData = file.read();           // burn the 'space' char
                jawEnable = false;
                if (file.read() == 'E') {         //
                  jawEnable = true;
                }
                //Serial.print("J=");
                //Serial.println(jawEnable);
                break;

              case 'A':
                readData = file.read();           // burn the 'space' char
                playAmbiantAtPowerUpEnable = false;
                if (file.read() == 'E') {         //
                  playAmbiantAtPowerUpEnable = true;
                }
                //Serial.print("PAPU=");
                //Serial.println(playAmbiantAtPowerUpEnable);
                break;

              case 'P':
                readData = file.read();           // burn the 'space' char
                playClipTwoAtStartEnable = false;
                if (file.read() == 'E') {         //
                  playClipTwoAtStartEnable = true;
                }
                //Serial.print("PC2AS=");
                //Serial.println(playClipTwoAtStartEnable);
                break;

              case 'F':
                //readData = file.read();    // burn the 'space' char
                readField(&file, str, sizeof(str), ",\n");
                jawFrequencyBands = (strtol(str, &ptr, 2));
                //Serial.print("AF=");
                //Serial.print(jawFrequencyBands);
                //Serial.print(" ");
                //Serial.print(str);
                break;

              case 'G':
                readData = file.read();           // burn the 'space' char
                JawGainPotEnable = false;
                if (file.read() == 'E') {         //
                  JawGainPotEnable = true;
                }
                //Serial.print("JGPE=");
                //Serial.println(JawGainPotEnable);
                break;

              default:
                break;
            }

          default:
            break;
        }
      }
    }

    file.close();  // close the file:

  } else {
    // if the file didn't open, print an error:
    Serial.println("EOC");
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Record Protect error flash

void RecordProtectError(void) {
  IOX.digitalWrite(PLAY_LED, LED_OFF);

  for (byte i = 1; i <= 4; i++) {
    IOX.digitalWrite(REC_LED, LED_ON);
    delay(200);
    IOX.digitalWrite(REC_LED, LED_OFF);
    delay(200);
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Card Not Present Error

void CardNotPresentError(void) {

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
// SD Card Error - flash error code
//
// 1 = SD Init fail
// 2 = File 1 open fail
// 3 = File 2 open fail
// 4 = File rename fail
// 5 = File delete fail
// 6 = Too few lines
// 7 = Bad number
// 8 = Extra characters in field
// 9 = Line with too few fields
// 10 = No endl

void SDerrorFlash(byte e) {
  Serial.print("SDE "); Serial.println(e);

  while (1) {
    for (byte i = 1; i <= e; i++) {
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(200);
      IOX.digitalWrite(REC_LED, LED_ON);
      delay(200);
    }
    delay(1000);
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Blink Code Version

void BlinkCodeVersion(void) {

  if (BETA == 1) {
    digitalWrite(PROG_LED, LED_ON);
  }
  while (1) {
    for (byte i = 1; i <= VERSION; i++) {
      IOX.digitalWrite(PLAY_LED, LED_ON);
      delay(200);
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      delay(200);
    }

    for (byte i = 1; i <= REVSION; i++) {
      IOX.digitalWrite(REC_LED, LED_ON);
      delay(200);
      IOX.digitalWrite(REC_LED, LED_OFF);
      delay(200);
    }
  }
  digitalWrite(PROG_LED, LED_OFF);

}

//--------------------------------------------------------------------------------------------------------------------
// Setup Audio board if enabled in the config file (*AE E)

void SetUpAudioBoard(void) {

  //Serial.print("A:");
  if (audioEnable == false) {
    //Serial.println("no");
    return;
  }

  delay(2000);

  if (JawGainPotEnable == true) {
    analogReference(EXTERNAL);
  } else {
    analogReference(DEFAULT);
  }

  IOX.pinMode(MSGEQ7_RESET, OUTPUT);    IOX.digitalWrite(MSGEQ7_RESET, HIGH);
  pinMode(MSGEQ7_STROBE, OUTPUT);       digitalWrite(MSGEQ7_STROBE, HIGH);

  pinMode(MP3_PLAYER_nBUSY, INPUT);

  Serial1.begin(9600);
  myMP3.begin(Serial1);

  StopAudioClip2(); // stop any clips and start ambiant
}

//--------------------------------------------------------------------------------------------------------------------
// Read MSGEQ7
//
//   Bands
// 1 = 63Hz
// 2 = 160Hz
// 3 = 400Hz
// 4 = 1KHz
// 5 = 2.5KHz
// 6 = 6.25KHz
// 7 = 16KHz

void ReadMSGEQ7(void) {

  IOX.digitalWrite(MSGEQ7_RESET, HIGH);
  IOX.digitalWrite(MSGEQ7_RESET, LOW);
  delayMicroseconds(75);

  for (byte i = 1; i <= 7; i++) {
    digitalWrite(MSGEQ7_STROBE, LOW);
    delayMicroseconds(40);
    MSGEQ7bands[i] = analogRead(MSGEQ7_AUDIO_OUT);
    digitalWrite(MSGEQ7_STROBE, HIGH);
    delayMicroseconds(40);
  }

  MSGEQ7bands[0] = 0;
  for (byte i = 1; i <= 7; i++) {
    if (bitRead(jawFrequencyBands, i) && (MSGEQ7bands[i] > MSGEQ7bands[0])) {
      MSGEQ7bands[0] = MSGEQ7bands[i];
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Start audio clip two if playClipTwoAtStart is enabled in the config file (*AP E)

void StartAudioClip2(void) {

  if (audioEnable) {
    if (playAmbiantAtPowerUpEnable) {
      //Serial.println("S1");
      myMP3.stop();

      while (!digitalRead(MP3_PLAYER_nBUSY)) { // wait for nBUSY pin to signal play had stopped
      }
      clip1Playing = false;
    }

    if (playClipTwoAtStartEnable) {
      //Serial.print("P2");
      delay(100);
      myMP3.volume(audioVolume2);
      delay(500);
      myMP3.play(2);

      while (digitalRead(MP3_PLAYER_nBUSY)) { // wait for nBUSY pin to signal play had started
      }
      clip2Playing = true;
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------
// Stop audio clip two and restart ambiant clip if enabled in the config file (*AA E)

void StopAudioClip2(void) {

  if (audioEnable) {
    //Serial.println("S2");
    myMP3.stop();

    while (!digitalRead(MP3_PLAYER_nBUSY)) { // wait for nBUSY pin to signal play had stopped
    }
    clip2Playing = false;

    if (playAmbiantAtPowerUpEnable) {
      //Serial.print("P1");
      delay(100);
      myMP3.volume(audioVolume1);
      delay(500);
      myMP3.loop(1);

      while (digitalRead(MP3_PLAYER_nBUSY)) { // wait for nBUSY pin to signal play had started
      }
      clip1Playing = true;
    }
  }
}


//--------------------------------------------------------------------------------------------------------------------
// Save servo rest positions in EEprom

void SaveServoRest(void) {

  for (byte i = 0; i <= 7; i++) {
    EEPROM.update(i, myServo[i].read());
  }
  EEPROM.update(1023, 0xAA); // flag EEPROM as initialized
}

//**********************************************************************************************************************
// end of file
//**********************************************************************************************************************
