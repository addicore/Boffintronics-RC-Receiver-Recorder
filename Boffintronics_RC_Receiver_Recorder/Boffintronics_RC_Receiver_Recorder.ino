//**********************************************************************************************************************
//
//                                    Steve Koci's DIY Remote Receiver Recorder
//                                        by Addicore.com & Boffintronics
//
//**********************************************************************************************************************
// Revision History:
//
//       06-16-2020   ARM     Version 1.0   Initial prduction Release
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

//**********************************************************************************************************************
// SDcard

const int SD_CS = 12; // SD card chip select pin

SdFat SD;
File file;


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

const byte OUT1 = 8;       // Outer left and shoulder button (Controller S4 & S8)
const byte OUT2 = 9;       // Left Toggle Switch (Controller S2)
const byte OUT3 = 10;      // Left Joystick button (Controller A1)
const byte OUT4 = 11;      // Center left button (Controller S5)
const byte OUT5 = 12;      // Center right button (Controller S6)
const byte OUT6 = 13;      // Right Joystick button (Controller A2)
const byte OUT7 = 14;      // Right Toggle Switch (Controller S3)
const byte OUT8 = 15;      // Outer right and shoulder button (Controller S7 & S9)

// Inputs
const byte SC_CP = 2;      // SD card present (low true)
const byte REC_BTN = 5;    // Record button (low true)
const byte PLAY_BTN = 6;   // Play button (low true)
const byte PROG_BTN = 7;   // Program mode button (low true)

MCP23S17 IOX(&SPI, IOX_CS, 0);

//**********************************************************************************************************************
// Misc Processor I/O

const int PROG_LED = 13;  // Program mode status LED
const int TRIG_HIGH = 7;  // external high trigger
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
  IOX.pinMode(0, INPUT_PULLUP);     // EXA0, high to low play trigger
  IOX.pinMode(1, INPUT);            // EXA1, low to high play trigger
  IOX.pinMode(2, INPUT_PULLUP);     // SD_CP, SD card present (low true)
  IOX.pinMode(3, OUTPUT);
  IOX.pinMode(4, OUTPUT);
  IOX.pinMode(5, INPUT_PULLUP);
  IOX.pinMode(6, INPUT_PULLUP);
  IOX.pinMode(7, INPUT_PULLUP);

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
  
  byte mask = EEPROM.read(1); // get init mask from EPROM and init bank B
  IOX.digitalWrite(OUT1, !bitRead(mask, 7));
  IOX.digitalWrite(OUT2, !bitRead(mask, 6));
  IOX.digitalWrite(OUT3, !bitRead(mask, 5));
  IOX.digitalWrite(OUT4, !bitRead(mask, 4));
  IOX.digitalWrite(OUT5, !bitRead(mask, 3));
  IOX.digitalWrite(OUT6, !bitRead(mask, 2));
  IOX.digitalWrite(OUT7, !bitRead(mask, 1));
  IOX.digitalWrite(OUT8, !bitRead(mask, 0));


  //**********************************************************************************************************************
  //Serial.begin(19200);  // Initialize serial communications with PC  // un comment for serial debug, must have IDE serial monitor
  //while (!Serial);      // wait for serial                           //  connected or code will hang
  Serial.println("------------------------------------");
  Serial.println("Steve Koci's DIY Remote Controller");
  Serial.println("         by Addicore.com");
  Serial.println("Receiver Recorder Beta Version 2_P_G");
  Serial.println("------------------------------------");

  if (IOX.digitalRead(SC_CP)) { 
    CardNotPresentError();
  }

  ReadSdConfig(); // load config file from sd card

  SetUpRadio();  // Setup Radio for receive

}

//**********************************************************************************************************************
// end of setup
//**********************************************************************************************************************


//**********************************************************************************************************************
// main loop
//**********************************************************************************************************************


const byte REALTIME = 1;
const byte PLAY = 2;
const byte REPLAY_DELAY = 3;
const byte TRIGGER_PLAY = 4;
const byte TRIGGER_LOCKOUT = 5;
const byte START_RECORD = 6;
const byte RECORD = 7;

const int PLAY_LOCKOUT_FLASH = 200;

const int PLAY_ROW_DIM = 1; // 1 X 7 play array to read from card
const int PLAY_COL_DIM = 7; //
int plArray[PLAY_ROW_DIM][PLAY_COL_DIM];

const int SERVO_PARAM_ROW_DIM = 6; // 6 X 5 servo parameter array to read from card
const int SERVO_PARAM_COL_DIM = 5; //
int spArray[SERVO_PARAM_ROW_DIM][SERVO_PARAM_COL_DIM];


byte mode = REALTIME;
unsigned long loopTime = millis();
unsigned long playLedFlashTime = millis();
bool recordProtect;
byte radioFrequency;
byte digitalOutputPolarity;
bool triggerEnable;
unsigned long triggerPowerUpDelayTime = millis();
unsigned long triggerPowerUpDelay;
unsigned long triggerLockOutTime;
unsigned long triggerDelay;
bool replayEnable;
unsigned long replayDelayTime;
unsigned long replayDelay;

//**********************************************************************************************************************
void loop() {

  CheckButtons();  // check play and record buttons

  switch (mode) {
    //---------------------------------------------------------------------------------------------
    case REALTIME:
      IOX.digitalWrite(PLAY_LED, LED_OFF);
      IOX.digitalWrite(REC_LED, LED_OFF);

      if (radio.available()) {
        radio.read(&data, sizeof(Data_Package)); // get new data package from radio
        WriteOutputs();
      }
      break;
    //---------------------------------------------------------------------------------------------
    case PLAY:
      IOX.digitalWrite(PLAY_LED, LED_ON);
      IOX.digitalWrite(REC_LED, LED_OFF);
      while (millis() < (loopTime + 33)) {
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
      if (millis() > (playLedFlashTime + PLAY_LOCKOUT_FLASH)) {
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
      while (millis() < (loopTime + 33)) {  // sync to 10 ms
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
      if (millis() > (playLedFlashTime + PLAY_LOCKOUT_FLASH)) {
        IOX.digitalWrite (PLAY_LED, !(IOX.digitalRead(PLAY_LED)));
        playLedFlashTime = millis();
      }

      if (millis() > (triggerLockOutTime + triggerDelay)) {
        mode = REALTIME;
        Serial.println("End of Lockout");
      }
      break;
    //---------------------------------------------------------------------------------------------
    case START_RECORD: // flash record led, recording starts on the third flash
      if (recordProtect) {
        RecordProtectError();
        mode = REALTIME;
        break;
      }

      InitSDcardWrite();
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
    default:
      break;
  }
}

//**********************************************************************************************************************
// end of main loop
//**********************************************************************************************************************

//--------------------------------------------------------------------------------------------------------------------
// Check play, record buttons and play trigger pins
//--------------------------------------------------------------------------------------------------------------------

int playButtonNow;
int playButtonPrev = 0;
unsigned long playButtonTime = 0;
int recButtonNow;
int recButtonPrev = 0;
unsigned long recButtonTime = 0;
unsigned long RP_DEBOUNCE = 200;   // button debounce time (ms)

void CheckButtons(void) {

  //--------------------------------------------------------------------------------------------------------------------
  // Check Play Button

  playButtonNow = IOX.digitalRead(PLAY_BTN);
  if (playButtonNow == 0 && playButtonPrev == 1 && (millis() - playButtonTime) > RP_DEBOUNCE) {
    switch (mode) {

      case REALTIME:
        mode = PLAY;
        Serial.println("Play");
        InitSDcard();
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
        mode = START_RECORD;
        Serial.println("Record");
        break;

      case RECORD:
        mode = REALTIME;
        file.println("EOF");
        file.println(F("******************************"));
        file.close();
        Serial.println("End Record");
        break;

      case PLAY:
        mode = PLAY;
        break;

      default:
        break;
    }
    recButtonTime = millis();
  }
  recButtonPrev = recButtonNow;

  //--------------------------------------------------------------------------------------------------------------------
  // Check Trigger inputs
  if (millis() > (triggerPowerUpDelayTime + triggerPowerUpDelay)) {        // don't check trigger inputs while in trigger power up delay
    if ((IOX.digitalRead(EXA0) == 0) || (digitalRead(TRIG_HIGH) == 1)) {   // check if either are active

      switch (mode) {

        case REALTIME:
          mode = TRIGGER_PLAY;
          Serial.println("Trigger Play");
          InitSDcard();
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

  Serial.println(F("Init SD card"));

  // Initialize the SD.
  if (!SD.begin(SD_CS, SPI_HALF_SPEED)) {
    //errorHalt("begin failed");
    Serial.println("begin failed");
  }

  // Create or open the file.
  file = SD.open("TRACK_1.TXT", FILE_WRITE);
  if (!file) {
    //errorHalt("open failed");
    Serial.println("open failed");
  }
  // Rewind file to the beginning
  file.rewind();

  Serial.println(F("Card init done"));
}

//--------------------------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------------------------

void InitSDcardWrite(void) {

  Serial.println(F("Init SD card for write"));

  // Initialize the SD.
  if (!SD.begin(SD_CS, SPI_HALF_SPEED)) {
    //errorHalt("begin failed");
    Serial.println("begin failed");
  }

  // Create or open the file.
  file = SD.open("TRACK_1.TXT", O_WRITE | O_CREAT | O_TRUNC ); // open/create file for write, clear file if it exists
  if (!file) {
    //errorHalt("open failed");
    Serial.println("open failed");
  }
  Serial.println(F("Card init done"));
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


void WriteOutputs(void) {

  // Write to servos with adjustments (see above)
  myServo1.write(AdjustServoData(data.LsPot, spArray[0][1], spArray[0][2], spArray[0][3], spArray[0][4])); // left slide pot
  myServo2.write(AdjustServoData(data.LjPotX, spArray[1][1], spArray[1][2], spArray[1][3], spArray[1][4])); // left joystick x pot
  myServo3.write(AdjustServoData(data.LjPotY, spArray[2][1], spArray[2][2], spArray[2][3], spArray[2][4])); // left joystick y pot
  myServo4.write(AdjustServoData(data.RjPotX, spArray[3][1], spArray[3][2], spArray[3][3], spArray[3][4])); // right joystick x pot
  myServo5.write(AdjustServoData(data.RjPotY, spArray[4][1], spArray[4][2], spArray[4][3], spArray[4][4])); // right joystick y pot
  myServo6.write(AdjustServoData(data.RsPot, spArray[5][1], spArray[5][2], spArray[5][3], spArray[5][4])); // right slide pot

  // Write digital data (buttons) to outputs
  data.Switches = data.Switches ^ digitalOutputPolarity; // flip bit polarity per the output polarity mask from config.txt
  IOX.digitalWrite(OUT1, bitRead(data.Switches, 7));
  IOX.digitalWrite(OUT2, bitRead(data.Switches, 6));
  IOX.digitalWrite(OUT3, bitRead(data.Switches, 5));
  IOX.digitalWrite(OUT4, bitRead(data.Switches, 4));
  IOX.digitalWrite(OUT5, bitRead(data.Switches, 3));
  IOX.digitalWrite(OUT6, bitRead(data.Switches, 2));
  IOX.digitalWrite(OUT7, bitRead(data.Switches, 1));
  IOX.digitalWrite(OUT8, bitRead(data.Switches, 0));

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
  int NRFfrequencyIndex = 0;
  int NRFpipeIndex = 1;

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


size_t n;      // Length of returned field with delimiter.
char str[20];  // Must hold longest field with delimiter and zero byte.
char *ptr;     // Test for valid field.

void ReadSdConfig(void) {

  Serial.println("Init SD Card");

  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("Card init done");

  // open the file for reading:
  file = SD.open("config.txt");
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

//**********************************************************************************************************************
// end of file
//**********************************************************************************************************************
