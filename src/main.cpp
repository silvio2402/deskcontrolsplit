#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// #include "app_api.h"  // only needed with flash breakpoints
// #include "avr8-stub.h"

HardwareSerial *interfaceSerial = &Serial1;
HardwareSerial *controllerSerial1 = &Serial2;
HardwareSerial *controllerSerial2 = &Serial3;

unsigned long int startTime;
unsigned long elapsedTime;
unsigned long elapsedControllerSendTime;

const unsigned long microSecond = 1;
const unsigned long milliSecond = 1000 * microSecond;
const unsigned long second = 1000 * milliSecond;
const unsigned long minute = 60 * second;

const double defaultUserSetSpeed = 0.0001;
const double snapUserSetSpeedThr = 0.0004;
const double maxUserSetSpeed = 0.001;
const double userSetSpeedMultiplier = 0.001;
const unsigned long shortTapThr = 0.5 * second;
const unsigned long longTapThr = 1.5 * second;
const unsigned long blinkTime = 0.7 * second;
const unsigned long hideDelay = 3 * second;
const unsigned long displayOffTime = 1 * minute;
const unsigned long controllerSendTime = 10 * milliSecond;
const unsigned long intResponseMaxLoops = 10;
const unsigned long ctrlResponseMaxLoops = 10;
const double minHeight = 620.0;
const double maxHeight = 1280;
const double allowedHeightDiff = 5;
const double errorHeightDiff = 20;

double preset1Height = minHeight;
double preset2Height = (minHeight + maxHeight) / 2;
double preset3Height = maxHeight;
bool presetsSet = false;

bool memoryMode = false;
uint8_t memoryModeSel = 0;

bool resetMode = true;
uint8_t resetting = 0;

bool errorMode = false;

double userSetHeight = 620;
double userSetSpeed = 0.0001;

double controller1Height = 620;
double controller2Height = 620;

unsigned long prevStateTime;
unsigned long sameStateFor;
unsigned long prevSameStateFor;
uint8_t prevState;

const uint8_t SEG7_MAP[36] = {B00111111, B00000110, B01011011, B01001111, B01100110, B01101101, B01111101, B00000111,
                              B01111111, B01101111, B01011111, B01111100, B01011000, B01011110, B01111001, B01110001,
                              B00111101, B01110100, B00010001, B00001101, B01110101, B00111001, B01010101, B01010100,
                              B01011100, B01110011, B01100111, B01010000, B00101101, B01111000, B00011100, B00101010,
                              B01101010, B00010100, B01101110, B00011011};

const uint8_t rstPattern[3] = {B01010000, B01101101, B00110001};
const uint8_t hotPattern[3] = {B01110110, B01011100, B01111000};

double bound(double value, double min, double max) {
  if (value >= min && value <= max)
    return value;
  else if (value < min)
    return min;
  else if (value > max)
    return max;
  return min;
}

void initSerial(unsigned long newControllerBaud) {
  // controllerSerial1->end();
  // controllerSerial2->end();
  // interfaceSerial->end();

  controllerSerial1->begin(newControllerBaud);
  controllerSerial2->begin(newControllerBaud);
  interfaceSerial->begin(newControllerBaud);
}

uint8_t calcChecksum(uint8_t *cmdBytes, unsigned int cmdLen) {
  unsigned int sum = 0;
  for (unsigned int i = 1; i < cmdLen - 1; i++) sum += cmdBytes[i];
  return (sum % (0xFF + 1));
}

void sendToDisplay(uint8_t digit0, uint8_t digit1, uint8_t digit2, bool buttonLights = true, bool timerLight = false) {
  // uint8_t cmdBytes[6] = {0x5A, digit0, digit1, digit2, 0x10, 0x00};

  uint8_t cmdBytes[6] = {0};
  cmdBytes[0] = 0x5A;
  cmdBytes[1] = digit0;
  cmdBytes[2] = digit1;
  cmdBytes[3] = digit2;
  cmdBytes[4] = B00000000 + buttonLights ? B00010000 : 0 + timerLight ? B00000001 : 0;
  cmdBytes[5] = calcChecksum(cmdBytes, 6);

  interfaceSerial->write(cmdBytes, 6);
}

void sendToController(HardwareSerial *controllerSerial, uint8_t state) {
  uint8_t cmdBytes[5] = {0};
  cmdBytes[0] = 0xA5;
  cmdBytes[1] = 0x00;
  cmdBytes[2] = state;
  cmdBytes[3] = 0x01;
  cmdBytes[4] = calcChecksum(cmdBytes, 5);
  controllerSerial->write(cmdBytes, 5);
}

void updatePresets() {
  EEPROM.put(sizeof(double) * 0, preset1Height);
  EEPROM.put(sizeof(double) * 1, preset2Height);
  EEPROM.put(sizeof(double) * 2, preset3Height);
}

void getPresets() {
  EEPROM.get(sizeof(double) * 0, preset1Height);
  EEPROM.get(sizeof(double) * 1, preset2Height);
  EEPROM.get(sizeof(double) * 2, preset3Height);
}

void clearSerialBuffer(HardwareSerial *ser) {
  int intSerAvail = ser->available();
  uint8_t buf[intSerAvail];
  ser->readBytes(buf, intSerAvail);
}

void sendControllerData(HardwareSerial *controllerSerial, double currHeight) {
  uint8_t state = 0;
  // state += B00000001;  // M button
  // state += B00000010;  // 1 button
  // state += B00000100;  // 2 button
  // state += B00001000;  // 3 button
  // state += B00010000;  // T button
  if (currHeight < userSetHeight - allowedHeightDiff || resetting > 0) {
    state += B00100000;  // Up button
  }
  if (currHeight > userSetHeight + allowedHeightDiff || resetting > 0) {
    state += B01000000;  // Down button
  }
  sendToController(controllerSerial, state);
}

uint8_t extractDigit(uint8_t digit, bool alphaNumeric = false) {
  uint8_t max = alphaNumeric ? 36 : 10;
  for (int i = 0; i < max; i++)
    if ((digit & B01111111) == SEG7_MAP[i]) return i;

  return 0xFF;
}

void checkControllerData(HardwareSerial *controllerSerial, double &heightVar) {
  unsigned long loopCounter = 0;

  while (controllerSerial->available() > 0 && loopCounter < ctrlResponseMaxLoops) {
    loopCounter++;
    uint8_t byte = controllerSerial->peek();
    if (byte != 0x5A) {
      controllerSerial->read();
      continue;
    }

    uint8_t cmdBytes[6];
    controllerSerial->readBytes(cmdBytes, 6);
    if (cmdBytes[5] == calcChecksum(cmdBytes, 6)) {
      uint8_t digit0 = cmdBytes[1];
      uint8_t digit1 = cmdBytes[2];
      uint8_t digit2 = cmdBytes[3];

      if ((cmdBytes[4] & B00010000) == 0) {
        // Display off
        return;
      }

      if (cmdBytes[1] == rstPattern[0] && cmdBytes[2] == rstPattern[1] && cmdBytes[3] == rstPattern[2]) {
        errorMode = true;
        return;
      }

      if (cmdBytes[1] == rstPattern[0] && cmdBytes[2] == rstPattern[1] && cmdBytes[3] == rstPattern[2]) {
        resetMode = true;
      } else if (resetting > 0) {
        if (resetting == 1)
          resetMode = false;
        else if (resetting == 2)
          resetting--;
      }

      uint8_t digit0Value = extractDigit(digit0, true);
      uint8_t digit1Value = extractDigit(digit1);
      uint8_t digit2Value = extractDigit(digit2);

      if (digit0Value >= 10) {
        Serial.print("E:");
        Serial.print(digit0, BIN);
        Serial.print(" ");
        Serial.print(digit1, BIN);
        Serial.print(" ");
        Serial.print(digit2, BIN);
        Serial.print(" ");
        Serial.print(cmdBytes[4], BIN);
        Serial.print(" E:");
        Serial.write(digit0Value - 10 + 'A');
        Serial.print(digit1Value);
        Serial.print(digit2Value);
        Serial.println();
        return;
      }

      if (digit0Value == 0xFF) {
        // errorMode = true;
        // return;
      }
      if (digit1Value == 0xFF) {
        // errorMode = true;
        // return;
      }
      if (digit2Value == 0xFF) {
        // errorMode = true;
        // return;
      }

      double height;
      // Check if dot is set
      if ((digit1 & B10000000) > 0) {
        height = digit0Value * 100 + digit1Value * 10 + digit2Value;
      } else {
        height = digit0Value * 1000 + digit1Value * 100 + digit2Value * 10;
      }

      Serial.print("H:");
      Serial.println(height);

      heightVar = height;
    }
  }
  if (loopCounter >= intResponseMaxLoops) {
    errorMode = true;
  }
}

void checkInterfaceData() {
  unsigned long loopCounter = 0;

  while (interfaceSerial->available() > 0 && loopCounter < intResponseMaxLoops) {
    loopCounter++;
    uint8_t byte = interfaceSerial->peek();
    if (byte != 0xA5) {
      interfaceSerial->read();
      continue;
    }

    uint8_t cmdBytes[5];
    interfaceSerial->readBytes(cmdBytes, 5);
    if (cmdBytes[4] == calcChecksum(cmdBytes, 5)) {
      bool prevOnlyPressedM = ((prevState & B00000001) > 0 && (prevState & B11111110) == 0);
      bool prevOnlyPressed1 = ((prevState & B00000010) > 0 && (prevState & B11111101) == 0);
      bool prevOnlyPressed2 = ((prevState & B00000100) > 0 && (prevState & B11111011) == 0);
      bool prevOnlyPressed3 = ((prevState & B00001000) > 0 && (prevState & B11110111) == 0);
      // bool prevOnlyPressedT = ((prevState & B00010000) > 0 && (prevState & B11101111) == 0);
      bool prevOnlyPressedUP = ((prevState & B00100000) > 0 && (prevState & B11011111) == 0);
      bool prevOnlyPressedDW = ((prevState & B01000000) > 0 && (prevState & B10111111) == 0);
      bool prevOnlyUPandDW = ((prevState & B01100000) == B01100000 && (prevState & B10011111) == 0);

      uint8_t state = cmdBytes[2];
      bool onlyPressedM = ((state & B00000001) > 0 && (state & B11111110) == 0);
      bool onlyPressed1 = ((state & B00000010) > 0 && (state & B11111101) == 0);
      bool onlyPressed2 = ((state & B00000100) > 0 && (state & B11111011) == 0);
      bool onlyPressed3 = ((state & B00001000) > 0 && (state & B11110111) == 0);
      // bool onlyPressedT = ((state & B00010000) > 0 && (state & B11101111) == 0);
      bool onlyPressedUP = ((state & B00100000) > 0 && (state & B11011111) == 0);
      bool onlyPressedDW = ((state & B01000000) > 0 && (state & B10111111) == 0);
      bool onlyUPandDW = ((state & B01100000) == B01100000 && (state & B10011111) == 0);

      if (prevState != state) {
        prevSameStateFor = sameStateFor;
        prevStateTime = startTime;
      }

      sameStateFor = startTime - prevStateTime;

      uint8_t digit0;
      uint8_t digit1;
      uint8_t digit2;

      bool overwriteDigits = false;

      if (memoryMode) {
        overwriteDigits = true;
        digit0 = memoryModeSel > 0 ? sameStateFor % blinkTime < blinkTime / 2 ? SEG7_MAP[10 + 'S' - 'A'] : B00000000
                                   : SEG7_MAP[10 + 'S' - 'A'];
        digit1 = sameStateFor % blinkTime < blinkTime / 2 ? B01000000 : B00000000;
        digit2 = memoryModeSel > 0 ? sameStateFor % blinkTime < blinkTime / 2 ? SEG7_MAP[memoryModeSel] : B00000000
                                   : B00000000;
        if (memoryModeSel > 0 && sameStateFor >= hideDelay) {
          updatePresets();
          memoryMode = false;
          memoryModeSel = 0;
        }
      }

      if (onlyUPandDW) {
        if (prevOnlyUPandDW) {
          // Update
          if (sameStateFor >= longTapThr) {
            resetting = 2;
            resetMode = true;
          }
        }
      } else if (prevOnlyUPandDW) {
        // Released
        resetting = 0;
        resetMode = false;
      }

      if (!resetMode) {
        if (onlyPressedUP) {
          if (prevOnlyPressedUP) {
            // Update
            if (sameStateFor >= shortTapThr) {
              userSetSpeed = bound(userSetSpeed * userSetSpeedMultiplier * (double)elapsedTime, defaultUserSetSpeed,
                                   maxUserSetSpeed);
              userSetHeight = bound(userSetHeight + userSetSpeed * (double)elapsedTime, minHeight, maxHeight);
            }
          } else {
            // Began
            userSetHeight = bound(userSetHeight + 1, minHeight, maxHeight);
          }
        } else if (prevOnlyPressedUP) {
          // Released
          if (userSetSpeed >= snapUserSetSpeedThr) {
            // Snap to 10mm
            userSetHeight = bound(ceil(userSetHeight / 10) * 10, minHeight, maxHeight);
          }
          userSetSpeed = defaultUserSetSpeed;
        }

        if (onlyPressedDW) {
          if (prevOnlyPressedDW) {
            // Update
            if (sameStateFor >= shortTapThr) {
              userSetSpeed = bound(userSetSpeed * userSetSpeedMultiplier * (double)elapsedTime, defaultUserSetSpeed,
                                   maxUserSetSpeed);
              userSetHeight = bound(userSetHeight - userSetSpeed * (double)elapsedTime, minHeight, maxHeight);
            }
          } else {
            // Began
            userSetHeight = bound(userSetHeight - 1, minHeight, maxHeight);
          }
        } else if (prevOnlyPressedDW) {
          // Released
          if (userSetSpeed >= snapUserSetSpeedThr) {
            // Snap to 10mm
            userSetHeight = floor(userSetHeight / 10) * 10;
          }
          userSetSpeed = defaultUserSetSpeed;
        }

        if (onlyPressedM) {
          if (prevOnlyPressedM) {
            // Update
            if (sameStateFor >= shortTapThr) {
              memoryMode = true;
            }
          }
        } else if (prevOnlyPressedM) {
          // Released
          if (memoryMode && prevSameStateFor < shortTapThr) memoryMode = false;
        }

        if (onlyPressed1) {
          if (prevOnlyPressed1) {
            // Update
            if (memoryMode) {
              memoryModeSel = 1;
              digit2 = SEG7_MAP[1];
              preset1Height = userSetHeight;
            }
          }
        } else if (prevOnlyPressed1) {
          // Released
          if (memoryMode) {
            memoryModeSel = 1;
          } else {
            userSetHeight = preset1Height;
            // bound(preset1Height, minHeight, maxHeight);
          }
        }

        if (onlyPressed2) {
          if (prevOnlyPressed2) {
            // Update
            if (memoryMode) {
              memoryModeSel = 2;
              digit2 = SEG7_MAP[2];
              preset2Height = userSetHeight;
            }
          }
        } else if (prevOnlyPressed2) {
          // Released
          if (memoryMode) {
            memoryModeSel = 2;
          } else {
            userSetHeight = preset2Height;
            // bound(preset2Height, minHeight, maxHeight);
          }
        }

        if (onlyPressed3) {
          if (prevOnlyPressed3) {
            // Update
            if (memoryMode) {
              memoryModeSel = 3;
              digit2 = SEG7_MAP[3];
              preset3Height = userSetHeight;
            }
          }
        } else if (prevOnlyPressed3) {
          // Released
          if (memoryMode) {
            memoryModeSel = 3;
          } else {
            userSetHeight = preset3Height;
            // bound(preset3Height, minHeight, maxHeight);
          }
        }

        if (!overwriteDigits) {
          if (userSetHeight >= 1000) {
            digit0 = SEG7_MAP[(int)floor(userSetHeight / 1000) % 10];
            digit1 = SEG7_MAP[(int)floor(userSetHeight / 100) % 10];
            digit2 = SEG7_MAP[(int)floor(userSetHeight / 10) % 10];
          } else {
            digit0 = SEG7_MAP[(int)floor(userSetHeight / 100) % 10];
            digit1 = SEG7_MAP[(int)floor(userSetHeight / 10) % 10];
            digit1 = (digit1 & B01111111) + B10000000;
            digit2 = SEG7_MAP[(int)userSetHeight % 10];
          }
        }
      } else {
        digit0 = SEG7_MAP[10 + 'r' - 'a'];
        digit1 = SEG7_MAP[10 + 's' - 'a'];
        digit2 = SEG7_MAP[10 + 't' - 'a'];
      }

      if (sameStateFor < displayOffTime) {
        sendToDisplay(digit0, digit1, digit2, true, errorMode);
      } else {
        sendToDisplay(0, 0, 0, false, errorMode);
      }

      prevState = state;
      return;
    }
  }
  if (loopCounter >= intResponseMaxLoops) {
    errorMode = true;
  }
}

void setup() {
  // initialize GDB stub
  // debug_init();

  Serial.begin(115200);
  Serial.println("Start");

  getPresets();

  initSerial(9600);

  sendToController(controllerSerial1, 0);
  sendToController(controllerSerial2, 0);

  checkControllerData(controllerSerial1, controller1Height);
  checkControllerData(controllerSerial2, controller2Height);
}

void loop() {
  elapsedTime = micros() - startTime;
  startTime = micros();
  elapsedControllerSendTime += elapsedTime;

  int intSerAvail = interfaceSerial->available();
  if (intSerAvail > 15)
    clearSerialBuffer(interfaceSerial);
  else if (intSerAvail > 0)
    checkInterfaceData();

  int ctrl1SerAvail = controllerSerial1->available();
  if (ctrl1SerAvail > 0) checkControllerData(controllerSerial1, controller1Height);

  int ctrl2SerAvail = controllerSerial2->available();
  if (ctrl2SerAvail > 0) checkControllerData(controllerSerial2, controller2Height);

  if (errorMode) {
    // Serial.print("E");
    return;
  }

  if (elapsedControllerSendTime >= controllerSendTime) {
    sendControllerData(controllerSerial1, controller1Height);
    sendControllerData(controllerSerial2, controller2Height);
    elapsedControllerSendTime = 0;
    Serial.println("Sent");
  }

  delay(1);
}