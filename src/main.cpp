#include <Arduino.h>
#include <math.h>

#include "app_api.h"  // only needed with flash breakpoints
#include "avr8-stub.h"

HardwareSerial *interfaceSerial = &Serial1;
HardwareSerial *controllerSerial1 = &Serial2;
HardwareSerial *controllerSerial2 = &Serial3;

unsigned long int startTime;
unsigned long elapsedTime;

const double defaultUserSetSpeed = 0.0001;
const double snapUserSetSpeedThr = 0.0004;
const double maxUserSetSpeed = 0.001;
const double userSetSpeedMultiplier = 0.001;
const unsigned long buttonHoldThr = 500000;
const unsigned int minHeight = 620;
const int maxHeight = 1280;

double userSetHeight = 620;
double userSetSpeed = 0.0001;

unsigned long prevStateTime;
uint8_t prevState;

uint8_t SEG7_MAP[36] = {B00111111, B00000110, B01011011, B01001111, B01100110, B01101101, B01111101, B00000111,
                        B01111111, B01101111, B01011111, B01111100, B01011000, B01011110, B01011110, B01110001,
                        B00111101, B01110100, B00010001, B00001101, B01110101, B00111001, B01010101, B01010100,
                        B01011100, B01110011, B01100111, B01010000, B00101101, B01111000, B00011100, B00101010,
                        B01101010, B00010100, B01101110, B00011011};

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

  // controllerSerial1->begin(newControllerBaud);
  // controllerSerial2->begin(newControllerBaud);
  interfaceSerial->begin(newControllerBaud);
}

uint8_t calcChecksum(uint8_t *cmdBytes, unsigned int cmdLen) {
  unsigned int sum = 0;
  for (unsigned int i = 1; i < cmdLen - 1; i++) sum += cmdBytes[i];
  return (sum % (0xFF + 1));
}

void sendToDisplay(uint8_t digit0, uint8_t digit1, uint8_t digit2) {
  // uint8_t cmdBytes[6] = {0x5A, digit0, digit1, digit2, 0x10, 0x00};

  uint8_t cmdBytes[6] = {0};
  cmdBytes[0] = 0x5A;
  cmdBytes[1] = digit0;
  cmdBytes[2] = digit1;
  cmdBytes[3] = digit2;
  cmdBytes[4] = 0x10;
  cmdBytes[5] = calcChecksum(cmdBytes, 6);

  interfaceSerial->write(cmdBytes, 6);
}

void checkInterfaceData() {
  while (interfaceSerial->available() > 0) {
    uint8_t byte = interfaceSerial->peek();
    if (byte != 0xA5) {
      interfaceSerial->read();
      continue;
    }

    uint8_t cmdBytes[5];
    interfaceSerial->readBytes(cmdBytes, 5);
    if (cmdBytes[4] == calcChecksum(cmdBytes, 5)) {
      // bool prevOnlyPressedM = ((prevState & B00000001) > 0 && (prevState & B11111110) == 0);
      // bool prevOnlyPressed1 = ((prevState & B00000010) > 0 && (prevState & B11111101) == 0);
      // bool prevOnlyPressed2 = ((prevState & B00000100) > 0 && (prevState & B11111011) == 0);
      // bool prevOnlyPressed3 = ((prevState & B00001000) > 0 && (prevState & B11110111) == 0);
      // bool prevOnlyPressedT = ((prevState & B00010000) > 0 && (prevState & B11101111) == 0);
      bool prevOnlyPressedUP = ((prevState & B00100000) > 0 && (prevState & B11011111) == 0);
      bool prevOnlyPressedDW = ((prevState & B01000000) > 0 && (prevState & B10111111) == 0);

      uint8_t state = cmdBytes[2];
      // bool onlyPressedM = ((state & B00000001) > 0 && (state & B11111110) == 0);
      // bool onlyPressed1 = ((state & B00000010) > 0 && (state & B11111101) == 0);
      // bool onlyPressed2 = ((state & B00000100) > 0 && (state & B11111011) == 0);
      // bool onlyPressed3 = ((state & B00001000) > 0 && (state & B11110111) == 0);
      // bool onlyPressedT = ((state & B00010000) > 0 && (state & B11101111) == 0);
      bool onlyPressedUP = ((state & B00100000) > 0 && (state & B11011111) == 0);
      bool onlyPressedDW = ((state & B01000000) > 0 && (state & B10111111) == 0);

      if (prevState != state) prevStateTime = startTime;

      unsigned long sameStateFor = startTime - prevStateTime;

      if (onlyPressedUP) {
        if (prevOnlyPressedUP) {
          // Update
          if (sameStateFor >= buttonHoldThr) {
            userSetSpeed = bound(userSetSpeed * userSetSpeedMultiplier * (double)elapsedTime, defaultUserSetSpeed,
                                 maxUserSetSpeed);
            userSetHeight = bound(userSetHeight + userSetSpeed * (double)elapsedTime, minHeight, maxHeight);
          }
        } else {
          // Pressed
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
          if (sameStateFor >= buttonHoldThr) {
            userSetSpeed = bound(userSetSpeed * userSetSpeedMultiplier * (double)elapsedTime, defaultUserSetSpeed,
                                 maxUserSetSpeed);
            userSetHeight = bound(userSetHeight - userSetSpeed * (double)elapsedTime, minHeight, maxHeight);
          }
        } else {
          // Pressed
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

      uint8_t digit0;
      uint8_t digit1;
      uint8_t digit2;

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

      // digit0 = B10101010;
      // digit1 = B01010101;
      // digit2 = B11001100;

      sendToDisplay(digit0, digit1, digit2);

      prevState = state;
    }
  }
}

void setup() {
  // initialize GDB stub
  // debug_init();

  // Serial.begin(115200);
  // Serial.println("Start");

  initSerial(9600);
}

void loop() {
  elapsedTime = micros() - startTime;
  startTime = micros();

  int av = interfaceSerial->available();
  if (av > 0) checkInterfaceData();

  delay(1);
}