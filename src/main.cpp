#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h" // only needed with flash breakpoints

const unsigned long defaultControllerBaud = 9600;

HardwareSerial *interfaceSerial = &Serial1;
HardwareSerial *controllerSerial1 = &Serial2;
HardwareSerial *controllerSerial2 = &Serial3;

// Protocol
const uint8_t PROT_SER_INT = B00000001;
const uint8_t PROT_SER_CTRL1 = B00000010;
const uint8_t PROT_SER_CTRL2 = B00000011;
const uint8_t PROT_SER_MASK_CTRL = B00000010;
const uint8_t PROT_SER_INT_LEN = 5;

const uint8_t PROT_START = 0xA5;
const uint8_t PROT_CMD_CTRL_STATE = 0x00;
const uint8_t PROT_CMD_CTRL_STATE_M = 0x01;
const uint8_t PROT_CMD_CTRL_STATE_1 = 0x02;
const uint8_t PROT_CMD_CTRL_STATE_2 = 0x04;
const uint8_t PROT_CMD_CTRL_STATE_3 = 0x08;
const uint8_t PROT_CMD_CTRL_STATE_T = 0x10;
const uint8_t PROT_CMD_CTRL_STATE_UP = 0x20;
const uint8_t PROT_CMD_CTRL_STATE_DW = 0x40;

void initSerial(unsigned long newControllerBaud)
{
  (*controllerSerial1).end();
  (*controllerSerial2).end();
  (*interfaceSerial).end();

  (*controllerSerial1).begin(newControllerBaud);
  (*controllerSerial2).begin(newControllerBaud);
  (*interfaceSerial).begin(newControllerBaud);
}

HardwareSerial *getSerial(uint8_t which)
{
  switch (which)
  {
  case PROT_SER_INT:
    return interfaceSerial;
    break;

  case PROT_SER_CTRL1:
    return controllerSerial1;
    break;

  case PROT_SER_CTRL2:
    return controllerSerial2;
    break;

  default:
    return nullptr;
    break;
  }
}

bool isCtrlSerial(uint8_t which)
{
  return ((which & PROT_SER_MASK_CTRL) > 0);
}

void sendCmd(uint8_t to, uint8_t *cmdBytes, uint8_t cmdLen)
{
  HardwareSerial *outputSerial = getSerial(to);
  (*outputSerial).write(cmdBytes, cmdLen);
  (*outputSerial).flush();
}

void handleCtrlCmd00(uint8_t *cmdBytes, uint8_t cmdLen)
{
  if (cmdLen != 5)
    return;
  uint8_t state = cmdBytes[2];
  bool pressedM = state & PROT_CMD_CTRL_STATE_M > 0;
  bool pressed1 = state & PROT_CMD_CTRL_STATE_1 > 0;
  bool pressed2 = state & PROT_CMD_CTRL_STATE_2 > 0;
  bool pressed3 = state & PROT_CMD_CTRL_STATE_3 > 0;
  bool pressedT = state & PROT_CMD_CTRL_STATE_T > 0;
  bool pressedUP = state & PROT_CMD_CTRL_STATE_UP > 0;
  bool pressedDW = state & PROT_CMD_CTRL_STATE_DW > 0;
}

bool checkSum(uint8_t *cmdBytes, uint8_t cmdLen)
{
  uint16_t sum = 0;
  for (int i = 1; i < cmdLen - 1; i++)
    sum += cmdBytes[i];
  return (sum % (0xFF + 1));
}

void sendCtrlCmd00(uint8_t to, bool pressedM, bool pressed1, bool pressed2, bool pressed3, bool pressedT, bool pressedUP, bool pressedDW)
{
  uint8_t state = 0;
  if (pressedM)
    state += PROT_CMD_CTRL_STATE_M;
  if (pressed1)
    state += PROT_CMD_CTRL_STATE_1;
  if (pressed2)
    state += PROT_CMD_CTRL_STATE_2;
  if (pressed3)
    state += PROT_CMD_CTRL_STATE_3;
  if (pressedT)
    state += PROT_CMD_CTRL_STATE_T;
  if (pressedUP)
    state += PROT_CMD_CTRL_STATE_UP;
  if (pressedDW)
    state += PROT_CMD_CTRL_STATE_DW;

  uint8_t cmdBytes[5] = {PROT_START, PROT_CMD_CTRL_STATE, state, 0x01};
  cmdBytes[4] = checkSum(cmdBytes, 5);
  sendCmd(to, cmdBytes, 5);
}

void handleCmd(uint8_t from, uint8_t *cmdBytes, uint8_t cmdLen)
{
  switch (cmdBytes[1])
  {
  case PROT_CMD_CTRL_STATE:
    break;

  default:
    break;
  }
}

void receiveCmd(uint8_t from, uint8_t cmdLen)
{
  HardwareSerial *inputSerial = getSerial(from);
  while ((*inputSerial).available() > 0)
  {
    uint8_t byte = (*inputSerial).peek();
    if (byte != PROT_START)
    {
      (*inputSerial).read();
      continue;
    }

    uint8_t *cmdBytes = new uint8_t[cmdLen];
    (*inputSerial).readBytes(cmdBytes, cmdLen);
    if (cmdBytes[cmdLen - 1] == checkSum(cmdBytes, cmdLen))
      handleCmd(from, cmdBytes, cmdLen);
  }
}

void setup()
{
  // initialize GDB stub
  debug_init();

  initSerial(defaultControllerBaud);
}

void loop()
{
  // receiveCmd(PROT_SER_INT, PROT_SER_INT_LEN);

  sendCtrlCmd00(PROT_SER_CTRL1, 0, 0, 0, 0, 0, 0, 0);
  delay(100);
}