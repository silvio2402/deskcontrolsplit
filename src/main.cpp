#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h" // only needed with flash breakpoints

const unsigned long defaultControllerBaud = 9600;

HardwareSerial *interfaceSerial = &Serial1;
HardwareSerial *controllerSerial1 = &Serial2;
HardwareSerial *controllerSerial2 = &Serial3;

void initSerial(unsigned long newControllerBaud)
{
  (*controllerSerial1).end();
  (*controllerSerial2).end();
  (*interfaceSerial).end();

  (*controllerSerial1).begin(newControllerBaud);
  (*controllerSerial2).begin(newControllerBaud);
  (*interfaceSerial).begin(newControllerBaud);
}

uint8_t *forwardSerial(HardwareSerial *inputSerial, HardwareSerial *outputSerial, int len)
{
  uint8_t *inputData = new uint8_t[len];

  if (len <= 0)
    return inputData;

  (*inputSerial).readBytes(inputData, len);

  (*outputSerial).write(inputData, len);

  return inputData;
}

char *debugByte(uint8_t data)
{
  char *out = new char[sizeof(data)];
  for (uint8_t pos = 0; pos < sizeof(data); pos++)
  {
    out[pos] = (data & (1 << pos)) ? '0' : '1';
  }
  return out;
}

void setup()
{
  // initialize GDB stub
  debug_init();

  initSerial(defaultControllerBaud);
}

void loop()
{
  int availInterfaceSerial = (*interfaceSerial).available();
  int availControllerSerial1 = (*controllerSerial1).available();
  // int availControllerSerial2 = (*controllerSerial2).available();

  /// controller 1 -> interface
  uint8_t *controller1Data = forwardSerial(controllerSerial1, interfaceSerial, availControllerSerial1);
  (*controllerSerial1).flush();

  /// controller 2 -> interface
  // uint8_t *controller2Data = forwardSerial(controllerSerial2, interfaceSerial);

  /// interface -> controller 1
  uint8_t *interfaceData = forwardSerial(interfaceSerial, controllerSerial1, availInterfaceSerial);
  (*interfaceSerial).flush();

  /// interface -> controller 2
  // uint8_t *interfaceData = forwardSerial(interfaceSerial, controllerSerial2, availInterfaceSerial);
}