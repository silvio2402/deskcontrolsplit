#include <Arduino.h>

const unsigned long usbBaud = 115200;
const unsigned long defaultControllerBaud = 9600;

HardwareSerial *usbSerial = &Serial;
HardwareSerial *controllerSerial = &Serial1;
HardwareSerial *interfaceSerial = &Serial2;

byte cmdBuffer[7];

void setControllerBaud(unsigned long newControllerBaud)
{
  (*controllerSerial).end();
  (*interfaceSerial).end();
  (*controllerSerial).begin(newControllerBaud);
  (*interfaceSerial).begin(newControllerBaud);
}

void handleCmd_00(byte data[7])
{
  uint16_t height;
  memcpy(&height, &data[3], 2);
  height = (height >> 8) | (height << 8);
  (*usbSerial).print("Set height: ");
  (*usbSerial).println(height);
}

void handleCmd_Unkn(byte data[7])
{
  char hexstr[22];
  for (int i = 0; i < 7; i++)
  {
    sprintf(hexstr + i * 3, " %02x", data[i]);
  }

  (*usbSerial).print("Unknown:");
  (*usbSerial).println(hexstr);
}

// Concept not working
void sendCmd_00(uint16_t height)
{
  byte cmd[7] = {0x5a, 0x06, 00};
  height = (height >> 8) | (height << 8);
  memcpy(&cmd[3], &height, 2);

  uint16_t chksum = cmd[1] + cmd[2] + cmd[3] + cmd[4];
  chksum = (chksum >> 8) | (chksum << 8);
  memcpy(&cmd[5], &chksum, 2);

  for (int i = 0; i < 7; i++)
  {
    (*controllerSerial).write(cmd[i]);
  }
}

void debugCommands()
{
  // loop as long as bytes are available
  while ((*controllerSerial).available() > 0)
  {
    // circulate cmdBuffer and fill by reading a byte from serial
    memmove(&cmdBuffer[0], &cmdBuffer[1], 6);
    cmdBuffer[6] = (*controllerSerial).read();

    // detect and handle start command
    if ((cmdBuffer[0] == 0x5a && cmdBuffer[1] == 0x06))
    {
      switch (cmdBuffer[2])
      {
      case 0x00:
        handleCmd_00(cmdBuffer);
        break;

      default:
        handleCmd_Unkn(cmdBuffer);
        break;
      }
    }
  }
}

void setup()
{
  (*usbSerial).begin(usbBaud);
  (*usbSerial).println("Start");
  setControllerBaud(defaultControllerBaud);

  // sendCmd_00(700);
}

void loop()
{
  // debugCommands();

  // check how many bytes are available in the serial ports
  const int availControllerBytes = (*controllerSerial).available();
  const int availInterfaceBytes = (*interfaceSerial).available();

  // read out the serial ports and write the data to the each other's serial port
  byte controllerData[availControllerBytes];
  byte interfaceData[availInterfaceBytes];

  if (availControllerBytes > 0)
  {

    (*controllerSerial).readBytes(controllerData, availControllerBytes);
    (*usbSerial).write(controllerData, availControllerBytes);
    // (*interfaceSerial).write(controllerData, availControllerBytes);
  }
  if (availInterfaceBytes > 0)
  {
    (*interfaceSerial).readBytes(interfaceData, availInterfaceBytes);
    // (*usbSerial).write(interfaceData, availInterfaceBytes);
    // (*controllerSerial).write(interfaceData, availInterfaceBytes);
  }
}