/**
 * ============================================================================
 *  Name        : RealDash_CAN.ino
 *  Part of     : RealDash
 *  Author      : Jani Immonen
 *  Created     : 15.10.2017
 *
 * Arduino example sketch of how to use RealDash CAN protocol.
 * 
 * This example code is free for any use.
 * 
 * www.realdash.net
 * 
 * Bluetooth setting
 * ------------------
 * UART:9600,0,0
 * NAME:REALDASH-CAN
 * CMODE:0 
 * ROLE:1
 * ------------------------
 * Modify By : Yudi Patriot
 * Date 13-10-2020
 * ============================================================================
**/
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // RX, TX

// Arduino digital and analog pins
unsigned int digitalPins = 0;
int analogPins[7] = {0};
unsigned int textCounter = 0;
bool BitVal;


void setup()
{
  // init serial
  BTSerial.begin(9600);
  delay(100);
}


void loop()
{
  ReadDigitalStatuses();
  ReadAnalogStatuses();
  SendCANFramesToSerial();

  if (textCounter++ > 4000)
  {
    textCounter = 0;
  }
  
  delay(5);
}


void ReadDigitalStatuses()
{
  // read status of digital pins (1-13)
  digitalPins = 0;

  int bitposition = 0;
    for (int i=0; i<14; i++)
  {
    if ( i==9 || i==11 || i==12) 
    {
      BitVal = !digitalRead(i); 
    } else 
    {
    BitVal = digitalRead(i);
    }
    if (BitVal == HIGH) digitalPins |= (1 << bitposition);
    bitposition++;
  }
}


void ReadAnalogStatuses()
{
  int SV = analogRead(A0);
  
  // read analog pins (0-7)
  for (int i=0; i<7; i++)
  {
    analogPins[i] = analogRead(i);
  }
}


void SendCANFramesToSerial()
{
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values
  // build 2nd CAN frame, Arduino digital pins and 2 analog values
  memcpy(buf, &digitalPins, 2);
  memcpy(buf + 2, &analogPins[0], 2); 
  memcpy(buf + 4, &analogPins[1], 2);
  memcpy(buf + 6, &analogPins[2], 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, buf);

  // build 3rd CAN frame, rest of Arduino analog values
  memcpy(buf, &analogPins[3], 2);
  memcpy(buf + 2, &analogPins[4], 2);
  memcpy(buf + 4, &analogPins[5], 2);
  memcpy(buf + 6, &analogPins[6], 2);

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, buf);

  // build 4th frame, this is a text extension frame
  // only send once at 1000 loops
  if (textCounter == 0)
  {
    SendTextExtensionFrameToSerial(3203, "Hello RealDash, this is Arduino sending some text data");
  }
  else if (textCounter == 1000)
  {
    SendTextExtensionFrameToSerial(3203, "Tomorrow's forecast: Lots of sun and 30 degrees centigate");
  }
  else if (textCounter == 2000)
  {
    SendTextExtensionFrameToSerial(3203, "Now Playing: Insert your favorite song info here");
  }
  else if (textCounter == 3000)
  {
    SendTextExtensionFrameToSerial(3203, "Message from Arduino: All systems running at nominal efficiency");
  }
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  BTSerial.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  BTSerial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  BTSerial.write(frameData, 8);
}


void SendTextExtensionFrameToSerial(unsigned long canFrameId, const char* text)
{
  if (text)
  {
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
    const byte textExtensionBlockTag[4] = { 0x55, 0x33, 0x22, 0x11 }; 
    BTSerial.write(textExtensionBlockTag, 4);

    // the CAN frame id number (as 32bit little endian value)
    BTSerial.write((const byte*)&canFrameId, 4);

    // text payload
    BTSerial.write(text, strlen(text) + 1);
  }
}
