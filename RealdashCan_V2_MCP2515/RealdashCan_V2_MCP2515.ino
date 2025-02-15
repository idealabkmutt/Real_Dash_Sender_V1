/**
 * ============================================================================
 * FileName        : RealDashCan_V2_MCP2515.ino
 * modified by Yudi Patriot
 * ypatriot@gmail.com
 * last update 04-9-2021
 * 
 *  Part of     : RealDash
 *  Author      : Jani Immonen
 *  Created     : 15.10.2017
 * Arduino example sketch of how to use RealDash CAN protocol.
 * 
 * This example code is free for any use.
 * www.realdash.net
 *---------------------------------------
 *
 * combined with obd2-mcp2515
 * as published on : https://forum.arduino.cc/t/reading-ecu-data-via-can-bus-and-mcp2515/597914
 * library :  https://github.com/coryjfowler/MCP_CAN_lib
 * Thanks to : sherzaad@forum.arduino.cc
 * Last update: 19-june-2019
 * ============================================================================
**/
#include <mcp_can.h>
#include <SPI.h>
//------------------------------------------------
// Arduino digital and analog pins
// Realdash CAN variables
unsigned int digitalPins = 0;
int analogPins[7] = {0};
unsigned int textCounter = 0;
int AI0;
int AI1;
int AI2;
bool BitVal;
int RPM;
int SPD;
int TPS;
int MAF;
int KPL;

// end of Realdash CAN variables
//------------------------------
//MCP2515 variables
#define PID_THROTTLE 0x11
#define PID_ENGINE_RPM  0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_MAF_FLOW 0x10
#define CAN_ID_PID 0x7DF //OBD-II CAN frame ID
#define CAN0_INT 2       // Set INT to pin 2  <--------- CHANGE if using different pin number
MCP_CAN CAN0(10);        // Set CS to pin 10 <--------- CHANGE if using different pin number
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];       
uint16_t tps;
uint16_t rpm;
uint16_t spd;
uint16_t maf;
// end of  MCP2515 variables
void sendPID(unsigned char __pid)
{
  unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};

  byte sndStat = CAN0.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);

  if (sndStat == CAN_OK) {
//    Serial.print("PID sent: 0x");
//   Serial.println(__pid, HEX);
  }
  else {
    Serial.println("Error Sending Message...");
  }
}

void receivePID(unsigned char __pid)
{
    if (!digitalRead(CAN0_INT)) {                      // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    sprintf(msgString, "Standard ID: 0x%.3lX, DLC: %1d, Data: ", rxId, len);
//    Serial.print(msgString);

    for (byte i = 0; i < len; i++) {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
//      Serial.print(msgString);
    }
//    Serial.println("");
    switch (__pid) {
      case PID_THROTTLE:
        if(rxBuf[2] == PID_THROTTLE){
          
          tps = (rxBuf[3] - 28 ) * 100/172;      
          TPS = tps;    
  //       Serial.print("Engine Coolant Temp (degC): ");
  //       Serial.println(throttle, DEC);
        }
      break;
      case PID_ENGINE_RPM:
        if(rxBuf[2] == PID_ENGINE_RPM){
          
          rpm = ((256 * rxBuf[3]) + rxBuf[4]) / 4;
          RPM = rpm;
//         Serial.print("Engine Speed (rpm): ");
//         Serial.println(rpm, DEC);
        }
      break;
      case PID_VEHICLE_SPEED:
        if(rxBuf[2] == PID_VEHICLE_SPEED){
          
          spd = rxBuf[3];
          SPD = spd;
//         Serial.print("Vehicle Speed (kmh): ");
//         Serial.println(spd, DEC);
        }
      break;      
      case PID_MAF_FLOW:
        if(rxBuf[2] == PID_MAF_FLOW){
          
          maf = ((256 * rxBuf[3]) + rxBuf[4]) / 100;
          MAF = maf;
//          Serial.print("Mass Air Flow (g/s): ");
//          Serial.println(maf, DEC);
        }
      break;     
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) { //< -------- - CHANGE if using different board
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }

  //initialise mask and filter to allow only receipt of 0x7xx CAN IDs
  CAN0.init_Mask(0, 0, 0x07000000);              // Init first mask...
  CAN0.init_Mask(1, 0, 0x07000000);              // Init second mask...

  for (uint8_t i = 0; i < 6; ++i) {
    CAN0.init_Filt(i, 0, 0x07000000);           //Init filters
  }
  
  CAN0.setMode(MCP_NORMAL);   // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);   // Configuring pin for /INT input

  Serial.println("Sending and Receiving OBD-II_PIDs Example...");
}

//------------------------------
void loop()
{
    //request THROTTLE
  sendPID(PID_THROTTLE);
  delay(10); //to allow time for ECU to reply
  receivePID(PID_THROTTLE);
  //request engine speed
  sendPID (PID_ENGINE_RPM);
  delay(10); //to allow time for ECU to reply
  receivePID(PID_ENGINE_RPM);
//request VSS
  sendPID(PID_VEHICLE_SPEED);
  delay(10); //to allow time for ECU to reply
  receivePID(PID_VEHICLE_SPEED);
//request MAF
  sendPID(PID_MAF_FLOW);
  delay(10); //to allow time for ECU to reply
  receivePID(PID_MAF_FLOW);
//abitrary loop delay
  delay(10);
  ReadDigitalStatuses();
  ReadAnalogStatuses();
  SendCANFramesToSerial();
  delay(10);   
}

//------------------------------ 

void ReadDigitalStatuses()
{
  // read status of digital pins (1-13)
  digitalPins = 0;

  int bitposition = 0;
    for (int i=0; i<13; i++)
  {
    if ( i==7 || i==8 || i==9) 
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
//------------------------------
void ReadAnalogStatuses()
{
  // read analog pins (0-7)
  for (int i=0; i<7; i++)
  {
    analogPins[i] = analogRead(i);
  }
}
//------------------------------
void SendCANFramesToSerial()
{
  byte buf[8];
  // build 1st CAN frame, RPM, SPD, throtle, maf
  //KPL = SPD/3600/MAF*(14.7*710);
  memcpy(buf, &rpm, 2);
  memcpy(buf + 2, &spd, 2);
  memcpy(buf + 4, &TPS, 2);
  memcpy(buf + 6, &maf, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  // build 2nd CAN frame, Arduino digital pins and 2 analog values
 AI0 = (analogPins[0] - 595)*(-1)/1.17;
// AI1 = 28 + ((788-analogPins[1])/53); 
AI1 = 85 -((analogPins[1]-267)/(78/10));     
  AI2 = (analogPins[2]*0.1274);
  memcpy(buf, &digitalPins, 2);
  memcpy(buf + 2, &AI0, 2); 
  memcpy(buf + 4, &AI1, 2);
  memcpy(buf + 6, &AI2, 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, buf);

  // build 3rd CAN frame, rest of Arduino analog values
  memcpy(buf, &analogPins[3], 2);
  memcpy(buf + 2, &analogPins[4], 2);
  memcpy(buf + 4, &analogPins[5], 2);
  memcpy(buf + 6, &analogPins[6], 2);

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, buf);

}
//------------------------------
void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}
