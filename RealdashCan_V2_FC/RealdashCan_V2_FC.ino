/**
 * ============================================================================
 * FileName        : RealDashCanV2-1.ino
 * modified by Yudi Patriot
 * ypatriot@gmail.com
 * last update 28-08-2021
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
 * combined with dual channel frequency counter program
 * as published on https://www.esologic.com/multiple-frequency-counter-arduino/
 * Made by Devon Bray
 * Last update: 03/30/2018
 * ============================================================================
**/
// Arduino digital and analog pins
#include<FreqCount.h>
// Realdash CAN variables
unsigned int digitalPins = 0;
int analogPins[7] = {0};
unsigned int rpm = 0;
unsigned int kpa = 0; 
unsigned int tps = 0;
unsigned int clt = 0; 
unsigned int textCounter = 0;
int AI0;
int AI1;
int AI2;
int SPEED;
int RPM;
int dummy23;
int dummy24;
bool BitVal;
// end of Realdash CAN variables
//------------------------------
// Frequency counter vriables
int freq_pin_1 = 2; 
int freq_pin_2 = 3; 
 
#define BUFFSIZE 3 // a rolling average of the frequency/period is computed, and this is the size of that buffer
//#define BUFFSIZE 100
#define NUMSIGS 2
#define FREQ1INDEX 0
#define FREQ2INDEX 1
 
volatile int period_buffer_indices[NUMSIGS] = { 0 }; // the location of the index for adding to the rolling buffer average
volatile unsigned long period_buffers[NUMSIGS][BUFFSIZE] = { 0 }; // the buffers
volatile unsigned long previous_edge_times_us[NUMSIGS] = { 0 }; // the time that the previous edge came in in microseconds
volatile float period_averages_ms[NUMSIGS] = { 0 }; // the period time of a given signal in milliseconds
volatile float frequency_averages_hz[NUMSIGS] = { 0 }; // the frequency of a given signal in hertz
volatile bool period_buffer_locked[NUMSIGS] = { false }; // spin locks for the different buffers
// end of frequency counter variables
void setup()
{
  Serial.begin(115200);
     
pinMode(freq_pin_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(freq_pin_1), new_freq1_edge, RISING); // you could change this mode to whatever you were looking for, FALLING, CHANGE etc.
 
  pinMode(freq_pin_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(freq_pin_2), new_freq2_edge, FALLING);
}
//------------------------------
void loop()
{
  compute_counts();
  RPM = (frequency_averages_hz[FREQ1INDEX])*750/25;
  SPEED = (frequency_averages_hz[FREQ2INDEX])*1000/710;
  ReadDigitalStatuses();
  ReadAnalogStatuses();
  SendCANFramesToSerial();
  delay(10);   
}
//------------------------------
void compute_counts() {
 
  // computes the average of the buffer for a given signal. Must be called before using the period_averages_ms or frequency_averages_hz buffers.
  
  for (int p_index = 0; p_index < NUMSIGS; p_index++) {
  
    float buffer_sum = 0;
 
    while (period_buffer_locked[p_index]) {}; // wait around for the ISR to finish
    
    period_buffer_locked[p_index] = true; // ISR won't add new data to `period_buffers`
    if ((micros() - previous_edge_times_us[p_index]) < 1000000) {
      for (int j = 0; j < BUFFSIZE; j++) {
        buffer_sum += period_buffers[p_index][j];
      }
    }
    period_buffer_locked[p_index] = false; // ISR will now add new data to `period_buffers`
    
    if (buffer_sum > 0){
      period_averages_ms[p_index] = ((buffer_sum / (float)BUFFSIZE)) / 1000;
      frequency_averages_hz[p_index] = (1 / period_averages_ms[p_index]) * 1000;  
    } 
    else {
      period_averages_ms[p_index] = 0;
      frequency_averages_hz[p_index] = 0;
    }
        
  }
}
//------------------------------
void new_edge(int period_index) {
 
  unsigned long current = micros();
 
  if (period_buffer_locked[period_index] == false) { // if compute_counts is using the buffer, skip adding to it because that process isn't atomic
 
    period_buffer_locked[period_index] = true;
    
    period_buffers[period_index][period_buffer_indices[period_index]] = current - previous_edge_times_us[period_index];
 
    period_buffer_locked[period_index] = false;
    
    period_buffer_indices[period_index]++;
    if (period_buffer_indices[period_index] >= BUFFSIZE) {
      period_buffer_indices[period_index] = 0; 
    }  
  }
  
  previous_edge_times_us[period_index] = current; // but make sure the new time is set because this operation is atomic
  
}
//------------------------------ 
void new_freq1_edge() {
  new_edge(FREQ1INDEX);
}
 
void new_freq2_edge() {
  new_edge(FREQ2INDEX);
}

void ReadDigitalStatuses()
{
  // read status of digital pins (1-13)
  digitalPins = 0;

  int bitposition = 0;
    for (int i=0; i<13; i++)
  {
    if ( i==9 || i==10 || i==11) 
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
  // build 1st CAN frame, RPM, SPD, dummy23, dummy24

  memcpy(buf, &RPM, 2);
  memcpy(buf + 2, &SPEED, 2);
  memcpy(buf + 4, &dummy23, 2);
  memcpy(buf + 6, &dummy24, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  // build 2nd CAN frame, Arduino digital pins and 2 analog values
  AI0 = (analogPins[0] - 595)*(-1)/1.17;
  AI1 = (analogPins[1] - 595)*(-1)/1.17;
  AI2 = (analogPins[2]/7.76);
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
