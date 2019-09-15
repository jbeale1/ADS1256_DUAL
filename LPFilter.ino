#include <SPI.h>
#include <ads12xx.h>

// Use two ADC1256 chips to measure two differential inputs
// at 500 Hz. Low-pass FIR filter. 2ch output at 50 Hz.
// Tested with Teensy 4

// modified from https://github.com/Flydroid/ADS12xx-Library
// uses ads12xx.cpp, ads12xx.h, ads1256.h
// changed DRDY interrupt to polled, added delays to SPI readout
// by J.Beale 14-Sept-2019

// Teensy pins connected to ADC #1
int  START = 3;  // _SYNC input (unused- ADS pin tied high)
int  CS =   21;
int  DRDY = 22;
int _RESET = 8;

// Teensy pins connected to ADC #2
int  START2 = 3;  // _SYNC input (unused- ADS pin tied high)
int  CS2 = 19;
int  DRDY2 = 20;
int _RESET2 = 7;


// two physical ADC chips
ads12xx ADS1;
ads12xx ADS2;

const int LED1 = 13;         // this pin both SPI clock, and LED

#define FIRsize 251
float inbuf1[FIRsize]; // buffer for input data from ADC1, prior to FIR filter
float inbuf2[FIRsize]; // buffer for input data from ADC2, prior to FIR filter
unsigned char bptr=0; // pointer into data buffer
float fres1=0;         // result of FIR filter
float fres2=0;         // result of FIR filter
float f=0.001;         // low pass filter constant
float dc1=0;
float dc2=0;           // dc average value of ch1,ch2


// 251 element FIR filter kernel
float kern[] = {
0.000307981,0.000313008,0.000319196,0.000326593,0.000335244,0.000345194,
0.000356488,0.000369167,0.000383272,0.000398845,0.000415923,0.000434543,
0.000454742,0.000476554,0.00050001,0.000525144,0.000551982,0.000580554,
0.000610884,0.000642996,0.000676913,0.000712653,0.000750236,0.000789676,
0.000830987,0.000874182,0.000919269,0.000966255,0.00101515,0.00106594,
0.00111865,0.00117326,0.00122977,0.00128818,0.00134847,0.00141064,
0.00147467,0.00154055,0.00160825,0.00167776,0.00174905,0.0018221,
0.00189688,0.00197336,0.0020515,0.00213128,0.00221266,0.00229559,
0.00238003,0.00246595,0.0025533,0.00264202,0.00273207,0.00282339,
0.00291594,0.00300966,0.00310448,0.00320035,0.00329721,0.00339499,
0.00349364,0.00359308,0.00369324,0.00379406,0.00389547,0.00399739,
0.00409975,0.00420248,0.0043055,0.00440873,0.0045121,0.00461553,
0.00471895,0.00482226,0.00492539,0.00502827,0.0051308,0.00523291,
0.00533452,0.00543554,0.00553589,0.0056355,0.00573427,0.00583213,
0.005929,0.0060248,0.00611943,0.00621284,0.00630493,0.00639563,
0.00648487,0.00657256,0.00665863,0.006743,0.00682561,0.00690637,
0.00698523,0.00706211,0.00713694,0.00720967,0.00728021,0.00734852,
0.00741453,0.00747818,0.00753942,0.00759819,0.00765443,0.00770811,
0.00775917,0.00780756,0.00785324,0.00789617,0.00793631,0.00797363,
0.00800808,0.00803965,0.0080683,0.008094,0.00811673,0.00813648,
0.00815322,0.00816693,0.00817761,0.00818525,0.00818983,0.00819136,
0.00818983,0.00818525,0.00817761,0.00816693,0.00815322,0.00813648,
0.00811673,0.008094,0.0080683,0.00803965,0.00800808,0.00797363,
0.00793631,0.00789617,0.00785324,0.00780756,0.00775917,0.00770811,
0.00765443,0.00759819,0.00753942,0.00747818,0.00741453,0.00734852,
0.00728021,0.00720967,0.00713694,0.00706211,0.00698523,0.00690637,
0.00682561,0.006743,0.00665863,0.00657256,0.00648487,0.00639563,
0.00630493,0.00621284,0.00611943,0.0060248,0.005929,0.00583213,
0.00573427,0.0056355,0.00553589,0.00543554,0.00533452,0.00523291,
0.0051308,0.00502827,0.00492539,0.00482226,0.00471895,0.00461553,
0.0045121,0.00440873,0.0043055,0.00420248,0.00409975,0.00399739,
0.00389547,0.00379406,0.00369324,0.00359308,0.00349364,0.00339499,
0.00329721,0.00320035,0.00310448,0.00300966,0.00291594,0.00282339,
0.00273207,0.00264202,0.0025533,0.00246595,0.00238003,0.00229559,
0.00221266,0.00213128,0.0020515,0.00197336,0.00189688,0.0018221,
0.00174905,0.00167776,0.00160825,0.00154055,0.00147467,0.00141064,
0.00134847,0.00128818,0.00122977,0.00117326,0.00111865,0.00106594,
0.00101515,0.000966255,0.000919269,0.000874182,0.000830987,0.000789676,
0.000750236,0.000712653,0.000676913,0.000642996,0.000610884,0.000580554,
0.000551982,0.000525144,0.00050001,0.000476554,0.000454742,0.000434543,
0.000415923,0.000398845,0.000383272,0.000369167,0.000356488,0.000345194,
0.000335244,0.000326593,0.000319196,0.000313008,0.000307981};
// =======================================================


void setup()
{
  pinMode(LED1,OUTPUT);       // for visual startup signal
  Serial.begin(115200);
  delay(200);

  for (int i=0;i<2;i++) {  // blink 2x to show startup
    digitalWrite(LED1,HIGH);   
    delay(250);
    digitalWrite(LED1,LOW);  
    delay(250);
  }

  ADS1.begin(CS, START, DRDY,_RESET);  //initialize ADC object of the ads12xx class
  ADS2.begin(CS2, START2, DRDY2,_RESET2);  //initialize 2nd ADC 

  ADS1.Reset();  // reset each ADC chip
  ADS2.Reset();
  
  delay(100);

  // Serial.println("Self Gain and Offset Calibration");
  ADS1.SendCMD(SELFCAL);
  ADS2.SendCMD(SELFCAL);
  delay(5);

// setup input mux pins and data rate for each ADC
  ADS1.SetRegisterValue(MUX, P_AIN0 | N_AIN1); // diff-in: (A0-A1)
  ADS1.SetRegisterValue(DRATE, DR_500);

  ADS2.SetRegisterValue(MUX, P_AIN0 | N_AIN1); // diff-in: (A0-A1)
  ADS2.SetRegisterValue(DRATE, DR_500);
  delay(1);

  int startNum = 50;  // how many to average
  for (int i=0;i<startNum;i++) {
    dc1 += (int32_t) ADS1.GetConversion();
    dc2 += (int32_t) ADS2.GetConversion();
  }
  dc1 /= startNum;  // calculate average DC value on each ADC
  dc2 /= startNum;
}

unsigned char i=0;
int32_t dPerLine = 5;  // decimation ratio
unsigned char inter=0;  // interleaving index

// =================================================================
void loop() {

unsigned char j,k;  // temp index into FIR[] and inbuf[]
int32_t raw1,raw2;  // raw ADC samples

  raw1 = (int32_t) ADS1.GetConversion(); // get one raw value
  raw2 = (int32_t) ADS2.GetConversion();

// ---------------------------------------------------------
  inbuf1[bptr] = raw1;     // store in buffer
  inbuf2[bptr] = raw2;     // store in buffer
  bptr = ++bptr % FIRsize;

  k = bptr;
  inter = ++inter % 4;  // 4-phase interleaving index. Don't do everything at once
  switch (inter) {
    case 0:          // calculate FIR filter on Ch.1
      fres1 = 0;
      for (j=0; j<FIRsize; j++) {  // implementation of n-tap FIR filter
        fres1 += kern[j] * inbuf1[k];
        k = ++k % FIRsize;
      }
      break;
    case 1:          // calculate FIR filter on Ch.2
      fres2 = 0;
      for (j=0; j<FIRsize; j++) {  // implementation of n-tap FIR filter
        fres2 += kern[j] * inbuf2[k];
        k = ++k % FIRsize;
      }
      break;      
    case 2:
    case 3:
      // at this point, fres1,2 has the FIR-filtered output
      // output is delayed (FIRsize/2) samples relative to input

      if (++i >= dPerLine) {
        dc1 = (1.0-f)*dc1 + f*fres1;  // long-term average DC value
        dc2 = (1.0-f)*dc2 + f*fres2;
        Serial.print(int(fres1-dc1));
        Serial.print(",");
        Serial.print(int(fres2-dc2));
        Serial.println();
        i = 0;  
      }
  }
} // end loop()
