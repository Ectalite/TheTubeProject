#ifndef HCSR04_H
#define HCSR04_H

#include "mbed.h"
using namespace std;

#define meas_factor 0.034 / 2
#define meas_timout 10000 //10ms (il faut environ 8ms pour une mesure de 1m (mesuré))

/*You can calculate the range through the time interval between sending trigger signal and
receiving echo signal. Formula: uS / 58 = centimeters or uS / 148 =inch; or: the
range = high level time * velocity (340M/S) / 2; we suggest to use over 60ms
measurement cycle, in order to prevent trigger signal to the echo signal. */

struct HeightData {
  float heightPercent;
  float heightCm;
};

const HeightData heightTable[] = {
  {94.5, 0},
  {93, 2},
  {90.6, 4},
  {88.2, 6},
  {86.3, 8},
  {85.8, 10},
  {82.6, 12},
  {80.5, 14},
  {78.6, 16},
  {76.3, 18},
  {74.6, 20},
  {72.6, 22},
  {70.4, 24},
  {68.7, 26},
  {67, 28},
  {64.9, 30},
  {62.9, 32},
  {61, 34},
  {59.1, 36},
  {57.2, 38},
  {55.3, 40},
  {53.5, 42},
  {51.1, 44},
  {49.2, 46},
  {47.2, 48},
  {45.4, 50},
  {43.3, 52},
  {41.4, 54},
  {39.5, 56},
  {37.6, 58},
  {35.7, 60},
  {33.8, 62},
  {32.2, 64},
  {30.8, 66},
  {27.7, 68},
  {25.8, 70},
  {24.2, 72},
  {22.3, 74},
  {19.9, 76},
  {18.1, 78},
  {17.1, 80},
  {14.1, 82},
  {11.7, 84},
  {9.6, 86},
  {8.1, 88},
  {6.7, 90},
  {3.6, 92},
  {3, 92.5},
};

const int tableSize = sizeof(heightTable) / sizeof(HeightData);

class HCSR04
{
  public:
  HCSR04(byte trigPin, byte echoPin);
  float measureDistance();
  float heightInCm();

  private:  
  byte _trigPin;
  byte _echoPin;
  unsigned long _duration; 
};

#endif