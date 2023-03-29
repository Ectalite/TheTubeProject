#ifndef FAN_MANAGER_H
#define FAN_MANAGER_H

#include "mbed.h"
using namespace std;

#define ANALOG_MAX 4096

class FanManager
{
  public:
  FanManager(byte fanPWMPin, byte fanHallSensorPin, byte fanEnablePin, byte CurrentMeasPin);
  int     setSpeed(float speedRPM);
  float   getPWMRatio();
  int  computeSpeedRPM();
  int computeCurrentmA();
  void enableRotation(bool state);
  void  incRpmCounter(); //Called by interupt on HallSensorPin
  byte getHallPinNumber();
 

  private:
  byte _fanPWMPin; // Arduino pin number for the Fan PWM
  byte _fanHallSensorPin; // Arduino pin number for the Fan speed
  byte _fanEnablePin; // Arduino pin number for the Fan enable
  byte _CurrentMeasPin; // Arduino pin number for reading of th measured current

  mbed::PwmOut* pwm;
  const unsigned int PWMPeriod = 1e6/25000; // us (minimum value is 2) -> Freq 25KHz
  const unsigned int maxRPM =  14500;
  const unsigned int minRPM = 3500;

  double _PWMDutyCycle; // ratio PWM

  int _NbTopsFan;
  unsigned long _lastSpeedMeasurement;
  //Defines the structure for multiple fans and their dividers 
  typedef struct{
  char fantype;
  unsigned int fandiv;
  }fanspec;
 
  //Definitions of the fans
  //This is the varible used to select the fan and it's divider,
  //set 1 for unipole hall effect sensor
  //set 2 for unipole hall effect sensor (R/F pulse detection)
  //and 3 for bipole hall effect sensor
  fanspec fanspace[4]={{0,1},{1,2},{2,4},{3,8}};
  const char _fanType = 2;

};


#endif