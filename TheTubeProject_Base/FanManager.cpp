#include "FanManager.h"
using namespace std;
#include "mbed.h"

FanManager::FanManager(byte fanPWMPin, byte fanHallSensorPin, byte fanEnablePin, byte CurrentMeasPin)
{
  _fanPWMPin=fanPWMPin;
  _fanHallSensorPin=fanHallSensorPin;
  _fanEnablePin=fanEnablePin;
  _CurrentMeasPin=CurrentMeasPin;

/*
  // set up the pwm object for PWMPin
  pwm = new mbed::PwmOut(digitalPinToPinName(_fanPWMPin));
  // set the PWM period to PWMPeriod
  pwm->period_us(PWMPeriod);
*/
  pinMode(_fanPWMPin, OUTPUT);
  
  _lastSpeedMeasurement = millis();
  _NbTopsFan = 0;

  pinMode(_fanEnablePin, OUTPUT);
}

int FanManager::setSpeed(float speedRPM)
{
  //Bornage des valeur de vitesse du ventilateur
  speedRPM = ((speedRPM > maxRPM) ? maxRPM : speedRPM);
  speedRPM = ((speedRPM < minRPM) ? minRPM : speedRPM);
  int _SetpointRpm = speedRPM;

  _PWMDutyCycle = float(_SetpointRpm-minRPM)/(maxRPM-minRPM);
  //pwm->write(_PWMDutyCycle);
  analogWrite(_fanPWMPin, _PWMDutyCycle*255);
  return _SetpointRpm;
}

void FanManager::setSpeedProp(float speedProp)
{
  analogWrite(_fanPWMPin, speedProp*255);
}
 
void FanManager::incRpmCounter()
//This is the function that the interupt calls
{ 
  _NbTopsFan++;
}

int FanManager::computeSpeedRPM()
{
  unsigned long now = millis();
  double tmp = now - _lastSpeedMeasurement;
  _lastSpeedMeasurement = now;
  double speed = ((_NbTopsFan * 60000 / tmp)/fanspace[_fanType].fandiv);
  _NbTopsFan = 0;
  return  int(speed);
}

int FanManager::computeCurrentmA()
{
 // read the input on analog pin
  float analogValue = analogRead(_CurrentMeasPin);

  return  int((analogValue/ANALOG_MAX)*1300);
}

void FanManager::enableRotation(bool state)
{
  digitalWrite(_fanEnablePin, state);
}

byte FanManager::getHallPinNumber()
{
  return _fanHallSensorPin;
}