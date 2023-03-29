#include "Potentiometer.h"

#include "mbed.h"
using namespace std;

Potentiometer::Potentiometer(byte potValuePin)
{
  _potValuePin = potValuePin;
}

float Potentiometer::getValuePercent()
{
  float sensorValue = float(analogRead(_potValuePin));

  //Bornage des valeurs du potentiomètre pour avoir une plage à 0% et une plage à 100%
  sensorValue = ((sensorValue > ANALOG_MAX - _potValueMargin) ? ANALOG_MAX - _potValueMargin : sensorValue);
  sensorValue = ((sensorValue < _potValueMargin) ? _potValueMargin : sensorValue);

  return (sensorValue-_potValueMargin/(ANALOG_MAX - 2*_potValueMargin))/40;
}
