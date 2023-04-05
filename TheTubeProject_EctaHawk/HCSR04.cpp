#include "HCSR04.h"

#include "mbed.h"
using namespace std;

HCSR04::HCSR04(byte trigPin, byte echoPin)
{ 
  _trigPin = trigPin;
  _echoPin = echoPin;

  pinMode(_trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(_echoPin, INPUT); // Sets the echoPin as an Input
}

float HCSR04::measureDistance()
{
  // Clears the trigPin
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  _duration = pulseIn(_echoPin, HIGH, meas_timout);
  // Calculating the distance
  return _duration * meas_factor;
}

float HCSR04::heightPercentToCm(float heightPercent) {
  for (int i = 0; i < tableSize - 1; i++) {
    if (heightPercent >= heightTable[i].heightPercent && heightPercent <= heightTable[i + 1].heightPercent) {
      float t = (heightPercent - heightTable[i].heightPercent) / (heightTable[i + 1].heightPercent - heightTable[i].heightPercent);
      return heightTable[i].heightCm + t * (heightTable[i + 1].heightCm - heightTable[i].heightCm);
    }
  }
  return -1; // Retournez -1 en cas d'erreur (par exemple, si heightPercent est en dehors des limites du tableau)
}