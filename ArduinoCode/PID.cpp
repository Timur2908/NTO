#include "PID.h"

PID::PID(double kp, double ki, double kd, double wantedValue, double initValue) {
  this->Kp = kp;
  this->Ki = ki;
  this->Kd = kd;
  this->period.start();
  this->wantedValue = wantedValue;
  this->curentValue = initValue;
  this->P = 0.0;
  this->I = 0.0;
  this->D = 0.0;
}
double PID::getControlValue(double curValue) {
  //Serial.println(curValue);
  double e = wantedValue - curValue;
  double dt = period.getPeriod();
  P = Kp * e;
  I = I + Ki * e * dt;
  D = Kd * (curValue - curentValue) / dt;
  curentValue = curValue;
  return P + I + D;
}
