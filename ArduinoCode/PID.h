#pragma once
#include "Period.h"
/*
  Класс PID-регулятора
  kp, ki, kd - коэффициенты
  dt - шаг по веремени для интегрирования
  wantedValue - требуемое значение
  curValue - текущее значение
*/
class PID {
  private:
    //Параметры регулятора
    double Kp;              // пропорциональный коэффициент
    double Ki;              // интегральный коэффициент
    double Kd;              // дифференциальный коэффициент
    Period period;          // таймер на millis для шага по времени интегрирования
    double wantedValue;     // требуемое значение
    double curentValue;     // текущее значение
    //Управляющие компоненты
    double P = 0;
    double I = 0;
    double D = 0;
  public:
    //Конструктор
    PID(double kp, double ki, double kd, double wantedValue, double initValue);
    //Вычисление управляющего воздействия по текущему значению
    double getControlValue(double curValue);
};
