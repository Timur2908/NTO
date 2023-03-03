#pragma once
#include<Arduino.h>
/*
  Класс драйвера двигателя
  При инициализакии указать порты подключения драйвера
  ВАЖНО!!! Порты должны поддерживать ШИМ
*/
class Driver {
  private:
    int motorPinA;    //пин A на драйвере
    int motorPinB;    //пин B на драйвере
  public:
    //Пустой конструктор с пигами по умолчанию
    Driver();
    Driver(int pinA, int pinB);
    //Инициализация пинов
    void init();
    //Подача напряжения на двигатель
    void setMotorVoltage(int voltage);
};
