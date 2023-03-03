#pragma once
#include<Arduino.h>
#include<EncButton2.h>

#define ANGLE_PER_TICK   18       //Количество тиков за оборот

class Encoder {
  private:
    EncButton2<EB_ENC> enc;
  public:
    double Speed;
    double Theta;
    Encoder();
    Encoder(int pinA, int pinB);
    //Обновление информации => пересчёт угла и скорости
    void Update();
    //Функция для обновления информации бибилиотеки энкодера
    //Надо вызывать постояноо, а лучше засунуть в аппаратное прерывание
    void Tick();
};
