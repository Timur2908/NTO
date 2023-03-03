#pragma once
#include "Driver.h"
#include "Encoder.h"
/*
  Класс двигателя
  При инициализакии передать тип двигателя = > L или R для правильного опрелелния при обновлении
  А так же пины для подключения драйвера и энкодера
*/
#define MAX_MOTOR_VOLTAGE   255     //Максимальное возможное напряжения на двигателе
class Motor {
  private:
    Driver motorDriver;     //Драйвер двигателя
    Encoder motorEncoder;   //Энкодер двигателя
    char motorType;     //Тип мотора (L or R)
  public:
    //Пустой конструктор
    Motor();
    //Инициализация
    Motor(char type, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB);
    //Подача напряжения на двигатель
    void runMotor(int voltage);
    // Обновление информации
    void Update();
    //Тикер для энкодера
    void Tick();
    //Getters для угла и скорости
    double getTheta();
    double getSpeed();
};
