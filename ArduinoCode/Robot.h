#pragma once
#include <math.h>
#include<Arduino.h>
#include "PID.h"
#include "Motor.h"
/*
  Класс робота - основной класс
  При инициализации передать праметры робота:
    Радиус колёс
    Расстояние между колесaми - база
  Также необходимо передавать начальные координаты и ориентацию робота
*/
//####################################ПИНЫ ПОДЛЮЧЕНИЯ################################
#define MOTOR_L_PIN_A       5
#define MOTOR_L_PIN_B       6
#define MOTOR_R_PIN_A       3
#define MOTOR_R_PIN_B       11

#define ENCODER_L_PIN_A     12
#define ENCODER_L_PIN_B     2
#define ENCODER_R_PIN_A     7
#define ENCODER_R_PIN_B     8
//###################################################################################

const int PID_ITERATIONS = 200;         // Колическо итераций для ПИД - регулятора
const int MOVE_ITERATIONS = 500;       // Количество итераций для функции Ляпунова
const double MOVE_ERROR = 0.0001;       // Предельная ошибка регулирования фуекции Ляпунова

#define MOTOR_WORK_DELAY    100         // Задержка перед обновлением после подачи напряжения на моторы

class Robot {
  private:
    double angle0;
    double path;
    double base;
    double wheelRadius;
    Motor LeftMotor;
    Motor RightMotor;
  public:
    double x;
    double y;
    double angle;
    //Функция инициализации
    Robot();
    Robot(double x0, double y0, double angle0, double wheelRadius, double base);
    //Движение робота
    void drive(double Lvoltage, double Rvoltage);
    //Остановка робота
    void Stop();
    //Обновление информации и роботе
    void Update();
    //Тикеры для энкодеров моторов
    //Его и надо сунуть в аппаратное прерывание на пины энкодера в Setup
    void TickL();
    void TickR();
    //Поворот в указанную ориентацию при помощи ПИД - регулятора
    //Возвращает массив с историей регулирования для построения графика
    void rotateToAngle(double purposeAngle);
    //Движение в указанную точку функцией Ляпунова
    void moveToPoint(double targetX, double targetY);
};

//Расстояние между 2 точками
double dist(double x1, double y1, double x2, double y2);
// Нормализация значений угла в промежутке от 0 до 360
void angleNormal360(double& angle);
// Нормализация угла в пределах[-180, 180]
void angleNormal180(double& angle);
//Нормализация углы тупым остатком от деления - эффективно для устронения проблемы перхода PID регулятора
void angleNormal(double& angle);
