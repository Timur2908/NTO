#include "Driver.h"

//Пустой конструктор с пигами по умолчанию
Driver::Driver() {
  //motorPinA = 9;
  //motorPinB = 10;
}
Driver::Driver(int pinA, int pinB) {
  motorPinA = pinA;
  motorPinB = pinB;
  init();
}
//Инициализация пинов
void Driver::init() {
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}
//Подача напряжения на двигатель
void Driver::setMotorVoltage(int voltage) {
  //Задаём управляющее напряжение на оба двигателя
  int voltageA = abs(voltage);
  int voltageB = 0;
  //Меняем местами если надо крутить в обратную сторону
  if (voltage < 0) {
    voltageA = 0;
    voltageB = abs(voltage);
  }
  //Подаём напряжение на двигатель
  analogWrite(motorPinA, voltageA);
  analogWrite(motorPinB, voltageB);
}
