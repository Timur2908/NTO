#include "Period.h"
//Конструктор
Period::Period(){
  period = 0;
}
//Метод для старта
void Period::start(){
  period = millis();
}
//Получение периода
unsigned long Period::getPeriod(){
  unsigned long res = millis() - period;
  period = millis();
  return res;
}
