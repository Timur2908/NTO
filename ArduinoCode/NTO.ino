#include "Robot.h"
#include "TimerOne.h"
//Глобальный параметры робота
#define BASE              0.15
#define WHEEL_RADIUS      0.05

//Создали неинициализированного робота
Robot rbt;

void setup() {
  //Настоили связь
  Serial.begin(9600);
  Serial.setTimeout(50);
  //Установили таймер на энкодеры
  Timer1.initialize(1000);            // установка таймера на каждые 1000 микросекунд (= 1 мс)
  Timer1.attachInterrupt(encodersUpdate);   // запуск таймера
  //Ждём и читаем данный инициализации
  while (true) {
    String inputData = Serial.readString();
    if (parcer(inputData)) break;
  }
  Serial.println("DONE");
//  Serial.print("x = ");
//  Serial.println(rbt.x);
//  Serial.print("y = ");
//  Serial.println(rbt.y);
//  Serial.print("angle = ");
//  Serial.println(rbt.angle);
}

//Функция для автоматического опроса энкодеров по таймеру,
//Работает независимо от общей программы и задержек в ней => не пропустим тики))
void encodersUpdate() {
  rbt.TickL();
  rbt.TickR();
}

void loop() {
  if (Serial.available()) {
    //Получили задание
    String inputData = Serial.readString();
    //Парсим и выполняем команду
    if (parcer(inputData)) {
      Serial.println("DONE");
    }
  }
}

//Парсер данных полученных из COM порта
//Вызывает функции выполнения задания
bool parcer(String inputData) {
  if(inputData.length() < 2) return false;
  //Выделяем тип задания
  /*
    I - инициализационны данные
    R - поворот в ориентацию
    M - движение в точку
  */
  //Запомнили тип задания
  char taskType = inputData[0];
  //Serial.println(taskType);
  //Убрали инфу об этом
  inputData.remove(0, 1);
  //Команда инициализации в ориентацию
  if (taskType == 'I') {
    //------------------------------Распарсиваем строку по значениям------------------
    String buf = "";
    double values[3];
    int cnt = 0;
    for (int i = 0; i < inputData.length(); i++) {
      if (inputData[i] == ';') {
        values[cnt] = buf.toDouble();
        cnt++;
        buf = "";
        continue;
      }
      buf += inputData[i];
    }
    values[cnt] = buf.toDouble();
    //------------------------------------Инициализируем робота-----------------------------------
    rbt = {values[0], values[1], values[2], WHEEL_RADIUS, BASE};
    return true;
  }
  //Команда поворота в ориентацию
  if (taskType == 'R') {
    double theta = inputData.toDouble();
    //Serial.println(theta);
    //Становимся в ориентацию
    rbt.rotateToAngle(theta);
    return true;
  }
  //Движение в точку
  if (taskType == 'M') {
    //Ищем разделитель
    int c = 0;
    for (c = 0; c < inputData.length(); c++) if (inputData[c] == ';') break;
    //Выделяем переменный из строки
    String copy = inputData;
    copy.remove(0, c + 1);
    inputData.remove(c + 1);
    double x = inputData.toDouble();
    double y = copy.toDouble();
    //Serial.print(x);
    //Serial.print(" ");
    //Serial.println(y);
    //Едем в точку
    rbt.moveToPoint(x, y);
    return true;
  }
  return false;
}
