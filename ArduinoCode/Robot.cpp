#include "Robot.h"

Robot::Robot(double x0, double y0, double angle0, double wheelRadius, double base) {
  this->x = x0;
  this->y = y0;
  this->angle = angle0;
  this->angle0 = angle0;
  this->path = 0;
  this->base = base;
  this->wheelRadius = wheelRadius;
  this->LeftMotor = Motor('L', MOTOR_L_PIN_A, MOTOR_L_PIN_B, ENCODER_L_PIN_A, ENCODER_L_PIN_B);
  this->RightMotor = Motor('R', MOTOR_R_PIN_A, MOTOR_R_PIN_B, ENCODER_R_PIN_A, ENCODER_R_PIN_B);
}

Robot::Robot() {
  this->x = 0;
  this->y = 0;
  this->angle = 0;
  this->angle0 = 0;
  this->path = 0;
  this->base = 0;
  this->wheelRadius = 0;
  this->LeftMotor = Motor('L', MOTOR_L_PIN_A, MOTOR_L_PIN_B, ENCODER_L_PIN_A, ENCODER_L_PIN_B);
  this->RightMotor = Motor('R', MOTOR_R_PIN_A, MOTOR_R_PIN_B, ENCODER_R_PIN_A, ENCODER_R_PIN_B);
}

void Robot::TickL(){
  LeftMotor.Tick();
}
void Robot::TickR(){
  RightMotor.Tick();
}

void Robot::drive(double Lvoltage, double Rvoltage) {
  LeftMotor.runMotor(Lvoltage);
  RightMotor.runMotor(Rvoltage);
  delay(MOTOR_WORK_DELAY);
  Update();
}
void Robot::Stop() {
  drive(0, 0);
  delay(MOTOR_WORK_DELAY);
  Update();
}
//Обновление информации и роботе
void Robot::Update() {
  //Обновляем информацию о двигателях
  LeftMotor.Update();
  RightMotor.Update();

  //Запоминаем старые значения координат, пути и ориентации
  double lastPath = path;
  double  lastX = x;
  double lastY = y;

  //----------------------------Вычисляем новые значения---------------------------------
  // вычисление пройденного пути
  path = (LeftMotor.getTheta() + RightMotor.getTheta()) * wheelRadius / 2.0;
  // вычисление курсового угла
  angle = angle0 + (RightMotor.getTheta() - LeftMotor.getTheta()) * wheelRadius / base;
  angleNormal(angle);
  x = lastX + (path - lastPath) * cos(angle);  // вычисление координаты X
  y = lastY + (path - lastPath) * sin(angle); // вычисление координаты Y
}
//Поворот в указанную ориентацию при помощи ПИД - регулятора
//Возвращает массив с историей регулирования для построения графика
void Robot::rotateToAngle(double purposeAngle) {
  angleNormal(purposeAngle);
  double Kp = 20.0;
  double Ki = 0.0;
  double Kd = 0.0;
  PID pid(Kp, Ki, Kd, purposeAngle, angle);
  for (int i = 0; i < PID_ITERATIONS; i++) {
    double voltage = pid.getControlValue(angle);
    drive(-voltage, voltage);
  }
  Stop();
}

//Движение в указанную точку функцией Ляпунова
void Robot::moveToPoint(double targetX, double targetY) {
  for (int i = 0; i < MOVE_ITERATIONS; i++) {
    double distanceError = dist(x, y, targetX, targetY);
    if (distanceError < MOVE_ERROR)  break;
    double  bearing = atan2(targetY - y, targetX - x);
    double courseAngle = bearing - angle;
    angleNormal180(courseAngle);
    double control_l = 30.0 * cos(courseAngle) * tanh(distanceError) - 10.0 * courseAngle;
    double control_r = 30.0 * cos(courseAngle) * tanh(distanceError) + 10.0 * courseAngle;
    drive(control_l, control_r);
  }
  Stop();
}

//Расстояние между 2 точками
double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// Нормализация значений угла в промежутке от 0 до 360
void angleNormal360(double& angle) {
  angle = fmod(angle, PI * 2.0);
  if (angle < 0.0) {
    angle += PI * 2.0;
  }
}
// Нормализация угла в пределах[-180, 180]
void angleNormal180(double& angle) {
  if (angle == PI) return;
  angle = fmod(angle + PI, PI * 2.0);
  if (angle < 0.0) {
    angle += PI * 2.0;
  }
  angle -= PI;
}
//Нормализация углы тупым остатком от деления - эффективно для устронения проблемы перхода PID регулятора
void angleNormal(double& angle) {
  angle = fmod(angle, PI * 4.0);
}
