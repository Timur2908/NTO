#include"Motor.h"

Motor::Motor() {
  motorType = '-';
}
Motor::Motor(char type, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB) {
  motorType = type;
  motorDriver = {motorPinA, motorPinB};
  motorEncoder = {encoderPinA, encoderPinB};
}
void Motor::runMotor(int voltage) {
  if (voltage > MAX_MOTOR_VOLTAGE) voltage = MAX_MOTOR_VOLTAGE;
  if (voltage < -MAX_MOTOR_VOLTAGE) voltage = -MAX_MOTOR_VOLTAGE;
  motorDriver.setMotorVoltage(voltage);
}
void Motor::Update() {
  motorEncoder.Update();
}
void Motor::Tick(){
  motorEncoder.Tick();
}
double Motor::getTheta() {
  return motorEncoder.Theta / 180.0 * PI;
}
double Motor::getSpeed() {
  return motorEncoder.Speed;
}
