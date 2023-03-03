from Motor import Motor
from PID import PID
import math
import time
from ArduinoControl import ArduinoControl
"""
Класс робота - основной класс
При инициализации передать праметры робота:
    Радиус колёс
    Расстояние между колесaми - база
Также необходимо передавать начальные координаты и ориентацию робота
"""
ARDUINO_PORT = 'COM3'       #Порт подключения платы
CONNECTION_SPEED = 9600     #Скорость передачи данных последовательного прота
CONNECTION_TIMEOUT = 50     #Время ожидания ответа от контроллера при передаче данных
class Robot:
    #Функция инициализации
    def __init__(self, x0, y0, angle0):
        self.controller = ArduinoControl(ARDUINO_PORT, CONNECTION_SPEED, CONNECTION_TIMEOUT)
        self.controller.run()
        #Инициализаруем робота
        initComand = "I" + str(x0) + ";" + str(y0) + ";" + str(angle0)
        self.controller.write(initComand)
        # Ждём пока выполнится операция
        while self.controller.readLine() != "DONE": pass

    #Поворот в указанную ориентацию при помощи ПИД-регулятора
    def rotateToAngle(self, purposeAngle):
        #Отправляем команду на поворот
        comand = "R" + str(purposeAngle)
        self.controller.write(comand)
        #Ждём пока выполнится операция
        while self.controller.readLine() != "DONE": pass

    #Движение в указанную точку функцией Ляпунова
    def moveToPoint(self, targetX, targetY):
        comand = "M" + str(targetX) + ";" + str(targetY)
        self.controller.write(comand)
        # Ждём пока выполнится операция
        while self.controller.readLine() != "DONE": pass