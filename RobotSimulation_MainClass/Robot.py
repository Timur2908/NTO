from Motor import Motor
from PID import PID
import math
"""
Класс робота - основной класс
При инициализации передать праметры робота:
    Радиус колёс
    Расстояние между колесaми - база
Также необходимо передавать начальные координаты и ориентацию робота
"""
PID_ITERATIONS = 200        # Колическо итераций для ПИД-регулятора
MOVE_ITERATIONS = 6000       # Количество итераций для функции Ляпунова
MOVE_ERROR = 0.0001              # Предельная ошибка регулирования фуекции Ляпунова
SIMULATION_DT = 0.01
class Robot:
    #Функция инициализации
    def __init__(self, x0, y0, angle0, wheelRadius, base):
        self.x = x0
        self.y = y0
        self.angle = angle0
        self.angle0 = angle0
        self.path = 0
        self.base = base
        self.wheeRadius = wheelRadius
        self.LeftMotor = Motor('L', SIMULATION_DT)
        self.RightMotor = Motor('R', SIMULATION_DT)

    #Движение робота
    def drive(self, Lvoltage, Rvoltage):
        self.LeftMotor.runMotor(Lvoltage)
        self.RightMotor.runMotor(Rvoltage)
        self.update()

    #Обновление информации и роботе
    def update(self):
        #Оновляем информацию о двигателях
        self.LeftMotor.update()
        self.RightMotor.update()

        #Запоминаем старые значения координат, пути и ориентации
        lastPath = self.path
        lastX = self.x
        lastY = self.y

        #Вычисляем новые значения
        self.path = (self.LeftMotor.theta + self.RightMotor.theta) * self.wheeRadius / 2  # вычисление пройденного пути
        self.angle = self.angle0 + (self.RightMotor.theta - self.LeftMotor.theta) * self.wheeRadius / self.base  # вычисление курсового угла
        self.angle = angleNormal180(self.angle)
        self.x = lastX + (self.path - lastPath) * math.cos(self.angle)  # вычисление координаты X
        self.y = lastY + (self.path - lastPath) * math.sin(self.angle)  # вычисление координаты Y

        #print('Information about robot`s odomeria was successfully updated!')

    #Поворот в указанную ориентацию при помощи ПИД-регулятора
    #Возвращает массив с историей регулирования для построения графика
    def rotateToAngle(self, purposeAngle):
        purposeAngle = angleNormal180(purposeAngle)
        Kp = 5
        Ki = 0
        Kd = 0
        pid = PID(Kp, Ki, Kd, SIMULATION_DT, purposeAngle, self.angle)
        for i in range(PID_ITERATIONS):
            voltage = pid.getControlValue(self.angle)
            self.drive(-voltage, voltage)
        return pid.getHistory()

    #Движение в указанную точку функцией Ляпунова
    def moveToPoint(self, targetX, targetY):
        moveHistoryX = []
        moveHistoryY = []
        for i in range(MOVE_ITERATIONS):
            moveHistoryX.append(self.x)
            moveHistoryY.append(self.y)
            distanceError = dist([self.x, self.y], [targetX, targetY])
            if distanceError < MOVE_ERROR:
                break
            bearing = -math.atan2(self.y - targetY, targetX - self.x)
            bearing = angleNormal180(bearing)
            courseAngle = bearing - self.angle
            control_l = 8 * math.cos(courseAngle) * math.tanh(distanceError) - 5 * courseAngle
            control_r = 8 * math.cos(courseAngle) * math.tanh(distanceError) + 5 * courseAngle
            self.drive(control_l, control_r)
        return moveHistoryX, moveHistoryY

#Расстояние между 2 точками
def dist(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
# Нормализация значений угла в промежутке от 0 до 360
def angleNormal360(angle):
    angle %= math.pi * 2.0
    if angle < 0: angle += math.pi * 2.0
    return angle
#Нормализация угла в пределах [-180, 180]
def angleNormal180(angle):
    if angle == math.pi:
        return angle
    angle = (angle + math.pi) % (math.pi * 2.0)
    if (angle < 0.0):
        angle += math.pi * 2.0
    return angle - math.pi