import math
"""
Класс PID-регулятора
kp, ki, kd - коэффициенты
dt - шаг по веремени для интегрирования
wantedValue - требуемое значение
curValue - текущее значение
"""
class PID:
    #Массив для зранения истории регулирования и построения графиков
    RegulationHistory = []
    def __init__(self, kp, ki, kd, dt, wantedValue, curValue):
        self.Kp = kp                    # пропорциональный коэффициент
        self.Ki = ki                    # интегральный коэффициент
        self.Kd = kd                    # дифференциальный коэффициент
        self.dt = dt                    # шаг по времени
        self.wantedValue = wantedValue  # требуемое значение
        self.curValue = curValue        # текущее значение
        self.RegulationHistory.append(curValue)
        self.P = 0
        self.I = 0
        self.D = 0
    #Вычисление управляющего воздействия по текущему значению
    def getControlValue(self, curValue):
        self.RegulationHistory.append(curValue)
        e = self.wantedValue - curValue
        self.P = self.Kp * e
        self.I = self.I + self.Ki * e * self.dt
        self.D = self.Kd * (curValue - self.curValue) / self.dt
        self.curValue = curValue
        return self.P + self.I + self.D
    def getHistory(self):
        return self.RegulationHistory