import numpy as np
import matplotlib.pyplot as plt
import math as m
from scipy.integrate import odeint
T_m = 0.08                                   # электромеханическая постоянная двигателя
k_e = 0.5                                    # конструктивная постоянная
def motor(state,t,u):
    speed, angle = state                                # обозначение состояния двигателя как скорости и угла поворота
    state_dt = [-speed/T_m + u / (T_m * k_e), speed ]    # задание производной состояния двигателя
    return state_dt

class motorSimulator:
    def __init__(self, speed, angle, dt):
        self.init = [speed, angle]
        self.dt = dt

    #Подача напряжения на двигатель
    def setMotorPower(self, voltage):
        y = odeint(motor, self.init, [self.dt, self.dt*2], args=(voltage,))  # вычисляем значения скорости и угла поворота
        # odeint возвращает две пары значений угла и скорости для моментов времени t[i-1] и t[i]
        self.init = y[1, :]  # запоминаем последние значения состояния двигателя

    #Возвращает состояние двигателя в виде пары
    #angle, speed
    def getMotorState(self):
        return self.init[1], self.init[0]