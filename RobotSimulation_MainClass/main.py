import numpy as np
import matplotlib.pyplot as plt
import math
import sys
sys.path.append("C:\\Users\\hache\\PycharmProjects\\pythonProject\\Geometry_")
from Robot import Robot
from Geometry_.Geometry import *

#Визуализация изменения угла поврота при вращении на месте
def rotationVisualisation(rbt):
    angles = []
    PI = []
    PI_ = []
    for i in range(100):
        rbt.drive(-5, 5)
        angles.append(rbt.angle)
        PI.append(math.pi)
        PI_.append(-math.pi)
    plt.plot(angles, 'b-')
    plt.plot(PI, 'r--')
    plt.plot(PI_, 'r--')
    #plt.ylim([-4, 4])
    plt.grid()
    plt.show()
#Визуализация поворота в указанную ориентацию
def PIDVisualisation(rbt, value):
    PIDHistory = rbt.rotateToAngle(value)
    values = np.zeros(len(PIDHistory))
    values[:] = value
    plt.plot(PIDHistory, 'b-')
    plt.plot(values, 'r--')
    plt.grid()
    plt.show()
#Визуализация здижения в точку
def moveToPointVisualisation(rbt, x, y):
    plt.scatter(rbt.x, rbt.y)
    mx, my = rbt.moveToPoint(x, y)
    plt.plot(mx, my)
    plt.scatter(x, y)
    plt.grid()
    plt.show()

def main():
    rbt = Robot(x0 = 0, y0 = 0, angle0 = math.pi, wheelRadius = 0.3, base = 0.2)
    PIDVisualisation(rbt, math.pi / 2.0)

if __name__ == '__main__':
    main()

