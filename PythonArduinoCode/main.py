import numpy as np
import matplotlib.pyplot as plt
import math
import time
import sys
sys.path.append("C:\\Users\\hache\\PycharmProjects\\pythonProject\\Geometry_")
from Robot import Robot
from Geometry_.Geometry import *

def main():
    rbt = Robot(x0 = 2, y0 = 5, angle0 = math.pi)
    rbt.rotateToAngle(math.pi * 2.0)
    rbt.moveToPoint(2, 5)

if __name__ == '__main__':
    main()

