import math

###########################################################################
#                         Входные данные
# Первая строка -
#     r > 0, b > 0, 0 < h < 0.1
#         где r - радиус, b - расстояние между колесами, h - временной шаг
# Вторая строка -
#     -100 < x, y, o < 100
#         где x, y - начальное положение, o - ориентация относительно x
# Третья строка -
#     n - кол-во послед. строк
# Четвертая строка -
#     содержит кол-во пар значений углов с двигателей
#
#
##########################################################################
#                          Выходные данные
#
# Первая строка -
#     n - число результатов вычисленний
# Вторая строка -
#     x, y, o
#         (Первая строка содержит начальные условия)
#
#
##########################################################################

# Sample Input 1:
#
# 1.457470 59.241677 0.00021798140509691257
# 0.000000 0.000000 1.000000
# 10
# 0.000000 0.000000
# 0.010899 0.010899
# 0.021798 0.021798
# 0.032697 0.032697
# 0.043596 0.043596
# 0.054495 0.054495
# 0.065394 0.065394
# 0.076293 0.076293
# 0.087193 0.087193
# 0.098092 0.098092

# Sample Output 1:
#
# 10
# 0.000000 0.000000 1.000000
# -0.013367 0.008583 1.000000
# -0.026733 0.017165 1.000000
# -0.040100 0.025748 1.000000
# -0.053467 0.034331 1.000000
# -0.066834 0.042913 1.000000
# -0.080200 0.051496 1.000000
# -0.093567 0.060079 1.000000
# -0.106935 0.068662 1.000000
# -0.120302 0.077245 1.000000


class robot:
    radius: float = 0
    base: float = 0
    time_step: float = 0
    speed_left: float = 0
    speed_right: float = 0
    speed_rotate: float = 0
    speed_linear: float = 0
    x: float = 0
    y: float = 0
    rotate: float = 0
    last_angle_left: float = 0
    last_angle_right: float = 0

    def __init__(self, r: float, b: float, h: float):
        self.radius = r
        self.base = b
        self.time_step = h

    def set_position(self, x: float, y: float, rotate: float):
        self.x = x
        self.y = y
        self.rotate = rotate

    def calculate_position(self, angle_left: float, angle_right: float):
        d_left = angle_left - self.last_angle_left
        d_right = angle_right - self.last_angle_right
        self.speed_left = d_left / self.time_step
        self.speed_right = d_right / self.time_step
        self.speed_linear = (self.speed_right + self.speed_left) * self.radius / 2.0
        self.speed_rotate = (self.speed_right - self.speed_left) * self.radius / self.base

        x = self.x - self.speed_linear * self.time_step * math.sin(self.rotate)
        y = self.y + self.speed_linear * self.time_step * math.cos(self.rotate)
        a = self.rotate + self.time_step * self.speed_rotate

        self.rotate = self.speed_rotate
        self.x = x
        self.y = y
        self.rotate = a

        self.last_angle_left = angle_left
        self.last_angle_right = angle_right
        return {'x': self.x, 'y': self.y, 'rotate': self.rotate}


radius, base, h = map(float, input().split())
x, y, rotate = map(float, input().split())
n = int(input())

angles = []
for i in range(n):
    a_l, a_r = map(float, input().split())
    angles.append([a_l, a_r])

r = robot(radius, base, h)
r.set_position(x, y, rotate)

print(n)

for i in range(n):
    pos = r.calculate_position(angles[i][0], angles[i][1])
    #print(round(pos['x'], 6), round(pos['y'], 6), round(pos['rotate'], 6))
    print('{:.6f}'.format(round(pos['x'], 6)), '{:.6f}'.format(round(pos['y'], 6)), '{:.6f}'.format(round(pos['rotate'], 6)))
    #print('{:.6f}'.format(round(pos['x'], 6), '{:.6f}'.format(pos['y']), '{:.6f}'.format(pos['rotate']))
