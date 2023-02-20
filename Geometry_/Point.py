import math
"""
Класс точки
Инициализируется координатами
Параллельно определяет радиус вектор в данную точку => поддержка всех операций над векторами
"""
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    #------------------------------ОПЕРАТОРЫ-----------------------------------------------
    # Получение текстовое интерпритации точки
    def __str__(self):
        return '{' + str(self.x) + ', ' + str(self.y) + '}'
    #Равенство точек
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    # -------------------Базовые Арифметические функции----------------------------
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Point(self.x * other, self.y * other)
    # ------------------------------------------------------------------------------
    # Полярный угол точки
    def getPolarAngle(self):
        return math.atan2(self.y, self.x)
    # Получение вектора перпендикулярного данному с той же длинной
    def getNormalVector(self):
        return Point(-self.y, self.x)
    #Получение точки в виде кортежа координат
    def getTuple(self):
        return (int(self.x), int(self.y))
    # Длинна вектора
    def len(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

