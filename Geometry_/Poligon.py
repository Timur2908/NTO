import math
from Point import Point
from functools import cmp_to_key

#------------------ФУНКЦИИ КОМПАРАТОРОВ СОРТИРОВКИ ДЛЯ ТОЧЕК------------------------------------
#Функция компоратора для сортировки точек по прлярному углу
def polarSortComporator(p1, p2):
    if p1 == p2: return 0
    if p1.getPolarAngle() == p2.getPolarAngle():
        if p1.len() < p1.len():
            return -1
        return 1
    if p1.getPolarAngle() < p2.getPolarAngle():
        return -1
    return 1
#Функция для сортировки по координате
#Ищет самую левую нижнюю точку
def xyComporator(p1, p2):
    if p1 == p2: return 0
    if p1.x == p2.x:
        if p1.y < p2.y:
            return -1
        return 1
    if p1.x < p2.x:
        return -1
    return 1

"""
Класс многоугольника
Задаётся массивом точек
Автоматически сортирует по левой нижней точке и полярному углу
Может вернуть площадь
"""
class Poligon:
    def __init__(self, points):
        points = sorted(points, key=cmp_to_key(xyComporator))
        points = sorted(points, key=cmp_to_key(polarSortComporator))
        self.points = points
        self.n = len(points)
    #Прощадь многоугольника в координатах
    def getArea(self):
        S = 0
        for i in range(self.n):
            S += self.points[i].x*self.points[(i + 1)%self.n].y
        for i in range(self.n):
            S -= self.points[i].y*self.points[(i + 1)%self.n].x
        return S/2.0