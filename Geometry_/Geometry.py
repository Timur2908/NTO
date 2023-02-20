import math
from Point import Point
from Poligon import Poligon
#--------------------------Работа с точками-------------------------------------
"""
Расстояние между двумя точками
gets two Points
return: float value
"""
def distance(p1, p2):
    return (p2 - p1).len()
"""
Средняя точка (середина отрезка)
gets two Points
return: Point
"""
def middlePoint(p1, p2):
    return (p1 + p2) * 0.5
#----------------Дополнительный функции для векторов----------------------------
"""
Векторное произведение векторов
gets two Vectors
return: float value
"""
def crossProduct(v1, v2):
        return v1.x * v2.y - v1.y * v2.x
"""
Скалярное произведение векторов
gets two Vectors
return: float value
"""
def scalarProduct(v1, v2):
    return v1.x * v2.x + v1.y * v2.y
"""
Угол между векторами
gets two Vectors
return: float value
"""
def angleBetweenVectors(v1, v2):
    return math.atan2(crossProduct(v1, v2), scalarProduct(v1, v2))
"""
Полученеи вектора заданной длинны из данного
gets Vector and float(len)
return: Vector
"""
def getResizedVector(v, a):
    k = a/v.len()
    return v*k
#---------------------------------------Работа с прямоугольниками---------------------------------------
"""
Прямоугольник по центру, сторонам и направлюшему вектору
get Vector(Задаёт и точку центра и направляющий вектор одновременно), a,b(стороны)
Возвращает Poligon из 4 точек
"""
def RectangleWithCentre(v, a, b):
    #Доводим вектор до середины ближайшей стороны
    vc = getResizedVector(v, a/2.0) + v
    #Получаем 2 верхние точки прямоугольника
    p1 = getResizedVector(vc.getNormalVector(), b/2.0) + vc
    p2 = getResizedVector(vc.getNormalVector(), b/2.0) * -1 + vc
    #Зеркалим вектор центра вниз
    vc = getResizedVector(v, a/2.0)*-1 + v
    #Получаем ещё 2 точки
    p3 = getResizedVector(vc.getNormalVector(), b / 2.0) + vc
    p4 = getResizedVector(vc.getNormalVector(), b / 2.0) * -1 + vc

    rect = Poligon([p1, p2, p3, p4])
    return rect