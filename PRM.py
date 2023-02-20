import cv2
import numpy as np
import random
import sys
sys.path.append("C:\\Users\\hache\\PycharmProjects\\pythonProject\\Geometry_")
from Geometry_.Geometry import *
from Geometry_.Point import Point

ROBOT_RADIUS = 35       #Условный безопасный радиус робота
BOARD_EPS = 50          #Отступ от краёв изображения
NODES = 2000            #Кол-во вершин в графе
MAX_EDGE_LEN = 50       #Максимальная длинна ребра
MIN_EDGE_LEN = 20       #Минимальная длинна ребра
MAX_CNT_OF_EDGES = 30   #Максимальное количество рёбер из вершины
INF = 10**18

#Класс вершины графа
class Node:
    def __init__(self, point, neighbours):
        self.point = point
        self.neighbours = neighbours

#Проверка точки на доступность
def chekPoint(obstacleImg, point):
    #Создаём изображение маску для анализа потенциальной области(круш безосного радиуса)
    mask = np.zeros(obstacleImg.shape, dtype='uint8')
    cv2.circle(mask, point.getTuple(), ROBOT_RADIUS, (255, 255, 255), -1)
    #Оставляем только анализируемую область изображения
    maskedImg = cv2.bitwise_and(obstacleImg, obstacleImg, mask = mask)
    #Ищем все контура
    contour, hir = cv2.findContours(maskedImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Если контуров нет то всё ОК
    if len(contour) == 0: return True
    return False
#Получение координат робота
def getRobotPos(sceneImg):
    #Фильтруем робота и ищем контуры
    robotFilter = ((149, 149, 149), (151, 151, 151))

    robotImg = cv2.inRange(sceneImg, *robotFilter)

    contour, hir = cv2.findContours(robotImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Выделяем самый большой контур и моментами находим центр
    for cnt in contour:
        if cv2.contourArea(cnt) < 5: continue
        body_moments = cv2.moments(cnt)
        x_body, y_body = int(body_moments['m10'] / body_moments['m00']), int(
            body_moments['m01'] / body_moments['m00'])  # центр масс
        return Point(x_body, y_body)
#Поиск всех маркеров
def getAllMarks(img):
    points = []
    #Фильтруем маркеры и ищем контуры
    gray_filter = ((203, 203, 203), (205, 205, 205))
    gray_img = cv2.inRange(img, *gray_filter)
    contours_all, _ = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Моментами ищем центрв отсеивая мусор
    for i, cnt in enumerate(contours_all):
        if cv2.contourArea(cnt) < 5: continue
        points.append(Point((cnt[:, :, 0].max() + cnt[:, :, 0].min()) // 2, (cnt[:, :, 1].max() + cnt[:, :, 1].min()) // 2))
    return points

#генератор  PRM рафа
def generatePRMGraf(grafImg, obstacleImg, g):
    # Пошли генерить точки
    while len(g) < NODES:
        print(len(g))
        #Создали рандомную точку
        x = random.randint(BOARD_EPS, obstacleImg.shape[1] - BOARD_EPS)
        y = random.randint(BOARD_EPS, obstacleImg.shape[0] - BOARD_EPS)
        p = Point(x, y)
        #Если точка безопасна то пытаемся тобавить её в граф
        if chekPoint(obstacleImg, p):
            curNone = Node(p, [])
            for i, node in enumerate(g):
                p1 = node.point
                if distance(p, p1) > MIN_EDGE_LEN and distance(p, p1) < MAX_EDGE_LEN \
                        and len(node.neighbours) < MAX_CNT_OF_EDGES and len(curNone.neighbours) < MAX_CNT_OF_EDGES:
                    #Рисуем ребро графа
                    cv2.line(grafImg, p.getTuple(), p1.getTuple(), (255, 0, 0), 1)
                    #Добавляем рёбра
                    curNone.neighbours.append(i)
                    node.neighbours.append(len(g))
            #Добавляем точку в граф
            g.append(curNone)
    # Рисуем вершины
    for n in g:
        p = n.point
        cv2.circle(grafImg, p.getTuple(), 3, (0, 0, 255), -1)
    cv2.imshow("PRM", grafImg)
    cv2.waitKey(0)

#Алгоритм дейкстры
#Принимает граф g
#Возвращает dad
def dijkstraAlgorithm(g, start):
    D = []
    dad = []
    used = []
    for i in range(NODES):
        D.append(INF)
        used.append(0)
        dad.append(0)
    D[start] = 0
    dad[start] = -1
    for k in range(NODES - 1):
        minv = INF
        w = 0
        for i in range(NODES):
            if D[i] < minv and used[i] == 0:
                minv = D[i]
                w = i
        used[w] = 1
        for to in g[w].neighbours:
            if used[to] == 0:
                if D[w] + distance(g[w].point, g[to].point) < D[to]:
                    D[to] = D[w] + distance(g[w].point, g[to].point)
                    dad[to] = w
    return dad
#Генерирует путь между вершинами 0 и node
def generatePath(dad, node):
    path = []
    cur = node
    while cur > -1:
        path.append(cur)
        cur = dad[cur]
    path = path[::-1]
    return path

def showPath(sceneImg, path, g):
    for p in path:
        p1 = g[p].point
        cv2.circle(sceneImg, p1.getTuple(), 3, (0, 0, 255), -1)
    for i in range(len(path)-1):
        p1 = g[path[i]].point
        p2 = g[path[i+1]].point
        cv2.line(sceneImg, p1.getTuple(), p2.getTuple(), (255, 0, 0), 1)
    cv2.imshow("PRM", sceneImg)
    cv2.waitKey(0)

def main():
    sceneImg = cv2.imread('MapTask4.png')

    #Нашди центр робота
    pc = getRobotPos(sceneImg)
    #Нашли все маркеры
    marks = getAllMarks(sceneImg)

    #Создали изображения с препятствиями
    yellowBoxFilter = ((120, 205, 240),(125, 215, 250))
    blackWallFilter = ((0, 0, 0),(5, 5, 5))

    yellowBoxImg = cv2.inRange(sceneImg, *yellowBoxFilter)
    blackWallImg = cv2.inRange(sceneImg, *blackWallFilter)

    obstacleImg = yellowBoxImg + blackWallImg

    #Создали массив вершин графа
    g = []
    #Добавили вершину с роботом и маркерами
    g.append(Node(pc, []))
    for p in marks:
        g.append(Node(p, []))

    #Сгенерировали весь граф
    generatePRMGraf(sceneImg.copy(), obstacleImg, g)
    #Запускам дейкстру из узла робота
    dad = dijkstraAlgorithm(g, 0)
    #Генерим и показываем путь до маркеров
    for i in range(len(marks)):
        path = generatePath(dad, i + 1)
        showPath(sceneImg.copy(), path, g)


if __name__ == '__main__':
    main()
