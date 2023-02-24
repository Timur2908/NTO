import os
import math
import time
import numpy as np

import sys
sys.path.insert(0, '../lib/')

from py_nto_task4 import Task
import cv2

import signal

run = False

###################
# класс точки
class Point:
    x: int = None   # координата X
    y: int = None   # координата Y

    # конструктор - ф-ция, вызывающаяся при создании экземпляра класса, напр: p = Point(1, 2)
    def __init__(self, x: int, y: int):
        self.x, self.y = x, y

    # ф-ция перевода в строковый вид, нужна только в целях отладки,
    # чтобы в отладчике видеть объект не в виде чего-то плохочитаемого, а в виде "(1, 2)"
    def __str__(self):
        return f'({self.x}, {self.y})'

    # ф-ция вычисления рассточния до точки
    # pt - экземпляр объекта Point
    def get_dist_evk(self, pt):
        return math.sqrt((pt.x - self.x)**2 + (pt.y - self.y)**2)

    # ф-ция поиска ближайшей точки в указанном path
    def get_nearest(self, path):
        diff = path - np.array([self.x, self.y])
        distance = np.einsum('ij,ij->i', diff, diff)    # вычисляем расстояние до каждой точки в пути, возвращается массив расстояний
        return np.argmin(distance), distance   # ищем индекс элемента в массиве с минимальным значениемб возвращаем кортеж индекса и массива расстояний


###################
# класс точки пути, расширяет класс Point
class PathPoint(Point):
    index: int = None   # индекс точки, если двумерную матрицу представить как одномерный массив

    # уровень близости точки к границе допустимой области перемещений, чем больше число, тем ближе к границе
    # тем "дороже" должен быть переход на эту точку
    level: float = None

    # расстояние до точки финиша
    dist: float = None

    # длина пройденнго пути от старта до этой точки
    path_len: float = None

    # "стоимость" перехода в эту точку
    func_value: float = None

    def __init__(self, x: int, y: int, w: int):
        super(PathPoint, self).__init__(x, y)   # вызываем конструктор из родительского класса
        self.index = PathPoint.get_pt_index(x, y, w)

    # статический метод расчета индекса точки, см. index
    @staticmethod
    def get_pt_index(x: int, y: int, w):
        return w*y + x

    # рассчет расстояния методом кварталов,
    # см. "Расстояние городских кварталов" в википедии
    def set_dist(self, ptTarget: Point):
        self.dist = abs(ptTarget.x - self.x) + abs(ptTarget.y - self.y)

    # расчет стоимости перехода
    def get_h(self):
        return self.dist + self.level*2

###################
# класс вектора
class Vector:
    pt: Point = None
    len: float = None

    def __init__(self, pt1: Point, pt2: Point):
        self.pt = Point(pt2.x - pt1.x, pt2.y - pt1.y)
        self.len = math.sqrt(self.pt.x**2 + self.pt.y**2)

    def __str__(self):
        return f'pt={self.pt}'

###################
# класс прямоугольника
class Rect:
    pt1: Point      # верхний левый угол
    pt2: Point      # нижний правый угол
    center: Point

    def __init__(self, pt1: Point, pt2: Point):
        self.pt1, self.pt2 = pt1, pt2
        self.center = self.get_center()

    def __str__(self):
        return f'{self.pt1}-{self.pt2}'

    # вычисляем центр
    def get_center(self):
        return Point(int((self.pt1.x + self.pt2.x)/2), int((self.pt1.y + self.pt2.y)/2))

    # проверка входит ли точка в прямоугольник
    def point_in(self, pt: Point):
        return self.pt1.x <= pt.x <= self.pt2.x and self.pt1.y <= pt.y <= self.pt2.y

###################
# класс зоны (старта или финиша)
class Zone:
    bbox: Rect      # граница зоны
    is_start: bool = None   # флаг старта или финиша

    def __init__(self, bbox: Rect):
        self.bbox = bbox

###################
# класс робота и его параметры
class Robo:
    MAX_SPEED = 10                      # макс. скорость
    PID_P = 0.7 #0.0003                 # коэф P ПИД-алгоритма
    PID_I = 0.01 #0.000001 #0.0000001   # коэф I ПИД-алгоритма
    PID_D = 0.3 #0.001 #0.007           # коэф D ПИД-алгоритма
    I_SUM_LIMIT: float = 10000          # лимит I сумматора
    iSum: float = 0
    prevErr = 0

    body_range_hsv = ((0, 0, 149),  (255, 255, 150))        # мин и макс HSV значения поиска тела робота
    robo_weels_hsv = ((0, 1, 255), (255, 255, 255))         # мин и макс HSV значения поиска колес робота

    center: Point = None    # центр робота
    pointer: Point = None   # условный "перед" робота
    vector: Vector = None   # вектор направления робота
    angle: float = 0        # угол поворота робота

    base = 50
    r = 5
    wl = 0
    wr = 0
    angle_l = 0
    angle_r = 0
    speed_linear = 0
    speed_rotate = 0

    body_ellipse = None
    wheel_ellipses = [None, None, None]

    # поиск робота в кадре
    def find(self, hsvScene):
        self.get_body(hsvScene)
        self.get_wheels(hsvScene)

    # поиск тела робота в кадре
    def get_body(self, scene):
        ellipses = self.get_ellipses(scene, self.body_range_hsv, 1)
        if len(ellipses) == 1:
            self.body_ellipse = ellipses[0]
            self.center = Point(int(self.body_ellipse[0][0]), int(self.body_ellipse[0][1]))

    # поиск колес робота в кадре
    def get_wheels(self, scene):
        ellipses = self.get_ellipses(scene, self.robo_weels_hsv, 3)
        if len(ellipses) == 3:
            self.wheel_ellipses = ellipses
            self.find_angle()

    # поиск эллипса по заданным параметрам
    def get_ellipses(self, hsvScene, ranges, count):
        maskWheels = cv2.inRange(hsvScene, ranges[0], ranges[1])
        contours, hierarchy = cv2.findContours(maskWheels, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ellipses = []
        contours = list(filter(lambda c: len(c) > 10, contours))
        if len(contours) == count:
            for c in contours:
                ellipses.append(cv2.fitEllipse(c))
        return ellipses;

    # расчет угла поворота робота
    def find_angle(self):
        # строим треугольник по колесам,
        # длинная сторона проходит через центр робота,
        # короткие стороны проходят от боковых колес к заднему черному прямоугольнику
        # исходя из этогои находим положение робота и его направление
        a2 = self.get_length2(self.wheel_ellipses[0][0], self.wheel_ellipses[1][0])
        b2 = self.get_length2(self.wheel_ellipses[0][0], self.wheel_ellipses[2][0])
        c2 = self.get_length2(self.wheel_ellipses[1][0], self.wheel_ellipses[2][0])
        a = self.get_length(a2)
        b = self.get_length(b2)
        c = self.get_length(c2)
        a_cos = ( b2 + c2 - a2 ) / (2*b*c)
        b_cos = ( a2 + c2 - b2 ) / (2*a*c)
        c_cos = ( a2 + b2 - c2 ) / (2*a*b)

        p = self.wheel_ellipses[2][0]
        cos_min = a_cos
        if b_cos < cos_min:
            p = self.wheel_ellipses[1][0]
            cos_min = b_cos
        if c_cos < cos_min:
            p = self.wheel_ellipses[0][0]
            cos_min = c_cos
        v = (self.center.x - p[0], self.center.y - p[1])
        self.pointer = Point(int(self.center.x+v[0]), int(self.center.y+v[1]))
        self.angle = math.atan2(-v[1], v[0])
        self.vector = Vector(self.center, self.pointer)

    # поиск квадрата расстояния
    def get_length2(self, a, b):
        return (a[0] - b[0])**2 + (a[1] - b[1])**2

    # поиск расстояния (корень из квадрата)
    def get_length(self, a2):
        return math.sqrt(a2)

    iSum: float = 0
    iSumLimit: float = 10000
    prevErr = 0

    # сброс параметров PID
    def resetPid(self):
        self.iSum = 0
        self.prevErr = 0

    # вычисление параметров PID
    def get_pid(self, err: float, nearCenter: PathPoint, nearPointer: PathPoint, pathVector: Vector):
        # print(f'err={err}; dir={dir}; ang={ang}; center=({center.x},{center.y}); pointer=({pointer.x},{pointer.y})')
        robotVector = Vector(self.center, self.pointer)
        centerOffsetVector = Vector(nearCenter, self.center)
        pointerOffsetVector = Vector(nearCenter, self.center)

        offsetDir = (centerOffsetVector.pt.x * pathVector.pt.y - centerOffsetVector.pt.y * pathVector.pt.x)
        robotDir = (robotVector.pt.x * pathVector.pt.y - robotVector.pt.y * pathVector.pt.x)
        # if offsetDir > 0 and robotDir > 0:
        if offsetDir > 0:
            err = err
        # elif offsetDir < 0 and robotDir < 0:
        elif offsetDir < 0:
            err = -err

        # print(f'err={err}; offsetDir={offsetDir}; robotDir={robotDir}')

        up = self.PID_P * err
        self.iSum = self.iSum + err

        if self.iSum > self.I_SUM_LIMIT:
            self.iSum = self.I_SUM_LIMIT
        elif self.iSum < -self.I_SUM_LIMIT:
            self.iSum = -self.I_SUM_LIMIT

        ui = self.PID_I * self.iSum
        ud = self.PID_D * (err - self.prevErr)
        self.prevErr = err

        v = up + ui + ud
        pid = (self.MAX_SPEED + v, self.MAX_SPEED - v)
        # print(f'PID: v={v} : {pid}')
        return pid

###################
# класс метки (куда ехать)
class RoboTask:
    # исходное описание
    descr: str = None
    # имя маркера
    marker: str = None
    # направление - как нужно остановиться в маркере
    direction: str = None
    # направление - в радианах
    final_angle: float = None

    def __init__(self, taskDescr: str):
        self.descr = taskDescr
        info = taskDescr.split('_')
        if len(info) > 0:
            self.marker = info[0]
        if len(info) > 1:
            self.direction = info[1]
            if self.direction == 'S':
                self.final_angle = math.atan2(-1, 0) #-90
            elif self.direction == 'SW':
                self.final_angle = math.atan2(-1, -1) #-135
            elif self.direction == 'W':
                self.final_angle = math.atan2(0, -1) #180
            elif self.direction == 'NW':
                self.final_angle = math.atan2(1, -1) #135
            elif self.direction == 'N':
                self.final_angle = math.atan2(1, 0) #90
            elif self.direction == 'NE':
                self.final_angle = math.atan2(1, 1) #45
            elif self.direction == 'E':
                self.final_angle = math.atan2(0, 1) #0
            elif self.direction == 'SE':
                self.final_angle = math.atan2(-1, 1) #-45

###################
# класс элемента очереди с приоритетами
class QueueItem:
    priority: float     # приоритет элемента для сортировки
    item: Point         # точка, которую нужно обработать
    next = None         # следующий элемент очереди (QueueItem)
    prev = None         # предыдущий элемент очереди (QueueItem)

    def __init__(self, priority: float, pt: PathPoint):
        self.priority = priority
        self.item = pt

###################
# класс очереди с приоритетами
# вставляет новый элемент в очередь в порядке сортировки в зависимости от значения priority
class SortedQueue:
    head: QueueItem = None  # голова очереди
    tail: QueueItem = None  # хвост очереди
    size: int = None        # длина очереди
    itemsSet: set = None    # сет (множество) всех индексов точек в очереди
                            # - чтобы быстро проверить, есть в очереди точка или нет

    def __init__(self):
        self.size = 0
        self.itemsSet = set()

    # добавить точку в очередь
    def put(self, priority: float, pt: Point):
        self.size += 1
        self.itemsSet.add(pt.index)

        newItem = QueueItem(priority, pt)
        if self.head is None:   # если очередь пустая, доавляем в начало
            self.head = newItem
            self.tail = self.head
        else:
            # иначе ищем куда вставить, в зависимости от значения priority
            curr: QueueItem = self.head
            while curr is not None and curr.priority < priority:
                curr = curr.next

            if curr is None:
                self.tail.next = newItem
                newItem.prev = self.tail
                self.tail = newItem
            else:
                prev: QueueItem = curr.prev
                if prev is None:
                    newItem.next = self.head
                    self.head.prev = newItem
                    self.head = newItem
                else:
                    prev.next = newItem
                    newItem.prev = prev
                    newItem.next = curr
                    curr.prev = newItem
        return newItem

    # взять следующую точку из начала очереди и удалить эту точку из очереди
    def get(self):
        self.size -= 1
        item: QueueItem = self.head
        self.head = self.head.next
        if self.head is not None:
            self.head.prev = None
        else:
            self.head = None
            self.tail = None

        self.itemsSet.remove(item.item.index)
        return item.item

    # удалить элемент из очереди
    def remove(self, item: QueueItem):
        prev: QueueItem = item.prev
        next: QueueItem = item.next

        if prev is None:
            self.head = next
            if self.head is not None:
                self.head.prev = None
        else:
            prev.next = next

        if next is None:
            self.tail = prev
            if self.tail is not None:
                self.tail.next = None
        else:
            next.prev = prev

        self.itemsSet.remove(item.item.index)
        return item

    # проверить есть ли такая точка в очереди
    def contains(self, pt: Point):
        return pt.index in self.itemsSet

    # проверить пустая ли очередь
    def empty(self) -> bool:
        return self.head is None

###################
# класс сцены
class Scene:
    startZone: Zone = None              # зона старта
    finishZone: Zone = None             # зона финиша
    robo: Robo = None                   # робот
    markers: {} = None  #{name: Rect}   # словарь (dict) маркеров
    tasks: [RoboTask] = None            # список заданий для робота (куда ехать)
    maskDist = None                     # маска расстояний до границ
    maxLevel = None                     # значение максимального расстояния

    # строим сцену из первого кадра и кадра с зонами
    def build(self, imgSceneZones, imgScene):
        self.maskDist = None
        # подготавливаем входящие кадры для последующей обработки
        hsvSceneZones = cv2.cvtColor(imgSceneZones, cv2.COLOR_BGR2HSV_FULL)
        hsvScene = cv2.cvtColor(imgScene, cv2.COLOR_BGR2HSV_FULL)

        # ищем маркеры
        self.get_markers(hsvScene)
        # ищем нашего робота
        self.robo = Robo()
        self.robo.find(hsvScene)
        # ищем зоны
        self.get_zones(hsvSceneZones)
        # ищем допустимые границы перемещения робота, чтобы не задеть стенки лабиринта
        self.prepare_borders(hsvScene)

        del hsvScene
        del hsvSceneZones

    # поиск робота на текущем кадре
    def find_robo(self, imgScene):
        hsvScene = cv2.cvtColor(imgScene, cv2.COLOR_BGR2HSV_FULL)
        robo = Robo()
        robo.find(hsvScene)
        del hsvScene
        return robo

    # ищем зоны старта и финиша
    def get_zones(self, hsvSceneZones):
        min_zone_hsv = (0, 10, 255)
        max_zone_hsv = (255, 255, 255)
        maskZones = cv2.inRange(hsvSceneZones, min_zone_hsv, max_zone_hsv)
        contoursZones, hierarchy = cv2.findContours(maskZones, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cntr in contoursZones:
            bb = cv2.boundingRect(cntr)
            rc = Rect(Point(int(bb[0]), int(bb[1])), Point(int(bb[0]+bb[2]), int(bb[1]+bb[3])))
            if rc.point_in(self.robo.center):
                self.startZone = Zone(rc)
            else:
                self.finishZone = Zone(rc)

        del contoursZones
        del maskZones

    # ищем маркеры
    def get_markers(self, hsvScene):
        self.markers = {}
        min_markers_hsv = (0, 0, 203)
        max_markers_hsv = (255, 255, 204)
        maskMarkers = cv2.inRange(hsvScene, min_markers_hsv, max_markers_hsv)
        contoursMarkers, hierarchy = cv2.findContours(maskMarkers, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursMarkers = list(filter(lambda c: len(c) > 3, contoursMarkers))

        # именуем маркеры по порядку в формате P1, P2, ....
        # и определяем границы маркера
        for i, cntr in enumerate(contoursMarkers):
            name = f'P{i+1}'
            bb = cv2.boundingRect(cntr)
            self.markers[name] = Rect(Point(int(bb[0]), int(bb[1])), Point(int(bb[0]+bb[2]), int(bb[1]+bb[3])))

        del contoursMarkers
        del maskMarkers

    # ищем допустимые границы перемещения робота, чтобы не задеть стенки лабиринта
    def prepare_borders(self, hsvScene):
        min_borders_hsv = (0, 0, 90)
        max_borders_hsv = (25, 255, 255)
        maskBorders = cv2.inRange(hsvScene, min_borders_hsv, max_borders_hsv)
        #contoursBorders, hierarchy = cv2.findContours(maskBorders, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # вычисление расстояний до границ, ф-ция OpenCV
        self.maskDist = cv2.distanceTransform(maskBorders, cv2.DIST_L2, 3)
        # обнуляем значения, где расстояние меньше 25, т.к. туда заезжать нельзя, превращаем в черный (0)
        self.maskDist[self.maskDist < 25] = 0
        # нормализуем значения в диапазон 0..255
        self.maskDist = cv2.normalize(self.maskDist, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # ищем максимальный уровень + 1
        self.maxLevel = self.maskDist.max() + 1

    # парсинг входящего списка тасок
    def parseTasks(self, tasksStr: str):
        tasksDescr = tasksStr.split()
        self.tasks = []
        for t in tasksDescr:
            self.tasks.append(RoboTask(t))

    # посик пути по алгоритму А*, почти копипаста из википедии
    def get_path(self, ptStart: Point, ptFinish: Point):
        h, w = self.maskDist.shape

        ptFrom = PathPoint(ptStart.x, ptStart.y, w)     # старт
        ptTo = PathPoint(ptFinish.x, ptFinish.y, w)     # финиш

        ptFrom.set_dist(ptTo)
        ptFrom.level = self.maxLevel - self.maskDist[ptFrom.y][ptFrom.x]
        ptFrom.path_len = 0

        ptTo.level = self.maxLevel - self.maskDist[ptTo.y][ptTo.x]

        queue = SortedQueue()   # очередь обработки точек
        qi = queue.put(0, ptFrom)   # помещаем первую точку - точку старта

        cameFromMap = {}        # карта откуда пришли в эту точку - {"текущая точка": "откуда пришли в эту точку"}
        cameFromMap[ptFrom.index] = None

        # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        gScore = {}
        gScore[ptFrom.index] = 0

        # For node n, fScore[n] := gScore[n] + h(n).
        # fScore[n] represents our current best guess as to
        # how cheap a path could be from start to finish if it goes through n.
        fScore = {}
        fScore[ptFrom.index] = ptFrom.get_h()

        visited = set()

        pointsMap = {}
        pointsMap[ptFrom.index] = ptFrom

        iter = 0
        # 1 - стоимость перехода по горизонтали/вертикали
        # 1.4 - стоимость перехода по диагонали
        moves = [
            [0, -1, 1],
            [1, -1, 1.4],
            [1, 0, 1],
            [1, 1, 1.4],
            [0, 1, 1],
            [-1, 1, 1.4],
            [-1, 0, 1],
            [-1, -1, 1.4]
        ]
        # пока очередь точек не пустая, обрабатываем имеющиеся точки
        while not queue.empty():
            iter += 1

            current: PathPoint = queue.get()
            visited.add(current.index)
            pointsMap[current.index] = current

            if current.index == ptTo.index:
                # достигли финальной точки, прерываем цикл
                break

            # проверяем возможные варианты перехода из текущей точки в новую
            for move in moves:
                x = current.x + move[0]
                y = current.y + move[1]

                if x >= 0 and x < w and y >= 0 and y < h and self.maskDist[y][x] > 0:
                    ptNext = PathPoint(x, y, w)

                    if ptNext.index in visited:
                        continue

                    ptNext.set_dist(ptTo)
                    ptNext.level = self.maxLevel - self.maskDist[ptNext.y][ptNext.x]

                    # d(current,neighbor) is the weight of the edge from current to neighbor
                    # tentative_gScore is the distance from start to the neighbor through current
                    # d(current,neighbor) === move[2]
                    tentative_gScore = gScore[current.index] + move[2]
                    if ptNext.index not in gScore or tentative_gScore < gScore[ptNext.index]:
                        # This path to neighbor is better than any previous one. Record it!
                        cameFromMap[ptNext.index] = current.index
                        gScore[ptNext.index] = tentative_gScore
                        fScore[ptNext.index] = tentative_gScore + ptNext.get_h()
                        if not queue.contains(ptNext):
                            queue.put(fScore[ptNext.index], ptNext)


        fullPath: [PathPoint] = []
        fullPath.append(ptTo)
        # print(f'ptTo.index={ptTo.index}')
        # if ptTo.index in cameFromMap:
        #     print(f'cameFromMap[ptTo.index]={cameFromMap[ptTo.index]}')
        # else:
        #     print(f'cameFromMap[ptTo.index]=missed')
        idx = cameFromMap[ptTo.index]

        # строим итоговый путь перехода по точкам
        while idx is not None:
            pt = pointsMap[idx]
            fullPath.append(pt)
            idx = cameFromMap[idx]

        fullPath.reverse()
        npPath = np.array(list(map(lambda p: [p.x, p.y], fullPath)))
        return fullPath, npPath


###################
# класс текущего состояния алгоритма
class State:
    GET_NEXT_TASK = 0           # проверить следущее задание
    BUILD_PATH = 1              # построить путь до след точки
    POSITION_ON_PATH = 2        # повернуться по направлению пути
    GO_TO_MARKER = 3            # идем к след точке
    POSITION_ON_MARKER = 4      # повернуться на маркере
    NOTIFY_COMPLETE_MARKER = 5  # отправить сообщение о достижении маркера
    GO_TO_EXIT = 6              # идем на выход
    FINISH = 10                 # закончили

# вычислить вектор направления пути
def getPathVector(path:[PathPoint], idxFrom: int) -> Vector:
    # тупо берем 20-ю точку пути и строим вектор от текущего положения до этой 20-й точки
    checkLen = 20
    idxTo = min(len(path)-1, idxFrom + checkLen)
    idxFrom = idxTo - checkLen
    return Vector(path[idxFrom], path[idxTo])


###################
# ОСНОВНОЙ АЛГОРИТМ

run = False

def SIGINT_handler(signum, frame):
    global run
    # print('SIGINT')
    run = False

## Здесь должно работать ваше решение
def solve():
    global run

    # наша сцена
    scene = Scene()

    ## Запуск задания и таймера (внутри задания)
    task = Task()
    task.start()
    run = True

    ## Загружаем изображение из задачи
    # сцена отправляется с определенной частотй
    # для этого смотри в документацию к задаче
    mapWithZones = task.getTaskMapWithZones()
    sceneImg     = task.getTaskScene()

    # строим сцену
    scene.build(mapWithZones, sceneImg)

    # отправляем список найденных маркеров
    task.sendMessage(str(len(scene.markers)))
    for mName in scene.markers:
        m: Rect = scene.markers[mName]
        descr = mName + f' {m.center.x} {m.center.y}'
        task.sendMessage(descr)
        # print('marker = ' + descr)

    #tasksCount = task.getTask()

    # берем текущую таску
    tasksPointsStr: str = task.getTask()
    # tasksPointsStr: str = 'P17_NE'
    # tasksPointsStr: str = 'P17_NE P18_NW P19_SW P20_SE'
    # tasksPointsStr: str = 'P10_SW P3_S P14_N'
    # print('tasks: ' + tasksPointsStr)

    # парсим полученные данные
    scene.parseTasks(tasksPointsStr)


    # cv2.imshow('some', mapWithZones)
    # cv2.waitKey(0)

    # msg_out = 'some message'
    # task.sendMessage(msg_out)
    #msg_in = task.getTask()

    robots = task.getRobots()
    # print('Number of robots:', len(robots))
    # print("Robot 0")
    # print("Left Motor angle:", robots[0].state.leftMotorAngle)
    # print("Right Motor angle:", robots[0].state.rightMotorAngle)
    # print("Left Motor speed:", robots[0].state.leftMotorSpeed)
    # print("Right Motor speed:", robots[0].state.rightMotorSpeed)

    # список маркеров для посещения
    tasksLists: [RoboTask] = []
    tasksLists.extend(scene.tasks)

    currentStatus = State.GET_NEXT_TASK
    currentTask = None
    prevMarker: Rect = None
    currentMarker: Rect = None
    taskPath: [PathPoint] = None
    npPath: np.array = None
    pathDirection = None
    lastNearestIdx = 0

    vl = 0
    vr = 0
    vv = [vl, vr]

    while (run):
        # текущая сцена
        sceneImg = task.getTaskScene()

        # ищем нашего робота
        robo = scene.find_robo(sceneImg)

        if currentStatus == State.GET_NEXT_TASK:    # проверить следущее задание
            prevMarker = currentMarker
            if len(tasksLists) > 0:
                currentTask = tasksLists.pop(0)
                # print('currentTask.marker = ' + currentTask.marker)
            else:
                currentTask = None

            currentStatus = State.BUILD_PATH    # строить путь
            if currentTask is not None:
                currentMarker = scene.markers[currentTask.marker]

        elif currentStatus == State.BUILD_PATH: # строить путь
            lastNearestIdx = 0
            if currentTask is None: # go to Exit
                taskPath, npPath = scene.get_path(prevMarker.center, scene.finishZone.bbox.center)
            else:
                taskPath, npPath = scene.get_path(robo.center, currentMarker.center)

            pathVector: Vector = getPathVector(taskPath, 0)
            pathDirection = math.atan2(-pathVector.pt.y, pathVector.pt.x)
            currentStatus = State.POSITION_ON_PATH

        elif currentStatus == State.POSITION_ON_PATH:   # повернуться по направлению пути
            diff = pathDirection - robo.angle
            if abs(diff) < 0.1:
                if currentTask is None:
                    currentStatus = State.GO_TO_EXIT
                else:
                    currentStatus = State.GO_TO_MARKER
                vl = 0
                vr = 0
                robo.resetPid()
            else:
                diff = pathDirection - robo.angle
                if 0 <= abs(diff) <= 0.3:
                    a = 5*diff
                else:
                    a = 10 * diff
                vl = -a
                vr = a
                # print(f'align a={a}')
            vv = [vl, vr]
            robots[0].setMotorVoltage(vv)

        elif currentStatus == State.GO_TO_MARKER or currentStatus == State.GO_TO_EXIT:  # идем к маркеру или на выход
            targetRect: Rect = None
            if currentStatus == State.GO_TO_EXIT:
                targetRect = scene.finishZone.bbox
            else:
                targetRect = currentMarker

            if targetRect.point_in(robo.center):
                vv = [0, 0]
                if currentStatus == State.GO_TO_EXIT:
                    currentStatus = State.FINISH
                    run = False
                else:
                    currentStatus = State.POSITION_ON_MARKER
            else:
                # idx, distance = robo.center.get_nearest(npPath)
                idxPnt, pointerDistance = robo.pointer.get_nearest(npPath)
                idxCntr, robotDistance = robo.center.get_nearest(npPath)
                if idxPnt > lastNearestIdx:
                    lastNearestIdx = idxPnt
                else:
                    idxPnt = lastNearestIdx

                pathVector: Vector = getPathVector(taskPath, idxPnt)
                err = math.sqrt(pointerDistance[idxPnt])
                vl, vr = robo.get_pid(err, taskPath[idxCntr], taskPath[idxPnt], pathVector)
                vv = [vl, vr]
            robots[0].setMotorVoltage(vv)

        elif currentStatus == State.POSITION_ON_MARKER:     # повернуться на маркере
            if currentTask.final_angle is None:
                currentStatus = State.NOTIFY_COMPLETE_MARKER    # известить о достижении маркера
            else:
                diff = currentTask.final_angle - robo.angle
                if abs(diff) < 0.09:
                    currentStatus = State.NOTIFY_COMPLETE_MARKER    # известить о достижении маркера
                    vl = 0
                    vr = 0
                    robo.resetPid()
                else:
                    diff = currentTask.final_angle - robo.angle
                    if 0 <= abs(diff) <= 0.5:
                        a = 5*diff
                    else:
                        a = 10*diff
                    vl = -a
                    vr = a
            vv = [vl, vr]
            robots[0].setMotorVoltage(vv)

        elif currentStatus == State.NOTIFY_COMPLETE_MARKER: # известить о достижении маркера
            msg = currentTask.descr + ' OK'
            task.sendMessage(msg)
            currentStatus = State.GET_NEXT_TASK     # взять следующую точку для посещения

        # elif currentStatus == State.GO_TO_EXIT:
        #     a = 0
        elif currentStatus == State.FINISH:     # закончили, выход
            run = False
            break


        # if taskPath is not None:
        #     ptPrev = None
        #     for pt in taskPath:
        #         if ptPrev is not None:
        #             cv2.line(sceneImg, (ptPrev.x, ptPrev.y), (pt.x, pt.y), (255,0,0), 1)
        #         ptPrev = pt
        #
        # cv2.line(sceneImg, (robo.center.x, robo.center.y), (robo.pointer.x, robo.pointer.y), (0,0,255), 1)


        # for m in scene.markers:
        #     cv2.rectangle(img=sceneImg, pt1=(scene.markers[m].pt1.x, scene.markers[m].pt1.y),
        #                   pt2=(scene.markers[m].pt2.x, scene.markers[m].pt2.y), color=(0, 255, 255), thickness=0,
        #                   lineType=cv2.LINE_AA)
        #     cv2.putText(img=sceneImg, text=m, org=(scene.markers[m].center.x, scene.markers[m].center.y),
        #                 fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 255))

        # cv2.imshow('Scene', sceneImg)
        # cv2.waitKey(10) # мс
        time.sleep(0.002)

    task.stop()

if __name__ == '__main__':

    signal.signal(signal.SIGINT, SIGINT_handler)
    solve()
