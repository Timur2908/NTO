from typing import Dict, Tuple, Union, Any

import cv2
import math
import numpy as np

# ---------фильтры--------------
body_filter = ((150, 100, 100), (250, 180, 180))
wheels_filter = ((103, 231, 253), (107, 235, 255))
head_filter = ((30, 30, 30), (80, 60, 60))
gray_filter = ((202, 202, 202), (206, 206, 206))
wall_filter = ((120, 207, 240), (124, 211, 244))
black_filter = ((0, 0, 0), (2, 2, 2))
red_filter = ((0, 0, 253), (0, 0, 255))


# считаем моменты изображения и вычислияем центр
def get_center_coords(cnt: np.ndarray) -> (int, int):
    """
    :param cnt: контуры маски объекта
    :return: координаты центра
    """
    moments = cv2.moments(cnt)
    x = int(moments['m10'] / moments['m00'])
    y = int(moments['m01'] / moments['m00'])
    return x, y


# считаем моменты изображения и вычислияем угол поворота
def get_angle_by_moments(cnt: np.ndarray) -> float:
    """
    для объекта без центральной симметрии
    :param cnt: контуры маски объекта
    :return: угол поворота объекта
    """
    moments = cv2.moments(cnt)
    huMoments = cv2.HuMoments(moments)
    for i in range(0, 7):
        huMoments[i] = -1 * math.copysign(1.0, huMoments[i]) * math.log10(abs(huMoments[i]))

    x, y = moments['m10'] / moments['m00'], moments['m01'] / moments['m00']  # центр масс

    hu = {'h20': moments['m20'] / moments['m00'] - (x ** 2),
          'h11': moments['m11'] / moments['m00'] - (x * y),
          'h02': moments['m02'] / moments['m00'] - (y ** 2)}

    angle = 0.5 * math.atan(hu['h11'] / (hu['h20'] - hu['h02'])) + (hu['h20'] < hu['h02']) * math.pi / 2
    return angle


# Нормализация значений угла
def angle_norm(angle: float) -> float:
    if round(angle, 6) <= 0.0:
        angle += math.pi
    else:
        angle -= math.pi
    return angle


# Находим угол поворота
def get_angle(x1, y1, x2, y2):
    """
    :param x1: x головы
    :param y1: y головы
    :param x2: x тела
    :param y2: y тела
    :return:
    """
    x, y = x1 - x2, y2 - y1  # вектор направления
    angle = angle_norm(math.atan2(x, y))
    return angle


# todo переделать в класс, добавить фильтров и т.д.
def image_processing(img: np.ndarray) -> np.ndarray:
    erode_img = cv2.erode(img, np.ones((5, 5)))  # сужение (убрать лишние пиксели)
    erode_dilate_img = cv2.dilate(erode_img, np.ones((7, 7)))  # расширение (вернуть к исходному состоянию)
    return erode_dilate_img


def get_robot_pos(img: np.ndarray) -> (int, int, float):
    """
    :param img:
    :return: координаты центра, угол поворота
    """
    # получаем контуры тела роббота
    body_img = cv2.inRange(img, *body_filter)
    body_img = image_processing(body_img)
    body_cnt, _ = cv2.findContours(body_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # получаем контуры головы робота
    head_img = cv2.inRange(img, *head_filter)
    head_cnt, _ = cv2.findContours(head_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # координаты центра тела и головы робота
    x_body, y_body = get_center_coords(body_cnt[0])
    x_head, y_head = get_center_coords(head_cnt[0])

    # угол поворота робота
    angle = get_angle(x_head, y_head, x_body, y_body)

    return x_body, y_body, angle


# получаем координаты центров всех маркеров
def get_marks(img: np.ndarray) -> dict[int, tuple[int, int]]:
    """
    :param img:
    :return: словарь с координатами центров
    """
    points = {}
    gray_img = cv2.inRange(img, *gray_filter)
    contours_all, _ = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # ходим по всем контурам и вычисляем координаты центра
    # todo переделать через моменты
    for i, cnt in enumerate(contours_all):
        points[i] = ((cnt[:, :, 0].max() + cnt[:, :, 0].min()) // 2,
                     (cnt[:, :, 1].max() + cnt[:, :, 1].min()) // 2)
    return points


def get_obstacles_mask(img: np.array):
    """
    :param img:
    :return:
    """
    wall_img = cv2.inRange(img, *wall_filter)
    black_img = cv2.inRange(img, *black_filter)
    obstacle_img = wall_img + black_img
    kernel = np.ones((50, 50))
    mask = cv2.bitwise_not(cv2.filter2D(obstacle_img, -1, kernel))
    return mask
