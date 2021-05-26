import pygame
import numpy as np
from types import SimpleNamespace
from modules.constants import *


def distance(p1, p2):
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def ccw(p1, p2, p3):  # check if three points p1, p2, p3 are oriented CCW
    return (p2[1] - p1[1]) * (p3[0] - p1[0]) <= (p3[1] - p1[1]) * (p2[0] - p1[0])


def transform(point, shift=(0, 0), angle=0):
    angle = -np.deg2rad(angle)
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.matmul(point, rotation_matrix) + shift


def mirror(point, flip_x=True, flip_y=True):
    return (-1 if flip_x else 1) * point[0], (-1 if flip_y else 1) * point[1]


def to_draw_coords(p, offset=(0, 0)):
    return round(FIELD.half_dims[0] + p[0] - 0.5 + offset[0]), round(FIELD.half_dims[1] - p[1] - 0.5 + offset[1])


def to_center_coords(p):
    return p[0] - FIELD.half_dims[0] + 0.5, -p[1] + FIELD.half_dims[1] + 0.5


class Line:
    def __init__(self, point1, point2, color=COLOR.black):
        self.p1 = point1
        self.p2 = point2
        self.color = color
        self.cache = None

    def mirror(self, flip_x=True, flip_y=True):
        return Line(mirror(self.p1, flip_x=flip_x, flip_y=flip_y), mirror(self.p2, flip_x=flip_x, flip_y=flip_y), color=self.color)

    def transform(self, shift=(0, 0), angle=0):
        angle = -np.deg2rad(angle)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        p1 = np.matmul(self.p1, rotation_matrix) + shift
        p2 = np.matmul(self.p2, rotation_matrix) + shift
        return Line(p1, p2, color=self.color)

    def draw(self, screen):
        if self.cache is None:
            self.cache = SimpleNamespace(start=to_draw_coords(self.p1), end=to_draw_coords(self.p2))
        pygame.draw.line(screen, self.color, self.cache.start, self.cache.end)

    def intersects(self, line: 'Line'):
        return ccw(self.p1, line.p1, line.p2) != ccw(self.p2, line.p1, line.p2) and \
               ccw(self.p1, self.p2, line.p1) != ccw(self.p1, self.p2, line.p2)

    def get_side(self, point):  # 1/-1/0 for a point on right/left/top of line
        side = (self.p2[1] - self.p1[1]) * (point[0] - self.p1[0]) - (self.p2[0] - self.p1[0]) * (point[1] - self.p1[1])
        if side > 0:
            return 1
        elif side < 0:
            return -1
        else:
            return 0


class Rectangle:
    def __init__(self, width, height, x_center=0, y_center=0, image=None, padding=0):
        width, height = width + 2 * padding, height + 2 * padding
        self.padding = padding
        self.dimensions = (width, height)
        self.center = (x_center, y_center)
        self.left = x_center - width / 2
        self.right = x_center + width / 2
        self.bottom = y_center - height / 2
        self.top = y_center + height / 2
        self.image = image
        self.cache = None

    def mirror(self, flip_x=True, flip_y=True):
        return Rectangle(self.dimensions[0] - 2 * self.padding, self.dimensions[1] - 2 * self.padding,
                         *mirror(self.center, flip_x=flip_x, flip_y=flip_y), image=self.image, padding=self.padding)

    def pygame_rect(self):
        return pygame.Rect(*to_draw_coords((self.left, self.top), offset=(self.padding, self.padding)),
                           self.dimensions[0] - 2 * self.padding, self.dimensions[1] - 2 * self.padding)

    def draw(self, screen: pygame.Surface):
        if self.cache is None:
            assert self.image is not None, 'need an image file to draw'
            self.cache = SimpleNamespace(rect=self.pygame_rect(), image=pygame.image.load(self.image))
        screen.blit(self.cache.image, self.cache.rect)

    def contains(self, point, strict=False):
        if strict:
            return self.left <= point[0] <= self.right and self.bottom <= point[1] <= self.top
        return self.left < point[0] < self.right and self.bottom < point[1] < self.top

    def intersects(self, line: Line):
        if any([line.p1[0] < self.left and line.p2[0] < self.left, line.p1[0] > self.right and line.p2[0] > self.right,
                line.p1[1] < self.bottom and line.p2[1] < self.bottom, line.p1[1] > self.top and line.p2[1] > self.top]):
            return False
        if all([self.left < line.p1[0] < self.right, self.left < line.p2[0] < self.right,
                self.bottom < line.p1[1] < self.top, self.bottom < line.p2[1] < self.top]):
            return False
        sides = [line.get_side((x, y)) for x in (self.left, self.right) for y in (self.bottom, self.top)]
        if all(s < 0 for s in sides) or all(s > 0 for s in sides):
            return False
        return True
