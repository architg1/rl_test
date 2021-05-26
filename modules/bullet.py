import pygame
import numpy as np
from types import SimpleNamespace
from modules.constants import FIELD, ROBOT, IMAGE
from modules.geometry import to_draw_coords


class Bullet(object):
    cache = None

    def __init__(self, center, angle, owner_id):
        self.center = center.copy()
        self.angle = angle
        self.owner_id = owner_id

    def step(self):
        self.center[0] += ROBOT.bullet_speed * np.cos(np.deg2rad(self.angle))
        self.center[1] += ROBOT.bullet_speed * np.sin(np.deg2rad(self.angle))

    def draw(self, screen: pygame.Surface):
        if Bullet.cache is None:
            Bullet.cache = SimpleNamespace(image=pygame.image.load(IMAGE.bullet))
        rect = Bullet.cache.image.get_rect()
        rect.center = to_draw_coords(self.center)
        screen.blit(Bullet.cache.image, rect)
