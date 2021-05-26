import numpy as np
import pygame
from typing import Union
from types import SimpleNamespace
from modules.constants import *
from modules.geometry import mirror, to_draw_coords, Line, Rectangle

chassis_outline = [Line(mirror(ROBOT.chassis_points[0], x, y), mirror(ROBOT.chassis_points[1], x, y), COLOR.green)
                   for x in [False, True] for y in [False, True]] + \
                  [Line(mirror(ROBOT.chassis_points[2], x, y), mirror(ROBOT.chassis_points[3], x, y), COLOR.green)
                   for x in [False, True] for y in [False, True]]
shield_outline = [Line(ROBOT.shield_dims, mirror(ROBOT.shield_dims, flip_y=False), COLOR.green),
                  Line(ROBOT.shield_dims, mirror(ROBOT.shield_dims, flip_x=False), COLOR.green),
                  Line(mirror(ROBOT.shield_dims), mirror(ROBOT.shield_dims, flip_y=False), COLOR.green),
                  Line(mirror(ROBOT.shield_dims), mirror(ROBOT.shield_dims, flip_x=False), COLOR.green)]
# front/right/left/back armors
armor_panels = [Line(ROBOT.armor_points[0], mirror(ROBOT.armor_points[0], flip_x=False), COLOR.orange),
                Line(ROBOT.armor_points[1], mirror(ROBOT.armor_points[1], flip_y=False), COLOR.orange),
                Line(mirror(ROBOT.armor_points[1]), mirror(ROBOT.armor_points[1], flip_x=False), COLOR.orange),
                Line(mirror(ROBOT.armor_points[0]), mirror(ROBOT.armor_points[0], flip_y=False), COLOR.orange)]


def control_response(value, command, rate, maximum):
    command = np.clip(command, -maximum, maximum)
    dv = np.clip(command - value, 0, rate)    
    if command < value:
        dv = np.clip(command - value, -rate, 0)
    value += dv
    return value

class Robot:
    cache = None

    def __init__(self, id_):
        assert 0 <= id_ <= 3, 'maximum 4 robots with ids [0-3] allowed'
        self.id_ = id_
        self.is_blue = (self.id_ % 2 == 0)

        self.center = np.array(mirror(FIELD.spawn_center, self.is_blue, self.id_ in {0, 3}), dtype='float32')
        self.rotation: int = 0 if self.is_blue else 180
        self.yaw = 0
        self.x_speed = 0
        self.y_speed = 0
        self.rotation_speed = 0
        self.yaw_speed = 0

        self.ammo = 50 if id_ in {0, 1} else 0
        self.shot_cooldown = 0
        self.heat = 0

        self.hp = 2000
        self.barrier_hits = 0
        self.bullet_hits = 0
        self.robot_hits = 0
        self.can_move = True
        self.can_shoot = True
        self.timeout = 0
        self.commands = np.zeros(5)  # x, y, rotate, yaw, shoot

    def draw(self, screen: pygame.Surface, font: pygame.font.Font, stat=False):
        if Robot.cache is None:
            Robot.cache = SimpleNamespace(
                blue_chassis_image=pygame.image.load(IMAGE.blue_robot).convert_alpha(),
                red_chassis_image=pygame.image.load(IMAGE.red_robot).convert_alpha(),
                dead_chassis_image=pygame.image.load(IMAGE.dead_robot).convert_alpha(),
                gimbal_image=pygame.image.load(IMAGE.gimbal).convert_alpha())

        chassis_image = Robot.cache.blue_chassis_image if self.is_blue else Robot.cache.red_chassis_image
        if self.hp <= 0:
            chassis_image = Robot.cache.dead_chassis_image

        chassis_image = pygame.transform.rotate(chassis_image, self.rotation)
        gimbal_image = pygame.transform.rotate(Robot.cache.gimbal_image, self.yaw + self.rotation)
        chassis_rect = chassis_image.get_rect()
        gimbal_rect = gimbal_image.get_rect()
        chassis_rect.center = gimbal_rect.center = to_draw_coords(self.center)
        screen.blit(chassis_image, chassis_rect)
        screen.blit(gimbal_image, gimbal_rect)
        label = font.render(f'{self.id_} | {self.hp:.0f}', False, COLOR.blue if self.is_blue else COLOR.red)
        screen.blit(label, to_draw_coords(self.center, offset=TEXT.robot_label_offset))

        if stat:
            for line in [*chassis_outline, *shield_outline, *armor_panels]:
                line.transform(self.center, self.rotation).draw(screen)

    def collides_chassis(self, rect: Rectangle):
        lines = [l.transform(self.center, self.rotation) for l in chassis_outline]
        if any(rect.intersects(l) for l in lines):
            self.barrier_hits += 1
            return True
        return False

    def collides_armor(self, rect: Rectangle):

        #armor_collides, armor_penalty = self._check_armor(rect)

        if self._check_armor(rect):
            self.barrier_hits += 1
            return True
        return False

    def hits_armor(self, line: Line):

        #armor_hit, armor_penalty = self._check_armor(line)

        if self._check_armor(line):
            self.bullet_hits += 1
            return True
        lines = [l.transform(self.center, self.rotation) for l in shield_outline]
        return any(line.intersects(l) for l in lines)

    def _check_armor(self, geometry: Union[Line, Rectangle]):
        lines = [l.transform(self.center, self.rotation) for l in armor_panels]
        if geometry.intersects(lines[0]):
            self.hp -= 20

            #armor_penalty = 
            #Penalty for lossing heal 

        elif geometry.intersects(lines[1]) or geometry.intersects(lines[2]):
            self.hp -= 40

            #armor_penalty = 
            #Penalty for lossing heal

        elif geometry.intersects(lines[3]):
            self.hp -= 60

            #armor_penalty = 
            #Penalty for lossing heal

        else:
            return False #,armor_penalty = 0
        return True #,armor_penalty

    def commands_to_actions(self):
        self.x_speed = control_response(self.x_speed, self.commands[0], ROBOT.x_accel, ROBOT.x_speed)
        self.y_speed = control_response(self.y_speed, self.commands[1], ROBOT.y_accel, ROBOT.y_speed)
        self.rotation_speed = control_response(self.rotation_speed, self.commands[2], ROBOT.rotation_accel, ROBOT.rotation_speed)
        self.yaw_speed = control_response(self.yaw_speed, self.commands[3], ROBOT.yaw_accel, ROBOT.yaw_speed)

    def status_dict(self):
        return {
            'x_center': self.center[0],
            'y_center': self.center[1],
            'rotation': self.rotation,
            'yaw': self.yaw,
            'x_speed': self.x_speed,
            'y_speed': self.y_speed,
            'rotation_speed': self.rotation_speed,
            'yaw_speed': self.yaw_speed,
            'ammo': self.ammo,
            'shot_cooldown': self.shot_cooldown,
            'heat': self.heat,
            'hp': self.hp,
            'barrier_hits': self.barrier_hits,
            'bullet_hits': self.bullet_hits,
            'robot_hits': self.robot_hits,
            'can_move': self.can_move,
            'can_shoot': self.can_shoot,
            'timeout': self.timeout
        }

    def status_list(self):  # for navigation transitions
        return [float(self.center[0]), float(self.center[1]), self.rotation]
