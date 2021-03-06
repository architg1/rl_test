import numpy as np
from globals import *
import pygame


class record_player(object):
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.map_length, self.map_width))
        pygame.display.set_caption('RM AI Challenge Simulator')

        self.barriers_img = []
        self.barriers_rect = []
        for index, barrier in enumerate(HIGH_BARRIERS):
            image_file = f"./images/low_barrier_{'horizonal' if index < 4 else 'vertical'}.png"
            self.barriers_img.append(pygame.image.load(image_file))
            self.barriers_rect.append(self.barriers_img[-1].get_rect())
            self.barriers_rect[-1].center = find_rect_center(barrier)
        # load areas imgs
        self.areas_img = []
        self.areas_rect = []
        for oi, o in enumerate(['red', 'blue']):
            for ti, t in enumerate(['bonus', 'supply', 'start', 'start']):
                self.areas_img.append(pygame.image.load('./imgs/area_{}_{}.png'.format(t, o)))
                self.areas_rect.append(self.areas_img[-1].get_rect())
                self.areas_rect[-1].center = [self.areas[oi, ti][0:2].mean(), self.areas[oi, ti][2:4].mean()]
        # load supply head imgs
        self.head_img = [pygame.image.load('./imgs/area_head_{}.png'.format(i)) for i in ['red', 'blue']]
        self.head_rect = [self.head_img[i].get_rect() for i in range(len(self.head_img))]
        self.head_rect[0].center = [self.areas[0, 1][0:2].mean(), self.areas[0, 1][2:4].mean()]
        self.head_rect[1].center = [self.areas[1, 1][0:2].mean(), self.areas[1, 1][2:4].mean()]
        self.chassis_img = pygame.image.load('images/chassis.png')
        self.gimbal_img = pygame.image.load('images/gimbal.png')
        self.bullet_img = pygame.image.load('images/bullet.png')
        self.info_bar_img = pygame.image.load('../images/stats_panel.png')
        self.bullet_rect = self.bullet_img.get_rect()
        self.info_bar_rect = self.info_bar_img.get_rect()
        self.info_bar_rect.center = [200, self.map_width/2]
        pygame.font.init()
        self.font = pygame.font.SysFont('info', 20)
        self.clock = pygame.time.Clock()

    def play(self, file):
        self.memory = np.load(file)
        i = 0
        stop = False
        flag = 0
        while True:
            self.time = self.memory[i].time
            self.cars = self.memory[i].robots_status
            self.car_num = len(self.cars)
            self.compet_info = self.memory[i].compet_info
            self.detect = self.memory[i].detect
            self.vision = self.memory[i].vision
            self.bullets = self.memory[i].bullets
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            pressed = pygame.key.get_pressed()
            if pressed[pygame.K_TAB]: self.dev = True
            else: self.dev = False
            self.one_epoch()
            if pressed[pygame.K_SPACE] and not flag:
                flag = 50
                stop = not stop
            if flag > 0: flag -= 1
            if pressed[pygame.K_LEFT] and i > 10: i -= 10
            if pressed[pygame.K_RIGHT] and i < len(self.memory)-10: i += 10
            if i < len(self.memory)-1 and not stop: i += 1
            self.clock.tick(200)

    def one_epoch(self):
        self.screen.fill(COLOR_GRAY)
        for i in range(len(self.barriers_rect)):
            self.screen.blit(self.barriers_img[i], self.barriers_rect[i])
        for i in range(len(self.areas_rect)):
            self.screen.blit(self.areas_img[i], self.areas_rect[i])
        for i in range(len(self.bullets)):
            self.bullet_rect.center = self.bullets[i].center
            self.screen.blit(self.bullet_img, self.bullet_rect)
        for n in range(self.car_num):
            chassis_rotate = pygame.transform.rotate(self.chassis_img, -self.cars[n, 3]-90)
            gimbal_rotate = pygame.transform.rotate(self.gimbal_img, -self.cars[n, 4]-self.cars[n, 3]-90)
            chassis_rotate_rect = chassis_rotate.get_rect()
            gimbal_rotate_rect = gimbal_rotate.get_rect()
            chassis_rotate_rect.center = self.cars[n, 1:3]
            gimbal_rotate_rect.center = self.cars[n, 1:3]
            self.screen.blit(chassis_rotate, chassis_rotate_rect)
            self.screen.blit(gimbal_rotate, gimbal_rotate_rect)
            select = np.where((self.vision[n] == 1))[0]+1
            select2 = np.where((self.detect[n] == 1))[0]+1
            info = self.font.render('{} | {}: {} {}'.format(int(self.cars[n, 6]), n+1, select, select2), True, COLOR_BLUE if self.cars[n, 0] else COLOR_RED)
            self.screen.blit(info, self.cars[n, 1:3]+[-20, -60])
            info = self.font.render('{} {}'.format(int(self.cars[n, 10]), int(self.cars[n, 5])), True, COLOR_BLUE if self.cars[n, 0] else COLOR_RED)
            self.screen.blit(info, self.cars[n, 1:3]+[-20, -45])
        self.screen.blit(self.head_img[0], self.head_rect[0])
        self.screen.blit(self.head_img[1], self.head_rect[1])
        info = self.font.render('time: {}'.format(self.time), False, (0, 0, 0))
        self.screen.blit(info, (8, 8))
        if self.dev:
            for n in range(self.car_num):
                wheels = self.check_points_wheel(self.cars[n])
                for w in wheels:
                    pygame.draw.circle(self.screen, COLOR_BLUE if self.cars[n, 0] else COLOR_RED, w.astype(int), 3)
                armors = self.check_points_armor(self.cars[n])
                for a in armors:
                    pygame.draw.circle(self.screen, COLOR_BLUE if self.cars[n, 0] else COLOR_RED, a.astype(int), 3)
            self.screen.blit(self.info_bar_img, self.info_bar_rect)
            for n in range(self.car_num):
                tags = ['owner', 'x', 'y', 'angle', 'yaw', 'heat', 'hp', 'freeze_time', 'is_supply',
                        'can_shoot', 'bullet', 'stay_time', 'wheel_hit', 'armor_hit', 'car_hit']
                info = self.font.render('car {}'.format(n), False, (0, 0, 0))
                self.screen.blit(info, (8+n*100, 100))
                for i in range(self.cars[n].size):
                    info = self.font.render('{}: {}'.format(tags[i], int(self.cars[n, i])), False, (0, 0, 0))
                    self.screen.blit(info, (8+n*100, 117+i*17))
            info = self.font.render('red   supply: {}   bonus: {}   bonus_time: {}'.format(self.compet_info[0, 0], \
                                    self.compet_info[0, 1], self.compet_info[0, 3]), False, (0, 0, 0))
            self.screen.blit(info, (8, 372))
            info = self.font.render('blue   supply: {}   bonus: {}   bonus_time: {}'.format(self.compet_info[1, 0], \
                                self.compet_info[1, 1], self.compet_info[1, 3]), False, (0, 0, 0))
            self.screen.blit(info, (8, 389))
        pygame.display.flip()
