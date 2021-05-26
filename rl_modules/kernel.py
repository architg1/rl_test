import numpy as np
import json
import pygame
from modules.objects import *
from modules.bullet import Bullet
from modules.robot import Robot
from modules.zones import Zones
from modules.geometry import distance, mirror, Line, Rectangle
from modules.constants import *

C1 = Rectangle(100, 100, -354, -174, image='images/area/blue.png')
C3 = Rectangle(100, 100, 354, 174, image='images/area/red.png')
B5 = Rectangle(35.4, 35.4, 0, 0, image='images/area/lcm.png')
B2 = Rectangle(80, 20, -214, 0, image='images/area/lhm.png')
B1 = Rectangle(100, 20, -354, 114, image='images/area/hhu.png')
B3 = Rectangle(20, 100, -244, -174, image='images/area/hvu.png')
B4 = Rectangle(100, 20, 0, 120.5, image='images/area/hhm.png')

field = Rectangle(*FIELD.dims)
coords = Rectangle(31, 31, image=IMAGE.coords)
stats_panel = Rectangle(520, 340, image=IMAGE.stats_panel)
spawn_rects = [C1, C1.mirror(flip_x=False), C3, C3.mirror(flip_x=False)]  # areas C1-4
low_barriers = [B2, B2.mirror(), B5]  # areas B2, B5, B8
high_barriers = [B1, B3, B4, B4.mirror(), B3.mirror(), B1.mirror()]  # areas B1, B3, B4, B6, B7, B9


class Kernel(object):
    def __init__(self, robot_count=4, render=False):
        self.car_count = robot_count
        self.render = render
        self.time, self.bullets, self.epoch, self.n, self.stat, self.memory, self.transitions, self.robots = None, None, None, None, None, None, None, None
        self.zones = Zones()
        self.reset()

        if render:
            import pygame
            pygame.init()
            self.screen = pygame.display.set_mode(FIELD.dims)
            pygame.display.set_caption('UBC RoboMaster AI Challenge Simulator')
            pygame.display.set_icon(pygame.image.load(IMAGE.logo))
            pygame.font.init()
            self.font = pygame.font.SysFont('mono', 12)
            self.clock = pygame.time.Clock()

    def reset(self):
        self.time = TIME.match / TIME.unit
        self.bullets = []
        self.epoch = 0
        self.n = 0
        self.end = None    # winning team
        self.stat = False
        self.memory = []
        self.transitions = []
        self.robots = [Robot(i) for i in range(self.car_count)]
        self.zones.reset()   
        return State(self.time, self.zones, self.robots)

    def play(self):
        assert self.render, 'play() requires render==True'
        state = None

        while True:
            if not self.epoch % TIME.step:
                if state is not None:
                    new_reward = [distance(r.center, mirror(FIELD.spawn_center)) for r in self.robots]
                    transition = Transition(State(self.epoch / 200, self.zones, self.robots), state, actions,
                                            [o - n for n, o in zip(new_reward, reward)][0])
                    self.transitions.append(transition)
                if self.receive_commands():
                    break
                state = State(self.epoch / 200, self.zones, self.robots)
                actions = [r.commands[:3].tolist() for r in self.robots]
                reward = [distance(r.center, mirror(FIELD.spawn_center)) for r in self.robots]
            self.one_epoch()
        self._end()

    def step(self, commands):
        for robot, command in zip(self.robots, commands):
            robot.commands = command
        for _ in range(TIME.step):
            self.one_epoch()
        return State(self.time, self.zones, self.robots)

    def one_epoch(self):
        pygame.time.wait(2)
        dead_b, dead_r = 0, 0
        for robot in self.robots:  # update robots
            if robot.hp == 0:
                dead_b += robot.is_blue
                dead_r += not robot.is_blue
                continue
            if not self.epoch % TIME.step:
                robot.commands_to_actions()
            self.zones.apply(self.robots)
            self.move_robot(robot)

            if not self.epoch % TIME.unit and robot.timeout > 0:
                robot.timeout -= 1

            robot.shot_cooldown = max(0, robot.shot_cooldown - 1)

            if not self.epoch % TIME.heat:  # Barrel Heat (Rules 4.1.2)
                if robot.heat >= 360:
                    robot.hp -= (robot.heat - 360) * 40
                    robot.heat = 360
                elif robot.heat > 240:
                    robot.hp -= (robot.heat - 240) * 4
                robot.heat -= 12 if robot.hp >= 400 else 24
            robot.heat = max(robot.heat, 0)
            robot.hp = max(robot.hp, 0)

        if not self.epoch % TIME.unit:
            self.time -= 1
        if not self.epoch % TIME.zone_reset:
            self.zones.reset()

        i = 0
        while i < len(self.bullets):  # update bullets
            if self.move_bullet(self.bullets[i]):
                del self.bullets[i]
            else:
                i += 1

        self.epoch += 1
        if self.render:
            self.draw()

    def move_robot(self, robot: Robot):
        if robot.timeout == 0:  # remove debuff if expired
            robot.can_move = True
            robot.can_shoot = True

        if robot.can_move and (robot.x_speed or robot.y_speed or robot.rotation_speed):  # move chassis
            angle = robot.rotation
            center = robot.center.copy()
            angle_rad = np.deg2rad(robot.rotation)
            robot.rotation = (robot.rotation + robot.rotation_speed) % 360
            robot.center[0] += robot.x_speed * np.cos(angle_rad) - robot.y_speed * np.sin(angle_rad)
            robot.center[1] += robot.x_speed * np.sin(angle_rad) + robot.y_speed * np.cos(angle_rad)

            if self.check_interference(robot):
                robot.rotation_speed = -robot.rotation_speed * ROBOT.rebound_coeff
                robot.x_speed *= -ROBOT.rebound_coeff
                robot.y_speed *= -ROBOT.rebound_coeff
                robot.rotation = angle
                robot.center = center

        if robot.yaw_speed:  # rotate gimbal
            robot.yaw += robot.yaw_speed
            robot.yaw = np.clip(robot.yaw, -90, 90)

        if robot.commands[4] and robot.can_shoot and (robot.ammo != 0) and (robot.shot_cooldown == 0):  # handle firing
            robot.ammo -= 1
            robot.shot_cooldown = ROBOT.shot_cooldown
            robot.heat += ROBOT.bullet_speed
            self.bullets.append(Bullet(robot.center, robot.yaw + robot.rotation, robot.id_))

    def move_bullet(self, bullet: Bullet):
        old_center = bullet.center.copy()
        bullet.step()
        trajectory = Line(old_center, bullet.center)
        
        if not field.contains(bullet.center, strict=True):
            return True
        if any(b.intersects(trajectory) for b in high_barriers):
            return True
        
        for robot in self.robots:
            if robot.id_ == bullet.owner_id:
                continue
            if robot.hits_armor(trajectory):
                return True
        return False

    def draw(self):
        assert self.render, 'draw() requires render==True'
        self.screen.fill(COLOR.gray)
        self.zones.draw(self.screen)
        for rect in [*spawn_rects, *low_barriers, *high_barriers]:
            rect.draw(self.screen)
        for robot in self.robots:
            robot.draw(self.screen, self.font, stat=self.stat)
        coords.draw(self.screen)
        for bullet in self.bullets:
            bullet.draw(self.screen)
        time_label = self.font.render(f'time: {self.time}', False, COLOR.black)
        self.screen.blit(time_label, TEXT.time_position)

        if self.stat:
            stats_panel.draw(self.screen)
            for n, robot in enumerate(self.robots):
                x_position = TEXT.stat_position[0] + n * TEXT.stat_increment[0]
                header = self.font.render(f'robot {robot.id_}', False, COLOR.blue if robot.is_blue else COLOR.red)
                self.screen.blit(header, (x_position, TEXT.stat_position[1]))
                for i, (label, value) in enumerate(robot.status_dict().items()):
                    data = self.font.render(f'{label}: {value:.1f}', False, COLOR.black)
                    self.screen.blit(data, (x_position, TEXT.stat_position[1] + (i + 1) * TEXT.stat_increment[1]))
        pygame.display.flip()

    #def opponent_hit(self):
        #oponent_status = self.robot.status_dict()
        #opponent_hit = opponent_status[13]
        #if opponent_hit != 0:
            #hit_reward = 
            #return hit_reward

    #def rl_robot_command(self,robot_rl = Robot[1],movement, fire): #movement is send to robot as a list
        # if movement[0] == 1: robot.commands[0] += 1
        # if movement[0] == 2: robot.commands[0] -= 1
        # if movement[1] == 1: robot.commands[1] += 1
        # if movement[1] == 2: robot.commands[1] -= 1
        # if movement[2] == 1: robot.commands[2] += 1
        # if movement[2] == 2: robot.commands[2] -= 1
        # if movement[3] == 1: robot.commands[3] += 1
        # if movement[3] == 2: robot.commands[3] -= 1
        # robot.commands[4] = fire

        #movement = [0,0,0,0]
        #fire = False

        

    def receive_commands(self, ignored_robot_ids=[None]):
        """ Get keyboard commands for controlling the robot.

        Parameters
        ----------
        ignored_robot_ids: list, optional
            ID of robots to ignore (starting from 0), default is None. The `commands`
            member variable for the Robots that are ignored will not be modified by
            this method, so they will not be overwritten.
        
        Returns
        -------
        stop: bool
            `True` if `pygame` receives `QUIT` event or the user presses Escape key.
        """
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            if (event.type == pygame.QUIT) or pressed[pygame.K_ESCAPE]:
                return True

        self.stat = pressed[pygame.K_TAB]
        
        if pressed[pygame.K_1]: self.n = 0
        if pressed[pygame.K_2]: self.n = 1
        if pressed[pygame.K_3]: self.n = 2
        if pressed[pygame.K_4]: self.n = 3
        
        id_to_control = min(self.n, len(self.robots) - 1)
        if id_to_control in ignored_robot_ids:
            return False

        robot = self.robots[id_to_control]
        
        robot.commands[:] = 0
        if pressed[pygame.K_UP]: robot.commands[0] += 1
        if pressed[pygame.K_DOWN]: robot.commands[0] -= 1
        if pressed[pygame.K_a]: robot.commands[1] += 1
        if pressed[pygame.K_d]: robot.commands[1] -= 1
        if pressed[pygame.K_LEFT]: robot.commands[2] += 1
        if pressed[pygame.K_RIGHT]: robot.commands[2] -= 1
        if pressed[pygame.K_q]: robot.commands[3] += 1
        if pressed[pygame.K_e]: robot.commands[3] -= 1

        robot.commands[4] = pressed[pygame.K_SPACE]
        return False

    def check_interference(self, robot: Robot, ignore_interference = False):
        if ignore_interference:
            return False
        if robot.collides_chassis(field):
            return True
        for barrier in [*low_barriers, *high_barriers]:
            if distance(robot.center, barrier.center) < ROBOT.size + BARRIER.size and \
                    (robot.collides_chassis(barrier) or robot.collides_armor(barrier)):
                return True
        for other_robot in self.robots:
            if other_robot.id_ == robot.id_:
                continue
            if distance(robot.center, other_robot.center) < 2 * ROBOT.size:
                robot.robot_hits += 1
                return True
        return False

    def get_robots_commands(self):
        robots_commands = [robot.commands.tolist() for robot in self.robots]
        return robots_commands
    
    def save_record(self, file_name):
        # np.save(file, self.memory)
        with open(file_name, 'w+') as file:
            json.dump([vars(t) for t in self.transitions], file, indent=2)
