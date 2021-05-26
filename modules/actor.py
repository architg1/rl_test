from numpy.lib.function_base import angle
from modules.objects import State
import numpy as np
import pathlib
import math
import random
import time

# Indices of important robot properties in state.agents[car_num]
from modules.robot import Robot
from modules.constants import ROBOT, ZONE
from modules.waypoints.navigation_graph import NavigationGraph
from modules.waypoints.navigator import Navigator
from modules.zones import Zones
from modules.geometry import distance, Rectangle, Line
from modules.kernel import B1, B2, B3, B4, B5

OWNER = 0
POS_X = 1
POS_Y = 2
ANGLE = 3
YAW = 4
BULLET_COUNT = 10

# Control constants for aim_and_shoot()
ROTATE_KP = 0.05
ROTATE_KD = 0.01
YAW_KP = 0.2
YAW_KD = 0.03


'''
Returns the determinant of the matrix made from two 2d column vectors, det((v0 v1))
'''
def det(v0, v1):
    return v0[0]*v1[1] - v1[0]*v0[1]


def floatEquals(a, b, error_threshold=0.01):
    return abs(a - b) < error_threshold


class Actor:
    def __init__(self, car_num, robot):
        self.car_num = car_num
        self.is_blue = robot.is_blue
        self.prev_commands = None
        self.prev_error_rotation = 0
        self.prev_error_yaw = 0
        self.sum_rotation_error = 0
        self.next_waypoint = None
        self.destination = None
        self.reached = True
        self.dest_node = None
        self.nav = Navigator(pathlib.Path('modules', 'waypoints', 'revised_graph.json'))
        self.avoid_nodes = []
        self.path = []

        self.robot = robot
        self.zones = None
        
        self.timer_start = False
        self.start_time = time.time()
        self.time_delay = 5


        # TODO replace dummy values with proper ones representing starting state
        self.next_state_commands = {}
        self.has_ammo = False
        self.not_shooting = True
        self.has_buff = False
        self.is_at_centre = False
        self.centre_waypoint = None
        self.is_at_spawn_zone = False
        self.spawn_waypoint = None
        self.buff_waypoint = None
        self.ammo_waypoint = None
    
    def commands_from_state(self, state):
        """Given the current state of the arena, determine what this robot should do next
        using a simple rule-based algorithm. These commands are represented as an array of numbers,
        which are interpreted by game.step()"""
        x = y = rotate = yaw = shoot = 0

        rotate, yaw, shoot = self.aim_and_shoot(state)
        x, y = self.navigate(state)
        commands = [x, y, rotate, yaw, shoot]

        self.prev_commands = commands
        
        return commands
        
    def aim_and_shoot(self, state):
        """
        Modify the rotate, yaw, and shoot commands to aim and shoot the enemy.
        """
        enemy_id = int(not self.robot.id_ % 2)
        enemy_state = state.robots_status[enemy_id]
        enemy_center = np.array([enemy_state['x_center'], enemy_state['y_center']])
        desired_angle_rotation, desired_angle_yaw = self.target_armor_coordinate(state,enemy_state)
        # convert (-180 to 180) to (0 to 360)
        if desired_angle_rotation < 0:
            desired_angle_rotation += 360
        if desired_angle_yaw < 0:
            desired_angle_yaw += 360

        error_angle_yaw = desired_angle_yaw - (self.robot.rotation + self.robot.yaw)
        error_angle_rotation = desired_angle_rotation - self.robot.rotation
        if error_angle_yaw > 180:
            error_angle_yaw -= 360
        elif error_angle_yaw < -180:
            error_angle_yaw += 360
        if error_angle_rotation > 180:
            error_angle_rotation -= 360
        elif error_angle_rotation < -180:
            error_angle_rotation += 360

        rotate = ROTATE_KP*error_angle_rotation + ROTATE_KD*(error_angle_rotation - self.prev_error_rotation)
        yaw = YAW_KP*error_angle_yaw + YAW_KD*(error_angle_yaw - self.prev_error_yaw)
        self.prev_error_yaw = error_angle_yaw
        self.prev_error_rotation = error_angle_rotation

        shoot = self.check_overheat()
        fire_line = Line(self.robot.center, enemy_center)
        for barrier in [B1, B2, B3, B4, B5, B1.mirror(), B2.mirror(), B3.mirror(), B4.mirror() ]:
            if barrier.intersects(fire_line):
                # continue aiming and do not shoot
                shoot = 0
                break
        return rotate, yaw, shoot

    def target_armor_coordinate(self, state, enemy_state):
        enemy_center_X = enemy_state['x_center']
        enemy_center_Y = enemy_state['y_center']
        enemy_rotation = enemy_state['rotation']
        angle_difference = enemy_rotation - self.robot.rotation
        if angle_difference < 0:
            angle_difference = 360 + angle_difference
        if angle_difference >= 35 and angle_difference < 145:
            enemy_armor = np.array( [enemy_center_X - 24*math.sin(enemy_rotation*math.pi/180), enemy_center_Y + 24*math.cos(enemy_rotation*math.pi/180)] )
        elif angle_difference >= 145 and angle_difference < 215:
            enemy_armor = np.array( [enemy_center_X + 29*math.cos(enemy_rotation*math.pi/180), enemy_center_Y + 29*math.sin(enemy_rotation*math.pi/180)] )
        elif angle_difference >= 215 and angle_difference < 325:
            enemy_armor = np.array( [enemy_center_X + 24*math.sin(enemy_rotation*math.pi/180), enemy_center_Y - 24*math.cos(enemy_rotation*math.pi/180)] )
        else:
            enemy_armor = np.array( [enemy_center_X - 29*math.cos(enemy_rotation*math.pi/180), enemy_center_Y - 29*math.sin(enemy_rotation*math.pi/180)] )
        desired_angle_rotation = np.rad2deg(np.arctan2(enemy_center_Y-self.robot.center[1], enemy_center_X-self.robot.center[0]))
        desired_angle_yaw = np.rad2deg(np.arctan2(enemy_armor[1]-self.robot.center[1], enemy_armor[0]-self.robot.center[0])) #+ (random.random()*10-5)
        return desired_angle_rotation, desired_angle_yaw

    def nearest_waypoint(self, pos):
        return np.argmin([np.linalg.norm(node - pos) for node in self.nav.nodes])
    def nearest_coordinate(self, pos):
        ind = self.nearest_waypoint(pos)
        return self.nav.nodes[ind]

    def check_overheat(self):
        if self.robot.heat < 220:
            return 1
        else:
            return 0

    def get_path(self, from_waypoint, to_waypoint, avoid_nodes=None):
        path = self.nav.navigate(from_waypoint, to_waypoint, avoid_nodes)
        if path:
            path = path[1:] if len(path) > 1 else path
        return self.nav.straight_line(path) if path is not None else np.array([])

    def avoid_robots(self, state):
        avoid_nodes = []
        for id, robot in enumerate(state.robots_status):
            if id != self.robot.id_:
                robot_pos = np.array((robot["x_center"], robot["y_center"]))
                avoid_nodes.append(self.nearest_waypoint(robot_pos))
        return avoid_nodes

    '''
        Sets the destination to the nearest waypoint, stored as the waypoint number
        @arg dest should be np.array([x, y])
    '''
    def set_destination(self, dest):
        '''Update the robots (x,y) destination co-ordinates'''
        self.reached = False
        print(dest)
        self.destination = dest[1:]
        self.dest_node = self.nearest_waypoint(dest[0])
        self.current_destination = self.nav.nodes[self.dest_node]
        

    def update_path(self, state):
        pos_node = self.nearest_waypoint(self.robot.center)
        self.path = self.get_path(pos_node, self.dest_node, avoid_nodes= self.avoid_robots(state))

    def navigate(self, state):
        '''Pathfind to the destination. Returns the x,y'''
        def is_goal_reached(pos, target, radius = 3):
            return (np.linalg.norm(pos - target) < radius)
        def all_goal_reached():
            return not self.destination.size
        def set_new_goal():
            print(self.robot.id_, self.destination)
            try:
                self.dest_node = self.nearest_waypoint(self.destination[0])
                self.current_destination = self.nav.nodes[self.dest_node]
                self.destination = self.destination[1:]
            except:
                self.reached = True
        self.update_path(state)
        if self.reached or not self.path.size:
            return 0, 0
        # pos_vec = np.array([np.cos(pos_angle * np.pi / 180), np.sin(pos_angle * np.pi / 180)])
        pos = np.array([self.robot.center[0], self.robot.center[1]])
        target = np.array([self.path[0][0], self.path[1][0]])
        if len(self.path[0]) == 1:
            if is_goal_reached(pos, self.current_destination, radius = 2):
                self.reached = all_goal_reached()
                if not self.reached:
                    set_new_goal()
            else:
                if not self.timer_start:
                    self.timer_start = True
                    self.start_time = time.time()
                elif time.time() - self.start_time > self.time_delay:
                    set_new_goal()
                    self.timer_start = False
        else:
            self.timer_start = False

        print(self.reached)
        print('-------')
        pos_angle = self.robot.rotation
        # Wrap angle from 0 - 360 to -180 to 180
        wrap_pos_angle = ((pos_angle + 180)%360) - 180
        # Compensate for robot position angle
        target_angle = np.degrees(np.arctan2(target[1] - pos[1], target[0] - pos[0])) - wrap_pos_angle
        print(wrap_pos_angle)
        print(pos)
        print(target)
        print(target_angle)
        max_speed = min(ROBOT.x_speed, ROBOT.y_speed)
        ux_vec, uy_vec = np.cos(np.deg2rad(target_angle)), np.sin(np.deg2rad(target_angle))
        if abs(ux_vec) >= abs(uy_vec):
            forward_movement = np.sign(ux_vec) * max_speed
            side_movement = np.sign(uy_vec) * abs(np.tan(target_angle)) * max_speed
        else:
            side_movement = np.sign(uy_vec) *  max_speed
            forward_movement = np.sign(ux_vec) * max_speed * 1/abs(np.tan(target_angle))
        print('-------')
        return forward_movement, side_movement

    def get_property(self, state, prop):
        # TODO: Update this method to use the new standard for accessing robot properties
        return state.agents[self.car_num][prop]
    
    def take_action(self, state):
        """
        Called on every frame by the Actor, it first updates the board state as stored in Actor memory
        Then it checks the current robot state to determine what the next line of action should be.
        It then accordingly modifies the next state command that is returned to kernel on every frame
        :return: The decisions to be made in the next time frame
        """
        self.update_board_zones(state.zones)
        if self.has_ammo:
            if self.is_hp_zone_active():
                if self.has_buff:
                    if self.is_at_centre:
                        self.wait()
                    else:
                        self.move_to(self.centre_waypoint)
                else:
                    self.move_to(self.buff_waypoint)
            else:
                if self.is_at_centre:
                    self.wait()
                else:
                    self.move_to(self.centre_waypoint)
        else:
            if self.is_ammo_zone_active():
                self.rush_to(self.ammo_waypoint)
            else:
                self.rush_to(self.spawn_waypoint)

        return self.next_state_commands

    def update_board_zones(self, zones):
        """
        Updates the Actor's brain with known values of the buff/debuff zones
        :return:
        """
        """
        TODO Dummy function
        Can either be called on every 60, 120 and 180 second time mark if competition time info is passed
        to the robot, or manually checking if there is a mismatch between Actor brain and board zone's as passed in
        by outpost/competition info, and updating Actor brain accordingly
        """
        self.zones = zones

    def scan_for_enemies(self):
        """
        TODO scans the nearby vicinity of the robot using LiDAR + Camera implementation and returns a list
        of enemies that can be aimed at
        :return:
        """
        return {}

    def wait(self):
        """
        TODO Scan's the robot's nearby environment for enemies to shoot, does not move
        :return:
        """
        scanned_enemies = self.scan_for_enemies()
        if scanned_enemies is not None:
            self.aim_then_shoot(scanned_enemies)
        else:
            # Does not make changes to next_state_command
            pass

    def move_to(self, waypoint):
        """
        TODO Scans the robot's nearby environment for enemies to shoot and sets the robots next_state_commands
        such that it moves towards its set waypoint
        :param waypoint:
        :return:
        """
        scanned_enemies = self.scan_for_enemies()
        if scanned_enemies is not None:
            self.aim_then_shoot(scanned_enemies)
        else:
            # TODO Insert navigation implementation
            # self.navigate(state, self.current_waypoint, self.destination)
            pass

    def rush_to(self, waypoint):
        """
        TODO Sets robot to navigate to the specified waypoint without checking for enemies
        :param waypoint:
        :return:
        """
        pass

    def is_ammo_zone_active(self):
        """
        TODO Checks if the supply zone is has not been activated yet from the current board zone info
        :return:
        """
        if self.is_blue:
            return self.zones.is_zone_active('ammo_blue')
        else:
            return self.zones.is_zone_active('ammo_red')

    def is_hp_zone_active(self):
        """
        TODO Checks if the ammo zone has not been activated yet from the current board zone info
        :return:
        """
        if self.is_blue:
            return self.zones.is_zone_active('hp_blue')
        else:
            return self.zones.is_zone_active('hp_red')

    def get_robot_id(self):
        return self.robot.id_