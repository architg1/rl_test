class RL_Actor:
    def __init__(self, car_num, robot):
        self.car_num = car_num
        self.robot = robot

    def commands_from_state(self, state):
        x = y = rotate = yaw = shoot = 0
        return [x, y, rotate, yaw, shoot]

    def get_robot_id(self):
        return self.robot.id_