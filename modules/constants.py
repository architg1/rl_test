EPSILON = 1E-5


class FIELD:
    dims = (808, 448)
    half_dims = (404, 224)
    spawn_center = (354, 174)
    spawn_dims = (100, 100)


class BARRIER:
    size = 51


class TIME:
    unit = 200  # one simulation second
    step = 10
    heat = 10
    zone_reset = 6000
    match = 18000


class MATCH:
    team = dict(blue=0, red=1)


class ROBOT:
    size = 30
    dims = (30, 25)
    shield_dims = (25, 20)
    chassis_points = [(9, 25), (29, 25), (30, 24), (30, 13)]  # order is important
    armor_points = [(29, 7), (7, 24)]  # order is important
    x_accel, x_speed = 0.25, 1.5
    y_accel, y_speed = 0.2, 1.2
    rotation_accel, rotation_speed = 0.2, 1.2
    yaw_accel, yaw_speed = 1, 3
    bullet_speed = 20
    shot_cooldown = 12  # cycles
    rebound_coeff = 1


class ZONE:
    centers = [(354, 55), (214, -59), (0, 179.5)]  # F1-3
    dims = (54, 48)
    activation_dims = (36, 32)
    types = dict(hp_blue=0, hp_red=1, ammo_blue=2, ammo_red=3, no_move=4, no_shoot=5)


class TEXT:
    robot_label_offset = (-20, -45)
    time_position = (375, 3)
    stat_position = (160, 70)
    stat_increment = (125, 15)


class COLOR:
    blue = (0, 0, 210)
    red = (210, 0, 0)
    green = (0, 210, 0)
    orange = (255, 165, 0)
    gray = (112, 119, 127)
    black = (0, 0, 0)


class IMAGE:
    bullet = 'images/robot/bullet.png'
    inactive_zone = 'images/zone/inactive.png'
    activated_zone = 'images/zone/activated.png'
    zone_icon = 'images/zone/{}.png'
    blue_robot = 'images/robot/blue.png'
    red_robot = 'images/robot/red.png'
    dead_robot = 'images/robot/dead.png'
    gimbal = 'images/robot/gimbal.png'
    stats_panel = 'images/stats_panel.png'
    coords = 'images/coords.png'
    logo = 'images/logo.png'


class FILE:
    transitions = 'records/transitions.json'
