from modules.constants import ZONE
from modules.geometry import mirror

class State(object):
    def __init__(self, time, zones, robots):
        self.time = time
        self.robots_status = [r.status_dict() for r in robots]
        self.zone_info = {}
        for zone_type in ZONE.types.keys():
            id = zones.get_index_by_type(zone_type)
            activation_status = zones.is_zone_active(zone_type)
            self.zone_info[zone_type] = dict(coord = mirror(ZONE.centers[id//2]) if id%2 else ZONE.centers[id//2], status = activation_status)


class Transition(object):
    def __init__(self, old_state: State, new_state: State, action, reward):
        self.old_state = vars(old_state)
        self.new_state = vars(new_state)
        self.action = action
        self.reward = reward


class Record(object):
    def __init__(self, time, robots, bullets):
        self.time = time
        self.robots = robots
        self.bullets = bullets
