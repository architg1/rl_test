import os
import numpy as np
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

from modules.rmaics import Rmaics
from modules.actor import Actor

#from rl_agent_actor import RL_Actor

if __name__ == '__main__':
    game = Rmaics(agent_num=2, render=True)

    # actor1 = RL_Actor(2, game.game.robots[1])

    actor0 = Actor(1, game.game.robots[0])
    actor0.set_destination(np.array([[354, 174], [-100, 100], [354, -174]]))

    game.play_against_actors(actor0)
