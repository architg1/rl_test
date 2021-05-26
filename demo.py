import os
from modules.rmaics import Rmaics
from modules.actor import Actor
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

if __name__ == '__main__':
    game = Rmaics(agent_num=2, render=True)
    actor0 = Actor(1, game.game.robots[0])
    actor1 = Actor(2, game.game.robots[1])
    game.play_against_actors(actor0, actor1)