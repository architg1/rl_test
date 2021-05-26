import numpy as np
from modules.kernel import Kernel


class Rmaics(object):
    def __init__(self, agent_num, render=False, num_epochs=1):
        self.agent_num = agent_num
        self.render = render
        self.game = Kernel(robot_count=agent_num, render=self.render)
        self.memory = []
        self.num_epochs = num_epochs

    def reset(self):
        return self.game.reset()

    def step(self, commands):
        state = self.game.step(commands)
        self.memory.append([state, commands])
        return state, False, None

    def play(self):
        self.game.play()

    def play_against_actors(self, *actors):
        """ Play with keyboard control against actor-controlled robots.

        Parameters
        ----------
        *actors: Actor() objects
            Actor objects to be used. Which robots to be controlled by the Actors is
            automatically determined by `Actor.get_robot_id()`. The robots which are
            not controlled by the Actors are controlled using keyboard.

        Notes
        -----
        This is an indefinitely-blocking method to play with keyboard control against
        Actors. This method is different from `Kernel.play()` method as this method uses
        `get_observation()` and `get_reward()`, which can be customized. The current
        implementation does not support `Transition()` object and `Kernel.save_record()`
        yet.
        """
        assert self.render, 'play_against_actor() requires render==True'

        commands = [ 5*[0] for _ in range(self.agent_num) ]
        actor_robot_ids = [actor.get_robot_id() for actor in actors]

        for i in range(self.num_epochs):
            print(f"Epoch {i+1}")
            while self.game.time > 0:
                obs, _, _ = self.step(np.array(commands))

                if self.game.receive_commands(ignored_robot_ids=actor_robot_ids):
                    break

                commands = self.game.get_robots_commands()
                for robot_id, actor in zip(actor_robot_ids, actors):
                    commands[robot_id] = actor.commands_from_state(obs)
                if self.game.red_hp == 0 or self.game.blue_hp == 0:
                    break
            winner = "blue" if self.game.blue_hp > self.game.red_hp else "red"
        print(f"The winner is {winner}")

    def save_record(self, file):
        self.game.save_record(file)
