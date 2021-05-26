# MLDA RMUA 2022 Reinforcement Learning Challenge 
**Before you start with this challenge please FORK this repo and implement your solution in the forked repo.**
## 1. Rationale for This Task
According to DJI, the Robomaster robot needs to be able to navigate in the battle arena autonomously and fire on target if opponent is spotted. Therefore, we believe Reinforcement Learning is best suitted for this task. Thus, we are recruiting students who are good at designing Reinforcement Learning Models.

## 2. Your Task
Your task, should you choose to accept it, is to create an agent throught Reinforcement Learning. Design an agent able to navigate in this Pygame arena autonomously and preferably eliminate the opponent(s) at the end of this simulation.

Please define your agent in the `rl_agent_actor.py`: [rl_agent_actor.py](rl_agent_actor.py)

## 3. Setting Up Environment
All the required libraries are found in `requirements.txt`

To install required libraries:
`
pip install -r requirements.txt
`

## 4. On File Modifications
You are free to modify *any files* in this repository, including the environment variables (HP of Robot, Movement Speed, etc). However, your submission will be tested with the default environment against the default, unmodified rule-based actor in `actor.py`.  

---
*This simulator is adapted from [UBC Robomaster AI Challenge Simulator](https://github.com/ubcrm/sim-2d)*

# RoboMaster AI Challenge Simulator

RoboMaster AI Challenge Simulator (RMAICS), is a 2D simulation environment designed for the [ICRA 2021 RoboMaster AI Challenge](https://www.robomaster.com/en-US/robo/icra).
 Its main function is to provide a simulation environment for intelligent decision-making groups to train neural networks.
 
![demo](docs/demo.gif)


## 1. Dependencies

* [numpy](https://numpy.org/)
* [pygame](https://www.pygame.org/) for visualization only
* [networkx](https://networkx.org/) for waypoint navigation
* opencv-python
* scipy


## 2. User Guides

The simulation consists of two levels:
> The high-level training interface class: `rmaics`  
> The low-level implementation class: `kernel`

See the following manuals for further information.
* High-level training interface in `rmaics.py`: [rmaics.md](docs/rmaics.md)
* Low-level implementation in `kernel.py`: [kernel.md](docs/kernel.md)
* Instructions for `record_player.py`: [record_player.md](docs/record_player.md)
* Instructions for controls: [operation.md](docs/operation.md)
* Parameter format: [params.md](docs/params.md)
* Further improvements to `kernel.py`: [future.md](docs/future.md)
