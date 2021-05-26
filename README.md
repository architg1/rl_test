# NTU RoboMaster AI Challenge Simulator

RoboMaster AI Challenge Simulator (RMAICS), is a 2D simulation environment designed for the [ICRA 2021 RoboMaster AI Challenge](https://www.robomaster.com/en-US/robo/icra).
 Its main function is to provide a simulation environment for intelligent decision-making groups to train neural networks.
 
![demo](docs/demo.gif)


## 1. Dependencies

* [numpy](https://numpy.org/)
* [pygame](https://www.pygame.org/) for visualization only
* [networkx](https://networkx.org/) for waypoint navigation


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

*This simulator is adapted from UBC Robomaster AI Challenge Simulator
