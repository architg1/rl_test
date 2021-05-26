# Kernel Manual

## 1. Simulation Parameters
| Item | Value | Description |
| --- | :---: | --- |
| Simulation frame frequency | 200Hz | epochs per simulation second |
| Operation frequency | 20Hz | rate at which new commands are received per simulation second |
| Scale | 10mm/px | 8×5m real-world field dimensions corresponds to 800×500px window |
| Car size | 60×50px | 600×500mm real-world |
| Maximum speed of car front/back | 1.5px/epoch | 3m/s real-world |
| Maximum speed of car left/right | 1px/epoch | 2m/s real-world |
| Maximum chassis rotation speed | 200°/s | - |
| Maximum speed of gimbal rotation | 600°/s | - |
| Bullet speed | 12.5px/epoch | 25m/s real-world |
| Barrel heat settlement frequency | 10Hz | - |

## 2. Implementation Notes

1. The speed of the chassis and pan/tilt will gradually increase to the maximum after pressing 
the corresponding command, and the speed will gradually decrease after the command is stopped.

2. The default team is divided into blue cars 1, 3 and red cars 2, 4. The game mode can be changed by setting `robot_count`.
For example, `robot_count=2` is a 1v1 confrontation and `robot_count=3` is a 2v1 confrontation.

3. `pygame` module is used only for visualization and will not be used if `render=False`.

4. In user-controlled operation `.play()`, only one car can be controlled at a time.

5. There is a rebound effect, but it does not fully comply with the laws of physics.

6. When the center of a car is within a preset area of a zone, the corresponding buff/debuff will be applied.

## 3. Game Configuration

All constants in `modules/constants.py` may be modified for a different game experience.
