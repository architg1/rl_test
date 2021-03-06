# Rmaics Manual

## 1. Training Neural Networks

Customize the `get_observation` and `get_reward` functions per your needs.


## 2. Running and testing

The usage of `rmaics` is similar to [gym](https://github.com/openai/gym) of [openai](https://openai.com/)

### 1. Initialization

Import, declare and initialize. When `render` is `True`,
the screen will be displayed, and keyboard operations can be used, but not vice versa

```python
from modules.rmaics import Rmaics
car_num = 4
game = rmaics(agent_num=car_num, render=True)
game.reset()
```

### 2. Game stepping

Stepping can be performed with the `step` function from `kernel.py`. The actions
to perform on each step are passed as a (4 * 8) array. This is described further in
[params.md](./params.md#actions).
The return values for `step` are defined in []()

```python
# action format (int, np.array): [['x', 'y', 'rotate', 'yaw', 'shoot', 'supply', 'shoot_mode', 'auto_aim'], ...]
# action.shape = (car_num, 8)
actions = [[1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]]
obs, reward, done, _ = game.step(actions)
```

### 3. Use keyboard control

The game can be controlled manually via keyboard when `render` is set to `True` and then calling the following 
function. Click the close icon to stop the game.

```python
game.play()
```

### 4. Saving memory

The memory here refers to the memory of the `kernel` object instantiated, and will be stored in a 
[.npy](https://stackoverflow.com/questions/4090080/what-is-the-way-data-is-stored-in-npy ) file, which 
stores the information needed to reproduce the game. For how to replay the memory, 
please refer to [record_player](./record_player.md).

Saving can be achieved by:

```python
game.save_record(file='./records/record.npy')
```