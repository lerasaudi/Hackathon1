# gym-panda

[![Build Status](https://travis-ci.org/mahyaret/gym-panda.svg?branch=master)](https://travis-ci.org/mahyaret/gym-panda)
[![Downloads](https://pepy.tech/badge/gym-panda)](https://pepy.tech/project/gym-panda)
[![PyPI version](https://badge.fury.io/py/gym-panda.svg)](https://badge.fury.io/py/gym-panda)


OpenaAI Gym Franka Emika Panda robot grasping environment implemented with PyBullet


## Links

https://www.youtube.com/watch?v=hjiQx8FC4N4

## Install

Install with `pip`:

    pip install gym-panda
    
Or, install from source:

    git clone https://github.com/mahyaret/gym-panda.git
    cd gym-panda
    pip install .

## Basic Usage

Running an environment:

```python
import gym
import gym_panda
env = gym.make('panda-v0')
env.reset()
for _ in range(100):
    env.render()
    obs, reward, done, info = env.step(
        env.action_space.sample()) # take a random action
env.close()
 ```
 


## Development

- clone the repo:
```bash
git clone https://github.com/mahyaret/gym-panda.git
cd gym-panda
```
    
- Create/activate the virtual environment:
```bash
pipenv shell --python=python3.6
```

- Install development dependencies:
```bash
pipenv install --dev
```
