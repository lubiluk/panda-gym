# panda-gym

[![PyPI version](https://img.shields.io/pypi/v/panda-gym.svg?logo=pypi&logoColor=FFE873)](https://pypi.org/project/panda-gym/)
[![PyPI downloads](https://static.pepy.tech/badge/panda-gym)](https://pypistats.org/packages/panda-gym) 
[![GitHub](https://img.shields.io/github/license/quenting44/panda-gym.svg)](LICENSE.txt)
[![Build Status](https://travis-ci.com/quenting44/panda-gym.svg?branch=master)](https://travis-ci.com/quenting44/panda-gym) 

OpenaAI Gym Franka Emika Panda robot environment based on Gazebo and ROS.

![](https://raw.githubusercontent.com/quenting44/panda-gym/master/docs/demo.gif) TODO: update the gif demo

## Installation

Using PyPI: TODO: test if it still works

    pip install panda-gym

From source:

    cd ~/catkin_ws/src/
    git clone https://github.com/quenting44/panda_description.git
    git clone https://github.com/quenting44/panda-gym.git
    cd ~/catkin_ws/
    rosdep install --from-paths src --ignore-src
    pip3 install -r src/panda-gym/requirements.txt
    catkin build

## Usage

```python
import gym
import panda_gym

env = gym.make('PandaReach-v0', render=True)

obs = env.reset()
done = False
while not done:
    action = env.action_space.sample() # random action
    obs, reward, done, info = env.step(action)

env.close()
```

## Environments

Following environments are widely inspired from [OpenAI Fetch environments](https://openai.com/blog/ingredients-for-robotics-research/). Video [here](https://youtu.be/TbISn3yu0CM)

`PandaReach-v0`: Panda has to move its end-effector to the desired goal position.
![](https://raw.githubusercontent.com/quenting44/panda-gym/master/docs/Reach.png) TODO: update this

`PandaSlide-v0`: Panda has to hit a puck across a long table such that it slides and comes to rest on the desired goal.
![](https://raw.githubusercontent.com/quenting44/panda-gym/master/docs/Slide.png) TODO: update this

`PandaPush-v0`: Panda has to move a box by pushing it until it reaches a desired goal position.
![](https://raw.githubusercontent.com/quenting44/panda-gym/master/docs/Push.png) TODO: update this

`PandaPickAndPlace-v0`: Panda has to pick up a box from a table using its gripper and move it to a desired goal above the table.
![](https://raw.githubusercontent.com/quenting44/panda-gym/master/docs/PickAndPlace.png) TODO: update this
