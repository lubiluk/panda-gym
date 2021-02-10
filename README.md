# panda_gym

## Installation

If you want to use it with `gazebo11`, you should first install it.

Clone the repo in your catkin workspace

    cd <catkin_ws>/src
    git clone https://github.com/qgallouedec/panda_gym

Clone the dependencies

    wstool init
    wstool merge panda_gym/dependencies.rosinstall
    wstool up

Install the system dependencies

    cd .. && rosdep install -y --from-path src --ignore-src

If `gazebo11` is installed, it will raise `ERROR: the following rosdeps failed to install apt: command [sudo -H apt-get install -y libgazebo9-dev] failed`. You can safely ignore it.

Build and source the environment:

    catkin build
    source devel/setup.bash

## Usage

Launch the simulated panda robot:

    roslaunch panda_gazebo panda_world.launch start_moveit:=false

<!-- Then, launch `moveit`:

    roslaunch panda_sim_moveit sim_move_group.launch -->

Then, you can try to interract like in `basic.py`

## Notes

### `start_moveit:=false`

The `panda_world.launch` file launches `moveit` and a `joint_trajectory_server_emulator` node simultaneously. But `joint_trajectory_server_emulator` takes a long time to start, and `moveit` needs this node to have started in order to work. 
The loading time is too long, so `moveit` goes into error.

    Action client not connected: position_joint_trajectory_controller/follow_joint_trajectory


Running `moveit` separately solves this problem.
