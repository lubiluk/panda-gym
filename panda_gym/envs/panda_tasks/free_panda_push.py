from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet
from panda_gym.envs.robots import FreePanda
from panda_gym.envs.tasks import Push


class FreePandaPushEnv(RobotTaskEnv):
    """Push task wih Panda robot.

    Args:
        render (bool, optional): Activate rendering. Defaults to False.
        reward_type (str, optional): "sparse" or "dense". Defaults to "sparse".
    """

    def __init__(self, render=False, reward_type="sparse", object_shape="cube"):
        self.sim = PyBullet(render=render)
        self.robot = FreePanda(self.sim, block_gripper=True, base_position=[-0.6, 0.0, 0.0])
        self.task = Push(self.sim, reward_type=reward_type, object_shape=object_shape)
        RobotTaskEnv.__init__(self)
