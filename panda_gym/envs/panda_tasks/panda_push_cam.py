import pybullet as p
import numpy as np
from gym import spaces

from ..panda_tasks.panda_push import PandaPushEnv
from panda_gym.envs.robots import Panda
from panda_gym.envs.tasks import Push
from panda_gym.pybullet import PyBullet


class PandaPushCamEnv(PandaPushEnv):
    def __init__(self, render=False, reward_type="sparse"):
        self._view_mat1 = p.computeViewMatrix(
            # [0.7, 0, 0.7], [0.45, 0, 0.45], [0.0,0.0,1.0]
            [0.0, 0.8, 0.8],
            [0.0, 0, 0.0],
            [0.0, 0.0, 1.0],
        )
        self._view_mat2 = p.computeViewMatrix(
            # [0.7, 0, 0.7], [0.45, 0, 0.45], [0.0,0.0,1.0]
            [0.0, -0.8, 0.8],
            [0.0, 0, 0.0],
            [0.0, 0.0, 1.0],
        )
        self._projection_mat = p.computeProjectionMatrixFOV(43.3, 1.0, 0.2, 2.0)

        self.sim = PyBullet(render=render)
        self.robot = Panda(self.sim, block_gripper=True, base_position=[-0.6, 0.0, 0.0])
        self.task = Push(self.sim, reward_type=reward_type)

        obs = self.reset()
        achieved_goal_shape = obs["achieved_goal"].shape
        desired_goal_shape = obs["achieved_goal"].shape
        self.observation_space = spaces.Dict(
            dict(
                observation=spaces.Dict(
                    dict(
                        robot_state=spaces.Box(
                            -np.inf,
                            np.inf,
                            shape=obs["observation"]["robot_state"].shape,
                        ),
                        camera1=spaces.Box(
                            -np.inf, np.inf, shape=obs["observation"]["camera1"].shape
                        ),
                        depth1=spaces.Box(
                            -np.inf, np.inf, shape=obs["observation"]["depth1"].shape
                        ),
                        camera2=spaces.Box(
                            -np.inf, np.inf, shape=obs["observation"]["camera2"].shape
                        ),
                        depth2=spaces.Box(
                            -np.inf, np.inf, shape=obs["observation"]["depth2"].shape
                        ),
                        goal_camera1=spaces.Box(
                            -np.inf,
                            np.inf,
                            shape=obs["observation"]["goal_camera1"].shape,
                        ),
                        goal_depth1=spaces.Box(
                            -np.inf,
                            np.inf,
                            shape=obs["observation"]["goal_depth1"].shape,
                        ),
                        goal_camera2=spaces.Box(
                            -np.inf,
                            np.inf,
                            shape=obs["observation"]["goal_camera2"].shape,
                        ),
                        goal_depth2=spaces.Box(
                            -np.inf,
                            np.inf,
                            shape=obs["observation"]["goal_depth2"].shape,
                        ),
                    )
                ),
                desired_goal=spaces.Box(-np.inf, np.inf, shape=desired_goal_shape),
                achieved_goal=spaces.Box(-np.inf, np.inf, shape=achieved_goal_shape),
            )
        )
        self.action_space = self.robot.action_space
        self.compute_reward = self.task.compute_reward
        self.render = self.sim.render
        self.seed(None)

    def _get_obs(self):
        obs = super()._get_obs()

        with self.sim.object_hidden("target"):
            cam1 = p.getCameraImage(
                width=100,
                height=100,
                viewMatrix=self._view_mat1,
                projectionMatrix=self._projection_mat,
            )
            cam2 = p.getCameraImage(
                width=100,
                height=100,
                viewMatrix=self._view_mat2,
                projectionMatrix=self._projection_mat,
            )

        with self.sim.object_hidden("object", True):
            with self.sim.object_hidden("target", False):
                tcam1 = p.getCameraImage(
                    width=100,
                    height=100,
                    viewMatrix=self._view_mat1,
                    projectionMatrix=self._projection_mat,
                )
                tcam2 = p.getCameraImage(
                    width=100,
                    height=100,
                    viewMatrix=self._view_mat2,
                    projectionMatrix=self._projection_mat,
                )

        obs["observation"] = {
            "camera1": cam1[2],
            "depth1": cam1[3],
            "camera2": cam2[2],
            "depth2": cam2[3],
            "goal_camera1": tcam1[2],
            "goal_depth1": tcam1[3],
            "goal_camera2": tcam2[2],
            "goal_depth2": tcam2[3],
            "robot_state": self.robot.get_obs(),
        }

        return obs
