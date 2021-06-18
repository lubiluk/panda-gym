import numpy as np
from gym import utils

from panda_gym.envs.core import Task
from panda_gym.utils import distance


class Push(Task):
    def __init__(
        self,
        sim,
        reward_type="sparse",
        distance_threshold=0.05,
        goal_xy_range=0.3,
        obj_xy_range=0.3,
        seed=None,
        object_shape="cube"
    ):
        self.sim = sim
        self.reward_type = reward_type
        self.distance_threshold = distance_threshold
        self.object_size = 0.04
        self.object_shape = object_shape
        self.np_random, self.seed = utils.seeding.np_random(seed)
        self.goal_range_low = np.array([-goal_xy_range / 2, -goal_xy_range / 2, 0])
        self.goal_range_high = np.array([goal_xy_range / 2, goal_xy_range / 2, 0])
        self.obj_range_low = np.array([-obj_xy_range / 2, -obj_xy_range / 2, 0])
        self.obj_range_high = np.array([obj_xy_range / 2, obj_xy_range / 2, 0])
        with self.sim.no_rendering():
            self._create_scene()
            self.sim.place_visualizer(target=[0, 0, 0], distance=0.9, yaw=45, pitch=-30)

    def _create_scene(self):
        self.sim.create_plane(z_offset=-0.4)
        self.sim.create_table(length=1.1, width=0.7, height=0.4, x_offset=-0.3)

        if self.object_shape == "sphere":
            self.sim.create_sphere(
                body_name="object",
                radius = self.object_size / 2,
                mass=2,
                position=[0.0, 0.0, self.object_size / 2],
                rgba_color=[0.9, 0.1, 0.1, 1],
                friction=1,  # increase friction. For some reason, it helps a lot learning
            )
            self.sim.create_sphere(
                body_name="target",
                radius = self.object_size / 2,
                mass=0.0,
                ghost=True,
                position=[0.0, 0.0, self.object_size / 2],
                rgba_color=[0.9, 0.1, 0.1, 0.3],
            )
        else:
            self.sim.create_box(
                body_name="object",
                half_extents=[
                    self.object_size / 2,
                    self.object_size / 2,
                    self.object_size / 2,
                ],
                mass=2,
                position=[0.0, 0.0, self.object_size / 2],
                rgba_color=[0.9, 0.1, 0.1, 1],
                friction=1,  # increase friction. For some reason, it helps a lot learning
            )
            self.sim.create_box(
                body_name="target",
                half_extents=[
                    self.object_size / 2,
                    self.object_size / 2,
                    self.object_size / 2,
                ],
                mass=0.0,
                ghost=True,
                position=[0.0, 0.0, self.object_size / 2],
                rgba_color=[0.9, 0.1, 0.1, 0.3],
            )

    def get_goal(self):
        return self.goal.copy()

    def get_obs(self):
        # position, rotation of the object
        object_position = np.array(self.sim.get_base_position("object"))
        object_rotation = np.array(self.sim.get_base_rotation("object"))
        object_velocity = np.array(self.sim.get_base_velocity("object"))
        object_angular_velocity = np.array(self.sim.get_base_angular_velocity("object"))
        observation = np.concatenate(
            [
                object_position,
                object_rotation,
                object_velocity,
                object_angular_velocity,
            ]
        )
        return observation

    def get_achieved_goal(self):
        object_position = np.array(self.sim.get_base_position("object"))
        object_velocity = np.array(self.sim.get_base_velocity("object"))
        return np.concatenate((object_position, object_velocity))

    def reset(self):
        self.goal = self._sample_goal()
        object_position = self._sample_object()
        self.sim.set_base_pose("target", self.goal[:3], [0, 0, 0, 1])
        self.sim.set_base_pose("object", object_position, [0, 0, 0, 1])

    def _sample_goal(self):
        """Randomize goal."""
        goal = [0.0, 0.0, self.object_size / 2]  # z offset for the cube center
        noise = self.np_random.uniform(self.goal_range_low, self.goal_range_high)
        goal += noise
        return np.concatenate((goal, (0,0,0)))

    def _sample_object(self):
        """Randomize start position of object."""
        object_position = [0.0, 0.0, self.object_size / 2]
        noise = self.np_random.uniform(self.obj_range_low, self.obj_range_high)
        object_position += noise
        return object_position

    def is_success(self, achieved_goal, desired_goal):
        if not np.allclose(achieved_goal[3:], 0, atol=0.01):
                return False

        d = distance(achieved_goal[:3], desired_goal[:3])
        return (d < self.distance_threshold).astype(np.float32)

    def compute_reward(self, achieved_goal, desired_goal, info):
        d = distance(achieved_goal[:3], desired_goal[:3])
        if self.reward_type == "sparse":
            if not np.allclose(achieved_goal[3:], 0, atol=0.01):
                return -1
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d
