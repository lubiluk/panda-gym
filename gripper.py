import copy
import actionlib
import rospy
from sensor_msgs.msg import JointState
from franka_gripper.msg import (GraspAction, GraspGoal,
                                HomingAction, HomingGoal,
                                MoveAction, MoveGoal,
                                StopAction, StopGoal,
                                GraspEpsilon)


class GripperInterface(object):
    """Interface class for the gripper on the Franka Panda robot."""

    def __init__(self):
        """Constructor."""

        self.name = '/franka_gripper'

        ns = self.name + '/'

        self._width = None

        self._joint_states_state_sub = rospy.Subscriber(
            ns + 'joint_states', JointState, self._joint_states_callback, queue_size=1, tcp_nodelay=True)

        self._homing_action_client = actionlib.SimpleActionClient(
            '{}homing'.format(ns), HomingAction)
        self._grasp_action_client = actionlib.SimpleActionClient(
            '{}grasp'.format(ns), GraspAction)
        self._move_action_client = actionlib.SimpleActionClient(
            '{}move'.format(ns), MoveAction)
        self._stop_action_client = actionlib.SimpleActionClient(
            '{}stop'.format(ns), StopAction)

        self._homing_action_client.wait_for_server()
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()

        self.MIN_FORCE = 0.01
        # documentation says upto 70N is possible as continuous force (max upto 140N)
        self.MAX_FORCE = 50

        self.MIN_WIDTH = 0.0001
        self.MAX_WIDTH = 0.2

    def _joint_states_callback(self, msg):
        self._width = 2*msg.position[0]

    def get_width(self):
        """Return gripper width."""
        return self._width

    def homing(self, wait_for_result=False):
        """Performs homing of the gripper.

        After changing the gripper fingers, a homing needs to be done.
        This is needed to estimate the maximum grasping width.

        Args:
            wait_for_result (bool): if True, this method will block until
                response is recieved from server.

        Returns:
            bool: success     

        """
        goal = HomingGoal()
        self._homing_action_client.send_goal(goal)

        if wait_for_result:
            return self._homing_action_client.wait_for_result(
                timeout=rospy.Duration(15.))
        else:
            return True

    def move(self, width, speed=0.05, wait_for_result=True):
        """Moves the gripper fingers to a specified width.

        Args:
            width (float): Intended opening width [m].
            speed (float, optionnal): Closing speed [m/s]. Default to 0.05.
            wait_for_result (bool): If True, this method will block until
                response is recieved from server.

        Returns:
            bool: Whether command was successful.
        """
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed

        if wait_for_result:
            return self._move_action_client.wait_for_result(
                timeout=rospy.Duration(15.))
        else:
            return True

    def open(self):
        """Open gripper to max possible width.

        Returns:
            bool: Whether command was successful.
        """
        return self.move(0.2)

    def close(self):
        """Close gripper.

        Returns:
            bool: Whether command was successful.
        """
        return self.move(0.0)

    def stop(self):
        """Stops a currently running gripper move or grasp.

        Returns:
            bool: Whether command was successful.
        """
        goal = StopGoal()

        self._stop_action_client.send_goal(goal)

        return self._stop_action_client.wait_for_result(rospy.Duration(15.))

    def grasp(self, width, force, speed=0.05, epsilon_inner=0.005, epsilon_outer=0.005, wait_for_result=True):
        """
        Grasps an object.

        An object is considered grasped if the distance $d$ between the gripper fingers satisfies
        $(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})$.

        Args:
            width (float): Size [m] of the object to grasp.
            force (float): Grasping force [N].
            speed (float, optionnal): Closing speed [m/s]. Default to 0.05.
            epsilon_inner (float, optionnal): Maximum inner tolerated deviation 
                [m]. Defaults to 0.005.
            epsilon_outer (float, optionnal): Maximum outter tolerated
                deviation [m]. Defaults to 0.005.

        Returns:
            bool: Whether command was successful.
        """
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)

        self._grasp_action_client.send_goal(goal)

        if wait_for_result:
            return self._grasp_action_client.wait_for_result(
                timeout=rospy.Duration(15.))
        else:
            return True


if __name__ == '__main__':
    rospy.init_node("test_robot", anonymous=True)
    p = GripperInterface()
    p.close()

    # rospy.spin()
