# -*- coding:utf-8 -*-

import time
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


def subscribe(name, data_class, callback, timeout=None):
    """TODO:"""
    msg = None
    while msg is None and not rospy.is_shutdown():
        try:
            rospy.logdebug("Waiting for message on topic %s" % name)
            msg = rospy.wait_for_message(name, data_class, timeout)
        except rospy.ROSInterruptException as e:
            rospy.logdebug(
                "Waiting for topic %s interrupted" % name)
            raise e
        except rospy.ROSException as e:
            rospy.logerr(
                "Timeout exceded, no message received on topic %s" % name)
            raise e
        else:
            rospy.logdebug("Subscriber to %s ready" % (name))
            rospy.Subscriber(name, data_class, callback)


def publisher(name, data_class, timeout=None):
    """TODO:"""
    start = time.time()
    pub = rospy.Publisher(name, data_class, queue_size=1)
    rate = rospy.Rate(5)  # 2hz
    rospy.logdebug("Looking for subscriber for topic %s" % name)
    while pub.get_num_connections() == 0:
        if timeout is not None and time.time() - start > timeout:
            rospy.logerr(
                "Timeout execeded, no subscriber found for topic %s" % name)
            raise rospy.ROSException()
        else:
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                # To avoid error when world is rested, time when backwards.
                rospy.logdebug("Time moved backward, ignoring")
            except rospy.ROSInterruptException as e:
                rospy.logdebug(
                    "Waiting for subscriber for topic %s interrupted" % name)
                raise e

    rospy.logdebug("Subscriber found for topic %s, publisher connected" % name)
    return pub


def service(name, service_class, timeout=None):
    """TODO:"""
    while not rospy.is_shutdown():
        try:
            rospy.logdebug("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout)
        except rospy.ROSInterruptException as e:
            rospy.logdebug(
                "Waiting for service %s interrupted" % name)
            raise e
        except rospy.ROSException as e:
            rospy.logerr(
                "Timeout exceded, no service %s found" % name)
            raise e
        else:
            rospy.logdebug("Service %s ready" % name)
            return rospy.ServiceProxy(name, service_class)


class Panda(object):

    def __init__(self):
        subscribe("/panda/joint_states",
                  JointState, self.joints_callback)
        self._my_pub = publisher(
            '/panda/panda_arm_controller/command', Float64MultiArray)

    def joints_callback(self, joints):
        self.state = joints

    def set_pos(self, pos):
        command = Float64MultiArray()
        command.data = pos
        self._my_pub.publish(command)

    def get_state(self):
        return self.state


def test_func():
    def pass_func(arg):
        pass
    rospy.init_node('test_node', anonymous=True, log_level=rospy.DEBUG)
    from gazebo_msgs.srv import SetPhysicsProperties
    service('/gazebo/set_physics_properties', SetPhysicsProperties, timeout=5)


if __name__ == "__main__":
    test_func()
