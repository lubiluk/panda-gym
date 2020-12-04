from panda_gym.gazebo_connection import Gazebo
import rospy
import time
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/quentin/catkin_ws/src/panda_description/launch/simple.launch"], is_core=True)
launch.start()
rospy.loginfo("started")


rospy.init_node("python_node", anonymous=True, log_level=rospy.DEBUG)

rospy.sleep(30)
# 3 seconds later
launch.shutdown()