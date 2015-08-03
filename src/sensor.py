"""ROS node that reads sensor data from the end effector and computes
grasping forces based on advanced magic.  """

from __future__ import division, print_function, absolute_import

import rospy
from std_msgs.msg import Empty

from baxter_interface import Gripper


class SensorizedGripper(Gripper):
    """Add some magic to Baxter's gripper.

    """
    def __init__(self, gripper, versioned=True):
        return super(SensorizedGripper, self).__init__(gripper, versioned)

    def compute_closing_force(self):
        f = 100
        return max(0, min(100, f))

    def close(self):
        f = self.compute_closing_force()
        self.set_moving_force(f)
        self.set_holding_force(f)
        return super(SensorizedGripper, self).close()


class GrippingNode(object):
    def __init__(self):
        pass


def gripping_server():
    rospy.init_node('gripping_server')
    gripper = SensorizedGripper('left')
    sclose = rospy.Service('close', Empty, gripper.close)
    sopen  = rospy.Service('open', Empty, gripper.open)
    rospy.spin()


if __name__ == '__main__':
    s = gripping_server()
