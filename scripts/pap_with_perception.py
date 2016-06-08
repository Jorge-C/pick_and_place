#! /usr/bin/env python
import rospy
from pap import PickAndPlaceNode


if __name__ == '__main__':
    n = PickAndPlaceNode('left')
    rospy.spin()
