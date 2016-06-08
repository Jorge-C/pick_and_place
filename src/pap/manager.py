from __future__ import division, print_function, absolute_import

import rospy

import tf

from std_msgs.msg import Int64
from keyboard.msg import Key

from .robot import Baxter, limb_pose


class PickAndPlaceNode(object):
    def __init__(self, limb_name):
        rospy.init_node("pp_node")
        self.limb_name = limb_name
        self.baxter = Baxter(limb_name)
        self.place_pose = limb_pose(limb_name)
        self.tf = tf.TransformListener()
        self.num_objects = 0
        # Would this work too? Both tf and tf2 have (c) 2008...
        # self.tf2 = tf2_ros.TransformListener()
        self.keyboard_sub = rospy.Subscriber("/keyboard/keyup",
                                             Key,
                                             self.handle_keyboard,
                                             queue_size=1)
        self.n_objects_sub = rospy.Subcriber("/num_objects", Int64,
                                             self.update_num_objects,
                                             queue_size=1)
        self.br = tf.TransformBroadcaster()

    def update_num_objects(self, msg):
        self.num_objects = msg.data

    def _pick(self, frame_name):
        rospy.loginfo("Picking object " + frame_name[-1:] + "...")
        if self.tf.frameExists("/base") and self.tf.frameExists(frame_name):
            t = self.tf.getLatestCommonTime("/base", frame_name)
            position, quaternion = self.tf.lookupTransform("/base",
                                                           frame_name,
                                                           t)
            print("position", position)
            print("quaternion", quaternion)
            position = list(position)
            # Vertical orientation
            self.br.sendTransform(position,
                                  [1, 0, 0, 0],
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/base")
            self.baxter.pick(position + [1, 0, 0, 0])

    def handle_keyboard(self, key):
        # First set mode (picking/placing)
        if key.code == ord('a'):
            self.mode = 'picking'
        elif key.code == ord('s'):
            self.mode = 'placing'
        if self.mode == 'picking':
            try:
                obj_to_get = int(chr(key.code))
            except ValueError:
                rospy.logerr("Please provide a number in picking mode")
                return
            frame_name = "object_pose_{}".format(obj_to_get)
            self._pick(frame_name)
        elif self.mode == 'placing':
            rospy.loginfo("Placing...")
            pose_list = ([getattr(self.place_pose['position'], i) for i in
                          ('x', 'y', 'z')] +
                         [getattr(self.place_pose['orientation'], i) for i in
                          ('x', 'y', 'z', 'w')])
            self.br.sendTransform(pose_list[:3],
                                  pose_list[3:],
                                  rospy.Time.now(),
                                  "place_pose",
                                  "/base")
            self.baxter.place(pose_list)
            self.place_pose['position'].x += 0.04
