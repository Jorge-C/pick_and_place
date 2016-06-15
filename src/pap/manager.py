from __future__ import division, print_function, absolute_import

from collections import defaultdict

import rospy

import tf

from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Pose
from keyboard.msg import Key
from teleop_interface.msg import IMarker


from .robot import Baxter, limb_pose


class PickAndPlaceNode(object):
    def __init__(self, limb_name):
        rospy.init_node("pp_node")

        _post_perceive_trans = defaultdict(lambda: self._pick)
        _post_perceive_trans.update({'c': self._calibrate})
        self.transition_table = {
            'initial': {'c': self._calibrate},
            'calibrate': {'q': self._perceive, 'c': self._calibrate},
            'perceive': {'q': self._post_perceive},
            'post_perceive': _post_perceive_trans,
            'postpick': {'s': self._place},
            'place': {'q': self._perceive, 'c': self._calibrate}
            }

        self.state = 'initial'

        self.limb_name = limb_name
        self.baxter = Baxter(limb_name)
        # Hardcoded place for now
        self.place_pose = [0.526025806104, 0.47801445746, -0.1613261530885,
                           1, 0, 0, 0]
        self.tf = tf.TransformListener()
        self.num_objects = 0
        # Would this work too? Both tf and tf2 have (c) 2008...
        # self.tf2 = tf2_ros.TransformListener()
        self.keyboard_sub = rospy.Subscriber("/keyboard/keyup",
                                             Key,
                                             self.handle_keyboard,
                                             queue_size=1)
        self.n_objects_sub = rospy.Subscriber("/num_objects", Int64,
                                              self.update_num_objects,
                                              queue_size=1)
        self.perception_pub = rospy.Publisher("/perception/enabled",
                                              Bool,
                                              queue_size=1)
        self.alignment_pub = rospy.Publisher("/alignment/doit",
                                             Bool,
                                             queue_size=1)
        self.br = tf.TransformBroadcaster()

        self.imarker_pub = rospy.Publisher("/imarker/add_pose",
                                           IMarker,
                                           queue_size=1)

    def update_num_objects(self, msg):
        self.num_objects = msg.data

    def _calibrate(self):
        self.state = "calibrate"
        self.alignment_pub.publish(Bool(True))

    def _perceive(self):
        self.state = "perceive"
        rospy.loginfo("Asking for perception...")
        self.perception_pub.publish(Bool(True))

    def _post_perceive(self):
        self.state = "post_perceive"
        rospy.loginfo("Asking to stop perception...")
        self.perception_pub.publish(Bool(False))

    def _place(self):
        self.state = "place"
        rospy.loginfo("Placing...")

        self.br.sendTransform(self.place_pose[:3],
                              self.place_pose[3:],
                              rospy.Time.now(),
                              "place_pose",
                              "/base")
        self.baxter.place(self.place_pose)
        # Order is x y z qx qy qz qw
        self.place_pose[2] += 0.04

    def _pick(self):
        # State not modified until picking succeeds
        try:
            obj_to_get = int(self.character)
        except ValueError:
            rospy.logerr("Please provide a number in picking mode")
            return

        frame_name = "object_pose_{}".format(obj_to_get)

        rospy.loginfo("Picking object {}...".format(obj_to_get))
        if self.tf.frameExists("/base") and self.tf.frameExists(frame_name):
            t = self.tf.getLatestCommonTime("/base", frame_name)
            position, quaternion = self.tf.lookupTransform("/base",
                                                           frame_name,
                                                           t)
            print("position", position)
            print("quaternion", quaternion)
            self.imarker_pub.publish(IMarker("imarker_{}".format(obj_to_get),
                                             Pose(position, quaternion)))
            position = list(position)
            # Vertical orientation
            self.br.sendTransform(position,
                                  [1, 0, 0, 0],
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/base")
            self.baxter.pick(position + [1, 0, 0, 0])
            self.state = 'postpick'

    def handle_keyboard(self, key):
        self.character = chr(key.code)
        try:
            next_state = self.transition_table[self.state][self.character]
        except KeyError:
            rospy.logerr(
                "Make sure input ({}) given"
                " state ({}) is valid!".format(input, self.state)
            )
            return

        next_state()
