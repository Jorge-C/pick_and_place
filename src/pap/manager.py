from __future__ import division, print_function, absolute_import

from collections import defaultdict

import rospy

import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer
    )

from .robot import Baxter
from .interactive_marker import make_interactive_marker
from .utils import Point2list, Quaternion2list


class Manager(object):
    def __init__(self, name):
        rospy.init_node(name)

        self.transition_table = None
        self.state = 'initial'

        self.keyboard_sub = rospy.Subscriber("/keyboard/keyup",
                                             Key,
                                             self.handle_keyboard,
                                             queue_size=1)

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


class PickAndPlaceNode(Manager):
    def __init__(self, limb_name):
        super(PickAndPlaceNode, self).__init__('pp_node')

        _post_perceive_trans = defaultdict(lambda: self._pick)
        _post_perceive_trans.update({'c': self._calibrate})
        self.transition_table = {
            # If calibration has already happened, allow skipping it
            'initial': {'c': self._calibrate, 'q': self._perceive},
            'calibrate': {'q': self._perceive, 'c': self._calibrate},
            'perceive': {'q': self._post_perceive},
            'post_perceive': _post_perceive_trans,
            'postpick': {'s': self._preplace},
            'preplace': {'s': self._place},
            'place': {'q': self._perceive, 'c': self._calibrate}
            }
        self.limb_name = limb_name
        self.baxter = Baxter(limb_name)
        # Hardcoded place for now
        self.place_pose = Pose(Point(0.526025806, 0.4780144, -0.161326153),
                               Quaternion(1, 0, 0, 0))
        self.tl = tf.TransformListener()
        self.num_objects = 0
        # Would this work too? Both tf and tf2 have (c) 2008...
        # self.tf2 = tf2_ros.TransformListener()
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

        self.int_marker_server = InteractiveMarkerServer('int_markers')
        # Dict to map imarker names and their updated poses
        self.int_markers = {}

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

    def _preplace(self):
        self.state = "preplace"
        rospy.loginfo("Adjusting place position...")
        imarker_name = 'place_pose'
        self.int_markers[imarker_name] = self.place_pose
        imarker = make_interactive_marker(imarker_name,
                                          self.place_pose)
        self.int_marker_server.insert(imarker, self.imarker_callback)
        self.int_marker_server.applyChanges()

    def imarker_callback(self, msg):
        # http://docs.ros.org/jade/api/visualization_msgs/html/msg/InteractiveMarkerFeedback.html # noqa
        name = msg.marker_name
        new_pose = msg.pose
        self.int_markers[name] = PoseStamped(msg.header, new_pose)

    def _place(self):
        self.state = "place"
        rospy.loginfo("Placing...")

        place_pose = self.int_markers['place_pose']
        # It seems positions and orientations are randomly required to
        # be actual Point and Quaternion objects or lists/tuples. The
        # least coherent API ever seen.
        self.br.sendTransform(Point2list(place_pose.pose.position),
                              Quaternion2list(place_pose.pose.orientation),
                              rospy.Time.now(),
                              "place_pose",
                              "/base")
        self.baxter.place(place_pose)

        place_pose.pose.position.z += 0.05
        self.place_pose = place_pose

    def _pick(self):
        # State not modified until picking succeeds
        try:
            obj_to_get = int(self.character)
        except ValueError:
            rospy.logerr("Please provide a number in picking mode")
            return

        frame_name = "object_pose_{}".format(obj_to_get)

        rospy.loginfo("Picking object {}...".format(obj_to_get))
        if self.tl.frameExists("/base") and self.tl.frameExists(frame_name):
            t = self.tl.getLatestCommonTime("/base", frame_name)
            position, quaternion = self.tl.lookupTransform("/base",
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
            # Ignore orientation from perception
            pose = Pose(Point(*position),
                        Quaternion(1, 0, 0, 0))
            h = Header()
            h.stamp = t
            h.frame_id = "base"
            stamped_pose = PoseStamped(h, pose)
            self.baxter.pick(stamped_pose)
            self.state = 'postpick'
