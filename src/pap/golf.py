#!/usr/bin/env python
from __future__ import division, print_function

from itertools import chain, repeat

import rospy
import baxter_interface
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import DigitalIOState

from std_msgs.msg import Int64
from keyboard.msg import Key

# Apparently tf was deprecated a long time ago? And we should use tf2?
import tf
# import tf2_ros

from baxter_pykdl import baxter_kinematics


class CuffOKButton(object):
    def __init__(self, limb_name):
        """Subscribe to the OK button on either cuff (selected using
        limb_name). action_function must be a function that accepts a data
        argument (DigitalIOState, where DigitalIOState.state is 1 if
        button is pressed, 0 otherwise).

        """
        self.limb_name = limb_name
        self.pressed = False
        node = '/robot/digital_io/%s_lower_button/state' % limb_name
        rospy.Subscriber(node, DigitalIOState, self.callback)

    def callback(self, data):
        self.pressed = data.state == 1


def limb_pose(limb_name):
    """Get limb pose at time of OK cuff button press.

    The returned pose matches """
    button = CuffOKButton(limb_name)
    rate = rospy.Rate(20)  # rate at which we check whether button was
                           # pressed or not
    rospy.loginfo(
        'Waiting for %s OK cuff button press to save pose' % limb_name)
    while not button.pressed and not rospy.is_shutdown():
        rate.sleep()
    endpoint_pose = baxter_interface.Limb(limb_name).endpoint_pose()
    return endpoint_pose
    # How is
    # baxter_kinematics(limb_name).forward_position_kinematics(
    #     baxter_interface.Limb(limb_name).joint_angles())
    # different from
    # baxter_interface.Limb(limb_name).endpoint_pose()
    # After some testing, we find that the differences are
    # negligible. In rviz, pretty much the same. In numerical terms,
    # they agree to at least two significant figures.
    # Eg:
    """
    baxter_interface endpoint pose:
    {'position': Point(x=0.8175678653720638,
                       y=0.11419185934519176,
                       z=-0.18813767395654984),
     'orientation': Quaternion(x=0.9983899777382322,
                               y=-0.008582355126430147,
                               z=0.051452248110337225,
                               w=-0.022281420437849815)}
    pykdl forward kinematics endpoint pose:
    [ 0.81954073  0.11447536 -0.18811824
      0.9983032  -0.00843487  0.0532936  -0.02189443]"""


class Baxter(object):
    """Add higher level operations to Baxter."""
    def __init__(self, limb_name, gripper=None):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        if gripper is None:
            self.gripper = baxter_interface.Gripper(limb_name)
        else:
            self.gripper = gripper(limb_name)

        # Enable actuators (doesn't seem to work?) I need to rosrun
        # robot_enable.py -e
        baxter_interface.RobotEnable().enable()
        # Calibrate gripper
        self.gripper.calibrate()

    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        """Go to pose + pick_direction * pick_distance, open, go to pose,
        close, go to pose + pick_direction * pick_distance.

        """
        pregrasp_pose = self.translate(pose, direction, distance)
        self.limb.set_joint_position_speed(0.02)
        self.move_ik(pregrasp_pose)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        self.gripper.open(block=True)
        self.move_ik(pose)
        self.gripper.close(block=True)
        self.move_ik(pregrasp_pose)

    def place(self, pose, direction=(0, 0, 1), distance=0.1):
        """Go to pose + place_direction * place_distance, go to pose,
        open, go to pose + pick_direction * pick_distance.

        """
        pregrasp_pose = self.translate(pose, direction, distance)
        self.move_ik(pregrasp_pose)
        self.move_ik(pose)
        self.gripper.open(block=True)
        self.move_ik(pregrasp_pose)

    @staticmethod
    def to_stamped_pose(pose):
        """Take a xyzrpy or xyz qxqyqzqw pose and convert it to a stamped
        pose (quaternion as orientation).

        """
        stamped_pose = conversions.list_to_pose_stamped(pose, "base")
        return stamped_pose

    @staticmethod
    def translate(pose, direction, distance):
        """Get an xyz rpy or xyz qxqyqzqw pose and add direction * distance to
        xyz. There's probably a better way to do this.

        """
        scaled_direction = [distance * di for di in direction]
        translated_pose = [pi + di for pi, di in
                           zip(pose,
                               chain(scaled_direction,
                                     repeat(0, len(pose) - 3)))]
        return translated_pose

    def ik_quaternion(self, quaternion_pose):
        """Take a xyz qxqyqzqw stamped pose and convert it to joint angles
        using IK. You can call self.limb.move_to_joint_positions to
        move to those angles

        """
        node = ("ExternalTools/{}/PositionKinematicsNode/"
                "IKService".format(self.limb_name))
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.loginfo('ik: waiting for IK service...')
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException) as err:
            rospy.logerr("ik_move: service request failed: %r" % err)
        else:
            if ik_response.isValid[0]:
                rospy.loginfo("ik_move: valid joint configuration found")
                # convert response to joint position control dictionary
                limb_joints = dict(zip(ik_response.joints[0].name,
                                       ik_response.joints[0].position))
                return limb_joints
            else:
                rospy.logerr('ik_move: no valid configuration found')

    def move_ik(self, pose):
        """Take a pose (either xyz rpy or xyz qxqyqzqw) and move the arm
        there.

        """
        stamped_pose = self.to_stamped_pose(pose)
        joint_pose = self.ik_quaternion(stamped_pose)
        return self.limb.move_to_joint_positions(joint_pose)

    def _compare_ik_fk(self):
        """All the logged values should match in theory. In practice they're
        fairly close.

        """
        kinematics = baxter_kinematics(self.limb_name)
        rospy.logerr(kinematics.forward_position_kinematics())
        rospy.logerr(kinematics.forward_position_kinematics(
            self.limb.joint_angles()))
        rospy.logerr(self.limb.endpoint_pose())

    def fk(self, joint_angles=None):
        """Compute end point pose (xyz qxqyqzqw) based on joint angles."""
        kinematics = baxter_kinematics(self.limb_name)
        return kinematics.forward_position_kinematics(joint_angles)

    def move_relative(self, pose):
        """Move endpoint according to the relative pose given.

        Examples
        --------
        baxter.move_relative([0, 0, 0, 0, 0, 0, 1])  # No movement
        baxter.move_relative([x, y, z, 0, 0, 0, 1])  # End effector translation
        """
        current_pose = self.limb.endpoint_pose()
        xyz = current_pose['position']
        qxqyqzqw = current_pose['orientation']
        current_pose = list(xyz) + list(qxqyqzqw)
        final_pose = self._compose(current_pose, pose)
        self.move_ik(final_pose)

    @staticmethod
    def _to_quaternion_pose(pose):
        if len(pose) == 6:
            q = tf.transformations.quaternion_from_euler(*pose[3:]).tolist()
            return list(pose[:3]) + q
        elif len(pose) == 7:
            return pose
        else:
            raise TypeError('Pose needs to be xyzrpy or xyzqxqyqzq')

    def _compose(self, pose1, pose2):
        """Compose two poses xyz qxqyqzqw. There must be a library to do it."""
        pose1 = self._to_quaternion_pose(pose1)
        pose2 = self._to_quaternion_pose(pose2)
        xyz = [x1 + x2 for x1, x2 in zip(pose1[:3], pose2[:3])]
        qxqyqzqw = tf.transformations.quaternion_multiply(pose1[3:],
                                                          pose2[3:]).tolist()
        return xyz + qxqyqzqw


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
            position, quaternion = self.tf.lookupTransform("/base", frame_name, t)
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


def main(limb_name, reset):
    """
    Parameters
    ----------
    limb : str
        Which limb to use. Choices are {'left', 'right'}
    reset : bool
        Whether to use previously saved picking and placing poses or
        to save new ones by using 0g mode and the OK cuff buttons.
    """
    # Initialise ros node
    rospy.init_node("pick_and_place", anonymous=True)

    # Either load picking and placing poses from the parameter server
    # or save them using the 0g mode and the circular buttons on
    # baxter's cuffs
    if reset or not rospy.has_param('~pick_and_place_poses'):
        rospy.loginfo(
            'Saving picking pose for %s limb' % limb_name)
        pick_pose = limb_pose(limb_name)
        rospy.sleep(1)
        place_pose = limb_pose(limb_name)
        # Parameter server can't store numpy arrays, so make sure
        # they're lists of Python floats (specifically not
        # numpy.float64's). I feel that's a bug in rospy.set_param.
        rospy.set_param('~pick_and_place_poses',
                        {'pick': pick_pose.tolist(),
                         'place': place_pose.tolist()})
        rospy.loginfo('pick_pose is %s' % pick_pose)
        rospy.loginfo('place_pose is %s' % place_pose)
    else:
        pick_pose = rospy.get_param('~pick_and_place_poses/pick')
        place_pose = rospy.get_param('~pick_and_place_poses/place')

    b = Baxter(limb_name)
    b.pick(pick_pose)
    b.place(place_pose)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Use baxter to pick and place objects')
    parser.add_argument(
        '-l', '--limb', choices=('left', 'right'),
        help='Choose which limb to use for picking and placing')
    parser.add_argument(
        '-r', '--reset', action='store_true', default=False)

    args = parser.parse_args(rospy.myargv()[1:])

    #main(args.limb, args.reset)
    c = PickAndPlaceNode('left')
    rospy.spin()
