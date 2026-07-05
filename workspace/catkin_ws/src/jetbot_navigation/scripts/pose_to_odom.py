#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pose_to_odom.py
Converts geometry_msgs/PoseStamped (from laser_scan_matcher) to:
  - nav_msgs/Odometry on /odom
  - TF transform:  odom → base_link

IMPORTANT: Set publish_tf: false in laser_scan_matcher so this node
is the single owner of the odom → base_link transform.

Subscribes:  /pose_stamped  (geometry_msgs/PoseStamped)
Publishes:   /odom          (nav_msgs/Odometry)
             TF             odom → base_link
"""

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseToOdom:
    def __init__(self):
        rospy.init_node('pose_to_odom', anonymous=False)

        # Parameters
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # [x,    y,    z,   roll, pitch, yaw]
        # z/roll/pitch = 1e6: tell EKF to ignore these axes (2D robot)
        pose_cov_diag  = rospy.get_param('~pose_covariance_diagonal',
                                         [0.05, 0.05, 1e6, 1e6, 1e6, 0.03])
        # vx/vy noisier than pose (numerically differentiated)
        twist_cov_diag = rospy.get_param('~twist_covariance_diagonal',
                                         [0.1, 0.1, 1e6, 1e6, 1e6, 0.05])

        self.pose_covariance  = self._diag_to_36(pose_cov_diag)
        self.twist_covariance = self._diag_to_36(twist_cov_diag)

        # State
        self.last_pose = None
        self.last_time = None

        # TF broadcaster — this node is the single owner of odom→base_link
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ROS interfaces
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/pose_stamped', PoseStamped,
                         self.pose_cb, queue_size=10)

        rospy.loginfo("[pose_to_odom] Ready. odom_frame='%s'  base_frame='%s'",
                      self.odom_frame, self.base_frame)
        rospy.spin()

    # ------------------------------------------------------------------
    def pose_cb(self, msg):
        msg_time = msg.header.stamp  # original stamp for velocity differentiation
        cur_time = rospy.Time.now()  # fresh stamp for TF & odom header

        # --- Broadcast TF: odom → base_link --------------------------
        # MUST use rospy.Time.now() here — using msg.header.stamp causes
        # ~0.3s lag through the topic queue, which exceeds Costmap2DROS
        # transform_tolerance (0.3s) and blocks move_base entirely.
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            cur_time,
            self.base_frame,   # child
            self.odom_frame    # parent
        )

        # --- Build Odometry message ----------------------------------
        odom = Odometry()
        odom.header.stamp    = cur_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose       = msg.pose
        odom.pose.covariance = self.pose_covariance

        # Estimate velocity from consecutive pose difference
        # Use msg_time for dt so velocity reflects actual scan rate,
        # not the wall-clock jitter introduced by rospy.Time.now()
        if self.last_pose is not None and self.last_time is not None:
            dt = (msg_time - self.last_time).to_sec()
            if dt > 1e-6:
                dx  = msg.pose.position.x - self.last_pose.position.x
                dy  = msg.pose.position.y - self.last_pose.position.y

                yaw_cur  = quat_to_yaw(msg.pose.orientation)
                yaw_last = quat_to_yaw(self.last_pose.orientation)
                dyaw     = self._angle_diff(yaw_cur, yaw_last)

                # Express velocities in robot (base_link) frame
                vx   = ( dx * math.cos(yaw_cur) + dy * math.sin(yaw_cur)) / dt
                vy   = (-dx * math.sin(yaw_cur) + dy * math.cos(yaw_cur)) / dt
                vyaw = dyaw / dt

                odom.twist.twist.linear.x  = vx
                odom.twist.twist.linear.y  = vy
                odom.twist.twist.angular.z = vyaw

        odom.twist.covariance = self.twist_covariance
        self.pub.publish(odom)

        self.last_pose = msg.pose
        self.last_time = msg_time   # keep using msg_time for velocity accuracy

    # ------------------------------------------------------------------
    @staticmethod
    def _angle_diff(a, b):
        """Shortest signed angular difference, result in [-pi, pi]."""
        diff = a - b
        while diff >  math.pi: diff -= 2.0 * math.pi
        while diff < -math.pi: diff += 2.0 * math.pi
        return diff

    @staticmethod
    def _diag_to_36(diag6):
        """Build a 36-element row-major covariance matrix from 6 diagonal values."""
        mat = [0.0] * 36
        for i, v in enumerate(diag6):
            mat[i * 6 + i] = float(v)  # explicit cast required for Python 2.7
        return mat


if __name__ == '__main__':
    try:
        PoseToOdom()
    except rospy.ROSInterruptException:
        pass
