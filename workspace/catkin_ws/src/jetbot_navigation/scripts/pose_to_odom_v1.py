#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pose_to_odom.py
Converts geometry_msgs/PoseStamped (from laser_scan_matcher) to
nav_msgs/Odometry with velocity estimation for move_base.

Subscribes:  /pose_stamped  (geometry_msgs/PoseStamped)
Publishes:   /odom          (nav_msgs/Odometry)
"""

import rospy
import math
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
        self.odom_frame  = rospy.get_param('~odom_frame',  'odom')
        self.base_frame  = rospy.get_param('~base_frame',  'base_link')

        # Covariance values (tune if AMCL/move_base complains)
        # [x, y, z, rot_x, rot_y, rot_z]  — 6x6 matrix, row-major
        pose_cov_diag   = rospy.get_param('~pose_covariance_diagonal',
                                          [0.05, 0.05, 1e6, 1e6, 1e6,0.03])
        twist_cov_diag  = rospy.get_param('~twist_covariance_diagonal',
                                          [0.1, 0.1, 1e6, 1e6, 1e6, 0.05])

        self.pose_covariance  = self._diag_to_36(pose_cov_diag)
        self.twist_covariance = self._diag_to_36(twist_cov_diag)

        # State
        self.last_pose = None
        self.last_time = None

        # ROS interfaces
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/pose_stamped', PoseStamped,
                         self.pose_cb, queue_size=10)

        rospy.loginfo("[pose_to_odom] Ready. "
                      "odom_frame='%s'  base_frame='%s'",
                      self.odom_frame, self.base_frame)
        rospy.spin()

    # ------------------------------------------------------------------
    def pose_cb(self, msg):
        cur_time = msg.header.stamp

        odom = Odometry()
        odom.header.stamp    = cur_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # Copy pose directly from laser_scan_matcher
        odom.pose.pose            = msg.pose
        odom.pose.covariance      = self.pose_covariance

        # Estimate velocity from pose difference
        if self.last_pose is not None and self.last_time is not None:
            dt = (cur_time - self.last_time).to_sec()
            if dt > 1e-6:                    # guard against zero dt
                dx  = msg.pose.position.x - self.last_pose.position.x
                dy  = msg.pose.position.y - self.last_pose.position.y

                yaw_cur  = quat_to_yaw(msg.pose.orientation)
                yaw_last = quat_to_yaw(self.last_pose.orientation)
                dyaw     = self._angle_diff(yaw_cur, yaw_last)

                # Velocities in the robot (base_link) frame
                vx   = (dx * math.cos(yaw_cur) + dy * math.sin(yaw_cur)) / dt
                vy   = (-dx * math.sin(yaw_cur) + dy * math.cos(yaw_cur)) / dt
                vyaw = dyaw / dt

                odom.twist.twist.linear.x  = vx
                odom.twist.twist.linear.y  = vy
                odom.twist.twist.angular.z = vyaw

        odom.twist.covariance = self.twist_covariance

        self.pub.publish(odom)

        self.last_pose = msg.pose
        self.last_time = cur_time

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
            mat[i * 6 + i] = float(v)
        return mat


if __name__ == '__main__':
    try:
        PoseToOdom()
    except rospy.ROSInterruptException:
        pass
