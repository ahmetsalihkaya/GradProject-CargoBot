#!/usr/bin/env python

import rospy
import actionlib
import sys
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert yaw (in radians) to quaternion
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('send_goal')

    # Get values from terminal arguments
    if len(sys.argv) != 4:
        print("Usage: send_goal.py x y yaw")
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])

    result = send_goal(x, y, yaw)
    print(result)
