#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geographic_msgs.msg import PoseStamped
import tf_transformations

def create_commander(navigator, x_pos, y_pos, rot_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rot_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x_pos
    goal_pose.pose.position.y = y_pos
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    goal_pose = create_commander(nav, 0.0, 0.0, 1.57)
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
