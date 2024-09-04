#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import csv
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommand, VehicleOdometry
import time
import math

class waypointsmc(Node):
    def __init__(self):
        super().__init__('mcWaypoints')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.handle_vehicle_odometry,
            qos_profile
        ) #should be fixed to local position and/or RTK

        # Publishers
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos_profile
        )

        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/offboard_position_cmd',
            qos_profile
        )

        self.arm_publisher = self.create_publisher(
            Bool,
            '/arm_message',
            qos_profile
        )

        # MAIN LOGIC 

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 20.0}, #WP1
            {'x': 3.0, 'y': 3.0, 'z': 20.0}, #WP2
            {'x': -3.0, 'y': 3.0, 'z': 20.0}, #WP3
            {'x': -3.0, 'y': -3.0, 'z': 20.0}, #WP4
            {'x': 3.0, 'y': -3.0, 'z': 20.0}, #WP5
            {'x': 0.0, 'y': 0.0, 'z': 20.0} #WP6
        ]

        self.curr_way_index = 0 #change
        self.position_tolerance = 1.0
        self.m_target = self.waypoints[0]
        self.timerCount = 0

        time.sleep(5)
        self.get_logger().info("Main launched")
        self.arm_drone(True)
        time.sleep(5)
        self.navigate_waypoints()


    def handle_vehicle_odometry(self, msg):
        self.current_position['x'] = float(msg.position[0])
        self.current_position['y'] = -float(msg.position[1])
        self.current_position['z'] = -float(msg.position[2])
        self.current_orientation = (
            float(msg.q[1]),
            -float(msg.q[2]),
            -float(msg.q[3]),
            float(msg.q[0])
        )


    def arm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone armed")

    def disarm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone disarmed")

    def arm_vtol(self, varm):
        vtol_hmg = Bool()
        vtol_hmg.data = varm
        self.vtol_publisher_fw.publish(vtol_hmg)
        self.get_logger().info("Go VTOL")
    
    def arm_mc(self, marm):
        vtol_mmg = Bool()
        vtol_mmg.data = marm
        self.vtol_publisher_mc.publish(vtol_mmg)
        self.get_logger().info("Go MC")

    def navigate_waypoints(self):
        self.wp_timer = self.create_timer(0.01, self.navigate_waypoint_callback)


    def navigate_waypoint_callback(self):
        # m_target = big target
        if self.curr_way_index < len(self.waypoints):
            target = self.waypoints[self.curr_way_index]
            if self.is_waypoint_reached(target):
                    self.curr_way_index += 1
                    self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
            else: 
                pose = self.calculate_position_command(target)
                self.position_publisher.publish(pose)
        else:
            self.get_logger().info("All waypoints reached")
            self.disarm_drone(False)
            self.destroy_timer(self.wp_timer)

    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 0.2
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y
        twist.linear.z = kp * error_z

        max_speed = 1.0
        norm = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if norm > max_speed:
            twist.linear.x = (twist.linear.x / norm) * max_speed
            twist.linear.y = (twist.linear.y / norm) * max_speed
            twist.linear.z = (twist.linear.z / norm) * max_speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        return twist
    
    def calculate_position_command(self, target):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['y']
        pose.pose.position.y = target['x']
        pose.pose.position.z = target['z']

        # Calculate desired yaw
        error_y = target['y'] - self.current_position['y']
        error_x = target['x'] - self.current_position['x']
        desired_yaw = math.atan2(error_y, error_x)

        # Set yaw in Euler angles directly
        pose.pose.orientation.x = desired_yaw + 1.5708
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        return pose


    def is_waypoint_reached(self, target):
        enu_current_position_x = -self.current_position['y']  
        enu_current_position_y = self.current_position['x']  
        enu_current_position_z = self.current_position['z']

        x_dist = abs(target['x'] - enu_current_position_x)
        y_dist = abs(target['y'] - enu_current_position_y)
        z_dist = abs(target['z'] - enu_current_position_z)

        self.get_logger().info(f"x: {round(x_dist, 2)} y: {round(y_dist, 2)}")

        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)

def main(args=None):
    rclpy.init(args=args)
    mission1node = waypointsmc()
    rclpy.spin(mission1node)
    print("node destroyed")
    mission1node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()