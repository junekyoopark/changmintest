#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from rclpy.clock import Clock
import numpy as np


class offboard_start(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )

        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile
        )

        self.vtol_msg_fw_sub = self.create_subscription(
            Bool,
            '/vtol_message_fw',
            self.vtol_msg_callback_fw,
            qos_profile
        )

        self.vtol_msg_mc_sub = self.create_subscription(
            Bool,
            '/vtol_message_mc',
            self.vtol_msg_callback_mc,
            qos_profile
        )

        self.position_setpoint_sub = self.create_subscription(
            PoseStamped,
            '/offboard_position_cmd',
            self.offboard_position_callback,
            qos_profile
        )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.publisher_velocity = self.create_publisher(
            Twist,
            '/fmu/in/setpoint_velocity/cmd_vel_unstamped',
            qos_profile
        )

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )



        # Timers
        arm_timer_period = 0.1
        self.arm_timer = self.create_timer(arm_timer_period, self.arm_timer_callback)

        cmd_timer_period = 0.05
        self.cmd_timer = self.create_timer(cmd_timer_period, self.cmdloop_callback)

        # Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0 # Yaw value send
        self.trueYaw = 0.0 # Current Yaw
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.vtol_msg_fw = False
        self.vtol_msg_mc = False

        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "OFFBOARD": self.state_offboard,
            "DONE": self.state_done
        }
        self.current_state = "IDLE"
        self.last_state = self.current_state
        

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm message recieved as {self.arm_message}")

    def vtol_msg_callback_fw(self, msg):
        self.vtol_msg_fw = msg.data
        self.get_logger().info("Transition message recieved (fw)")

    def vtol_msg_callback_mc(self, msg):
        self.vtol_msg_mc = msg.data
        self.get_logger().info("Transition message recieved (mc)")
    
    
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message):
                    self.current_state = "ARMING"
                    self.get_logger().info("idle case passed")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info("case arming failed: FC failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("case arming passed")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state == VehicleStatus.ARMING_STATE_DISARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info("case offboard failed: FC failed")
                self.state_offboard()
                self.current_state = "DONE"
            
            case "DONE":
                if (self.myCnt > 100):
                    self.destroy_timer(self.arm_timer)
                    self.get_logger().info("timer destroyed")

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False
        
        if(self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)
        
        self.myCnt += 1
    
    def state_init(self):
        self.myCnt = 0

    def state_arming(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm Command Sent")
    
    def state_takeoff(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7 = 5.0)
        self.get_logger().info("Takeoff command sent")

    def state_loiter(self):
        self.myCnt = 0
        self.get_logger().info("Loiter mode")

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboardMode = True
        self.get_logger().info("Offboard command sent")

    def state_done(self):
        self.myCnt = 0

    def vtol_is_a_go(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0)
        self.get_logger().info("VTOL transition imminent")

    def vtol_MC(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0)
        self.get_logger().info("VTOL back to MC")

    
    # Commands
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm sent by commands")
    
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7 = 5.0)
        self.get_logger().info("Takeoff sent by commands")

    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0, param7 = 0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        
        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlgihtCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def offboard_position_callback(self, msg):
        self.posx = msg.pose.position.x
        self.posy = msg.pose.position.y
        self.posz = -msg.pose.position.z
        self.posyaw = msg.pose.orientation.x

    def offboard_velocity_callback(self, msg):
        self.velocity.x = msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.x

    def attitude_callback(self, msg):
        orientation_q = msg.q

        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        

    def cmdloop_callback(self):
        if (self.offboardMode):
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)

            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            trajectory_msg = TrajectorySetpoint()
            if (self.vtol_msg_fw):
                self.vtol_is_a_go()
                trajectory_msg.yaw = float('nan')
                self.vtol_msg_fw = False
            elif (self.vtol_msg_mc):
                self.vtol_MC()
                trajectory_msg.yaw = float(self.posyaw)
                self.vtol_msg_mc = False
            else:
                trajectory_msg.yaw = float(self.posyaw)

            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = float('nan')#velocity_world_x
            trajectory_msg.velocity[1] = float('nan')#velocity_world_y
            trajectory_msg.velocity[2] = float('nan')#self.velocity.z
            trajectory_msg.position[0] = self.posx
            trajectory_msg.position[1] = self.posy
            trajectory_msg.position[2] = self.posz
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yawspeed = float('nan')
            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    offboard_node = offboard_start()
    rclpy.spin(offboard_node)
    offboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()