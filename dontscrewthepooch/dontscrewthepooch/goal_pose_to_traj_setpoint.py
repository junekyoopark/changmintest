import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from geometry_msgs.msg import Twist
from rclpy.clock import Clock

class MulticopterControlNode(Node):

    def __init__(self):
        super().__init__('multicopter_control_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        cmd_vel_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,  # Match publisher's settings
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # Subscriber for /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, cmd_vel_qos_profile
        )

        # Variables
        self.nav2_vel_x = 0.0
        self.nav2_vel_y = 0.0
        self.nav2_vel_z = 0.0  # Assuming you want to control Z as well
        self.nav2_yaw = 0.0
        self.offboardMode = False
        self.cmd_received = False  # New flag to check if cmd_vel has been updated

        # Go to offboard mode right away
        self.enable_offboard_mode()

        # Timer for sending commands
        self.cmd_timer = self.create_timer(0.05, self.cmdloop_callback)  # 20Hz

    def arm_vehicle(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Vehicle armed")

    def enable_offboard_mode(self):
        # Send a few setpoints before enabling offboard mode
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow some time for the messages to be processed
        
        # Now enable offboard mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboardMode = True
        self.get_logger().info("Offboard mode enabled")
    
    def cmdloop_callback(self):
        if self.offboardMode and self.cmd_received:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
        elif self.offboardMode and not self.cmd_received:
            self.get_logger().info("Waiting for /cmd_vel input...")

    def publish_offboard_control_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def publish_trajectory_setpoint(self):
        # REMEMBER PX4 and ROS use DIFFERENT COORDINATE SYSTEMS
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = self.nav2_vel_y  # Velocity in X direction
        trajectory_msg.velocity[1] = self.nav2_vel_x  # Velocity in Y direction
        trajectory_msg.velocity[2] = -self.nav2_vel_z  # Velocity in Z direction
        trajectory_msg.position.fill(float('nan'))
        trajectory_msg.acceleration.fill(float('nan'))
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = -self.nav2_yaw  # Yaw rate
        self.publisher_trajectory.publish(trajectory_msg)
        #self.get_logger().info(f"Setpoint: vx={trajectory_msg.velocity[0]}, vy={trajectory_msg.velocity[1]}, vz={trajectory_msg.velocity[2]}, yaw_rate={trajectory_msg.yawspeed}")

    def cmd_vel_callback(self, msg):
        # Update the velocity and yaw based on the received /cmd_vel message
        self.nav2_vel_x = msg.linear.x
        self.nav2_vel_y = msg.linear.y
        self.nav2_vel_z = msg.linear.z  # If provided, otherwise set to 0.0 or a constant value
        self.nav2_yaw = msg.angular.z  # Angular velocity in Z (yaw rate)
        self.cmd_received = True  # Set the flag to true when a message is received
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
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

def main(args=None):
    rclpy.init(args=args)
    multicopter_node = MulticopterControlNode()
    rclpy.spin(multicopter_node)
    multicopter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()