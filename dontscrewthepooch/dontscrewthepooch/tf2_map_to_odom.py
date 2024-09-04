import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry

class OdometryTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odometry_tf_broadcaster')
        # Subscribe to the vehicle odometry topic
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.handle_vehicle_odometry,
            qos_profile)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', qos_profile)

        self.get_logger().info("Vehicle Odometry TF broadcaster and Odometry publisher have been started")

    def handle_vehicle_odometry(self, msg):
        # Transform position from NED (PX4) to ENU (ROS2)
        t = TransformStamped()

        # Fill the TransformStamped data
        t.header.stamp = self.get_clock().now().to_msg()  # Adapt as needed for real timestamp
        t.header.frame_id = 'map'  # Assume map as the reference frame
        t.child_frame_id = 'odom'  # Assume odom as the child frame
        # Ensure to handle NaNs and set default values if necessary
        if not any(nan in msg.position for nan in (float('nan'),)):
            t.transform.translation.x = float(msg.position[0])
            t.transform.translation.y = -float(msg.position[1])
            t.transform.translation.z = -float(msg.position[2])
        if not any(nan in msg.q for nan in (float('nan'),)):
            t.transform.rotation.x = float(msg.q[1])
            t.transform.rotation.y = -float(msg.q[2])
            t.transform.rotation.z = -float(msg.q[3])
            t.transform.rotation.w = float(msg.q[0])
        # Broadcast the transform
        self.br.sendTransform(t)

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'odom'

        # Fill in the pose
        odom_msg.pose.pose.position.x = t.transform.translation.x
        odom_msg.pose.pose.position.y = t.transform.translation.y
        odom_msg.pose.pose.position.z = t.transform.translation.z
        odom_msg.pose.pose.orientation = t.transform.rotation

        # Covariance, assuming provided variances are in the correct order
        odom_msg.pose.covariance[0] = msg.position_variance[0]
        odom_msg.pose.covariance[7] = msg.position_variance[1]
        odom_msg.pose.covariance[14] = msg.position_variance[2]
        odom_msg.pose.covariance[21] = msg.orientation_variance[0]
        odom_msg.pose.covariance[28] = msg.orientation_variance[1]
        odom_msg.pose.covariance[35] = msg.orientation_variance[2]

        # Fill in the velocity
        # Transform velocity from NED (PX4) to ENU (ROS2)
        if not any(nan in msg.velocity for nan in (float('nan'),)):
            odom_msg.twist.twist.linear.x = float(msg.velocity[0])
            odom_msg.twist.twist.linear.y = -float(msg.velocity[1])
            odom_msg.twist.twist.linear.z = -float(msg.velocity[2])

        if not any(nan in msg.angular_velocity for nan in (float('nan'),)):
            odom_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
            odom_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
            odom_msg.twist.twist.angular.z = float(msg.angular_velocity[2])

        # Covariance for velocity, assuming provided variances are in the correct order
        odom_msg.twist.covariance[0] = msg.velocity_variance[0]
        odom_msg.twist.covariance[7] = msg.velocity_variance[1]
        odom_msg.twist.covariance[14] = msg.velocity_variance[2]
        # Angular velocity covariance could be similar; here assuming identity for simplicity
        odom_msg.twist.covariance[21] = 0.1
        odom_msg.twist.covariance[28] = 0.1
        odom_msg.twist.covariance[35] = 0.1

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
