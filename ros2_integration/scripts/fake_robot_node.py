#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
import math
import time


class FakeRobotNode(Node):
    """
    Simple fake robot node that publishes odometry for demonstration.
    """
    
    def __init__(self):
        super().__init__('fake_robot_node')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        
        self.get_logger().info('Fake robot node started')
    
    def cmd_callback(self, msg):
        """Update robot velocities from command."""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def publish_odometry(self):
        """Publish robot odometry."""
        dt = 0.1  # 10 Hz
        
        # Simple integration
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    
    node = FakeRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
