"""Drives the red ball on a circle via the planar_move plugin (/ball/cmd_vel)."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class BallMover(Node):
    def __init__(self):
        super().__init__('ball_mover')
        self.declare_parameter('v', 0.2)
        self.declare_parameter('w', 0.25)
        self.pub = self.create_publisher(Twist, '/ball/cmd_vel', 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        m = Twist()
        m.linear.x = self.get_parameter('v').value
        m.angular.z = self.get_parameter('w').value
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(BallMover())
