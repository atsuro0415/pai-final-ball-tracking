import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleStop(Node):
    """cmd_vel_raw を受け取り、前方に障害物があれば減速・停止して cmd_vel_in に出す。"""

    def __init__(self):
        super().__init__('obstacle_stop')
        self.declare_parameter('stop_dist', 0.5)    # この距離以内で前進停止 [m]
        self.declare_parameter('slow_dist', 1.0)    # この距離以内で減速開始 [m]
        self.declare_parameter('half_angle_deg', 30.0)  # 前方の監視範囲（片側）[deg]

        self.stop_dist = self.get_parameter('stop_dist').value
        self.slow_dist = self.get_parameter('slow_dist').value
        self.half_angle = math.radians(self.get_parameter('half_angle_deg').value)

        self.front_min = float('inf')   # 前方で最も近い障害物までの距離
        self.have_scan = False

        self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_in', 10)
        self.get_logger().info(
            f'停止 {self.stop_dist} m / 減速開始 {self.slow_dist} m で開始')

    def on_scan(self, msg):
        dmin = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self.half_angle and r < dmin:
                dmin = r
        self.front_min = dmin
        self.have_scan = True

    def on_cmd(self, msg):
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular

        if msg.linear.x > 0.0 and self.have_scan:
            d = self.front_min
            if d <= self.stop_dist:
                scale = 0.0
            elif d < self.slow_dist:
                scale = (d - self.stop_dist) / (self.slow_dist - self.stop_dist)
            else:
                scale = 1.0
            if scale < 1.0:
                self.get_logger().info(
                    f'前方 {d:.2f} m: 速度を {scale * 100:.0f}% に制限',
                    throttle_duration_sec=1.0)
            out.linear.x = msg.linear.x * scale

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
