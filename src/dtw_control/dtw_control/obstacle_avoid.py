import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoid(Node):
    """cmd_vel_raw を受け取り、前方の障害物を避けて cmd_vel_in に出す。

    前方が詰まったら、空いている側へその場旋回し、前方が開いたら前進を再開する。
    """

    def __init__(self):
        super().__init__('obstacle_avoid')
        self.declare_parameter('slow_dist', 1.5)     # 減速開始 [m]
        self.declare_parameter('avoid_dist', 0.8)    # 旋回開始 [m]
        self.declare_parameter('clear_dist', 1.2)    # 前進再開 [m]
        self.declare_parameter('front_half_deg', 40.0)   # 前方の監視範囲（片側）[deg]
        self.declare_parameter('side_half_deg', 90.0)    # 左右比較の範囲 [deg]
        self.declare_parameter('turn_speed', 0.5)    # 旋回の角速度 [rad/s]

        self.slow_dist = self.get_parameter('slow_dist').value
        self.avoid_dist = self.get_parameter('avoid_dist').value
        self.clear_dist = self.get_parameter('clear_dist').value
        self.front_half = math.radians(self.get_parameter('front_half_deg').value)
        self.side_half = math.radians(self.get_parameter('side_half_deg').value)
        self.turn_speed = self.get_parameter('turn_speed').value

        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.have_scan = False
        self.turn_dir = 0.0   # 0: 回避していない / +1: 左回り / -1: 右回り

        self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_in', 10)
        self.get_logger().info(
            f'減速 {self.slow_dist} m / 旋回 {self.avoid_dist} m / '
            f'再開 {self.clear_dist} m で開始')

    def on_scan(self, msg):
        front = left = right = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self.front_half:
                front = min(front, r)
            if self.front_half < a <= self.side_half:
                left = min(left, r)
            elif -self.side_half <= a < -self.front_half:
                right = min(right, r)
        self.front, self.left, self.right = front, left, right
        self.have_scan = True

    def on_cmd(self, msg):
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular

        # 前進指令のときだけ回避を働かせる（後退・旋回指令はそのまま通す）
        if msg.linear.x <= 0.0 or not self.have_scan:
            self.turn_dir = 0.0
            self.pub.publish(out)
            return

        d = self.front

        if self.turn_dir != 0.0:
            # 回避中：前方が十分に開くまで回り続ける
            if d > self.clear_dist:
                self.turn_dir = 0.0
                self.get_logger().info('前方が開いたので前進を再開')
        elif d < self.avoid_dist:
            # 空いている側を選んで旋回開始
            self.turn_dir = 1.0 if self.left >= self.right else -1.0
            side = '左' if self.turn_dir > 0 else '右'
            self.get_logger().info(
                f'前方 {d:.2f} m: {side}へ回避（左 {self.left:.2f} / 右 {self.right:.2f}）')

        if self.turn_dir != 0.0:
            out.linear.x = 0.0
            out.angular.z = self.turn_dir * self.turn_speed
        elif d < self.slow_dist:
            scale = (d - self.avoid_dist) / (self.slow_dist - self.avoid_dist)
            out.linear.x = msg.linear.x * max(0.2, min(1.0, scale))

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObstacleAvoid()
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
