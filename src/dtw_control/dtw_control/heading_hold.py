import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lim):
    return max(-lim, min(lim, x))


class HeadingHold(Node):
    """cmd_vel_in を受け取り、直進時は向きを保つ補正を加えて cmd_vel に出す。"""

    def __init__(self):
        super().__init__('heading_hold')
        self.declare_parameter('kp', 1.5)          # 向き誤差 -> 角速度
        self.declare_parameter('ki', 30.0)         # 曲率バイアスの学習ゲイン
        self.declare_parameter('max_w', 0.5)       # 角速度の上限 [rad/s]
        self.declare_parameter('max_k', 0.1)       # 曲率バイアスの上限 [rad/m]
        self.declare_parameter('settle_time', 0.5)  # 旋回後に学習を止める時間 [s]
        self.declare_parameter('learn_err_max', 0.15)  # これを超える誤差では学習しない [rad]
        self.declare_parameter('input_timeout', 1.0)  # 指令が途絶えたら停止 [s]
        self.declare_parameter(
            'bias_file', os.path.expanduser('~/.ros/dtw_heading_bias.json'))

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.max_w = self.get_parameter('max_w').value
        self.max_k = self.get_parameter('max_k').value
        self.settle = self.get_parameter('settle_time').value
        self.learn_err_max = self.get_parameter('learn_err_max').value
        self.timeout = self.get_parameter('input_timeout').value
        self.bias_file = self.get_parameter('bias_file').value

        self.k = clamp(self.load_bias(), self.max_k)  # 学習する曲率バイアス [rad/m]
        self.yaw = None
        self.target = None
        self.straight_since = None
        self.cmd = Twist()
        self.last_cmd_time = None
        self.stopped = True
        self.dt = 0.05
        self.tick = 0

        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(self.dt, self.loop)
        self.get_logger().info(f'学習済みバイアス k = {self.k:.4f} rad/m で開始')

    def load_bias(self):
        try:
            with open(self.bias_file) as f:
                return float(json.load(f)['k'])
        except Exception:
            return 0.0

    def save_bias(self):
        try:
            os.makedirs(os.path.dirname(self.bias_file), exist_ok=True)
            with open(self.bias_file, 'w') as f:
                json.dump({'k': self.k}, f)
        except Exception as e:
            self.get_logger().warn(f'バイアス保存に失敗: {e}')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_odom(self, msg):
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def loop(self):
        now = self.get_clock().now()
        # 指令が途絶えたら停止（Gazebo プラグインは最後の指令を保持するため）
        if (self.last_cmd_time is None or
                (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout):
            if not self.stopped:
                self.pub.publish(Twist())
                self.stopped = True
                self.save_bias()
            self.target = None
            self.straight_since = None
            return
        self.stopped = False

        v = self.cmd.linear.x
        w_in = self.cmd.angular.z
        out = Twist()
        out.linear.x = v

        if self.yaw is None:
            out.angular.z = w_in
        elif abs(w_in) < 1e-3 and abs(v) > 1e-3:
            # 直進モード
            if self.straight_since is None:
                self.straight_since = now
            t = (now - self.straight_since).nanoseconds * 1e-9
            if t < self.settle or self.target is None:
                # 旋回直後：向きに追従するだけ（学習しない）
                self.target = self.yaw
                out.angular.z = clamp(self.k * v, self.max_w)
            else:
                e = wrap(self.target - self.yaw)
                if abs(e) < self.learn_err_max:
                    self.k = clamp(self.k + self.ki * e * v * self.dt, self.max_k)
                out.angular.z = clamp(self.kp * e + self.k * v, self.max_w)
        else:
            # 旋回中・停止中：目標をリセットし、バイアス分だけ補正
            self.target = None
            self.straight_since = None
            out.angular.z = clamp(w_in + self.k * v, self.max_w)

        self.pub.publish(out)

        self.tick += 1
        if self.tick % int(5.0 / self.dt) == 0:
            self.get_logger().info(f'学習中の k = {self.k:.4f} rad/m')


def main():
    rclpy.init()
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_bias()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
