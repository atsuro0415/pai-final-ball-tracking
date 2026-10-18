import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

def clamp(x, lim):
    return max(-lim, min(lim, x))

class BallChase(Node):
    """カメラで緑のボールの方向と距離を求め、障害物を避けながら追いかける。

    距離は画像上のボールの大きさから推定する。LiDAR の値がそれより
    大幅に手前なら、ボールの手前に障害物があると判断して回り込む。
    回避のあとや見失ったときは、回転と前後移動を交互に行ってボールを探す。
    cmd_vel_in に直接出力する（heading_hold が受け取る）。

    モード: chase（追跡）/ turn（回避の旋回）/ bypass（脇を抜ける直進）/ search（探索）
    """

    def __init__(self):
        super().__init__('ball_chase')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('info_topic', 'camera/camera_info')
        self.declare_parameter('fallback_hfov', 1.396)   # camera_info が無いときの水平画角 [rad]
        self.declare_parameter('ball_diameter', 0.2)     # ボールの直径 [m]
        self.declare_parameter('stop_dist', 0.65)    # ボールの手前で停止する距離 [m]
        self.declare_parameter('ball_slow_dist', 1.5)  # ボールへの接近時の減速開始距離 [m]
        self.declare_parameter('max_speed', 0.3)     # 最大前進速度 [m/s]
        self.declare_parameter('kv', 0.6)            # 距離誤差 -> 速度
        self.declare_parameter('kw', 1.2)            # 方向誤差 -> 角速度
        self.declare_parameter('max_w', 0.5)         # 角速度の上限 [rad/s]
        self.declare_parameter('deadband', 0.03)     # この角度以内は直進 [rad]
        self.declare_parameter('min_pixels', 15)     # 検出とみなす最小画素数
        self.declare_parameter('green_margin', 50)   # 緑が他の色より強い量
        # 障害物の回避
        self.declare_parameter('blocked_margin', 0.7)   # ボール推定距離より手前なら障害物 [m]
        self.declare_parameter('slow_dist', 1.5)     # 障害物の手前で減速開始 [m]
        self.declare_parameter('avoid_dist', 0.8)    # 障害物の手前で旋回開始 [m]
        self.declare_parameter('clear_dist', 1.2)    # 前方がこれ以上開いたら旋回終了 [m]
        self.declare_parameter('front_half_deg', 40.0)   # 前方の監視範囲（片側）[deg]
        self.declare_parameter('turn_speed', 0.5)    # 回避の旋回速度 [rad/s]
        self.declare_parameter('bypass_time', 3.0)   # 脇を抜けるために直進する時間 [s]
        self.declare_parameter('bypass_speed', 0.25) # そのときの速度 [m/s]
        # 探索（往復回転）
        self.declare_parameter('sweep_time', 3.0)    # 最初の回転時間 [s]（次は2倍の時間で逆回転）
        self.declare_parameter('sweep_w', 0.5)       # 探索の回転速度 [rad/s]
        self.declare_parameter('lost_hold', 2.0)     # ボール近くで見失ったとき止まって待つ時間 [s]
        self.declare_parameter('arrived_reset_time', 10.0)  # 到達後に見失ってから探索再開する時間 [s]
        self.declare_parameter('arrive_hyst', 0.35)   # 到達状態を解除する距離の余裕 [m]

        g = lambda n: self.get_parameter(n).value
        self.image_topic = g('image_topic')
        self.info_topic = g('info_topic')
        self.fallback_hfov = g('fallback_hfov')
        self.ball_d = g('ball_diameter')
        self.stop_dist = g('stop_dist')
        self.max_speed = g('max_speed')
        self.ball_slow_dist = g('ball_slow_dist')
        if self.ball_slow_dist <= self.stop_dist:
            raise ValueError('ball_slow_dist must be greater than stop_dist')
        self.kv = g('kv')
        self.kw = g('kw')
        self.max_w = g('max_w')
        self.deadband = g('deadband')
        self.min_pixels = g('min_pixels')
        self.margin = g('green_margin')
        self.blocked_margin = g('blocked_margin')
        self.slow_dist = g('slow_dist')
        self.avoid_dist = g('avoid_dist')
        self.clear_dist = g('clear_dist')
        self.front_half = math.radians(g('front_half_deg'))
        self.turn_speed = g('turn_speed')
        self.bypass_time = g('bypass_time')
        self.bypass_speed = g('bypass_speed')
        self.sweep_time = g('sweep_time')
        self.sweep_w = g('sweep_w')
        self.lost_hold = g('lost_hold')
        self.arrive_hyst = g('arrive_hyst')
        self.arrived_reset_time = float(g('arrived_reset_time'))
        if not math.isfinite(self.arrived_reset_time) or self.arrived_reset_time <= 0.0:
            raise ValueError('arrived_reset_time must be positive and finite')
        self.arrived_lost_since = None

        self.fx = None
        self.cx = None
        self.scan = None
        self.last_sign = 1.0         # 最後にボールが見えた側（+1: 左, -1: 右）
        self.last_bearing = 0.0
        self.last_ball_dist = None   # 最後に見えたときのボールまでの距離
        self.warned_enc = False
        self.mode = 'chase'
        self.turn_dir = 0.0
        self.bypass_end = None
        self.search_dir = 1.0
        self.search_start = None
        self.lost_since = None
        self.arrived = False
        # 距離は指令速度×シミュレーション時間による概算。
        self.search_phase = 'rotate_forward'
        self.search_phase_start = None
        self.search_speed = 0.15
        self.search_forward_dist = 0.5
        self.search_reverse_dist = 1.0
        self.scan_received = None

        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, 'cmd_vel_in', 10)
        self.get_logger().info(
            f'ボール追跡を開始（画像: {self.image_topic}、停止距離 {self.stop_dist} m）')

    def on_info(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]

    def on_scan(self, msg):
        self.scan = msg
        self.scan_received = self.get_clock().now()

    def range_at(self, bearing, half=0.14):
        """bearing（左が正）を中心に ±half の範囲の最短距離。見えなければ inf。"""
        if self.scan is None:
            return float('inf')
        s = self.scan
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            a = s.angle_min + i * s.angle_increment
            d = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
            if abs(d) <= half and r < best:
                best = r
        return best

    def start_turn(self, ball_sign):
        """空いている側へ旋回を始める。左右がほぼ同じならボールのある側へ。"""
        left = min(self.range_at(1.2, 0.5), 8.0)
        right = min(self.range_at(-1.2, 0.5), 8.0)
        if abs(left - right) < 0.1:
            self.turn_dir = ball_sign
        else:
            self.turn_dir = 1.0 if left > right else -1.0
        self.mode = 'turn'
        side = '左' if self.turn_dir > 0 else '右'
        self.get_logger().info(
            f'障害物を避けるため{side}へ旋回（左 {left:.2f} / 右 {right:.2f}）')

    def start_search(self, now, direction):
        self.mode = 'search'
        self.search_dir = direction
        self.search_start = now
        self.search_phase = 'rotate_forward'
        self.search_phase_start = now
        self.get_logger().info('移動探索を開始：回転 → 前進0.5 m → 回転 → 後退1.0 m')

    def search_motion(self, now):
        """回転と直線移動を交互に行う。ボール検出は呼出元で毎画像確認。"""
        out = Twist()
        elapsed = (now - self.search_phase_start).nanoseconds * 1e-9
        phase = self.search_phase
        duration = (2.0 * self.sweep_time if phase.startswith('rotate') else
                    (self.search_forward_dist if phase == 'forward'
                     else self.search_reverse_dist) / self.search_speed)
        if elapsed < 0.0:
            self.start_search(now, self.search_dir)
            return out
        if elapsed >= duration:
            phases = ['rotate_forward', 'forward', 'rotate_reverse', 'reverse']
            self.search_phase = phases[(phases.index(phase) + 1) % len(phases)]
            self.search_phase_start = now
            if self.search_phase == 'rotate_forward':
                self.search_dir *= -1.0
            self.get_logger().info(f'[探索] {self.search_phase}')
            return out  # 切替時は一度停止
        if phase.startswith('rotate'):
            out.angular.z = self.search_dir * self.sweep_w
            return out

        bearing = 0.0 if phase == 'forward' else math.pi
        clear = self.search_path_clear(bearing, now)
        if not clear:
            # 計測できない方向・障害物のある方向には移動せず次の回転へ。
            self.search_phase = ('rotate_reverse' if phase == 'forward'
                                 else 'rotate_forward')
            self.search_phase_start = now
            if self.search_phase == 'rotate_forward':
                self.search_dir *= -1.0
            self.get_logger().info('[探索] 移動方向の計測不足または障害物：移動を省略')
            return out
        out.linear.x = self.search_speed if phase == 'forward' else -self.search_speed
        return out

    def search_path_clear(self, bearing, now):
        if self.scan is None or self.scan_received is None:
            return False
        age = (now - self.scan_received).nanoseconds * 1e-9
        if not 0.0 <= age <= 0.5:
            return False
        # +/-25度を5分割し、全区間に有効な計測があることを確認。
        half = math.radians(25.0)
        covered = set()
        for i, r in enumerate(self.scan.ranges):
            a = self.scan.angle_min + i * self.scan.angle_increment
            d = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
            if abs(d) > half:
                continue
            valid = (math.isfinite(r) and
                     self.scan.range_min <= r <= self.scan.range_max)
            # +inf は検出範囲内に反射なし。NaN、-inf、範囲外は不明。
            if not valid and r != float('inf'):
                continue
            covered.add(min(4, int((d + half) / (2.0 * half) * 5)))
            if valid and r <= self.avoid_dist:
                return False
        return len(covered) == 5

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            if not self.warned_enc:
                self.get_logger().warn(f'未対応の画像形式: {msg.encoding}')
                self.warned_enc = True
            return

        now = self.get_clock().now()
        front = self.range_at(0.0, self.front_half)
        out = Twist()

        # --- 回避中の動作 ---
        if self.mode == 'turn':
            if front > self.clear_dist:
                self.mode = 'bypass'
                self.bypass_end = now + Duration(seconds=self.bypass_time)
                self.get_logger().info('前方が開いた：脇を抜けるため直進')
            else:
                out.angular.z = self.turn_dir * self.turn_speed
                self.pub.publish(out)
                return
        if self.mode == 'bypass':
            if now < self.bypass_end and front > self.avoid_dist * 0.6:
                out.linear.x = self.bypass_speed
                self.pub.publish(out)
                return
            # 避けた向きの反対側にボールがある
            self.start_search(now, -self.turn_dir)

        # --- ボールの検出 ---
        h, w = msg.height, msg.width
        step = 2   # 計算量を減らすため、2画素おきに見る
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)[::step, ::step]
        if msg.encoding == 'rgb8':
            r, g_, b = arr[..., 0], arr[..., 1], arr[..., 2]
        else:
            b, g_, r = arr[..., 0], arr[..., 1], arr[..., 2]
        g16 = g_.astype(np.int16)
        mask = ((g16 - r.astype(np.int16) > self.margin) &
                (g16 - b.astype(np.int16) > self.margin) & (g_ > 80))
        ys, xs = np.nonzero(mask)

        if xs.size < self.min_pixels:
            # ---- 見失った ----
            if self.arrived:
                if self.arrived_lost_since is None:
                    self.arrived_lost_since = now
                    self.get_logger().info(
                        f'到達後にボールを見失った：{self.arrived_reset_time:.1f}秒待機')
                elapsed = (now - self.arrived_lost_since).nanoseconds * 1e-9
                if elapsed < 0.0:  # シミュレーション時刻が巻き戻った場合
                    self.arrived_lost_since = now
                elif elapsed >= self.arrived_reset_time:
                    self.arrived = False
                    self.arrived_lost_since = None
                    self.last_ball_dist = None
                    self.last_bearing = 0.0
                    self.lost_since = None
                    self.turn_dir = 0.0
                    self.bypass_end = None
                    self.get_logger().info('到達後の見失いが継続：到達状態をリセットして探索再開')
                    self.start_search(now, self.last_sign)
                    out = self.search_motion(now)
                self.pub.publish(out)
                return
            if self.mode == 'search':
                out = self.search_motion(now)
                self.pub.publish(out)
                self.get_logger().info(
                    f'ボールを探索中：{self.search_phase}', throttle_duration_sec=2.0)
                return

            lb = self.last_ball_dist
            r_last = self.range_at(self.last_bearing)
            # 最後に見たボールより明らかに手前に物があれば、障害物
            obstacle_ahead = (front < self.avoid_dist and
                              (lb is None or r_last < lb - self.blocked_margin))
            near_ball = (lb is not None and not obstacle_ahead and
                         lb <= self.avoid_dist + 0.4)

            if obstacle_ahead:
                self.start_turn(self.last_sign)
                out.angular.z = self.turn_dir * self.turn_speed
                self.pub.publish(out)
            elif near_ball:
                # ボールの近くで見失った：止まって少し待つ
                if self.lost_since is None:
                    self.lost_since = now
                    self.get_logger().info(
                        f'近くでボールを見失った（最後の距離 {lb:.2f} m / 前方 {front:.2f} m）：停止')
                if (now - self.lost_since).nanoseconds * 1e-9 < self.lost_hold:
                    self.pub.publish(out)   # 停止指令
                else:
                    self.lost_since = None
                    self.start_search(now, self.last_sign)
                    out = self.search_motion(now)
                    self.pub.publish(out)
            else:
                self.start_search(now, self.last_sign)
                out = self.search_motion(now)
                self.pub.publish(out)
            return

        # ---- ボールが見えた ----
        self.arrived_lost_since = None
        self.lost_since = None
        if self.mode == 'search':
            self.mode = 'chase'
            self.get_logger().info('ボールを発見：追跡を再開')

        u = float(xs.mean()) * step + step / 2.0
        width_px = (float(xs.max()) - float(xs.min()) + 1.0) * step
        cx = self.cx if self.cx is not None else w / 2.0
        if self.fx is not None:
            fx = self.fx
        else:
            fx = (w / 2.0) / math.tan(self.fallback_hfov / 2.0)
        bearing = math.atan2(cx - u, fx)   # 左が正
        self.last_sign = 1.0 if bearing >= 0.0 else -1.0
        self.last_bearing = bearing

        # --- 距離：画像から推定し、LiDAR と照らし合わせる ---
        dist_cam = fx * self.ball_d / max(width_px, 1.0)
        r_l = self.range_at(bearing)
        blocked = math.isfinite(r_l) and r_l < dist_cam - self.blocked_margin
        if math.isfinite(r_l) and not blocked:
            ball_dist = r_l       # LiDAR がボール自身を捉えている
        else:
            ball_dist = dist_cam  # 障害物に隠れている、または LiDAR の範囲外
        self.last_ball_dist = ball_dist

        scale = 1.0
        if blocked:
            if r_l < self.avoid_dist:
                self.start_turn(self.last_sign)
                out.angular.z = self.turn_dir * self.turn_speed
                self.pub.publish(out)
                return
            if r_l < self.slow_dist:
                scale = (r_l - self.avoid_dist) / (self.slow_dist - self.avoid_dist)
                scale = max(0.2, min(1.0, scale))

        # --- 到達判定（ヒステリシス付き） ---
        if not blocked:
            if ball_dist <= self.stop_dist:
                if not self.arrived:
                    self.get_logger().info(f'ボールに到達（距離 {ball_dist:.2f} m）：停止')
                self.arrived = True
            elif ball_dist > self.stop_dist + self.arrive_hyst:
                self.arrived = False
        else:
            self.arrived = False

        # --- 追跡の制御 ---
        if abs(bearing) > self.deadband:
            out.angular.z = clamp(self.kw * bearing, self.max_w)

        if self.arrived:
            out.angular.z = 0.0
            v = 0.0
        else:
            approach_ratio = max(0.0, min(1.0,
                (ball_dist - self.stop_dist) /
                (self.ball_slow_dist - self.stop_dist)))
            speed_cap = self.max_speed * approach_ratio
            v = max(0.0, min(self.kv * (ball_dist - self.stop_dist), speed_cap))
            v *= scale
            v *= max(0.0, 1.0 - abs(bearing) / 0.6)
        out.linear.x = v

        self.pub.publish(out)
        state = '障害物あり' if blocked else ('到達' if self.arrived else '追跡')
        self.get_logger().info(
            f'[{state}] 方向 {math.degrees(bearing):+.1f}° / ボール {ball_dist:.2f} m'
            f'（画像 {dist_cam:.2f} / LiDAR {r_l:.2f}）/ 速度 {v:.2f}',
            throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = BallChase()
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