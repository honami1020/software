import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist

class GpsNavigator(Node):
    """
    GPSとIMUデータを使用して、目標地点までロボットを誘導するROS 2ノード。（PI制御版）
    """
    def __init__(self):
        super().__init__('gps_navigator')

        # 購読（Subscription）
        self.create_subscription(NavSatFix, '/gps/fix', self.current_fix_callback, 10)
        self.create_subscription(NavSatFix, '/goal_fix', self.goal_fix_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # 速度司令のPublisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 状態保持
        self.current_lat = None
        self.current_lon = None
        self.goal_lat = None
        self.goal_lon = None
        self.yaw = None  # 現在の向き（ヨー角, rad）

        # PI制御のためのゲイン
        # 💡調整ポイント💡: ロボットの挙動に合わせて値を調整してください
        self.Kp_angular = 0.25 # 比例ゲイン (旋回能力を向上させるため、値を戻しました)
        self.Ki_angular = 0.03  # 積分ゲイン
        self.Kp_linear = 1.0  # 比例ゲイン

        # 積分誤差の保持
        self.angular_error_integral = 0.0

        # 最高速度
        self.MAX_LINEAR_SPEED = 10.0  # 最高直進速度 (m/s)
        self.MAX_ANGULAR_SPEED = 1.0 # 最高旋回速度 (rad/s)
        
        # ゴール判定距離
        self.GOAL_TOLERANCE = 1.5 # (m)

        self.get_logger().info("GPS Navigator node started.")

    # --- コールバック関数 ---
    def current_fix_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.update_motion()

    def goal_fix_callback(self, msg):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude
        self.get_logger().info(f"New goal received: Lat={self.goal_lat}, Lon={self.goal_lon}")
        # 新しいゴールが設定されたら、積分誤差をリセット
        self.angular_error_integral = 0.0
        self.update_motion()

    def imu_callback(self, msg):
        q = msg.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.update_motion()

    # --- 四元数→ヨー角変換 ---
    def quaternion_to_yaw(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        # ヨー角の範囲を[-π, π]に統一
        return math.atan2(t3, t4)

    # --- Haversine距離計算（m単位） ---
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000.0  # 地球の半径（メートル）
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    # --- 方位角計算 ---
    def bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)

        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        
        return math.atan2(y, x)
        
    def normalize_angle(self, angle):
        """
        角度を -π から π の範囲に正規化する。
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # --- 動作更新 ---
    def update_motion(self):
        # 必要なデータがすべて揃っているか確認
        if None in (self.current_lat, self.current_lon, self.goal_lat, self.goal_lon, self.yaw):
            return

        distance = self.haversine(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)
        target_bearing = self.bearing(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)

        # ターゲット方位と現在のヨー角の差分を計算
        angle_diff = self.normalize_angle(target_bearing - self.yaw)
       
        twist = Twist()
        if distance < self.GOAL_TOLERANCE:
            # ゴールに到達したら停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Goal reached.")
            # 積分誤差をリセット
            self.angular_error_integral = 0.0
        else:
            if distance < 20.0:
                self.angular_error_integral = 0.0
            else:
                self.angular_error_integral += angle_diff
            
            # P制御で直進速度を計算
            linear_speed_base = self.Kp_linear * distance
            # 角度差に応じて直進速度を調整
            angle_speed_ratio = 1.0 - (abs(angle_diff) / math.pi)
            linear_speed = linear_speed_base * angle_speed_ratio
            
            # 💡後退防止ロジックの追加💡
            # 線形速度が負にならないように0以上を保証する
            twist.linear.x = min(self.MAX_LINEAR_SPEED, max(0.0, linear_speed))
            
            # PI制御で角速度を計算
            #angular_speed = (self.Kp_angular * angle_diff) + (self.Ki_angular * self.angular_error_integral)
            #twist.angular.z = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, angular_speed))
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GpsNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # シャットダウン時にロボットを停止させる
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.get_logger().info("Node shutting down, sending zero velocity.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
