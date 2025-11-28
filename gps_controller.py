import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class GPSController(Node):
    def __init__(self):
        super().__init__('gps_controller')

        # ゴール座標の設定（例: ロボットの初期位置）
        self.goal_latitude = 35.681236
        self.goal_longitude = 139.767125

        # 地球の半径（メートル）
        self.earth_radius = 6371000

        # ROS 2のトピック設定
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )
        self.velocity_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.get_logger().info('GPS Controller Node has been started.')

    def gps_callback(self, msg):
        """GPSデータを受け取り、相対座標と移動速度を計算する"""
        current_latitude = msg.latitude
        current_longitude = msg.longitude

        # 緯度経度をラジアンに変換
        lat_rad = math.radians(current_latitude)
        lon_rad = math.radians(current_longitude)
        goal_lat_rad = math.radians(self.goal_latitude)
        goal_lon_rad = math.radians(self.goal_longitude)

        # 緯度経度から相対座標（X, Y）に変換
        delta_lon = lon_rad - goal_lon_rad
        x = delta_lon * self.earth_radius * math.cos(lat_rad)
        
        delta_lat = lat_rad - goal_lat_rad
        y = delta_lat * self.earth_radius
        
        self.get_logger().info(f'Relative position: X={x:.2f}m, Y={y:.2f}m')
        
        # 移動速度の計算
        self.move_to_goal_with_differential_drive(x, y)

    def move_to_goal_with_differential_drive(self, x, y):
        """相対座標に基づいて左右の車輪速度を決定しパブリッシュする"""
        twist_msg = Twist()
        
        # ゴールまでの距離と角度を計算
        distance = math.sqrt(x**2 + y**2)
        angle_to_goal = math.atan2(y, x)

        # ゴールに十分に近づいたら停止
        if distance < 0.5:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Goal reached!')
        else:
            # 鋭角か鈍角かを判定
            is_acute_angle = abs(angle_to_goal) < math.pi / 2
            
            # 最大速度を設定
            max_linear_speed = 0.5
            
            # ゴールへの角度に応じて左右の車輪の速度を計算
            if angle_to_goal > 0: # ゴールが左にある場合
                if is_acute_angle: # 鋭角
                    # 左車輪を減速して左に旋回
                    left_wheel_speed = max_linear_speed * (1 - angle_to_goal)
                    right_wheel_speed = max_linear_speed
                else: # 鈍角
                    # ロボットを大きく旋回させるため、右車輪の速度を高く維持
                    left_wheel_speed = max_linear_speed * (1 - angle_to_goal)
                    right_wheel_speed = max_linear_speed
            else: # ゴールが右にある場合
                if is_acute_angle: # 鋭角
                    # 右車輪を減速して右に旋回
                    left_wheel_speed = max_linear_speed
                    right_wheel_speed = max_linear_speed * (1 + angle_to_goal)
                else: # 鈍角
                    # ロボットを大きく旋回させるため、左車輪の速度を高く維持
                    left_wheel_speed = max_linear_speed
                    right_wheel_speed = max_linear_speed * (1 + angle_to_goal)
            
            # 速度を負の値にしないように調整
            left_wheel_speed = max(0.0, left_wheel_speed)
            right_wheel_speed = max(0.0, right_wheel_speed)

            # 左右の車輪速度からTwistメッセージを生成
            # linear.xは平均速度、angular.zは左右の速度差に比例
            twist_msg.linear.x = (left_wheel_speed + right_wheel_speed) / 2.0
            twist_msg.angular.z = (right_wheel_speed - left_wheel_speed) / 2.0
            
        self.velocity_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    gps_controller = GPSController()
    rclpy.spin(gps_controller)
    gps_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()