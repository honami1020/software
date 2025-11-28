import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math
import transforms3d


class GPSIMUNavigator(Node):
    def __init__(self):
        super().__init__('gps_imu_navigator')
        
        # 目的地座標の設定
        self.goal_latitude = 35.681236
        self.goal_longitude = 139.767125
        
        # 地球の半径（メートル）
        self.earth_radius = 6371000.0
        
        # ロボットのベース速度と停止距離
        self.base_speed = 1.5
        self.stop_distance = 1.0
        
        # データの保存用変数
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        
        # GPSヨー角とIMUヨー角を平滑化するための変数
        self.filtered_yaw = 0.0
        self.alpha = 0.5  # フィルタ係数（0〜1、1に近いほどIMUを重視）


        # 過去のGPS座標を保存
        self.prev_lat = None
        self.prev_lon = None
        
        # ROS 2のトピック設定
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.velocity_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # 制御ロジックを定期的に実行するタイマー
        self.timer = self.create_timer(0.1, self.navigation_timer)
        
        self.get_logger().info('GPS-IMU Navigator Node has been started.')


    def gps_callback(self, msg):
        """GPSデータを受け取る"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude


        # ヨー角の初期化と補正
        if self.prev_lat is None:
            self.prev_lat = self.current_lat
            self.prev_lon = self.current_lon
        else:
            # GPSデータから移動方向を計算してIMUのヨー角を補正
            gps_bearing = self.calculate_bearing(self.prev_lat, self.prev_lon, self.current_lat, self.current_lon)
            if gps_bearing is not None:
                # IMUヨー角とGPS方位角を統合
                self.filtered_yaw = self.alpha * self.current_yaw + (1 - self.alpha) * gps_bearing
            
            self.prev_lat = self.current_lat
            self.prev_lon = self.current_lon


    def imu_callback(self, msg):
        """IMUデータからヨー角を計算する"""
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # クォータニオンからオイラー角に変換
        _, _, self.current_yaw = transforms3d.euler.quat2euler(quaternion)
        
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """2つのGPS座標間の角度を計算する"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)


        delta_lon = lon2_rad - lon1_rad
        
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)


        if x == 0 and y == 0:
            return None


        bearing = math.atan2(x, y)
        return bearing


    def navigation_timer(self):
        """GPSとIMUのデータが揃ったら移動制御を実行する"""
        if self.current_lat is None or self.current_lon is None or self.current_yaw is None:
            self.get_logger().warn('Waiting for GPS and IMU data...')
            return


        goal_lat_rad = math.radians(self.goal_latitude)
        goal_lon_rad = math.radians(self.goal_longitude)
        current_lat_rad = math.radians(self.current_lat)
        current_lon_rad = math.radians(self.current_lon)


        delta_lon = goal_lon_rad - current_lon_rad
        x_diff = self.earth_radius * delta_lon * math.cos(current_lat_rad)
        delta_lat = goal_lat_rad - current_lat_rad
        y_diff = self.earth_radius * delta_lat
        
        distance = math.sqrt(x_diff**2 + y_diff**2)
        
        bearing_to_goal = math.atan2(x_diff, y_diff)
        
        angle_to_turn = bearing_to_goal - self.filtered_yaw
        
        if angle_to_turn > math.pi:
            angle_to_turn -= 2 * math.pi
        elif angle_to_turn < -math.pi:
            angle_to_turn += 2 * math.pi
        
        self.get_logger().info(f'Distance: {distance:.2f}m, Angle to turn: {math.degrees(angle_to_turn):.2f}deg')
        
        self.move_to_goal(distance, angle_to_turn)


    def move_to_goal(self, distance, angle_to_turn):
        """距離と角度に基づいて左右の車輪速度を決定しパブリッシュする"""
        twist_msg = Twist()
        
        if distance < self.stop_distance:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Goal reached and stopping.')
            return


        # 前進速度の調整
        linear_speed = min(3.0, self.base_speed + (distance * 0.5))
        
        # 回転速度の調整（過度な旋回を防ぐ）
        angular_speed = angle_to_turn * 1.5
        
        max_angular_speed = 1.0
        angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_speed))
        
        # デッドバンドを導入して、微小な角度補正による回転を防ぐ
        if abs(angle_to_turn) < math.radians(5):
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = 0.0
        else:
            left_wheel_speed = linear_speed - angular_speed
            right_wheel_speed = linear_speed + angular_speed
            if left_wheel_speed < self.base_speed:
                left_wheel_speed = self.base_speed
            if right_wheel_speed < self.base_speed:
                right_wheel_speed = self.base_speed


            twist_msg.linear.x = (left_wheel_speed + right_wheel_speed) / 2.0
            twist_msg.angular.z = (right_wheel_speed - left_wheel_speed) * 2.0
            
        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = GPSIMUNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



