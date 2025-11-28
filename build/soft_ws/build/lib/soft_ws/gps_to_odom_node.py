import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographiclib.geodesic import Geodesic

class GpsToOdomNode(Node):
    def __init__(self):
        super().__init__('gps_to_odom_node')

        # 原点（ローカル座標系の0,0にするGPS地点）
        self.origin_lat = 0.000029  # 必要に応じて変更
        self.origin_lon = 0.000101

        self.geod = Geodesic.WGS84

        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/current_pose', 10)

    def gps_callback(self, msg):
        if msg.status.status < 0:
            self.get_logger().warn('No GPS fix.')
            return

        # 原点からの相対座標（メートル単位）を計算
        result = self.geod.Inverse(self.origin_lat, self.origin_lon, msg.latitude, msg.longitude)

        x = result['s12'] * math.sin(math.radians(result['azi1']))
        y = result['s12'] * math.cos(math.radians(result['azi1']))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'  # 必ず 'odom' とする
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # 標高は今回は使わない

        # 向きは不明なので単位クォータニオン（無回転）
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        self.pub.publish(pose)
        self.get_logger().info(f'Published pose: x={x:.2f}, y={y:.2f}')

import math

def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdomNode()
    try:
        rclpy.spin(node)
    except:
        return
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
