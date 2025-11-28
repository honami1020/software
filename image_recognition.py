import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('image_recognition.py')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info('カメラ受信ノードを開始しました。')

    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'画像変換エラー: {e}')
            return
        image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 赤のしきい値
        lower_red1 = np.array([0, 0, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 0, 80])
        upper_red2 = np.array([179, 255, 255])

        # 二値化 & ノイズ除去
        mask1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((10, 10), np.uint8)
        mask_cleaned = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)

        # ピクセル数カウント
        white_pixels = np.sum(mask_cleaned == 255)
        black_pixels = np.sum(mask_cleaned == 0)
        all_pixels = white_pixels + black_pixels

        self.get_logger().info(f"白(赤)ピクセル: {white_pixels}, 合計: {all_pixels}")

        # 進むか止まるか判断
        if all_pixels > 0 and white_pixels < 180000:
            self.publish_velocity(0.2, 0.0)  # 前進
        else:
            self.publish_velocity(0.0, 0.0)  # 停止

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f"速度指令: linear_x={linear_x}, angular_z={angular_z}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ノードを停止します。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()