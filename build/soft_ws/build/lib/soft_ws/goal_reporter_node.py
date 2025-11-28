# goal_reporter_node.py（例）
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GoalReporter(Node):
    def __init__(self):
        super().__init__('goal_reporter_node')
        self.publisher = self.create_publisher(NavSatFix, '/goal_fix', 10)
        self.timer = self.create_timer(0.1, self.publish_goal)

    def publish_goal(self):
        goal = NavSatFix()
        goal.latitude = 0.000108    # 目標緯度
        goal.longitude = 0.000201# 目標経度
        goal.altitude = 0.0
        self.publisher.publish(goal)
        self.get_logger().info('Published goal GPS')

def main(args=None):
    rclpy.init(args=args)
    node = GoalReporter()
    try:
        rclpy.spin(node)
    except:
        return
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()