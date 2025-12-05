import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.log = self.get_logger()
        self.declare_parameter('tread', 0.5)
        self.declare_parameter('wheel_radius', 0.25)

        self.targets_pub = self.create_publisher(Float64MultiArray, "/rover/targets", 1)

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 1)

    def cmd_cb(self, twist):
        wheel_msg = Float64MultiArray()

        tread = self.get_parameter('tread').get_parameter_value()._double_value
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value()._double_value

        left_wheel = (twist.linear.x - twist.angular.z * 0.5 * tread) / wheel_radius
        right_wheel = (twist.linear.x + twist.angular.z * 0.5 * tread) / wheel_radius   

        wheel_msg.data = [left_wheel, left_wheel, right_wheel, right_wheel] 

        self.targets_pub.publish(wheel_msg)

        self.log.info(f'targets : {wheel_msg.data} rad/s')

def main():
    rclpy.init()
    rover = Rover()
    try:
        rclpy.spin(rover)
    except:
        return 
    rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
