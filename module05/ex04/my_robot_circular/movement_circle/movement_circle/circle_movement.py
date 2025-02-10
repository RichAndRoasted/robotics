from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class FrameListener(Node):

    def __init__(self):
        super().__init__('robot_frame_listener')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 1)


        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 1.0
        self.publisher.publish(msg)



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
