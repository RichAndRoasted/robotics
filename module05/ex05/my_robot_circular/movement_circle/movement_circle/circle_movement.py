from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

class FrameListener(Node):
    
    def __init__(self):
        super().__init__('robot_frame_listener')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 1)
        
        self.direction = 1.0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.duration = 6.2831 
        self.timer = self.create_timer(1e-6, self.on_timer)

    def on_timer(self):
        msg = Twist()
        msg.linear.x = 1.0
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time
        if elapsed_time > self.duration:
            self.start_time = current_time
            self.direction *= -1
        msg.angular.z = 0.5 * self.direction
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
