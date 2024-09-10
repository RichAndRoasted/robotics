import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TextToCmdVel(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        res = Twist()
        if msg.data == "turn_right":
            res.angular.z = -1.57079632679
            self.get_logger().info('Understood, turning right!')
        elif msg.data == "turn_left":
            res.angular.z = 1.57079632679 
            self.get_logger().info('Understood, turning left!')
        elif msg.data == "move_forward":
            res.linear.x = 2.0
            self.get_logger().info('Understood, moving forward!')
        elif msg.data == "move_backward":
            res.linear.x = -2.0
            self.get_logger().info('Understood, moving backward!')
        else:
            self.get_logger().info('"%s" is not a valid command!' % msg.data)
        self.publisher_.publish(res)


def main(args=None):
    rclpy.init(args=args)

    text_to_cmd_vel = TextToCmdVel()

    rclpy.spin(text_to_cmd_vel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    text_to_cmd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()