import math
from action_turtle_interfaces.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ActionTurtleServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self.started = False
        self.start_point = {'x': 0, 'y': 0, 'theta': 0}
        self.current_point = {'x': 0, 'y': 0, 'theta': 0}
        server_cb_group = MutuallyExclusiveCallbackGroup()
        subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turtle_commands',
            self.execute_callback,
            callback_group=server_cb_group)
        
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscription_callback, 
            10,
            callback_group=subscription_cb_group)
        self.subscription
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        request = goal_handle.request
        command = Twist()
        if request.command == 'forward':
            self.get_logger().info('Understood, moving forward!')
            command.linear.x = float(request.s)
        elif request.command == 'turn_right':
            self.get_logger().info('Understood, turning right!')
            command.angular.z = - (math.pi / 180 * request.angle)
        elif request.command == 'turn_left':
            self.get_logger().info('Understood, turning left')
            command.angular.z = math.pi / 180 * request.angle
        else:
            self.get_logger().info('"%s" is not a valid command!' % request.command)
            goal_handle.canceled()
            return result
        print(f"Start point: {self.start_point}")
        print(f"Current point: {self.current_point}")
        self.publisher_.publish(command)
        time.sleep(3)
        feedback_msg.odom = int(math.sqrt((self.start_point['x'] - self.current_point['x'])**2 + (self.start_point['y'] - self.current_point['y'])**2))
        goal_handle.publish_feedback(feedback_msg)
        result = MessageTurtleCommands.Result()
        print(f"Start point: {self.start_point}")
        print(f"Current point: {self.current_point}")
        if ( request.s == int(math.sqrt((self.start_point['x'] - self.current_point['x'])**2 + (self.start_point['y'] - self.current_point['y'])**2)) and
                abs(abs(self.start_point['theta'] - self.current_point['theta']) -  (math.pi / 180 * request.angle)) < 0.1):
            goal_handle.succeed()
            result.result = True
        self.start_point['x'] = self.current_point['x']
        self.start_point['y'] = self.current_point['y']
        self.start_point['theta'] = self.current_point['theta']
        return result
    
    def subscription_callback(self, msg):
        if not self.started:
            self.started = True
            self.start_point['x'] = msg.x
            self.start_point['y'] = msg.y
            self.start_point['theta'] = msg.theta
        self.current_point['x'] = msg.x
        self.current_point['y'] = msg.y
        self.current_point['theta'] = msg.theta
        

def main(args=None):
    rclpy.init(args=args)

    action_turtle_server = ActionTurtleServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_turtle_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()