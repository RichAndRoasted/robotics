from action_turtle_interfaces.action import MessageTurtleCommands
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionTurtleClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'message_turtle_commands')
        self.status = GoalStatus.STATUS_EXECUTING

    def send_goal(self, command, s, angle):
        self.status = GoalStatus.STATUS_EXECUTING
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        self.status = GoalStatus.STATUS_SUCCEEDED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.odom))


def main(args=None):
    rclpy.init(args=args)

    action_turtle_client = ActionTurtleClient()
    goals = [
        ['forward', 2, 0],
        ['turn_right', 0, 90],
        ['forward', 1, 0]
    ]
    for goal in goals:
        
        action_turtle_client.send_goal(*goal)

        while action_turtle_client.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(action_turtle_client)

    action_turtle_client.get_logger().info('All done!')
    action_turtle_client.destroy_node()

if __name__ == '__main__':
    main()