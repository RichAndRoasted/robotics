import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import rclpy
import math
from rclpy.node import Node


class MoveToGoal(Node):

    def __init__(self):
        super().__init__('move_to_goal')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.subscription_callback, 10)
        self.timer = self.create_timer(0.1, self.move_to_goal)
        self.pose = Pose()
        self.goal_reached = False

    def move_to_goal(self):
        goal = Pose()
        goal.x = float(sys.argv[1])
        goal.y = float(sys.argv[2])
        goal.theta = float(sys.argv[3])
        delta_x = goal.x - self.pose.x
        delta_y = goal.y - self.pose.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        turn = math.atan2(delta_y, delta_x)
        distance_tolerance = 0.1
        angle_tolerance = 0.01
        angle_error = turn - self.pose.theta
        kp = 1.4
        new_vel = Twist()
        if not self.goal_reached:
            if abs(angle_error) > angle_tolerance:
                new_vel.angular.z = kp * angle_error
            else :
                if( distance ) >= distance_tolerance:
                    new_vel.linear.x = kp * distance
                else :
                    new_vel.linear.x = 0.0
                    self.get_logger().info("Goal Reached ")
                    self.goal_reached = True
        else:
            angle_error = goal.theta - self.pose.theta
            if abs(angle_error) < angle_tolerance:
                quit()
            new_vel.angular.z = kp * angle_error
            

        self.cmd_vel_pub.publish(new_vel)


    def subscription_callback(self, msg):
        self.pose = msg
        


def main(args=None):
    rclpy.init(args=args)
    move_to_goal = MoveToGoal()
    rclpy.spin(move_to_goal)
    move_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()