#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import tf_transformations
from time import sleep

class RealignNode(Node):
    def __init__(self):
        super().__init__('realign_node')
        self.pre_xyz_data = 0.0
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.realign_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def realign_callback(self, amcl_data):
        orientation_q = amcl_data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw= tf_transformations.euler_from_quaternion(orientation_list)
        self.pre_xyz_data = yaw
        self.get_logger().info(f'Yaw from AMCL: {yaw:.4f}')

    def adjust_yaw(self, target_yaw):
        twist = Twist()
        tolerance = 0.05

        yaw_error = target_yaw - self.pre_xyz_data

        if abs(yaw_error) > tolerance:
            twist.angular.z = 0.2 * (1 if yaw_error > 0 else -1)
            self.publisher.publish(twist)
            self.get_logger().info(f'Realigning... Yaw error: {yaw_error:.4f}')
            return False 
        else:
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Yaw aligned!")
            return True 


def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose


def send_goal(nav, realign_node, x, y, yaw):
    goal_pose = create_pose_stamped(nav, x, y, yaw)
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            realign_node.get_logger().info(f'Navigation feedback: {feedback}')
    rate = realign_node.create_rate(10)
    while not realign_node.adjust_yaw(yaw):
        rclpy.spin_once(realign_node)


def main():
    rclpy.init()
    nav = BasicNavigator()
    realign_node = RealignNode()

    initial_pose = create_pose_stamped(nav, 1.84, -9.05, 3.14)
    nav.setInitialPose(initial_pose)

    nav.waitUntilNav2Active()
    send_goal(nav, realign_node, -0.12, -2.35, 3.14)
    sleep(5)
    send_goal(nav, realign_node, 1.86, 2.56, 0.97)
    sleep(5)
    send_goal(nav, realign_node, -3.84, 2.64, 2.78)

    rclpy.shutdown()


if __name__ == '__main__':
    main()