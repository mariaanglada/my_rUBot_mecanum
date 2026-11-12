#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler


class TrafficSignNavigation(Node):

    def __init__(self):
        super().__init__('traffic_sign_navigation')

        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('inspection_pose1', [0.5, 0.0, 0.0])
        self.declare_parameter('left_waypoint1', [1.0, 0.5, 0.0])
        self.declare_parameter('right_waypoint1', [1.0, -0.5, 0.0])
        self.declare_parameter('final_pose1', [3.0, 0.0, 1.57])
        self.declare_parameter('inspection_pose2', [2.0, 0.0, 3.14])
        self.declare_parameter('left_waypoint2', [1.0, -0.5, 0.0])
        self.declare_parameter('right_waypoint2', [1.0, 0.5, 0.0])
        self.declare_parameter('final_pose2', [0.0, 0.0, 0.0])
        self.declare_parameter('stop_duration', 3.0)

        self.initial_pose = self.get_parameter('initial_pose').value
        self.inspection_pose1 = self.get_parameter('inspection_pose1').value
        self.left_waypoint1 = self.get_parameter('left_waypoint1').value
        self.right_waypoint1 = self.get_parameter('right_waypoint1').value
        self.final_pose1 = self.get_parameter('final_pose1').value
        self.inspection_pose2 = self.get_parameter('inspection_pose2').value
        self.left_waypoint2 = self.get_parameter('left_waypoint2').value
        self.right_waypoint2 = self.get_parameter('right_waypoint2').value
        self.final_pose2 = self.get_parameter('final_pose2').value
        self.stop_duration = self.get_parameter('stop_duration').value

        self.navigator = BasicNavigator()
        self.detected_sign = None
        self.accept_sign = False

        # TODO 1: Create a String subscriber for /traffic_sign.
        self.sign_subscriber = self.create_subscription(
            String, '/traffic_sign', self.sign_callback, 10)

    def sign_callback(self, msg):
        # TODO 2: While accept_sign is True, store the sign in lower case.
        if self.accept_sign:
            self.detected_sign = msg.data.strip().lower()

    def create_pose(self, values):
        x, y, yaw = values
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def go_to(self, values, name):
        self.get_logger().info(f'Going to {name}')
        self.navigator.goToPose(self.create_pose(values))

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_sign(self):
        self.detected_sign = None
        self.accept_sign = True
        self.get_logger().info('Waiting for a traffic sign on /traffic_sign ...')

        # TODO 3: Process callbacks until detected_sign is not None.
        while rclpy.ok() and self.detected_sign is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.accept_sign = False
        return self.detected_sign

    def handle_sign(self, sign, left_wp, right_wp, final_pose):
        """Execute the behaviour for a sign. Returns True if the sign was
        recognised and the robot reached the final pose, False otherwise."""
        # TODO 4: Implement left, right, stop and unknown-sign behaviour.
        if sign == 'left':
            # Go to left point, then to final point
            self.go_to(left_wp, 'left waypoint')
            self.go_to(final_pose, 'final pose')
            return True
        elif sign == 'right':
            # Go to right point, then to final point
            self.go_to(right_wp, 'right waypoint')
            self.go_to(final_pose, 'final pose')
            return True
        elif sign == 'stop':
            # Wait for the specified duration, then go to the final pose
            self.get_logger().info(f'Stop sign: waiting {self.stop_duration:.1f} s')
            time.sleep(self.stop_duration)
            self.go_to(final_pose, 'final pose')
            return True
        else:
            self.get_logger().info(
                f'Unknown sign "{sign}": staying at inspection pose')
            return False

    def run(self):
        self.navigator.setInitialPose(self.create_pose(self.initial_pose))
        self.navigator.waitUntilNav2Active()

        # ---------- Stage 1: inspection pose 1 ----------
        self.go_to(self.inspection_pose1, 'inspection pose 1')
        sign = self.wait_for_sign()

        ok = self.handle_sign(
            sign,
            self.left_waypoint1,
            self.right_waypoint1,
            self.final_pose1)
        if not ok:
            return

        # ---------- Stage 2: from final pose 1 to inspection pose 2 ----------
        self.go_to(self.inspection_pose2, 'inspection pose 2')
        sign = self.wait_for_sign()

        ok = self.handle_sign(
            sign,
            self.left_waypoint2,
            self.right_waypoint2,
            self.final_pose2)
        if ok:
            self.get_logger().info('Route finished at final pose 2')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNavigation()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()