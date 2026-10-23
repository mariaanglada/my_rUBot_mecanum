#!/usr/bin/env python3
"""Student template for the holonomic obstacle wall follower."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicObstacleWallFollower(Node):
    """Complete the LiDAR processing, state machine and control laws."""

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        # Desired distance between the robot and the right wall (m).
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        # Maximum lateral distance used to classify a short obstacle (m).
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        # Forward velocity (m/s).
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.20).value
        )
        # Maximum lateral velocity (m/s).
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.20).value
        )
        # Turn speed and maximum angular correction (rad/s).
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 1.00).value
        )
        # Maximum LiDAR distance considered (m).
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )
        # Proportional gain for lateral-distance correction.
        self.lateral_gain = float(
            self.declare_parameter('lateral_gain', 2.0).value
        )
        # Proportional gain for angular alignment.
        self.alignment_gain = float(
            self.declare_parameter('alignment_gain', 2.0).value
        )

        self.RIGHT = 'RIGHT'
        self.FRONT = 'FRONT'
        self.FRONT_RIGHT = 'FRONT_RIGHT'
        self.BACK_RIGHT = 'BACK_RIGHT'

        self.diagonal_factor = 0.5
        self.region_confirmations = 3
        self.robust_rays = 3

        self.state = self.RIGHT
        self.obstacle_is_long = False
        self.passing_short_obstacle = False
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.right_angle = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            'Student template started. Complete the TODO sections.'
        )

    def publish_command(self):
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        """Store planar pose (x, y, yaw) from odometry."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.odom_pose = (position.x, position.y, yaw)

    def region_measurements(self, scan):
        """Return robust distances and angles for the four LiDAR regions."""
        rays = {
            self.FRONT: [],
            self.FRONT_RIGHT: [],
            self.RIGHT: [],
            self.BACK_RIGHT: [],
        }

        for i, distance in enumerate(scan.ranges):
            # Ignore invalid and out-of-range measurements
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

            if -20.0 <= angle <= 20.0:
                rays[self.FRONT].append((distance, angle))
            elif -70.0 <= angle < -20.0:
                rays[self.FRONT_RIGHT].append((distance, angle))
            elif -110.0 <= angle < -70.0:
                rays[self.RIGHT].append((distance, angle))
            elif -160.0 <= angle < -110.0:
                rays[self.BACK_RIGHT].append((distance, angle))

        measurements = {}
        for region, region_rays in rays.items():
            if not region_rays:
                measurements[region] = (math.inf, None)
                continue
            nearest = sorted(region_rays, key=lambda ray: ray[0])
            nearest = nearest[:self.robust_rays]
            mean_distance = sum(r[0] for r in nearest) / len(nearest)
            mean_angle = sum(r[1] for r in nearest) / len(nearest)
            measurements[region] = (mean_distance, mean_angle)
        return measurements

    def closest_region(self, measurements):
        """Return the closest region inside detection_limit."""
        best_region = None
        best_distance = self.detection_limit
        for region, (distance, _) in measurements.items():
            if distance < best_distance:
                best_distance = distance
                best_region = region
        return best_region

    def confirmed_region(self, measurements):
        """Confirm a region change over several consecutive scans."""
        closest = self.closest_region(measurements)

        if closest == self.stable_region:
            self.region_candidate = None
            self.region_candidate_count = 0
            return self.stable_region

        if closest == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = closest
            self.region_candidate_count = 1

        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = self.region_candidate
            self.region_candidate = None
            self.region_candidate_count = 0
        return self.stable_region

    def lateral_progress(self):
        """Return travel along the robot's initial lateral axis."""
        if self.odom_pose is None or self.front_start_pose is None:
            return 0.0
        x0, y0, yaw0 = self.front_start_pose
        x, y, _ = self.odom_pose
        dx = x - x0
        dy = y - y0
        return abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    def change_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'{self.state} -> {new_state}')
            self.state = new_state

    def follow_right(self, measurements, closest, cmd):
        """Follow the wall using lateral and angular correction."""
        if closest == self.FRONT:
            self.change_state(self.FRONT)
            self.front_start_pose = self.odom_pose
            self.obstacle_is_long = False
            self.follow_front(measurements, closest, cmd)
            return

        # Short obstacle passed: the right obstacle is now behind us
        if self.passing_short_obstacle and closest == self.BACK_RIGHT:
            self.change_state(self.BACK_RIGHT)
            self.follow_back_right(measurements, closest, cmd)
            return

        cmd.linear.x = self.forward_speed

        distance, angle = measurements[self.RIGHT]
        if not math.isfinite(distance):
            return

        # Too far from the wall -> move right (negative vy)
        distance_error = distance - self.follow_distance
        cmd.linear.y = clamp(
            -self.lateral_gain * distance_error,
            -self.lateral_speed,
            self.lateral_speed,
        )

        # Closest rays should be at -90 deg
        if not self.passing_short_obstacle and angle is not None:
            angle_error = math.radians(angle + 90.0)
            cmd.angular.z = clamp(
                self.alignment_gain * angle_error,
                -self.turn_speed,
                self.turn_speed,
            )
        else:
            cmd.angular.z = 0.0

    def follow_front(self, measurements, closest, cmd):
        """Move laterally and classify the obstacle as short or long."""
        cmd.linear.x = 0.0
        cmd.linear.y = self.lateral_speed

        progress = self.lateral_progress()

        if progress >= self.max_front_length:
            # Long wall: turn left until it becomes the new right wall
            self.obstacle_is_long = True
            self.change_state(self.FRONT_RIGHT)
            return

        if closest != self.FRONT:
            # FRONT cleared before max_front_length -> short obstacle
            self.passing_short_obstacle = True
            self.change_state(self.FRONT_RIGHT)

    def follow_front_right(self, measurements, closest, cmd):
        """Move diagonally around the front-right side."""
        if self.obstacle_is_long:
            if closest == self.RIGHT:
                self.obstacle_is_long = False
                self.passing_short_obstacle = False
                self.front_start_pose = None
                self.change_state(self.RIGHT)
                return
            cmd.angular.z = self.turn_speed
            return

        # Short obstacle: go forward and slightly left, away from the corner
        cmd.linear.x = self.forward_speed
        cmd.linear.y = self.diagonal_factor * self.lateral_speed

        if closest == self.RIGHT:
            self.change_state(self.RIGHT)   # temporary RIGHT (w = 0)
        elif closest == self.FRONT:
            self.front_start_pose = self.odom_pose
            self.change_state(self.FRONT)

    def follow_back_right(self, measurements, closest, cmd):
        """Complete the bypass and recover the right wall."""
        cmd.linear.x = self.forward_speed
        cmd.linear.y = -self.diagonal_factor * self.lateral_speed

        if closest == self.RIGHT:
            self.passing_short_obstacle = False   # alignment enabled again
            self.front_start_pose = None
            self.change_state(self.RIGHT)
        elif closest == self.FRONT:
            self.front_start_pose = self.odom_pose
            self.change_state(self.FRONT)

    def scan_callback(self, scan):
        """Read the LiDAR and calculate the next velocity command."""
        measurements = self.region_measurements(scan)
        closest = self.confirmed_region(measurements)
        cmd = Twist()

        if self.state == self.RIGHT:
            self.follow_right(measurements, closest, cmd)
        elif self.state == self.FRONT:
            self.follow_front(measurements, closest, cmd)
        elif self.state == self.FRONT_RIGHT:
            self.follow_front_right(measurements, closest, cmd)
        elif self.state == self.BACK_RIGHT:
            self.follow_back_right(measurements, closest, cmd)

        self.command = cmd

    def stop(self):
        """Publish a final zero command."""
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = HolonomicObstacleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

