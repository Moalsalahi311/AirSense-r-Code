#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        # Publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Motion parameters
        self.forward_speed = 0.25     # m/s
        self.turn_speed = 0.6         # rad/s
        self.stop_distance = 0.5      # m

        # Memory of last turn direction (1 = left, -1 = right)
        self.last_turn_direction = 1

        self.get_logger().info('Improved LiDAR obstacle avoidance node started')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Split regions (front ±30°, left 30–90°, right -90–-30°)
        front_range = self._region_min(ranges, msg.angle_min, msg.angle_increment, -30, 30)
        left_range  = self._region_min(ranges, msg.angle_min, msg.angle_increment, 30, 90)
        right_range = self._region_min(ranges, msg.angle_min, msg.angle_increment, -90, -30)

        self.get_logger().info(
            f"Front: {front_range:.2f}  Left: {left_range:.2f}  Right: {right_range:.2f}"
        )

        cmd = Twist()

        # --- Main decision logic ---
        if front_range > self.stop_distance:
            # Front clear → move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        else:
            # Obstacle ahead → stop and turn
            cmd.linear.x = 0.0

            # If both sides blocked, continue turning same direction as before
            if left_range < self.stop_distance and right_range < self.stop_distance:
                cmd.angular.z = self.turn_speed * self.last_turn_direction
                self.get_logger().info("Corner detected — continuing same turn direction")
            else:
                # Choose freer side
                if left_range > right_range:
                    cmd.angular.z = self.turn_speed      # turn left
                    self.last_turn_direction = 1
                else:
                    cmd.angular.z = -self.turn_speed     # turn right
                    self.last_turn_direction = -1

        self.cmd_pub.publish(cmd)

    def _region_min(self, ranges, angle_min, angle_inc, start_deg, end_deg):
        """Return minimum valid range value within given angular sector."""
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        start_idx = int((start_rad - angle_min) / angle_inc)
        end_idx = int((end_rad - angle_min) / angle_inc)
        sector = [r for r in ranges[min(start_idx, end_idx):max(start_idx, end_idx)] if not math.isinf(r)]
        if len(sector) == 0:
            return float('inf')
        return min(sector)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

