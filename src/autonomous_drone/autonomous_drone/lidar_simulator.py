#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSimulator(Node):

    def __init__(self):
        super().__init__('lidar_simulator')

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data
        )

        # LiDAR configuration
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)

        self.num_rays = 360
        self.range_min = 0.12
        self.range_max = 12.0

        # -------------------------------------------------
        # ENVIRONMENT GEOMETRY
        # Coordinates are in meters.
        # The LiDAR is currently positioned at (0, 0).
        # -------------------------------------------------

        # Walls are line segments:
        # (x1, y1, x2, y2)

        self.walls = [
            # Outer room: 8 m x 6 m
            (-4.0, -3.0, -0.6, -3.0),
            (0.6, -3.0, 4.0, -3.0),
            (-4.0, 3.0, 4.0, 3.0),
            (-4.0, -3.0, -4.0, 3.0),
            (4.0, -3.0, 4.0, 3.0),

            # Upper internal partitions
            (-2.8, 1.4, -2.8, 3.0),
            (-1.2, 2.0, -1.2, 3.0),
            (0.2, 1.8, 0.2, 3.0),
            (1.8, 2.0, 1.8, 3.0),

            # Left-side corridor and alcove
            (-4.0, 0.9, -2.5, 0.9),
            (-2.5, -0.6, -2.5, 0.9),
            (-2.5, -0.6, -1.6, -0.6),

            # Lower-left partitions
            (-3.2, -3.0, -3.2, -1.8),
            (-3.2, -1.8, -2.0, -1.8),
            (-1.1, -3.0, -1.1, -2.0),
            (-1.1, -2.0, -0.4, -2.0),

            # Right-side rooms and passages
            (2.5, 0.8, 4.0, 0.8),
            (2.5, -0.5, 2.5, 0.8),
            (2.5, -0.5, 3.3, -0.5),
            (3.3, -0.5, 3.3, -1.6),

            # Central zigzag corridor
            (-0.7, 1.0, 0.5, 1.0),
            (0.5, 1.0, 0.5, 0.3),
            (0.5, 0.3, 1.3, 0.3),

            # Lower-right partition
            (1.5, -2.0, 3.0, -2.0),
            (1.5, -2.0, 1.5, -1.4),

            # Additional offset walls
            (-0.8, -0.5, -0.8, 0.2),
            (-0.8, 0.2, -0.2, 0.2),
            (1.0, -0.9, 1.9, -0.9),
            (1.9, -0.9, 1.9, -0.2),
        ]

        # Rectangular objects:
        # (center_x, center_y, width, height)

        self.rectangles = [
            (-3.0, 2.0, 0.7, 0.5),
            (-2.0, 1.8, 0.4, 0.4),
            (-3.2, -1.2, 0.6, 0.8),
            (-1.7, -1.6, 0.8, 0.5),
            (-0.1, 2.1, 0.6, 0.4),
            (1.0, 2.0, 0.8, 0.5),
            (2.8, 2.2, 0.9, 0.5),
            (3.1, 1.3, 0.4, 0.7),
            (2.8, -1.2, 0.5, 0.5),
            (2.5, -2.5, 0.7, 0.4),
            (0.8, -2.4, 0.5, 0.6),
            (-0.2, -1.1, 0.4, 0.7),
        ]

        # Circular obstacles:
        # (center_x, center_y, radius)

        self.circles = [
            (-1.9, 1.0, 0.28),
            (-0.5, 2.0, 0.22),
            (1.4, 1.4, 0.30),
            (3.2, 0.0, 0.25),
            (-3.0, -2.3, 0.30),
            (-1.5, -1.1, 0.35),
            (0.8, -1.5, 0.25),
            (2.2, -1.0, 0.30),
            (2.8, -2.6, 0.20),
        ]

        # Curved wall segments:
        # (center_x, center_y, radius, start_angle, end_angle)
        # Angles are in radians.

        self.curved_walls = [
            (-1.7, 0.0, 0.85, -1.1, 1.1),
            (1.7, 0.0, 0.85, math.pi - 1.1, math.pi + 1.1),
            (0.0, -1.9, 0.55, 0.2, 2.5),
            (-3.0, 0.0, 0.45, -1.2, 1.2),
        ]

        # Convert rectangles into wall segments.
        for cx, cy, width, height in self.rectangles:
            x1 = cx - width / 2
            x2 = cx + width / 2
            y1 = cy - height / 2
            y2 = cy + height / 2

            self.walls.extend([
                (x1, y1, x2, y1),
                (x2, y1, x2, y2),
                (x2, y2, x1, y2),
                (x1, y2, x1, y1),
            ])

        self.timer = self.create_timer(
            0.1,
            self.publish_scan
        )

        self.get_logger().info(
            'Complex LiDAR simulator started. '
            'Publishing /scan at 10 Hz with 360 rays.'
        )

    @staticmethod
    def cross(ax, ay, bx, by):
        return ax * by - ay * bx

    def ray_segment_intersection(
        self, dx, dy, x1, y1, x2, y2
    ):
        """Return the distance to a ray/segment intersection."""

        sx = x2 - x1
        sy = y2 - y1

        denominator = self.cross(dx, dy, sx, sy)

        if abs(denominator) < 1e-9:
            return None

        t = self.cross(x1, y1, sx, sy) / denominator
        u = self.cross(x1, y1, dx, dy) / denominator

        if t > 0.0 and 0.0 <= u <= 1.0:
            return t

        return None

    def ray_circle_intersection(
        self, dx, dy, cx, cy, radius
    ):
        """Return the nearest positive ray/circle intersection."""

        projection = cx * dx + cy * dy
        discriminant = (
            projection * projection
            - (cx * cx + cy * cy - radius * radius)
        )

        if discriminant < 0.0:
            return None

        root = math.sqrt(discriminant)

        candidates = [
            projection - root,
            projection + root,
        ]

        positive = [t for t in candidates if t > 0.0]

        return min(positive) if positive else None

    def ray_arc_intersection(
        self, dx, dy, cx, cy, radius,
        start_angle, end_angle
    ):
        """Return an intersection if it lies on a curved wall."""

        distance = self.ray_circle_intersection(
            dx, dy, cx, cy, radius
        )

        if distance is None:
            return None

        hit_x = distance * dx
        hit_y = distance * dy

        hit_angle = math.atan2(
            hit_y - cy,
            hit_x - cx
        )

        hit_angle %= 2.0 * math.pi
        start_angle %= 2.0 * math.pi
        end_angle %= 2.0 * math.pi

        if start_angle <= end_angle:
            on_arc = start_angle <= hit_angle <= end_angle
        else:
            on_arc = (
                hit_angle >= start_angle
                or hit_angle <= end_angle
            )

        return distance if on_arc else None

    def calculate_range(self, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)

        nearest = self.range_max

        # Straight walls and rectangular obstacles
        for x1, y1, x2, y2 in self.walls:
            distance = self.ray_segment_intersection(
                dx, dy, x1, y1, x2, y2
            )

            if distance is not None and distance < nearest:
                nearest = distance

        # Circular obstacles
        for cx, cy, radius in self.circles:
            distance = self.ray_circle_intersection(
                dx, dy, cx, cy, radius
            )

            if distance is not None and distance < nearest:
                nearest = distance

        # Curved walls
        for cx, cy, radius, start, end in self.curved_walls:
            distance = self.ray_arc_intersection(
                dx, dy, cx, cy, radius, start, end
            )

            if distance is not None and distance < nearest:
                nearest = distance

        if nearest < self.range_min:
            return float('inf')

        return float(nearest)

    def publish_scan(self):
        scan = LaserScan()

        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.time_increment = 0.0
        scan.scan_time = 0.1

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = [
            self.calculate_range(
                self.angle_min + i * self.angle_increment
            )
            for i in range(self.num_rays)
        ]

        scan.intensities = []

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    node = LidarSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
