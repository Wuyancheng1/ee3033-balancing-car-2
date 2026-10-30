
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        # Parameters for waypoint generation
        self.declare_parameter('spacing', 0.8)
        self.declare_parameter('clearance', 0.35)
        self.declare_parameter('free_threshold', 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Receive the saved occupancy-grid map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Publish search waypoints for the mission controller
        self.waypoint_pub = self.create_publisher(
            PoseArray,
            '/search_waypoints',
            10
        )

        self.get_logger().info(
            'Exploration node started. Waiting for /map...'
        )

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        spacing = float(
            self.get_parameter('spacing').value
        )
        clearance = float(
            self.get_parameter('clearance').value
        )
        threshold = int(
            self.get_parameter('free_threshold').value
        )

        if width == 0 or height == 0 or resolution <= 0:
            return

        if spacing <= 0 or clearance < 0:
            self.get_logger().error('Invalid parameters')
            return

        grid = msg.data
        step = max(1, math.ceil(spacing / resolution))
        radius = math.ceil(clearance / resolution)

        # Generate candidates using a boustrophedon
        # (alternating left-to-right/right-to-left) route
        candidates = []

        rows = list(range(radius, height - radius, step))

        for row_index, y in enumerate(rows):
            columns = list(
                range(radius, width - radius, step)
            )

            if row_index % 2 == 1:
                columns.reverse()

            for x in columns:
                if self.is_safe(
                    grid, x, y, width, height,
                    radius, threshold
                ):
                    candidates.append((x, y))

        # Convert candidates to map-frame coordinates
        waypoints = PoseArray()
        waypoints.header.frame_id = 'map'
        waypoints.header.stamp = (
            self.get_clock().now().to_msg()
        )

        origin = msg.info.origin
        q = origin.orientation

        origin_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        for x, y in candidates:
            local_x = (x + 0.5) * resolution
            local_y = (y + 0.5) * resolution

            pose = Pose()
            pose.position.x = (
                origin.position.x
                + math.cos(origin_yaw) * local_x
                - math.sin(origin_yaw) * local_y
            )
            pose.position.y = (
                origin.position.y
                + math.sin(origin_yaw) * local_x
                + math.cos(origin_yaw) * local_y
            )
            pose.orientation.w = 1.0

            waypoints.poses.append(pose)

        self.waypoint_pub.publish(waypoints)

        self.get_logger().info(
            f'Generated {len(waypoints.poses)} '
            'candidate search waypoints'
        )

    def is_safe(
        self, grid, x, y, width, height,
        radius, threshold
    ):
        # Check a circular area around each waypoint
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue

                nx = x + dx
                ny = y + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    return False

                value = grid[ny * width + nx]

                # Reject unknown or occupied cells
                if value < 0 or value > threshold:
                    return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
