#!/usr/bin/env python3

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


class ContinuousWaypointLoop(Node):
    """Continuously send a list of waypoints to Nav2."""

    def __init__(self):
        super().__init__('continuous_waypoint_loop')

        # Waypoints are provided as:
        # [x1, y1, x2, y2, x3, y3, ...]
        self.declare_parameter(
            'waypoints_xy',
            [
                0.5, 0.2,
                1.5, 0.2,
                1.5, 1.0,
                0.5, 1.0,
            ],
        )

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('repeat_delay', 1.0)

        raw_waypoints = list(
            self.get_parameter('waypoints_xy').value
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.repeat_delay = float(
            self.get_parameter('repeat_delay').value
        )

        if len(raw_waypoints) < 4:
            raise ValueError(
                'At least two waypoints are required.'
            )

        if len(raw_waypoints) % 2 != 0:
            raise ValueError(
                'waypoints_xy must contain x-y pairs.'
            )

        self.waypoints = []

        for i in range(0, len(raw_waypoints), 2):
            x = float(raw_waypoints[i])
            y = float(raw_waypoints[i + 1])
            self.waypoints.append((x, y))

        self.action_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints',
        )

        self.goal_active = False
        self.next_send_time = time.monotonic()
        self.server_wait_message_printed = False

        # Check every 0.5 seconds whether a route should be sent.
        self.timer = self.create_timer(
            0.5,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints.'
        )

        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(
                f'Waypoint {index + 1}: '
                f'x={waypoint[0]:.2f}, '
                f'y={waypoint[1]:.2f}'
            )

    def timer_callback(self):
        """Send the waypoint route when Nav2 is ready."""

        if self.goal_active:
            return

        if time.monotonic() < self.next_send_time:
            return

        if not self.action_client.wait_for_server(
            timeout_sec=0.1
        ):
            if not self.server_wait_message_printed:
                self.get_logger().info(
                    'Waiting for follow_waypoints action server...'
                )
                self.server_wait_message_printed = True
            return

        self.server_wait_message_printed = False
        self.send_waypoint_route()

    def make_pose(self, index):
        """Create one PoseStamped waypoint."""

        x, y = self.waypoints[index]

        # Point the robot toward the next waypoint.
        next_index = (index + 1) % len(self.waypoints)
        next_x, next_y = self.waypoints[next_index]

        yaw = math.atan2(
            next_y - y,
            next_x - x,
        )

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_waypoint_route(self):
        """Send all waypoints as one FollowWaypoints goal."""

        goal = FollowWaypoints.Goal()

        goal.poses = [
            self.make_pose(index)
            for index in range(len(self.waypoints))
        ]

        self.get_logger().info(
            'Sending waypoint route: '
            + ' → '.join(
                str(index + 1)
                for index in range(len(self.waypoints))
            )
        )

        self.goal_active = True

        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        send_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle acceptance or rejection from Nav2."""

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to send waypoint goal: {error}'
            )
            self.schedule_retry()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                'Waypoint route was rejected.'
            )
            self.schedule_retry()
            return

        self.get_logger().info(
            'Waypoint route was accepted.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Display the waypoint currently being executed."""

        current_index = (
            feedback_msg.feedback.current_waypoint
        )

        self.get_logger().info(
            f'Moving toward waypoint '
            f'{current_index + 1}/'
            f'{len(self.waypoints)}'
        )

    def result_callback(self, future):
        """Restart the route after it has finished."""

        try:
            result_wrapper = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to obtain waypoint result: {error}'
            )
            self.schedule_retry()
            return

        status = result_wrapper.status
        result = result_wrapper.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Waypoint route completed.'
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                'Waypoint route was aborted.'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                'Waypoint route was canceled.'
            )
        else:
            self.get_logger().warn(
                f'Waypoint route ended with status {status}.'
            )

        if result.missed_waypoints:
            self.get_logger().warn(
                'Missed waypoint indices: '
                f'{list(result.missed_waypoints)}'
            )

        # After a short delay, send the same route again.
        self.schedule_retry()

    def schedule_retry(self):
        self.goal_active = False
        self.next_send_time = (
            time.monotonic() + self.repeat_delay
        )


def main(args=None):
    rclpy.init(args=args)

    node = ContinuousWaypointLoop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()