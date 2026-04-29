#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SimpleNavClient(Node):
    def __init__(self):
        super().__init__('simple_nav_client')

        # 初始化 action 客户端
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follower_client = ActionClient(self, FollowPath, 'follow_path')

        # 订阅 /goal_pose
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos
        )

        # Track in-flight requests to avoid mixing callbacks across goals.
        self.request_seq = 0
        self.latest_request_id = 0
        self.active_follow_goal_handle = None

        self.get_logger().info("SimpleNavClient node initialized, waiting for /goal_pose...")

    def goal_callback(self, goal_pose):
        self.request_seq += 1
        request_id = self.request_seq
        self.latest_request_id = request_id

        self.get_logger().info(
            f"[req={request_id}] Received new goal at: "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )

        # 直接下发新目标，FollowPath server 收到新 goal 后会自动替换当前执行。
        # 无需主动 cancel 上一个 goal。

        # 等待 action server 可用
        self.planner_client.wait_for_server()
        self.follower_client.wait_for_server()

        # 构造路径规划请求
        path_goal = ComputePathToPose.Goal()
        path_goal.goal = goal_pose
        path_goal.planner_id = 'GridBased'

        # 异步发送 ComputePathToPose
        self.get_logger().info(f"[req={request_id}] Sending ComputePathToPose goal...")
        send_goal_future = self.planner_client.send_goal_async(path_goal)
        send_goal_future.add_done_callback(
            lambda f, rid=request_id: self.on_planner_response(f, rid)
        )

    def on_planner_response(self, future, request_id):
        if request_id != self.latest_request_id:
            self.get_logger().warn(f"[req={request_id}] Ignoring stale planner response")
            return

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"[req={request_id}] ComputePathToPose goal was rejected.")
            return

        self.get_logger().info(f"[req={request_id}] ComputePathToPose goal accepted.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f, rid=request_id: self.on_path_received(f, rid)
        )

    def on_path_received(self, future, request_id):
        if request_id != self.latest_request_id:
            self.get_logger().warn(f"[req={request_id}] Ignoring stale planner result")
            return

        result = future.result().result
        path = result.path

        self.get_logger().info(
            f"[req={request_id}] Planner result: outcome={result.outcome}, "
            f"message='{result.message}', poses={len(path.poses)}"
        )

        if not path.poses:
            self.get_logger().error(
                f"[req={request_id}] Received empty path, aborting follow. "
                f"outcome={result.outcome}, message='{result.message}'"
            )
            return

        self.get_logger().info(
            f"[req={request_id}] Received path with {len(path.poses)} poses. Sending to FollowPath..."
        )

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        send_goal_future = self.follower_client.send_goal_async(follow_goal)
        send_goal_future.add_done_callback(
            lambda f, rid=request_id: self.on_follow_accepted(f, rid)
        )

    def on_follow_accepted(self, future, request_id):
        goal_handle = future.result()

        if request_id != self.latest_request_id:
            self.get_logger().warn(f"[req={request_id}] Stale FollowPath goal accepted; canceling it")
            if goal_handle and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"[req={request_id}] FollowPath goal was rejected.")
            return

        self.active_follow_goal_handle = goal_handle
        self.get_logger().info(f"[req={request_id}] FollowPath goal accepted.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f, rid=request_id: self.on_follow_completed(f, rid)
        )

    def on_follow_completed(self, future, request_id):
        if request_id != self.latest_request_id:
            self.get_logger().warn(f"[req={request_id}] Ignoring stale FollowPath result")
            return

        result = future.result().result
        self.get_logger().info(f"[req={request_id}] FollowPath execution complete!")
        if hasattr(result, 'message'):
            self.get_logger().info(
                f"[req={request_id}] outcome={result.outcome}, message='{result.message}', "
                f"dist_to_goal={result.dist_to_goal:.3f}, angle_to_goal={result.angle_to_goal:.3f}"
            )
        else:
            self.get_logger().info(
                f"[req={request_id}] FollowPath result received (no message field in this interface version)."
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
