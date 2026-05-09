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

        # 当前目标 & 请求序号
        self.current_goal_pose = None
        self.request_seq = 0
        self.latest_request_id = 0
        self.active_follow_goal_handle = None
        # 标记上一轮规划+跟踪流程是否已结束，避免并发
        self._planning_in_progress = False

        # 每 1 s 触发一次规划
        self.plan_timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("SimpleNavClient node initialized, waiting for /goal_pose...")

    def goal_callback(self, goal_pose):
        self.current_goal_pose = goal_pose
        self.get_logger().info(
            f"Received new goal at: "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )

    def timer_callback(self):
        if self.current_goal_pose is None:
            return

        if self._planning_in_progress:
            self.get_logger().warn("Previous plan still in progress, skipping this cycle")
            return

        if not self.planner_client.server_is_ready() or not self.follower_client.server_is_ready():
            self.get_logger().warn("Action servers not ready yet, skipping")
            return

        self._planning_in_progress = True
        self.request_seq += 1
        request_id = self.request_seq
        self.latest_request_id = request_id

        # 构造路径规划请求
        path_goal = ComputePathToPose.Goal()
        path_goal.goal = self.current_goal_pose
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
            self._planning_in_progress = False
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
            self._planning_in_progress = False
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
            self._planning_in_progress = False
            return

        self.active_follow_goal_handle = goal_handle
        self.get_logger().info(f"[req={request_id}] FollowPath goal accepted.")
        # 解锁，允许下一个 1s 周期重新规划（FollowPath server 收到新 goal 会自动抢占旧的）
        self._planning_in_progress = False
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
            # outcome=0 表示成功到达，清除目标，停止重规划
            if result.outcome == 0:
                self.get_logger().info(f"[req={request_id}] Goal reached, clearing target.")
                self.current_goal_pose = None
        else:
            self.get_logger().info(
                f"[req={request_id}] FollowPath result received (no message field in this interface version)."
            )

        self._planning_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
