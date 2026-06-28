#!/usr/bin/env python3
"""
使用 nav2_route ComputeAndTrackRoute 规划并跟踪路径，通过 FollowPath 执行。

用法:
    ros2 run navflex_costmap_nav route_nav.py

启动后按提示输入目标点，支持连续导航：
    x y [yaw]    -- 发送新目标
    q            -- 退出

ComputeAndTrackRoute 会持续跟踪机器人在路网上的进度，
当发生重规划时自动取消 FollowPath 并用新路径重新执行。
"""

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import ComputeAndTrackRoute, FollowPath


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RouteNavClient(Node):
    def __init__(self):
        super().__init__('route_nav_client')
        self._frame = 'map'
        self._busy = False
        self._active_route_handle = None   # ComputeAndTrackRoute handle
        self._active_follow_handle = None  # FollowPath handle
        self._following_path = False       # FollowPath 是否正在执行

        self._route_client = ActionClient(self, ComputeAndTrackRoute, 'compute_and_track_route')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self._path_pub = self.create_publisher(Path, 'route_plan', _latched_qos)

        self.get_logger().info('等待 action 服务器...')
        self._route_client.wait_for_server()
        self._follow_client.wait_for_server()
        self.get_logger().info('Action 服务器已就绪！')

    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        """发送新目标，若有正在执行的任务先取消"""
        if self._busy:
            self.get_logger().info('取消当前任务...')
            if self._active_follow_handle:
                self._active_follow_handle.cancel_goal_async()
            if self._active_route_handle:
                self._active_route_handle.cancel_goal_async()

        self._busy = True
        self._following_path = False
        self.get_logger().info(f'目标: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        goal_msg = ComputeAndTrackRoute.Goal()
        goal_msg.use_poses = True
        goal_msg.use_start = False
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.header.frame_id = self._frame
        goal_msg.goal.pose.position.x = x
        goal_msg.goal.pose.position.y = y
        goal_msg.goal.pose.orientation = yaw_to_quaternion(yaw)

        future = self._route_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_route_feedback)
        future.add_done_callback(self._on_route_accepted)

    def _on_route_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('ComputeAndTrackRoute goal 被拒绝！')
            self._busy = False
            return
        self._active_route_handle = handle
        self.get_logger().info('路网跟踪已启动，等待路径...')
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        path = fb.path

        if not path.poses:
            return

        # 首次收到路径，或路由重规划时，重启 FollowPath
        if not self._following_path or fb.rerouted:
            if fb.rerouted:
                self.get_logger().info(
                    f'路由重规划！新路径 {len(path.poses)} 个点，重启 FollowPath...')
                if self._active_follow_handle:
                    self._active_follow_handle.cancel_goal_async()
                    self._active_follow_handle = None
            else:
                self.get_logger().info(
                    f'收到初始路径：{len(path.poses)} 个点，开始执行...')

            self._path_pub.publish(path)
            self._send_follow_path(path)

        # 更新当前节点信息日志
        if fb.next_node_id or fb.last_node_id:
            self.get_logger().debug(
                f'路网进度: last_node={fb.last_node_id} → next_node={fb.next_node_id} '
                f'edge={fb.current_edge_id}')

    def _send_follow_path(self, path):
        self._following_path = True
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'

        future = self._follow_client.send_goal_async(follow_goal)
        future.add_done_callback(self._on_follow_accepted)

    def _on_follow_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal 被拒绝！')
            self._following_path = False
            return
        self._active_follow_handle = handle
        self.get_logger().info('机器人开始运动...')
        handle.get_result_async().add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future):
        self._active_follow_handle = None
        self._following_path = False
        result = future.result().result
        if hasattr(result, 'dist_to_goal'):
            self.get_logger().info(
                f'FollowPath 完成 | outcome={result.outcome} '
                f'dist={result.dist_to_goal:.3f}m '
                f'angle={result.angle_to_goal:.3f}rad')

    def _on_route_result(self, future):
        self._busy = False
        self._active_route_handle = None
        status = future.result().status
        # 取消 FollowPath（若还在运行）
        if self._active_follow_handle:
            self._active_follow_handle.cancel_goal_async()
            self._active_follow_handle = None
        self._following_path = False
        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('路网导航完成！')
        else:
            self.get_logger().warn(f'路网导航结束，status={status}')
        print('\n输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)


def input_loop(node: RouteNavClient):
    """在独立线程中处理用户输入"""
    print('\n输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            print('输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)
            continue
        if line.lower() == 'q':
            print('退出')
            rclpy.shutdown()
            return
        parts = line.split()
        if len(parts) < 2:
            print('格式错误，请输入: x y [yaw]')
            print('输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
        except ValueError:
            print('坐标必须是数字')
            print('输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)
            continue
        node.navigate_to(x, y, yaw)
        print('输入目标点 (x y [yaw]) 或 q 退出: ', end='', flush=True)


def main():
    rclpy.init()
    node = RouteNavClient()

    # 输入循环放在独立线程，rclpy.spin 在主线程
    t = threading.Thread(target=input_loop, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
