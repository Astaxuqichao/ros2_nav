#!/usr/bin/env python3
"""
使用 nav2_route 计算路径并通过 FollowPath 执行（交互式手动输入）。

用法:
    ros2 run navflex_costmap_nav route_nav.py

启动后按提示输入目标点，支持连续导航：
    x y [yaw]    -- 发送新目标
    q            -- 退出
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
from nav2_msgs.action import ComputeRoute, FollowPath


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RouteNavClient(Node):
    def __init__(self):
        super().__init__('route_nav_client')
        self._frame = 'map'
        self._busy = False  # 是否有任务正在执行
        self._active_follow_handle = None

        self._route_client = ActionClient(self, ComputeRoute, 'compute_route')
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
        if self._busy and self._active_follow_handle:
            self.get_logger().info('取消当前执行任务...')
            self._active_follow_handle.cancel_goal_async()

        self._busy = True
        self.get_logger().info(f'目标: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        goal_msg = ComputeRoute.Goal()
        goal_msg.use_poses = True
        goal_msg.use_start = False
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.header.frame_id = self._frame
        goal_msg.goal.pose.position.x = x
        goal_msg.goal.pose.position.y = y
        goal_msg.goal.pose.orientation = yaw_to_quaternion(yaw)

        future = self._route_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_route_accepted)

    def _on_route_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('ComputeRoute goal 被拒绝！')
            self._busy = False
            return
        self.get_logger().info('路径规划中...')
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future):
        result = future.result().result
        path = result.path
        if not path.poses:
            self.get_logger().error('路径为空，规划失败！')
            self._busy = False
            return
        self.get_logger().info(f'规划成功：{len(path.poses)} 个路径点，开始执行...')
        self._path_pub.publish(path)
        self._send_follow_path(path)

    def _send_follow_path(self, path):
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        future = self._follow_client.send_goal_async(follow_goal)
        future.add_done_callback(self._on_follow_accepted)

    def _on_follow_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal 被拒绝！')
            self._busy = False
            return
        self._active_follow_handle = handle
        self.get_logger().info('机器人开始运动...')
        handle.get_result_async().add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future):
        result = future.result().result
        self._busy = False
        self._active_follow_handle = None
        if hasattr(result, 'dist_to_goal'):
            self.get_logger().info(
                f'执行完毕 | outcome={result.outcome} '
                f'dist={result.dist_to_goal:.3f}m '
                f'angle={result.angle_to_goal:.3f}rad')
        else:
            self.get_logger().info('执行完毕')
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
