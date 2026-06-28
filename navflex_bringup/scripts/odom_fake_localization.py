#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_normalize(q):
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    rotated = quaternion_multiply(quaternion_multiply(q, qv), qc)
    return rotated[:3]


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return math.atan2(siny_cosp, cosy_cosp)


def compose_transform(a_translation, a_rotation, b_translation, b_rotation):
    rotated_b_translation = rotate_vector(a_rotation, b_translation)
    return (
        (
            a_translation[0] + rotated_b_translation[0],
            a_translation[1] + rotated_b_translation[1],
            a_translation[2] + rotated_b_translation[2],
        ),
        quaternion_normalize(quaternion_multiply(a_rotation, b_rotation)),
    )


def invert_transform(translation, rotation):
    inverted_rotation = quaternion_conjugate(rotation)
    inverted_translation = rotate_vector(
        inverted_rotation,
        (-translation[0], -translation[1], -translation[2]))
    return inverted_translation, inverted_rotation


class OdomFakeLocalization(Node):

    def __init__(self):
        super().__init__('odom_fake_localization')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initialpose_topic', 'initialpose')
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.z', 0.0)
        self.declare_parameter('initial_pose.yaw', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.initial_position = (
            float(self.get_parameter('initial_pose.x').value),
            float(self.get_parameter('initial_pose.y').value),
            float(self.get_parameter('initial_pose.z').value),
        )
        self.initial_orientation = yaw_to_quaternion(
            float(self.get_parameter('initial_pose.yaw').value))
        self.map_to_odom = None
        self.last_odom = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.on_odom,
            10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value,
            self.on_initial_pose,
            10)

        self.get_logger().info(
            f'Publishing simulated localization from {self.get_parameter("odom_topic").value}: '
            f'{self.map_frame} -> {self.odom_frame}')

    def on_odom(self, msg):
        self.last_odom = msg
        odom_to_base_translation = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        odom_to_base_rotation = quaternion_normalize((
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ))

        if self.map_to_odom is None:
            self.map_to_odom = self.compute_map_to_odom(
                self.initial_position,
                self.initial_orientation,
                odom_to_base_translation,
                odom_to_base_rotation)
            self.get_logger().info(
                'Initialized simulated localization from first odom sample')

        map_to_odom_translation, map_to_odom_rotation = self.map_to_odom

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_to_odom_translation[0]
        transform.transform.translation.y = map_to_odom_translation[1]
        transform.transform.translation.z = map_to_odom_translation[2]
        transform.transform.rotation.x = map_to_odom_rotation[0]
        transform.transform.rotation.y = map_to_odom_rotation[1]
        transform.transform.rotation.z = map_to_odom_rotation[2]
        transform.transform.rotation.w = map_to_odom_rotation[3]
        self.tf_broadcaster.sendTransform(transform)

    def on_initial_pose(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'Ignoring initial pose in frame {msg.header.frame_id}; expected {self.map_frame}')
            return
        if self.last_odom is None:
            self.get_logger().warn('Initial pose received before odom; waiting for odom')
            return

        odom_to_base_translation = (
            self.last_odom.pose.pose.position.x,
            self.last_odom.pose.pose.position.y,
            self.last_odom.pose.pose.position.z,
        )
        odom_to_base_rotation = quaternion_normalize((
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
            self.last_odom.pose.pose.orientation.w,
        ))
        requested_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        requested_orientation = quaternion_normalize((
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ))
        self.map_to_odom = self.compute_map_to_odom(
            requested_position,
            requested_orientation,
            odom_to_base_translation,
            odom_to_base_rotation)
        self.get_logger().info(
            f'Reset simulated localization to x={requested_position[0]:.3f}, '
            f'y={requested_position[1]:.3f}, yaw={quaternion_to_yaw(requested_orientation):.3f}')

    def compute_map_to_odom(
        self,
        map_to_base_translation,
        map_to_base_rotation,
        odom_to_base_translation,
        odom_to_base_rotation,
    ):
        base_to_odom = invert_transform(
            odom_to_base_translation,
            odom_to_base_rotation)
        return compose_transform(
            map_to_base_translation,
            map_to_base_rotation,
            base_to_odom[0],
            base_to_odom[1])

    def reset_localization(self):
        self.map_to_odom = None


def main():
    rclpy.init()
    node = OdomFakeLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
