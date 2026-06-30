#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class GeojsonMarkerArrayPublisher(Node):
    def __init__(self):
        super().__init__("geojson_marker_array_publisher")

        bringup_dir = Path(get_package_share_directory("navflex_bringup"))
        default_geojson = bringup_dir / "maps" / "aws_small_house_semantics.geojson"
        if not default_geojson.exists():
            default_geojson = bringup_dir / "params" / "sample_graph.geojson"

        self.declare_parameter("geojson_file", str(default_geojson))
        self.declare_parameter("topic", "/geojson_marker_array")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_once", False)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("line_width", 0.08)
        self.declare_parameter("node_scale", 0.35)
        self.declare_parameter("z", 0.03)
        self.declare_parameter("show_labels", True)

        geojson_file = Path(
            self.get_parameter("geojson_file").get_parameter_value().string_value
        ).expanduser()
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._publish_once = self.get_parameter("publish_once").value
        self._publish_period = max(
            0.1, self.get_parameter("publish_period").get_parameter_value().double_value
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(MarkerArray, topic, qos)

        self._marker_array = self._load_geojson(geojson_file)
        self._timer = None
        self._publish()
        if not self._publish_once:
            self._timer = self.create_timer(self._publish_period, self._publish)

        self.get_logger().info(
            f"Publishing {len(self._marker_array.markers)} markers from {geojson_file} on {topic}"
        )

    def _load_geojson(self, geojson_file):
        if not geojson_file.exists():
            raise FileNotFoundError(f"GeoJSON file does not exist: {geojson_file}")

        with geojson_file.open("r", encoding="utf-8") as geojson_stream:
            data = json.load(geojson_stream)

        features = data.get("features", [])
        markers = MarkerArray()
        marker_id = 0
        for feature in features:
            geometry = feature.get("geometry") or {}
            properties = feature.get("properties") or {}
            geometry_type = geometry.get("type")
            coordinates = geometry.get("coordinates")
            feature_id = properties.get("label") or properties.get("id", marker_id)

            if geometry_type == "Point":
                markers.markers.append(
                    self._point_marker(marker_id, coordinates, str(feature_id))
                )
                marker_id += 1
                if self.get_parameter("show_labels").value:
                    markers.markers.append(
                        self._label_marker(marker_id, coordinates, str(feature_id))
                    )
                    marker_id += 1
            elif geometry_type in ("LineString", "MultiLineString"):
                line_groups = coordinates if geometry_type == "MultiLineString" else [coordinates]
                for line in line_groups:
                    if len(line) < 2:
                        continue
                    markers.markers.append(self._line_marker(marker_id, line))
                    marker_id += 1
            elif geometry_type == "Polygon":
                for ring in coordinates:
                    if len(ring) < 2:
                        continue
                    markers.markers.append(self._line_marker(marker_id, ring, closed=True))
                    marker_id += 1
            elif geometry_type == "MultiPolygon":
                for polygon in coordinates:
                    for ring in polygon:
                        if len(ring) < 2:
                            continue
                        markers.markers.append(
                            self._line_marker(marker_id, ring, closed=True)
                        )
                        marker_id += 1
            else:
                self.get_logger().warn(f"Skipping unsupported GeoJSON geometry: {geometry_type}")

        return markers

    def _base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.ns = "geojson"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _point_marker(self, marker_id, coordinates, label):
        marker = self._base_marker(marker_id, Marker.SPHERE)
        marker.ns = "geojson_nodes"
        marker.pose.position = self._point(coordinates)
        marker.scale.x = self.get_parameter("node_scale").value
        marker.scale.y = self.get_parameter("node_scale").value
        marker.scale.z = self.get_parameter("node_scale").value
        marker.color.r = 0.05
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = label
        return marker

    def _label_marker(self, marker_id, coordinates, label):
        marker = self._base_marker(marker_id, Marker.TEXT_VIEW_FACING)
        marker.ns = "geojson_labels"
        marker.pose.position = self._point(coordinates)
        marker.pose.position.z += 0.45
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = label
        return marker

    def _line_marker(self, marker_id, coordinates, closed=False):
        marker = self._base_marker(marker_id, Marker.LINE_STRIP)
        marker.ns = "geojson_lines"
        marker.scale.x = self.get_parameter("line_width").value
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.points = [self._point(coordinate) for coordinate in coordinates]
        if closed and marker.points and marker.points[0] != marker.points[-1]:
            marker.points.append(marker.points[0])
        return marker

    def _point(self, coordinate):
        point = Point()
        point.x = _as_float(coordinate[0] if len(coordinate) > 0 else 0.0)
        point.y = _as_float(coordinate[1] if len(coordinate) > 1 else 0.0)
        point.z = _as_float(
            coordinate[2] if len(coordinate) > 2 else self.get_parameter("z").value
        )
        return point

    def _publish(self):
        now = self.get_clock().now().to_msg()
        for marker in self._marker_array.markers:
            marker.header.stamp = now
        self._publisher.publish(self._marker_array)


def main():
    rclpy.init()
    node = GeojsonMarkerArrayPublisher()
    try:
        if node.get_parameter("publish_once").value:
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
