import math

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from radar_interfaces.msg import RadarScan, TargetTrack
from target_tracker.coordinates import spherical_to_cartesian

CONFIRMED_COLOR = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)
TENTATIVE_COLOR = ColorRGBA(r=0.95, g=0.85, b=0.1, a=0.9)
DETECTION_COLOR = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8)
TRUTH_COLOR = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.9)


class VisualizerNode(Node):
    """Publishes RViz markers for tracks, detections, and ground truth.

    Per track: a sphere (green = confirmed, yellow = tentative), a velocity
    arrow, a text label 'id / confidence', and an optional 2-sigma
    covariance ellipsoid. Filtered detections appear as blue points —
    essential during hardware bring-up — and simulated ground truth as a
    red cube.
    """

    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('marker_lifetime_s', 0.5)
        self.declare_parameter('track_sphere_scale', 0.3)
        self.declare_parameter('velocity_arrow_scale', 0.5)
        self.declare_parameter('show_covariance', True)
        self.declare_parameter('show_detections', True)
        self.declare_parameter('detection_point_scale', 0.12)

        self.track_sub = self.create_subscription(
            TargetTrack, 'tracks', self.track_callback, 10)
        self.scan_sub = self.create_subscription(
            RadarScan, 'radar/detections_filtered', self.scan_callback, 10)
        self.truth_sub = self.create_subscription(
            PointStamped, 'radar/ground_truth', self.truth_callback, 10)
        self.publisher_ = self.create_publisher(MarkerArray, 'track_markers', 10)

    def _base_marker(self, header, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = Duration(
            seconds=self.get_parameter('marker_lifetime_s').value).to_msg()
        return marker

    def track_callback(self, msg):
        markers = MarkerArray()
        color = CONFIRMED_COLOR if msg.confidence >= 0.5 else TENTATIVE_COLOR
        scale = self.get_parameter('track_sphere_scale').value

        sphere = self._base_marker(msg.header, 'track_position',
                                   int(msg.track_id), Marker.SPHERE)
        sphere.pose.position.x = msg.position_x
        sphere.pose.position.y = msg.position_y
        sphere.pose.position.z = msg.position_z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
        sphere.color = color
        markers.markers.append(sphere)

        speed = math.sqrt(msg.velocity_x ** 2 + msg.velocity_y ** 2
                          + msg.velocity_z ** 2)
        if speed > 1e-3:
            arrow = self._base_marker(msg.header, 'track_velocity',
                                      int(msg.track_id), Marker.ARROW)
            arrow_scale = self.get_parameter('velocity_arrow_scale').value
            start = Point(x=msg.position_x, y=msg.position_y, z=msg.position_z)
            end = Point(x=msg.position_x + arrow_scale * msg.velocity_x,
                        y=msg.position_y + arrow_scale * msg.velocity_y,
                        z=msg.position_z + arrow_scale * msg.velocity_z)
            arrow.points = [start, end]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            arrow.color = color
            markers.markers.append(arrow)

        label = self._base_marker(msg.header, 'track_label',
                                  int(msg.track_id), Marker.TEXT_VIEW_FACING)
        label.pose.position.x = msg.position_x
        label.pose.position.y = msg.position_y
        label.pose.position.z = msg.position_z + 0.5
        label.pose.orientation.w = 1.0
        label.scale.z = 0.3
        label.color = color
        label.text = f'#{msg.track_id} ({msg.confidence:.2f})'
        markers.markers.append(label)

        if self.get_parameter('show_covariance').value:
            cov = self._base_marker(msg.header, 'track_covariance',
                                    int(msg.track_id), Marker.SPHERE)
            cov.pose.position.x = msg.position_x
            cov.pose.position.y = msg.position_y
            cov.pose.position.z = msg.position_z
            cov.pose.orientation.w = 1.0
            # 2-sigma ellipsoid from the position covariance diagonal
            # (row-major 6x6: indices 0, 7, 14).
            cov.scale.x = max(2.0 * math.sqrt(max(msg.covariance[0], 0.0)), 0.01)
            cov.scale.y = max(2.0 * math.sqrt(max(msg.covariance[7], 0.0)), 0.01)
            cov.scale.z = max(2.0 * math.sqrt(max(msg.covariance[14], 0.0)), 0.01)
            cov.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.15)
            markers.markers.append(cov)

        self.publisher_.publish(markers)

    def scan_callback(self, msg):
        if not self.get_parameter('show_detections').value:
            return
        points = self._base_marker(msg.header, 'detections', 0, Marker.POINTS)
        point_scale = self.get_parameter('detection_point_scale').value
        points.scale.x = points.scale.y = point_scale
        points.color = DETECTION_COLOR
        for d in msg.detections:
            x, y, z = spherical_to_cartesian(
                d.range_m, d.azimuth_rad, d.elevation_rad)
            points.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers = MarkerArray()
        markers.markers.append(points)
        self.publisher_.publish(markers)

    def truth_callback(self, msg):
        cube = self._base_marker(msg.header, 'ground_truth', 0, Marker.CUBE)
        cube.pose.position = msg.point
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = 0.2
        cube.color = TRUTH_COLOR
        markers = MarkerArray()
        markers.markers.append(cube)
        self.publisher_.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
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
