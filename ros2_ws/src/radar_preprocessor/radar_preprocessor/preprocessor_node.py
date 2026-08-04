import numpy as np
import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection, RadarScan
from radar_preprocessor.clustering import dbscan, reduce_clusters
from radar_preprocessor.filters import (
    range_filter,
    snr_filter,
    static_clutter_filter,
)
from target_tracker.coordinates import (
    cartesian_to_spherical,
    spherical_to_cartesian,
)


class PreprocessorNode(Node):
    """Filters and clusters raw radar frames.

    Chain: SNR threshold -> range bounds -> optional static-clutter
    (zero-doppler) rejection -> optional DBSCAN clustering
    (project_plan.md Phase 3, steps 1-3). Empty frames pass through:
    they are the tracker's missed-detection signal.
    """

    def __init__(self):
        super().__init__('preprocessor_node')

        self.declare_parameter('min_snr_db', 5.0)
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 30.0)
        self.declare_parameter('enable_static_clutter_filter', True)
        self.declare_parameter('min_abs_radial_velocity_mps', 0.05)
        self.declare_parameter('enable_clustering', True)
        self.declare_parameter('cluster_eps_m', 0.5)
        self.declare_parameter('cluster_min_samples', 2)
        self.declare_parameter('cluster_keep_noise', True)

        self.subscription = self.create_subscription(
            RadarScan, 'radar/detections', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(
            RadarScan, 'radar/detections_filtered', 10)

    def scan_callback(self, msg):
        p = self.get_parameter
        detections = list(msg.detections)

        detections = snr_filter(detections, p('min_snr_db').value)
        detections = range_filter(
            detections, p('min_range_m').value, p('max_range_m').value)
        if p('enable_static_clutter_filter').value:
            detections = static_clutter_filter(
                detections, p('min_abs_radial_velocity_mps').value)

        out = RadarScan()
        out.header = msg.header

        if detections and p('enable_clustering').value:
            points = np.array([
                spherical_to_cartesian(d.range_m, d.azimuth_rad, d.elevation_rad)
                for d in detections])
            labels = dbscan(points, p('cluster_eps_m').value,
                            p('cluster_min_samples').value)
            for centroid, doppler, snr in reduce_clusters(
                    detections, points, labels,
                    keep_noise=p('cluster_keep_noise').value):
                det = RadarDetection()
                det.header = msg.header
                r, az, el = cartesian_to_spherical(*centroid)
                det.range_m = float(r)
                det.azimuth_rad = float(az)
                det.elevation_rad = float(el)
                det.radial_velocity_mps = doppler
                det.signal_strength = snr
                out.detections.append(det)
        else:
            out.detections = detections

        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PreprocessorNode()
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
