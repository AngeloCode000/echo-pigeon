import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarScan, TargetTrack
from target_tracker.track_manager import TrackManager, TrackState


class TrackerNode(Node):
    """Converts filtered detection frames into persistent tracks.

    Runs an extended Kalman filter per track with nearest-Mahalanobis
    association and hit/miss based confirmation and deletion
    (project_plan.md Phase 3).
    """

    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('sigma_accel', 2.0)
        self.declare_parameter('sigma_range_m', 0.1)
        self.declare_parameter('sigma_azimuth_rad', 0.02)
        self.declare_parameter('sigma_elevation_rad', 0.02)
        self.declare_parameter('sigma_doppler_mps', 0.1)
        self.declare_parameter('gate_chi2', 9.488)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('initial_velocity_sigma', 5.0)
        self.declare_parameter('max_coast_dt_s', 1.0)

        p = self.get_parameter
        self.manager = TrackManager(
            sigma_accel=p('sigma_accel').value,
            measurement_noise_diag=(
                p('sigma_range_m').value,
                p('sigma_azimuth_rad').value,
                p('sigma_elevation_rad').value,
                p('sigma_doppler_mps').value,
            ),
            gate_chi2=p('gate_chi2').value,
            confirm_hits=p('confirm_hits').value,
            max_misses=p('max_misses').value,
            initial_velocity_sigma=p('initial_velocity_sigma').value,
            max_coast_dt_s=p('max_coast_dt_s').value,
        )

        self.subscription = self.create_subscription(
            RadarScan, 'radar/detections_filtered', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(TargetTrack, 'tracks', 10)

    def scan_callback(self, msg):
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        detections = [
            (d.range_m, d.azimuth_rad, d.elevation_rad, d.radial_velocity_mps)
            for d in msg.detections
        ]
        tracks = self.manager.process_scan(stamp_s, detections)

        for track in tracks:
            out = TargetTrack()
            out.header = msg.header
            out.track_id = track.track_id
            state = track.ekf.x
            out.position_x, out.position_y, out.position_z = state[:3]
            out.velocity_x, out.velocity_y, out.velocity_z = state[3:]
            out.covariance = list(track.ekf.P.flatten())
            age = track.age(stamp_s)
            out.track_age.sec = int(age)
            out.track_age.nanosec = int((age - int(age)) * 1e9)
            out.detection_count = track.hits
            out.miss_count = track.misses
            # Tentative tracks are capped at low confidence so downstream
            # consumers (visualizer) can distinguish them.
            out.confidence = (track.confidence
                              if track.state is TrackState.CONFIRMED
                              else min(track.confidence, 0.49))
            self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
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
