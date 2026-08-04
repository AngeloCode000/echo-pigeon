import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection, RadarScan
from radar_simulator.measurement_model import MeasurementModel
from radar_simulator.trajectories import make_trajectory


class TargetGeneratorNode(Node):
    """Publishes synthetic RadarScan frames of a simulated drone.

    Drives a circle or figure-eight trajectory with Gaussian measurement
    noise, random missed detections, and Poisson clutter
    (project_plan.md Phase 0). Ground truth goes out on radar/ground_truth.
    """

    def __init__(self):
        super().__init__('target_generator_node')

        self.declare_parameter('trajectory_type', 'figure_eight')
        self.declare_parameter('center_x', 8.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 3.0)
        self.declare_parameter('size_m', 3.0)
        self.declare_parameter('period_s', 20.0)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('sigma_range_m', 0.1)
        self.declare_parameter('sigma_azimuth_rad', 0.02)
        self.declare_parameter('sigma_elevation_rad', 0.02)
        self.declare_parameter('sigma_doppler_mps', 0.1)
        self.declare_parameter('detection_probability', 0.9)
        self.declare_parameter('clutter_mean_count', 0.5)
        self.declare_parameter('base_snr_db', 25.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('frame_id', 'radar_link')

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        self.trajectory = make_trajectory(
            p('trajectory_type').value,
            (p('center_x').value, p('center_y').value, p('center_z').value),
            p('size_m').value,
            p('period_s').value)
        seed = p('seed').value
        self.model = MeasurementModel(
            sigma_range_m=p('sigma_range_m').value,
            sigma_azimuth_rad=p('sigma_azimuth_rad').value,
            sigma_elevation_rad=p('sigma_elevation_rad').value,
            sigma_doppler_mps=p('sigma_doppler_mps').value,
            detection_probability=p('detection_probability').value,
            clutter_mean_count=p('clutter_mean_count').value,
            base_snr_db=p('base_snr_db').value,
            seed=seed if seed != 0 else None)

        self.scan_pub = self.create_publisher(RadarScan, 'radar/detections', 10)
        self.truth_pub = self.create_publisher(
            PointStamped, 'radar/ground_truth', 10)

        self.t = 0.0
        self.dt = 1.0 / p('update_rate_hz').value
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f"simulating '{p('trajectory_type').value}' trajectory at "
            f"{p('update_rate_hz').value:.1f} Hz")

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        position = self.trajectory.position(self.t)
        velocity = self.trajectory.velocity(self.t)
        self.t += self.dt

        scan = RadarScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.frame_id
        for m in self.model.frame(position, velocity):
            det = RadarDetection()
            det.header = scan.header
            det.range_m = m.range_m
            det.azimuth_rad = m.azimuth_rad
            det.elevation_rad = m.elevation_rad
            det.radial_velocity_mps = m.radial_velocity_mps
            det.signal_strength = m.signal_strength
            scan.detections.append(det)
        self.scan_pub.publish(scan)

        truth = PointStamped()
        truth.header = scan.header
        truth.point.x, truth.point.y, truth.point.z = position
        self.truth_pub.publish(truth)


def main(args=None):
    rclpy.init(args=args)
    node = TargetGeneratorNode()
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
