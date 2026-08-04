import rclpy
from rclpy.node import Node

from data_logger.csv_writer import CsvRunWriter
from radar_interfaces.msg import RadarScan, TargetTrack


class LoggerNode(Node):
    """Records raw detections and tracks to CSV for offline analysis.

    One timestamped run directory per launch (project_plan.md
    "Engineering requirements"). For full-fidelity replay use
    `ros2 bag record /radar/detections /tracks` alongside or instead.
    """

    def __init__(self):
        super().__init__('logger_node')

        self.declare_parameter('output_dir', '~/echo_pigeon_logs')
        self.declare_parameter('enable', True)
        self.declare_parameter('flush_every_n', 50)

        self.writer = None
        if self.get_parameter('enable').value:
            self.writer = CsvRunWriter(
                self.get_parameter('output_dir').value,
                flush_every_n=self.get_parameter('flush_every_n').value)
            self.get_logger().info(f'logging to {self.writer.run_dir}')
        else:
            self.get_logger().info('logging disabled by parameter')

        self.detection_sub = self.create_subscription(
            RadarScan, 'radar/detections', self.scan_callback, 10)
        self.track_sub = self.create_subscription(
            TargetTrack, 'tracks', self.track_callback, 10)

    def scan_callback(self, msg):
        if self.writer is not None:
            self.writer.write_scan(msg.header.stamp.sec,
                                   msg.header.stamp.nanosec, msg.detections)

    def track_callback(self, msg):
        if self.writer is not None:
            self.writer.write_track(msg.header.stamp.sec,
                                    msg.header.stamp.nanosec, msg)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
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
