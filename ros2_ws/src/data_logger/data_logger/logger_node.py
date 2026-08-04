import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection, TargetTrack


class LoggerNode(Node):
    """Records detections and tracks for offline analysis.

    TODO: write incoming messages to CSV or a ROS bag so datasets can be
    replayed without hardware (project_plan.md "Engineering requirements").
    """

    def __init__(self):
        super().__init__('logger_node')
        self.detection_sub = self.create_subscription(
            RadarDetection, 'radar/detections', self.detection_callback, 10)
        self.track_sub = self.create_subscription(
            TargetTrack, 'tracks', self.track_callback, 10)

    def detection_callback(self, msg):
        pass

    def track_callback(self, msg):
        pass


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
