import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection, TargetTrack


class TrackerNode(Node):
    """Converts filtered detections into persistent tracks.

    TODO: convert spherical measurements into Cartesian coordinates,
    initialize tentative tracks, run an extended Kalman filter, associate
    new detections via nearest-neighbor/Mahalanobis gating, and confirm or
    delete tracks based on hit/miss counts (project_plan.md Phase 3).
    """

    def __init__(self):
        super().__init__('tracker_node')
        self.subscription = self.create_subscription(
            RadarDetection, 'radar/detections_filtered', self.detection_callback, 10)
        self.publisher_ = self.create_publisher(TargetTrack, 'tracks', 10)

    def detection_callback(self, msg):
        pass


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
