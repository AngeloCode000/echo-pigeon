import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection


class PreprocessorNode(Node):
    """Filters and clusters raw radar detections.

    TODO: remove static clutter, reject points below a signal threshold,
    and cluster detections using DBSCAN or spatial gating
    (project_plan.md Phase 3, steps 1-3).
    """

    def __init__(self):
        super().__init__('preprocessor_node')
        self.subscription = self.create_subscription(
            RadarDetection, 'radar/detections', self.detection_callback, 10)
        self.publisher_ = self.create_publisher(RadarDetection, 'radar/detections_filtered', 10)

    def detection_callback(self, msg):
        self.publisher_.publish(msg)


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
