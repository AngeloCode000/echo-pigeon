import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from radar_interfaces.msg import TargetTrack


class VisualizerNode(Node):
    """Publishes RViz markers for persistent target tracks.

    TODO: convert each TargetTrack into a Marker (position, velocity
    arrow, track ID label) and publish as a MarkerArray for RViz.
    """

    def __init__(self):
        super().__init__('visualizer_node')
        self.subscription = self.create_subscription(
            TargetTrack, 'tracks', self.track_callback, 10)
        self.publisher_ = self.create_publisher(MarkerArray, 'track_markers', 10)

    def track_callback(self, msg):
        pass


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
