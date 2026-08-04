import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection


class TargetGeneratorNode(Node):
    """Publishes synthetic RadarDetection messages.

    TODO: drive a constant-velocity or figure-eight trajectory, add
    measurement noise and random missed detections (project_plan.md Phase 0).
    """

    def __init__(self):
        super().__init__('target_generator_node')
        self.publisher_ = self.create_publisher(RadarDetection, 'radar/detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = RadarDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar'
        self.publisher_.publish(msg)


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
