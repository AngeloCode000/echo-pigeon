import rclpy
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection


class RadarDriverNode(Node):
    """Serial/USB driver for the TI IWR6843ISK-ODS.

    TODO: open the serial port, load a TI demo chirp configuration, parse
    the point-cloud UART stream, and publish RadarDetection messages
    (project_plan.md Phase 1).
    """

    def __init__(self):
        super().__init__('radar_driver_node')
        self.publisher_ = self.create_publisher(RadarDetection, 'radar/detections', 10)

    # TODO: read from the serial port and publish parsed detections.


def main(args=None):
    rclpy.init(args=args)
    node = RadarDriverNode()
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
