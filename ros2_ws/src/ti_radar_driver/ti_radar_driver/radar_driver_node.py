import threading

import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from radar_interfaces.msg import RadarDetection, RadarScan
from target_tracker.coordinates import cartesian_to_spherical
from ti_radar_driver import cfg_loader
from ti_radar_driver.tlv_parser import TlvParser


class RadarDriverNode(Node):
    """Serial driver for the TI IWR6843ISK-ODS running the out-of-box demo.

    Sends the chirp configuration over the CLI UART, then parses the
    point-cloud TLV stream from the data UART and publishes RadarScan
    frames — including empty ones, which are the tracker's miss signal.
    Reconnects with backoff, since a board reset detaches usbipd devices.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self):
        super().__init__('radar_driver_node')

        self.declare_parameter('cli_port', '/dev/ttyACM0')
        self.declare_parameter('data_port', '/dev/ttyACM1')
        self.declare_parameter('cli_baud', 115200)
        self.declare_parameter('data_baud', 921600)
        self.declare_parameter('cfg_file', '')
        self.declare_parameter('frame_id', 'radar_link')

        self.publisher_ = self.create_publisher(RadarScan, 'radar/detections', 10)
        self.frame_id = self.get_parameter('frame_id').value

        self._cli = None
        self._data = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _cfg_path(self):
        path = self.get_parameter('cfg_file').value
        if path:
            return path
        share = get_package_share_directory('ti_radar_driver')
        return f'{share}/config/iwr6843_ods_default.cfg'

    def _run(self):
        while not self._stop.is_set():
            try:
                self._connect_and_stream()
            except (serial.SerialException, OSError,
                    cfg_loader.CfgError) as error:
                self.get_logger().warning(
                    f'radar connection failed ({error}); '
                    f'retrying in {self.RECONNECT_DELAY_S:.0f} s')
                self._close_ports()
                if self._stop.wait(self.RECONNECT_DELAY_S):
                    return

    def _connect_and_stream(self):
        cli_port = self.get_parameter('cli_port').value
        data_port = self.get_parameter('data_port').value
        self.get_logger().info(
            f'opening CLI {cli_port} and data {data_port}')
        self._cli = serial.Serial(
            cli_port, self.get_parameter('cli_baud').value, timeout=0.5)
        self._data = serial.Serial(
            data_port, self.get_parameter('data_baud').value, timeout=0.5)

        cfg_path = self._cfg_path()
        with open(cfg_path) as f:
            cfg_text = f.read()
        # A previous session may have left the sensor running.
        try:
            cfg_loader.sensor_stop(self._cli)
        except cfg_loader.CfgError:
            pass
        cfg_loader.send_cfg(self._cli, cfg_text)
        self._data.reset_input_buffer()
        cfg_loader.sensor_start(self._cli)
        self.get_logger().info(f'sensor started with {cfg_path}')

        parser = TlvParser()
        while not self._stop.is_set():
            chunk = self._data.read(4096)
            if not chunk:
                continue
            for frame in parser.feed(chunk):
                self._publish_frame(frame)

    def _publish_frame(self, frame):
        scan = RadarScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        for point in frame.points:
            # TI board frame (x right, y boresight, z up) -> REP-103
            # radar frame (x forward, y left, z up).
            x, y, z = point.y, -point.x, point.z
            r, az, el = cartesian_to_spherical(x, y, z)
            det = RadarDetection()
            det.header = scan.header
            det.range_m = float(r)
            det.azimuth_rad = float(az)
            det.elevation_rad = float(el)
            det.radial_velocity_mps = float(point.doppler)
            det.signal_strength = float(point.snr_db)
            scan.detections.append(det)
        self.publisher_.publish(scan)

    def _close_ports(self):
        for port in (self._cli, self._data):
            if port is not None:
                try:
                    port.close()
                except (serial.SerialException, OSError):
                    pass
        self._cli = None
        self._data = None

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._cli is not None:
            try:
                cfg_loader.sensor_stop(self._cli)
            except (cfg_loader.CfgError, serial.SerialException, OSError):
                pass
        self._close_ports()
        super().destroy_node()


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
