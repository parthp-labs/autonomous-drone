import math
import time

import rclpy
from rclpy.node import Node

from pymavlink import mavutil

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState
from std_msgs.msg import String


class PixhawkTelemetry(Node):
    """Receive ArduPilot MAVLink telemetry and publish ROS 2 topics."""

    def __init__(self):
        super().__init__('pixhawk_telemetry')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 57600)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        # --------------------------------------------------
        # ROS 2 Publishers
        # --------------------------------------------------
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, '/drone/attitude', 10
        )

        self.gps_pub = self.create_publisher(
            NavSatFix, '/drone/gps', 10
        )

        self.battery_pub = self.create_publisher(
            BatteryState, '/drone/battery', 10
        )

        self.flight_mode_pub = self.create_publisher(
            String, '/drone/flight_mode', 10
        )

        # --------------------------------------------------
        # MAVLink Connection State
        # --------------------------------------------------
        self.master = None

        self.target_system = None
        self.target_component = None

        self.last_heartbeat = None
        self.last_connection_attempt = 0.0

        self.heartbeat_timeout = 10.0
        self.reconnect_interval = 5.0

        self.telemetry_requested = False

        # --------------------------------------------------
        # Start Connection
        # --------------------------------------------------
        self.connect_pixhawk()

        # Process MAVLink without blocking ROS callbacks.
        self.timer = self.create_timer(
            0.05,
            self.read_mavlink
        )

        self.get_logger().info(
            'Pixhawk telemetry node started.'
        )

    # ======================================================
    # CONNECT TO PIXHAWK
    # ======================================================
    def connect_pixhawk(self):
        """Open the UART connection to the Pixhawk."""

        self.last_connection_attempt = time.monotonic()

        try:
            self.master = mavutil.mavlink_connection(
                self.port,
                baud=self.baud,
                source_system=255,
                source_component=190,
                autoreconnect=False
            )

            self.target_system = None
            self.target_component = None
            self.last_heartbeat = None
            self.telemetry_requested = False

            self.get_logger().info(
                f'Opened MAVLink UART: {self.port} '
                f'at {self.baud} baud. Waiting for heartbeat...'
            )

        except Exception as error:
            self.master = None

            self.get_logger().error(
                f'Unable to open Pixhawk UART: {error}'
            )

    # ======================================================
    # REQUEST MAVLINK TELEMETRY
    # ======================================================
    def request_telemetry(self):
        """Request MAVLink message intervals from ArduPilot."""

        if self.master is None or self.target_system is None:
            return

        streams = {
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 100000,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 200000,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1000000,
        }

        for message_id, interval_us in streams.items():

            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0
            )

            frequency = 1000000.0 / interval_us

            self.get_logger().info(
                f'Requested MAVLink message {message_id} '
                f'at {frequency:.1f} Hz'
            )

        self.telemetry_requested = True

    # ======================================================
    # READ MAVLINK
    # ======================================================
    def read_mavlink(self):
        """Read MAVLink messages and publish ROS 2 telemetry."""

        if self.master is None:

            elapsed = (
                time.monotonic() - self.last_connection_attempt
            )

            if elapsed >= self.reconnect_interval:
                self.connect_pixhawk()

            return

        try:
            for _ in range(50):

                msg = self.master.recv_match(blocking=False)

                if msg is None:
                    break

                msg_type = msg.get_type()

                if msg_type == 'BAD_DATA':
                    continue

                # ------------------------------------------
                # HEARTBEAT
                # ------------------------------------------
                if msg_type == 'HEARTBEAT':

                    # Ignore heartbeats from other components.
                    if (
                        msg.autopilot
                        != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                    ):
                        continue

                    self.last_heartbeat = time.monotonic()

                    if self.target_system is None:

                        self.target_system = msg.get_srcSystem()
                        self.target_component = msg.get_srcComponent()

                        self.get_logger().info(
                            'Pixhawk heartbeat received: '
                            f'system={self.target_system}, '
                            f'component={self.target_component}'
                        )

                    if not self.telemetry_requested:
                        self.request_telemetry()

                    self.publish_flight_mode(msg)

                # ------------------------------------------
                # ATTITUDE
                # ------------------------------------------
                elif msg_type == 'ATTITUDE':
                    self.publish_attitude(msg)

                # ------------------------------------------
                # GPS
                # ------------------------------------------
                elif msg_type == 'GPS_RAW_INT':
                    self.publish_gps(msg)

                # ------------------------------------------
                # BATTERY
                # ------------------------------------------
                elif msg_type == 'SYS_STATUS':
                    self.publish_battery(msg)

                # ------------------------------------------
                # COMMAND ACKNOWLEDGMENT
                # ------------------------------------------
                elif msg_type == 'COMMAND_ACK':

                    if (
                        msg.command
                        == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
                    ):
                        self.get_logger().info(
                            'Telemetry request ACK: '
                            f'result={msg.result}'
                        )

            # ----------------------------------------------
            # HEARTBEAT TIMEOUT
            # ----------------------------------------------
            if self.last_heartbeat is not None:

                elapsed = (
                    time.monotonic() - self.last_heartbeat
                )

                if elapsed > self.heartbeat_timeout:

                    self.get_logger().warning(
                        'Pixhawk heartbeat timeout. Reconnecting...'
                    )

                    self.disconnect_pixhawk()

            elif (
                time.monotonic() - self.last_connection_attempt
                > self.heartbeat_timeout
            ):

                self.get_logger().warning(
                    'No Pixhawk heartbeat detected. Retrying...'
                )

                self.disconnect_pixhawk()

        except Exception as error:

            self.get_logger().error(
                f'MAVLink communication error: {error}'
            )

            self.disconnect_pixhawk()

    # ======================================================
    # PUBLISH ATTITUDE
    # ======================================================
    def publish_attitude(self, msg):
        """Publish roll, pitch, yaw in radians."""

        attitude = Vector3Stamped()

        attitude.header.stamp = (
            self.get_clock().now().to_msg()
        )

        attitude.header.frame_id = 'base_link'

        attitude.vector.x = float(msg.roll)
        attitude.vector.y = float(msg.pitch)
        attitude.vector.z = float(msg.yaw)

        self.attitude_pub.publish(attitude)

    # ======================================================
    # PUBLISH GPS
    # ======================================================
    def publish_gps(self, msg):
        """Publish GPS position."""

        gps = NavSatFix()

        gps.header.stamp = (
            self.get_clock().now().to_msg()
        )

        gps.header.frame_id = 'gps_link'

        gps.latitude = msg.lat / 1e7
        gps.longitude = msg.lon / 1e7
        gps.altitude = msg.alt / 1000.0

        if msg.fix_type >= 3:
            gps.status.status = NavSatStatus.STATUS_FIX
        else:
            gps.status.status = NavSatStatus.STATUS_NO_FIX

        gps.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.gps_pub.publish(gps)

    # ======================================================
    # PUBLISH BATTERY
    # ======================================================
    def publish_battery(self, msg):
        """Publish battery voltage, current, and percentage."""

        battery = BatteryState()

        battery.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Voltage: millivolts -> volts
        if msg.voltage_battery != 65535:
            battery.voltage = msg.voltage_battery / 1000.0
        else:
            battery.voltage = math.nan

        # Current: centiamps -> amps
        if msg.current_battery != -1:
            battery.current = msg.current_battery / 100.0
        else:
            battery.current = math.nan

        # Remaining battery percentage: 0-100 -> 0.0-1.0
        if msg.battery_remaining >= 0:
            battery.percentage = msg.battery_remaining / 100.0
        else:
            battery.percentage = math.nan

        battery.present = True

        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )

        battery.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )

        battery.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )

        self.battery_pub.publish(battery)

    # ======================================================
    # PUBLISH FLIGHT MODE
    # ======================================================
    def publish_flight_mode(self, msg):
        """Publish the current ArduPilot flight mode."""

        mode = String()

        mode.data = mavutil.mode_string_v10(msg)

        self.flight_mode_pub.publish(mode)

    # ======================================================
    # DISCONNECT
    # ======================================================
    def disconnect_pixhawk(self):
        """Close the UART connection and reset connection state."""

        if self.master is not None:

            try:
                self.master.close()
            except Exception:
                pass

        self.master = None

        self.target_system = None
        self.target_component = None

        self.last_heartbeat = None
        self.telemetry_requested = False

        self.last_connection_attempt = time.monotonic()

    # ======================================================
    # CLEANUP
    # ======================================================
    def cleanup(self):
        """Release the MAVLink serial connection."""

        self.disconnect_pixhawk()


def main(args=None):
    rclpy.init(args=args)

    node = PixhawkTelemetry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.cleanup()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
