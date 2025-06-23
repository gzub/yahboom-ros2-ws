#!/usr/bin/env python3
"""
Yahboomcar Ackermann Driver Node

This module implements the main driver node for the Yahboomcar robot, handling motion commands,
sensor data publishing, and peripheral controls (RGB lights, buzzer) using the Rosmaster library.

Safety Features:
- Sets speed and steering to zero on startup
- Periodic safety checks during startup to ensure robot remains stationary
- Multiple redundant zero motion commands on shutdown
- Emergency stop on signal handling (SIGINT, SIGTERM)
"""
import math
import random
import signal
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Bool, Float32, Int32

from Rosmaster_Lib import Rosmaster

# Constants
CAR_TYPE_MAPPING: Dict[str, int] = {"R2": 5, "X3": 1, "NONE": -1}
DEGREES_TO_RADIANS = math.pi / 180.0
RGB_LIGHT_EFFECT_COUNT = 3
DEFAULT_PUBLISH_RATE = 10.0  # Hz


class YahboomcarDriver(Node):
    """
    ROS2 Node for Yahboomcar Ackermann driver.

    Handles velocity commands, sensor data publishing, and controls RGB lights and buzzer.
    Supports Ackermann steering geometry and publishes sensor data including IMU, magnetometer,
    battery voltage, and joint states.
    """

    def __init__(self, name: str):
        """
        Initialize the Yahboomcar driver node.

        Args:
            name: Name of the ROS2 node
        """
        super().__init__(name)

        # Initialize car controller
        self.car = Rosmaster()
        self._shutdown_requested = False

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()

        # Validate parameters
        self._validate_parameters()

        # Setup QoS profiles
        self._setup_qos_profiles()

        # Initialize Rosmaster library
        self._initialize_car()

        # Create subscribers
        self._create_subscribers()

        # Create publishers
        self._create_publishers()

        # Create timer for sensor data publishing
        publish_period = 1.0 / DEFAULT_PUBLISH_RATE
        self.timer = self.create_timer(publish_period, self._publish_sensor_data)

        # Create startup safety timer to ensure zero motion
        self._startup_safety_timer = self.create_timer(0.5, self._startup_safety_check)
        self._startup_checks_count = 0

        # Initialize message templates
        self._initialize_messages()

        # Start receiving thread for car communication
        try:
            self.car.create_receive_threading()

            # Safety: Set car to zero motion on startup
            self._set_zero_motion()

            self.get_logger().info("Yahboomcar driver node initialized successfully")
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Failed to initialize car communication: {e}")
            raise

    def _signal_handler(self, signum: int, _frame) -> None:
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f"Received signal {signum}, shutting down gracefully...")
        self._shutdown_requested = True

        # Immediately stop the car for safety
        try:
            self._set_zero_motion()
        except Exception as e:
            self.get_logger().error(f"Failed to set zero motion in signal handler: {e}")

        # Request ROS shutdown to break the spin loop
        rclpy.shutdown()

    def _declare_parameters(self) -> None:
        """Declare all node parameters with default values."""
        self.declare_parameter("car_type", "R2")
        self.declare_parameter("imu_link", "imu_link")
        self.declare_parameter("prefix", "")
        self.declare_parameter("xlinear_limit", 2.0)
        self.declare_parameter("ylinear_limit", 2.0)
        self.declare_parameter("angular_limit", 5.0)
        self.declare_parameter("nav_use_rotvel", False)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)

    def _get_parameters(self) -> None:
        """Retrieve and store parameter values."""
        self.car_type = (
            self.get_parameter("car_type").get_parameter_value().string_value
        )
        self.imu_link = (
            self.get_parameter("imu_link").get_parameter_value().string_value
        )
        self.prefix = self.get_parameter("prefix").get_parameter_value().string_value
        self.xlinear_limit = (
            self.get_parameter("xlinear_limit").get_parameter_value().double_value
        )
        self.ylinear_limit = (
            self.get_parameter("ylinear_limit").get_parameter_value().double_value
        )
        self.angular_limit = (
            self.get_parameter("angular_limit").get_parameter_value().double_value
        )
        self.nav_use_rotvel = (
            self.get_parameter("nav_use_rotvel").get_parameter_value().bool_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # Log parameters
        self.get_logger().info(f"Car type: {self.car_type}")
        self.get_logger().info(f"IMU link: {self.imu_link}")
        self.get_logger().info(f"Prefix: '{self.prefix}'")
        self.get_logger().info(
            f"Linear limits - X: {self.xlinear_limit}, Y: {self.ylinear_limit}"
        )
        self.get_logger().info(f"Angular limit: {self.angular_limit}")
        self.get_logger().info(
            f"Navigation uses rotational velocity: {self.nav_use_rotvel}"
        )
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")

    def _validate_parameters(self) -> None:
        """Validate parameter values and raise exceptions for invalid ones."""
        if self.car_type not in CAR_TYPE_MAPPING:
            valid_types = list(CAR_TYPE_MAPPING.keys())
            raise ValueError(
                f"Invalid car_type '{self.car_type}'. Valid types: {valid_types}"
            )

        if (
            self.xlinear_limit <= 0
            or self.ylinear_limit <= 0
            or self.angular_limit <= 0
        ):
            raise ValueError("Linear and angular limits must be positive")

        if self.publish_rate <= 0 or self.publish_rate > 100:
            raise ValueError("Publish rate must be between 0 and 100 Hz")

    def _setup_qos_profiles(self) -> None:
        """Setup QoS profiles for publishers and subscribers."""
        # Command velocity uses best effort for real-time performance
        self.cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Sensor data can tolerate some loss for performance
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Control commands need reliability
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

    def _initialize_car(self) -> None:
        """Initialize the car with the specified type."""
        car_type_id = CAR_TYPE_MAPPING.get(self.car_type, 5)
        self.car.set_car_type(car_type_id)
        self.get_logger().info(f"Car type set to {self.car_type} (ID: {car_type_id})")

    def _create_subscribers(self) -> None:
        """Create all subscribers with appropriate QoS profiles."""
        self.sub_cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self._cmd_vel_callback, self.cmd_vel_qos
        )
        self.sub_rgb_light = self.create_subscription(
            Int32, "RGBLight", self._rgb_light_callback, self.control_qos
        )
        self.sub_buzzer = self.create_subscription(
            Bool, "Buzzer", self._buzzer_callback, self.control_qos
        )

    def _create_publishers(self) -> None:
        """Create all publishers with appropriate QoS profiles."""
        self.edition_publisher = self.create_publisher(
            Float32, "edition", self.sensor_qos
        )
        self.voltage_publisher = self.create_publisher(
            Float32, "voltage", self.sensor_qos
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", self.sensor_qos
        )
        self.velocity_publisher = self.create_publisher(
            Twist, "vel_raw", self.sensor_qos
        )
        self.imu_publisher = self.create_publisher(
            Imu, "imu/data_raw", self.sensor_qos
        )
        self.magnetometer_publisher = self.create_publisher(
            MagneticField, "imu/mag", self.sensor_qos
        )

    def _initialize_messages(self) -> None:
        """Initialize message templates to avoid repeated allocations."""
        self.edition_msg = Float32()
        self.edition_msg.data = 1.0

        # Joint names for Ackermann steering
        prefix = self.prefix if self.prefix else ""
        self.joint_names = [
            f"{prefix}back_right_joint",
            f"{prefix}back_left_joint",
            f"{prefix}front_left_steer_joint",
            f"{prefix}front_left_wheel_joint",
            f"{prefix}front_right_steer_joint",
            f"{prefix}front_right_wheel_joint",
        ]

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback for /cmd_vel topic. Controls the car's motion based on received Twist messages.

        Args:
            msg: Twist message containing linear and angular velocity commands
        """
        if self._shutdown_requested:
            return

        try:
            # Clamp velocities to safe limits
            vx = max(min(msg.linear.x, self.xlinear_limit), -self.xlinear_limit)
            vy = max(min(msg.linear.y, self.ylinear_limit), -self.ylinear_limit)
            angular = max(min(msg.angular.z, self.angular_limit), -self.angular_limit)

            # Send motion command to car
            self.car.set_car_motion(vx, vy, angular)

            self.get_logger().debug(
                f"Motion command - vx: {vx:.3f}, vy: {vy:.3f}, angular: {angular:.3f}"
            )
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Error in cmd_vel callback: {e}")

    def _rgb_light_callback(self, msg: Int32) -> None:
        """
        Callback for RGBLight topic. Controls the RGB lights on the car.

        Args:
            msg: Int32 message containing the RGB light effect code
        """
        if self._shutdown_requested:
            return

        try:
            # Send RGB light command multiple times for reliability
            for _ in range(RGB_LIGHT_EFFECT_COUNT):
                self.car.set_colorful_effect(msg.data, 6, parm=1)

            self.get_logger().debug(f"RGB light effect: {msg.data}")
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Error in RGB light callback: {e}")

    def _buzzer_callback(self, msg: Bool) -> None:
        """
        Callback for Buzzer topic. Controls the buzzer on the car.

        Args:
            msg: Bool message to enable/disable the buzzer
        """
        if self._shutdown_requested:
            return

        try:
            self.car.set_beep(1 if msg.data else 0)
            self.get_logger().debug(f"Buzzer: {'ON' if msg.data else 'OFF'}")
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Error in buzzer callback: {e}")

    def _publish_sensor_data(self) -> None:
        """
        Timer callback to publish sensor and state data at regular intervals.
        Publishes IMU, magnetic field, battery, edition, velocity, and joint state data.
        """
        if self._shutdown_requested:
            return

        try:
            timestamp = self.get_clock().now()

            # Get data from car
            edition_value = self.car.get_version()
            battery_voltage = self.car.get_battery_voltage()
            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            mx, my, mz = self.car.get_magnetometer_data()
            vx, vy, angular = self.car.get_motion_data()

            # Publish edition
            edition_msg = Float32()
            edition_msg.data = float(edition_value)
            self.edition_publisher.publish(edition_msg)

            # Publish battery voltage
            battery_msg = Float32()
            battery_msg.data = float(battery_voltage)
            self.voltage_publisher.publish(battery_msg)

            # Publish IMU data
            self._publish_imu_data(timestamp, ax, ay, az, gx, gy, gz)

            # Publish magnetometer data
            self._publish_magnetometer_data(timestamp, mx, my, mz)

            # Publish velocity data
            self._publish_velocity_data(vx, vy, angular)

            # Publish joint states
            self._publish_joint_states(timestamp, vy, vx, angular)

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().error(f"Error publishing sensor data: {e}")

    def _publish_imu_data(
        self,
        timestamp,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
    ) -> None:
        """Publish IMU acceleration and gyroscope data."""
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp.to_msg()
        imu_msg.header.frame_id = self.imu_link

        # Linear acceleration
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)

        # Angular velocity
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)

        # Set covariance matrices (unknown covariance)
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.orientation_covariance[0] = -1

        self.imu_publisher.publish(imu_msg)

    def _publish_magnetometer_data(
        self, timestamp, mx: float, my: float, mz: float
    ) -> None:
        """Publish magnetometer data."""
        mag_msg = MagneticField()
        mag_msg.header.stamp = timestamp.to_msg()
        mag_msg.header.frame_id = self.imu_link

        mag_msg.magnetic_field.x = float(mx)
        mag_msg.magnetic_field.y = float(my)
        mag_msg.magnetic_field.z = float(mz)

        # Set covariance matrix (unknown covariance)
        mag_msg.magnetic_field_covariance[0] = -1

        self.magnetometer_publisher.publish(mag_msg)

    def _publish_velocity_data(self, vx: float, vy: float, angular: float) -> None:
        """Publish raw velocity data."""
        twist_msg = Twist()
        twist_msg.linear.x = vx
        # Note: vy is scaled by 1000.0 to match expected units for consumers of vel_raw
        twist_msg.linear.y = vy * 1000.0
        twist_msg.angular.z = angular

        self.velocity_publisher.publish(twist_msg)

    def _publish_joint_states(
        self, timestamp, vy: float, vx: float, angular: float
    ) -> None:
        """Publish joint states for Ackermann steering visualization."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = timestamp.to_msg()
        joint_state_msg.header.frame_id = "joint_states"
        joint_state_msg.name = self.joint_names

        # Calculate steering angle from vy (scaled steering input)
        steer_radians = vy * 1000.0 * DEGREES_TO_RADIANS

        # Set positions: [back_right, back_left, front_left_steer, front_left_wheel, front_right_steer, front_right_wheel]
        if vx != 0 or angular != 0:
            # Add some random motion to wheel joints when moving
            wheel_position = random.uniform(-math.pi, math.pi)
            joint_state_msg.position = [
                wheel_position,
                wheel_position,  # rear wheels
                steer_radians,
                wheel_position,  # front left steer and wheel
                steer_radians,
                wheel_position,  # front right steer and wheel
            ]
        else:
            # Stationary - only steering angle changes
            joint_state_msg.position = [
                0.0,
                0.0,
                steer_radians,
                0.0,
                steer_radians,
                0.0,
            ]

        self.joint_state_publisher.publish(joint_state_msg)

    def _set_zero_motion(self) -> None:
        """
        Set the car to zero motion (stop all movement and steering).
        This is a critical safety function used during startup and shutdown.
        """
        try:
            # Set all motion parameters to zero
            self.car.set_car_motion(0.0, 0.0, 0.0)

            # Turn off buzzer for safety
            self.car.set_beep(0)

            self.get_logger().info("Car motion set to zero for safety")

        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Failed to set zero motion: {e}")

    def _startup_safety_check(self) -> None:
        """
        Periodic safety check during startup to ensure the car remains stopped.
        Runs for the first few seconds after startup.
        """
        self._startup_checks_count += 1

        try:
            # Ensure car is stopped during startup
            self.car.set_car_motion(0.0, 0.0, 0.0)
            self.get_logger().debug(
                f"Startup safety check {self._startup_checks_count}"
            )

            # Stop safety checks after 6 cycles (3 seconds)
            if self._startup_checks_count >= 6:
                if hasattr(self, "_startup_safety_timer"):
                    self._startup_safety_timer.cancel()
                    self._startup_safety_timer = None
                self.get_logger().info("Startup safety checks completed")

        except (RuntimeError, OSError) as e:
            self.get_logger().warn(f"Startup safety check failed: {e}")

    def destroy_node(self) -> None:
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Cleaning up resources...")
        self._shutdown_requested = True

        # Cancel startup safety timer if it exists
        if (
            hasattr(self, "_startup_safety_timer")
            and self._startup_safety_timer is not None
        ):
            try:
                self._startup_safety_timer.cancel()
                self._startup_safety_timer = None
            except Exception:
                # Ignore timer cancellation errors during shutdown
                pass

        # Stop the car with redundant safety measures
        try:
            # Use the dedicated zero motion method
            self._set_zero_motion()

            # Additional safety: send zero motion multiple times
            for i in range(3):
                self.car.set_car_motion(0.0, 0.0, 0.0)
                self.get_logger().debug(f"Emergency stop command {i+1}/3 sent")

        except (RuntimeError, OSError) as e:
            self.get_logger().warn(f"Error stopping car during cleanup: {e}")
        except Exception as e:
            # Catch any other errors during shutdown
            self.get_logger().warn(f"Unexpected error during car shutdown: {e}")

        try:
            super().destroy_node()
        except Exception:
            # Ignore errors when destroying the node during shutdown
            pass

    def is_shutdown_requested(self) -> bool:
        """Return whether shutdown has been requested."""
        return self._shutdown_requested


def main(args=None) -> None:
    """
    Main entry point for the yahboomcar driver node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)

    driver = None
    try:
        driver = YahboomcarDriver("yahboomcar_driver_node")
        driver.get_logger().info("Yahboomcar driver node started successfully")

        # Spin until shutdown is requested
        while rclpy.ok() and not driver.is_shutdown_requested():
            rclpy.spin_once(driver, timeout_sec=0.1)

    except KeyboardInterrupt:
        if driver:
            driver.get_logger().info("Shutdown requested by user")
    except (RuntimeError, OSError, ValueError) as e:
        if driver:
            driver.get_logger().error(f"Unexpected error: {e}")
        else:
            print(f"Failed to initialize driver: {e}")
    finally:
        if driver:
            driver.get_logger().info("Shutting down yahboomcar driver node")
            driver.destroy_node()

        # Only shutdown if not already done by signal handler
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
