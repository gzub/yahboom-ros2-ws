#!/usr/bin/env python3
"""
Joystick Node.

This node subscribes to joystick inputs and publishes Twist commands for robot control.
Implements Ackermann steering geometry for accurate motion control and provides
continuous command publishing to ensure smooth operation and proper stopping behavior.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy


class JoystickNode(Node):
    """
    ROS2 Node that converts joystick input to Twist commands for robot control.

    Features:
    - Ackermann steering geometry for accurate motion control
    - Continuous command publishing for smooth operation
    - Configurable joystick axis mapping and scaling
    - Safety features including deadzone handling and parameter validation
    """

    def __init__(self):
        """
        Initialize the JoystickNode with parameters, publishers, and subscribers.
        """
        super().__init__("joystick_node")

        # Declare parameters with validation
        self._declare_parameters()

        # Get and validate parameters
        self._get_parameters()
        self._validate_parameters()

        # Setup QoS profiles
        self._setup_qos_profiles()

        # Create ROS2 interfaces
        self._create_publishers()
        self._create_subscribers()
        self._create_timers()

        # Initialize state variables
        self.last_joy_msg: Optional[Joy] = None
        self.last_nonzero = False
        self.last_command_time = self.get_clock().now()

        # Log successful initialization
        self._log_initialization()

    def _declare_parameters(self) -> None:
        """Declare all node parameters with default values."""
        # Robot configuration
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("wheelbase", 0.25)  # meters, typical for Yahboomcar
        self.declare_parameter("cmd_vel_namespace", "r2")  # New parameter for topic name
        self.declare_parameter("cmd_vel_topic", "cmd_vel")  # New parameter for topic name

        # Joystick mapping
        self.declare_parameter("steering_axis", 2)
        self.declare_parameter("throttle_axis", 1)

        # Control limits
        self.declare_parameter("max_steering_angle", 45.0)  # degrees
        self.declare_parameter("max_speed", 1.8)  # m/s

        # Behavior configuration
        self.declare_parameter("reverse_steering", False)
        self.declare_parameter("idle_publish_rate", 10.0)  # Hz
        self.declare_parameter("deadzone_threshold", 0.1)
        self.declare_parameter("enable_safety_stop", True)

    def _get_parameters(self) -> None:
        """Retrieve parameter values from the parameter server."""
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.wheelbase = (
            self.get_parameter("wheelbase").get_parameter_value().double_value
        )
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.cmd_vel_namespace = (
            self.get_parameter("cmd_vel_namespace").get_parameter_value().string_value
        )
        self.steering_axis = (
            self.get_parameter("steering_axis").get_parameter_value().integer_value
        )
        self.throttle_axis = (
            self.get_parameter("throttle_axis").get_parameter_value().integer_value
        )
        self.max_steering_angle = (
            self.get_parameter("max_steering_angle").get_parameter_value().double_value
        )
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.reverse_steering = (
            self.get_parameter("reverse_steering").get_parameter_value().bool_value
        )
        self.idle_publish_rate = (
            self.get_parameter("idle_publish_rate").get_parameter_value().double_value
        )
        self.deadzone_threshold = (
            self.get_parameter("deadzone_threshold").get_parameter_value().double_value
        )
        self.enable_safety_stop = (
            self.get_parameter("enable_safety_stop").get_parameter_value().bool_value
        )

    def _validate_parameters(self) -> None:
        """Validate parameter values and apply corrections if needed."""
        # Validate and correct idle publish rate
        if self.idle_publish_rate <= 0.0:
            self.get_logger().warn(
                f"Invalid idle_publish_rate: {self.idle_publish_rate}. Using default 10.0 Hz"
            )
            self.idle_publish_rate = 10.0

        # Validate wheelbase
        if self.wheelbase <= 0.0:
            raise ValueError(f"Wheelbase must be positive, got: {self.wheelbase}")

        # Validate max speed
        if self.max_speed <= 0.0:
            raise ValueError(f"Max speed must be positive, got: {self.max_speed}")

        # Validate max steering angle
        if not (0.0 < self.max_steering_angle <= 90.0):
            raise ValueError(
                f"Max steering angle must be between 0 and 90 degrees, got: {self.max_steering_angle}"
            )

        # Validate deadzone
        if not (0.0 <= self.deadzone_threshold < 1.0):
            raise ValueError(
                f"Deadzone threshold must be between 0 and 1, got: {self.deadzone_threshold}"
            )

    def _setup_qos_profiles(self) -> None:
        """Setup QoS profiles for different types of communication."""
        # Command QoS - reliable for control commands
        self.cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Joystick QoS - best effort for real-time input
        self.joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _create_publishers(self) -> None:
        """Create ROS2 publishers."""
        self.drive_publisher = self.create_publisher(Twist, self.cmd_vel_namespace + "/" + self.cmd_vel_topic, self.cmd_qos)

    def _create_subscribers(self) -> None:
        """Create ROS2 subscribers."""
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self._joy_callback, self.joy_qos
        )

    def _create_timers(self) -> None:
        """Create ROS2 timers."""
        idle_period = 1.0 / self.idle_publish_rate
        self.idle_timer = self.create_timer(idle_period, self._idle_publish_callback)

    def _log_initialization(self) -> None:
        """Log initialization parameters and status."""
        self.get_logger().info("Joystick Node initialized with parameters:")
        self.get_logger().info(f"  frame_id: '{self.frame_id}'")
        self.get_logger().info(f"  wheelbase: {self.wheelbase:.3f} m")
        self.get_logger().info(f"  steering_axis: {self.steering_axis}")
        self.get_logger().info(f"  throttle_axis: {self.throttle_axis}")
        self.get_logger().info(f"  max_steering_angle: {self.max_steering_angle:.1f}°")
        self.get_logger().info(f"  max_speed: {self.max_speed:.2f} m/s")
        self.get_logger().info(f"  reverse_steering: {self.reverse_steering}")
        self.get_logger().info(f"  idle_publish_rate: {self.idle_publish_rate:.1f} Hz")
        self.get_logger().info(f"  deadzone_threshold: {self.deadzone_threshold:.2f}")
        self.get_logger().info(f"  safety_stop_enabled: {self.enable_safety_stop}")
        self.get_logger().info("Joystick Node has been started.")

    def _joy_callback(self, msg: Joy) -> None:
        """
        Callback for joystick messages.

        Converts joystick input to Twist commands using Ackermann steering geometry.

        Args:
            msg: The incoming joystick message
        """
        try:
            # Validate joystick message
            if not self._validate_joy_message(msg):
                return

            # Apply deadzone and get raw input values
            steering_raw = self._apply_deadzone(msg.axes[self.steering_axis])
            throttle_raw = self._apply_deadzone(msg.axes[self.throttle_axis])

            # Scale inputs to physical units
            steering_angle = steering_raw * self.max_steering_angle
            if self.reverse_steering:
                steering_angle = -steering_angle

            speed = throttle_raw * self.max_speed

            # Convert to Twist command
            twist_msg = self._calculate_twist_command(steering_angle, speed)

            # Publish command
            self.drive_publisher.publish(twist_msg)
            self.last_command_time = self.get_clock().now()

            # Update state
            self.last_joy_msg = msg
            self.last_nonzero = abs(steering_angle) > 1e-6 or abs(speed) > 1e-6

            # Debug logging
            self.get_logger().debug(
                f"Joy input - steering: {steering_raw:.3f} → {steering_angle:.1f}°, "
                f"throttle: {throttle_raw:.3f} → {speed:.2f} m/s, "
                f"output - linear.x: {twist_msg.linear.x:.3f}, angular.z: {twist_msg.angular.z:.3f}"
            )

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing joystick input: {e}")

    def _validate_joy_message(self, msg: Joy) -> bool:
        """
        Validate that the joystick message has the required axes.

        Args:
            msg: Joystick message to validate

        Returns:
            True if message is valid, False otherwise
        """
        if len(msg.axes) <= max(self.steering_axis, self.throttle_axis):
            self.get_logger().error(
                f"Joystick message has {len(msg.axes)} axes, but need at least "
                f"{max(self.steering_axis, self.throttle_axis) + 1} for configured axes"
            )
            return False
        return True

    def _apply_deadzone(self, value: float) -> float:
        """
        Apply deadzone to joystick input to eliminate noise and drift.

        Args:
            value: Raw joystick axis value [-1.0, 1.0]

        Returns:
            Processed value with deadzone applied
        """
        if abs(value) < self.deadzone_threshold:
            return 0.0

        # Scale the remaining range to maintain full output range
        if value > 0:
            return (value - self.deadzone_threshold) / (1.0 - self.deadzone_threshold)
        else:
            return (value + self.deadzone_threshold) / (1.0 - self.deadzone_threshold)

    def _calculate_twist_command(
        self, steering_angle_deg: float, speed: float
    ) -> Twist:
        """
        Calculate Twist command from steering angle and speed using Ackermann geometry.

        Args:
            steering_angle_deg: Steering angle in degrees
            speed: Forward speed in m/s

        Returns:
            Twist message with calculated linear and angular velocities
        """
        twist_msg = Twist()

        # Set linear velocity
        twist_msg.linear.x = speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        # Calculate angular velocity using Ackermann steering geometry
        steering_rad = math.radians(steering_angle_deg)

        if abs(steering_rad) > 1e-6 and abs(speed) > 1e-6:
            # Angular velocity = v * tan(δ) / L
            # where v = speed, δ = steering angle, L = wheelbase
            twist_msg.angular.z = speed * math.tan(steering_rad) / self.wheelbase
        else:
            twist_msg.angular.z = 0.0

        # Set unused angular components to zero
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        return twist_msg

    def _idle_publish_callback(self) -> None:
        """
        Timer callback to publish zero command when joystick is idle.

        This ensures the robot stops when no joystick input is received
        and maintains a consistent command rate for the control system.
        """
        # Don't publish if no joystick message has been received
        if self.last_joy_msg is None:
            return

        # Check for safety stop condition
        if self.enable_safety_stop:
            current_time = self.get_clock().now()
            time_since_last_command = (
                current_time - self.last_command_time
            ).nanoseconds * 1e-9

            # If too much time has passed without joystick input, send stop command
            if time_since_last_command > 1.0:  # 1 second timeout
                self._publish_stop_command()
                return

        # If the last command was not zero, publish a zero command
        if not self.last_nonzero:
            self._publish_stop_command()

        # Reset the nonzero flag
        self.last_nonzero = False

    def _publish_stop_command(self) -> None:
        """Publish a stop command (all zeros)."""
        try:
            twist_msg = Twist()
            # All fields default to 0.0, so no need to set them explicitly
            self.drive_publisher.publish(twist_msg)

            self.get_logger().debug("Published stop command")
        except Exception:
            # Silently ignore publish errors during shutdown
            pass

    def destroy_node(self) -> None:
        """
        Clean up resources before shutting down the node.
        """
        self.get_logger().info("Shutting down Joystick Node...")

        # Send final stop command for safety (only if context is still valid)
        try:
            if hasattr(self, "drive_publisher") and rclpy.ok():
                self._publish_stop_command()
        except Exception:
            # Ignore errors during shutdown
            pass

        super().destroy_node()


def main(args=None) -> None:
    """
    Main entry point for the JoystickNode.

    Initializes ROS2, creates the joystick node, and handles graceful shutdown.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)

    node = None
    try:
        node = JoystickNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutdown requested by user")
    except rclpy.executors.ExternalShutdownException:
        # Normal shutdown via signal
        pass
    except (RuntimeError, ValueError) as e:
        if node:
            node.get_logger().error(f"Node error: {e}")
        else:
            print(f"Failed to initialize JoystickNode: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                # Ignore cleanup errors
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
