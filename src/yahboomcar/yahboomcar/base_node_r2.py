#!/usr/bin/env python3
"""
Base Node R2 for Yahboomcar (Python port of base_node_R2.cpp)

Publishes odometry and broadcasts TF from velocity commands. This node subscribes to /cmd_vel,
computes odometry using dead reckoning, and optionally publishes the odom->base_footprint transform.
Supports Ackermann steering geometry for accurate mobile robot localization.

Safety Features:
- Publishes zero velocity on startup to ensure robot starts stationary
- Publishes zero velocity on shutdown to ensure robot stops safely
"""
import math
import signal
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class BaseNodeR2(Node):
    """
    ROS2 Node for publishing odometry and broadcasting TF for Yahboomcar.

    This node integrates velocity commands to compute dead reckoning odometry
    and publishes both odometry messages and TF transforms for robot localization.
    """

    def __init__(self):
        """
        Initialize the BaseNodeR2 node with parameters and ROS2 interfaces.
        """
        super().__init__("base_node_r2")

        # Initialize shutdown flag
        self._shutdown_requested = False

        # Initialize all instance attributes
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading = 0.0
        self.current_velocity: Optional[Twist] = None
        self.last_velocity_time: Optional[Time] = None
        self.last_update_time: Optional[Time] = None
        self._startup_timer = None
        self._odom_timer = None

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Declare parameters with validation
        self._declare_parameters()

        # Get and validate parameters
        self._get_parameters()
        self._validate_parameters()

        # Initialize state variables
        self._initialize_state()

        # Setup QoS profiles
        self._setup_qos_profiles()

        # Create ROS2 interfaces
        self._create_subscribers()
        self._create_publishers()
        self._create_tf_broadcaster()

        # Log initialization success
        self._log_initialization()

        # Add timer for periodic odometry updates
        self._odom_timer = self.create_timer(
            1.0 / self.publish_rate, self._update_odometry
        )

    def _signal_handler(self, signum: int, _frame) -> None:
        """Handle shutdown signals gracefully and publish safety commands immediately."""
        self.get_logger().info(f"Received signal {signum}, executing emergency stop...")
        self._shutdown_requested = True

        # Immediately publish zero velocity while ROS context is still valid
        try:
            self._publish_emergency_stop()
        except Exception as e:
            self.get_logger().error(f"Failed to publish emergency stop: {e}")

        # Request ROS shutdown to break the spin loop
        rclpy.shutdown()

    def _declare_parameters(self) -> None:
        """Declare all node parameters with default values."""
        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("linear_scale_x", 1.0)
        self.declare_parameter("linear_scale_y", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("pub_odom_tf", False)
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("max_velocity_age", 0.5)  # seconds

    def _get_parameters(self) -> None:
        """Retrieve parameter values from the parameter server."""
        self.wheelbase = (
            self.get_parameter("wheelbase").get_parameter_value().double_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_footprint_frame = (
            self.get_parameter("base_footprint_frame")
            .get_parameter_value()
            .string_value
        )
        self.linear_scale_x = (
            self.get_parameter("linear_scale_x").get_parameter_value().double_value
        )
        self.linear_scale_y = (
            self.get_parameter("linear_scale_y").get_parameter_value().double_value
        )
        self.angular_scale = (
            self.get_parameter("angular_scale").get_parameter_value().double_value
        )
        self.pub_odom_tf = (
            self.get_parameter("pub_odom_tf").get_parameter_value().bool_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.max_velocity_age = (
            self.get_parameter("max_velocity_age").get_parameter_value().double_value
        )

    def _validate_parameters(self) -> None:
        """Validate parameter values and raise exceptions for invalid ones."""
        if self.wheelbase <= 0:
            raise ValueError("Wheelbase must be positive")
        if self.publish_rate <= 0 or self.publish_rate > 1000:
            raise ValueError("Publish rate must be between 0 and 1000 Hz")
        if self.max_velocity_age <= 0:
            raise ValueError("Max velocity age must be positive")
        if not self.odom_frame or not self.base_footprint_frame:
            raise ValueError("Frame names cannot be empty")

    def _initialize_state(self) -> None:
        """Initialize state variables for odometry tracking."""
        # Position and orientation state
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading = 0.0

        # Velocity state
        self.current_velocity: Optional[Twist] = None
        self.last_velocity_time: Optional[Time] = None

        # Timing - use ROS clock consistently
        self.last_update_time = self.get_clock().now()

        # Initialize with zero velocity for safety
        self._publish_initial_zero_velocity()

    def _setup_qos_profiles(self) -> None:
        """Setup QoS profiles for different types of communication."""
        # Reliable QoS for critical odometry data
        self.odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Best effort QoS for velocity commands (real-time priority)
        self.cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _create_subscribers(self) -> None:
        """Create ROS2 subscribers."""
        self.velocity_subscription = self.create_subscription(
            Twist, "cmd_vel", self._velocity_callback, self.cmd_vel_qos
        )

    def _create_publishers(self) -> None:
        """Create ROS2 publishers."""
        self.odom_publisher = self.create_publisher(Odometry, "odom", self.odom_qos)

        # Publisher for safety commands (zero velocity on shutdown)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "cmd_vel", self.cmd_vel_qos
        )

    def _create_tf_broadcaster(self) -> None:
        """Create TF broadcaster for coordinate frame transforms."""
        if self.pub_odom_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

    def _log_initialization(self) -> None:
        """Log initialization parameters and status."""
        self.get_logger().info("BaseNodeR2 initialized with parameters:")
        self.get_logger().info(f"  wheelbase: {self.wheelbase:.3f} m")
        self.get_logger().info(f"  odom_frame: '{self.odom_frame}'")
        self.get_logger().info(f"  base_footprint_frame: '{self.base_footprint_frame}'")
        self.get_logger().info(f"  linear_scale_x: {self.linear_scale_x:.3f}")
        self.get_logger().info(f"  linear_scale_y: {self.linear_scale_y:.3f}")
        self.get_logger().info(f"  angular_scale: {self.angular_scale:.3f}")
        self.get_logger().info(f"  pub_odom_tf: {self.pub_odom_tf}")
        self.get_logger().info(f"  publish_rate: {self.publish_rate:.1f} Hz")
        self.get_logger().info(f"  max_velocity_age: {self.max_velocity_age:.3f} s")

    def _velocity_callback(self, msg: Twist) -> None:
        """
        Callback for /cmd_vel topic. Stores velocity and updates timestamp.

        Args:
            msg: Twist message containing linear and angular velocity commands
        """
        current_time = self.get_clock().now()

        # Store current velocity and timestamp
        self.current_velocity = msg
        self.last_velocity_time = current_time

        # Update odometry based on the new velocity
        self._update_odometry()

        self.get_logger().debug(
            f"Velocity command - vx: {msg.linear.x:.3f}, vy: {msg.linear.y:.3f}, "
            f"vth: {msg.angular.z:.3f}"
        )

    def _update_odometry(self) -> None:
        """
        Update odometry based on current velocity and time differential.
        Publishes odometry message and optionally TF transform.
        """
        if self.current_velocity is None or self.last_velocity_time is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds * 1e-9

        # Skip update if time differential is invalid
        if dt <= 0:
            self.get_logger().warn(
                "Non-positive time differential, skipping odometry update"
            )
            self.last_update_time = current_time
            return

        # Check if velocity is too old
        velocity_age = (current_time - self.last_velocity_time).nanoseconds * 1e-9
        if velocity_age > self.max_velocity_age:
            self.get_logger().debug(
                f"Velocity too old ({velocity_age:.3f}s), using zero velocity"
            )
            # Use zero velocity if command is too old
            vx = vy = vth = 0.0
        else:
            # Apply scaling to velocities
            vx = self.current_velocity.linear.x * self.linear_scale_x
            vy = self.current_velocity.linear.y * self.linear_scale_y
            vth = self.current_velocity.angular.z * self.angular_scale

        # Integrate velocities to update pose (dead reckoning)
        delta_x = (vx * math.cos(self.heading) - vy * math.sin(self.heading)) * dt
        delta_y = (vx * math.sin(self.heading) + vy * math.cos(self.heading)) * dt
        delta_th = vth * dt

        # Update pose
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_th

        # Normalize heading to [-pi, pi] for consistent orientation representation
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # Update timestamp
        self.last_update_time = current_time

        # Publish odometry and TF
        self._publish_odometry(current_time)
        if self.pub_odom_tf:
            self._publish_tf_transform(current_time)

        self.get_logger().debug(
            f"Odometry updated - x: {self.x_pos:.3f}, y: {self.y_pos:.3f}, "
            f"heading: {self.heading:.3f}, dt: {dt:.3f}"
        )

    def _publish_odometry(self, timestamp: Time) -> None:
        """
        Publish odometry message.

        Args:
            timestamp: Current timestamp for the message
        """
        odom_msg = Odometry()

        # Header
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_footprint_frame

        # Position
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation = self._yaw_to_quaternion(self.heading)

        # Velocity (use current velocity if available and recent)
        if (
            self.current_velocity is not None
            and self.last_velocity_time is not None
            and (timestamp - self.last_velocity_time).nanoseconds * 1e-9
            < self.max_velocity_age
        ):

            odom_msg.twist.twist.linear.x = (
                self.current_velocity.linear.x * self.linear_scale_x
            )
            odom_msg.twist.twist.linear.y = (
                self.current_velocity.linear.y * self.linear_scale_y
            )
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = (
                self.current_velocity.angular.z * self.angular_scale
            )
        else:
            # Zero velocity if no recent command
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

        # Set covariance matrices (diagonal values indicate uncertainty)
        # Position covariance (conservative estimates)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[14] = 1e6  # z (not used)
        odom_msg.pose.covariance[21] = 1e6  # roll (not used)
        odom_msg.pose.covariance[28] = 1e6  # pitch (not used)
        odom_msg.pose.covariance[35] = 0.2  # yaw

        # Velocity covariance
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[7] = 0.1  # vy
        odom_msg.twist.covariance[14] = 1e6  # vz (not used)
        odom_msg.twist.covariance[21] = 1e6  # vroll (not used)
        odom_msg.twist.covariance[28] = 1e6  # vpitch (not used)
        odom_msg.twist.covariance[35] = 0.2  # vyaw

        # Publish the message
        self.odom_publisher.publish(odom_msg)

    def _publish_tf_transform(self, timestamp: Time) -> None:
        """
        Publish TF transform from odom to base_footprint.

        Args:
            timestamp: Current timestamp for the transform
        """
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()

        # Header
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_footprint_frame

        # Translation
        transform.transform.translation.x = self.x_pos
        transform.transform.translation.y = self.y_pos
        transform.transform.translation.z = 0.0

        # Rotation (quaternion from yaw)
        transform.transform.rotation = self._yaw_to_quaternion(self.heading)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

        # Info-level log for TF publishing
        self.get_logger().debug(
            f"TF published: odom -> {self.base_footprint_frame} | x={self.x_pos:.3f}, y={self.y_pos:.3f}, heading={self.heading:.3f}"
        )

    def _publish_initial_zero_velocity(self) -> None:
        """
        Publish an initial zero velocity command to ensure the robot starts stationary.
        This is a safety feature to prevent unexpected movement on startup.
        """
        # Create a timer to publish zero velocity after a short delay
        # This allows the system to fully initialize before sending commands
        self._startup_timer = self.create_timer(0.1, self._send_zero_velocity_once)

    def _send_zero_velocity_once(self) -> None:
        """
        Send a single zero velocity command and destroy the timer.
        This ensures the robot is stationary on startup.
        """
        try:
            # Create zero velocity message
            zero_velocity = Twist()
            # All fields are already 0.0 by default

            # Simulate receiving this as a velocity command
            current_time = self.get_clock().now()
            self.current_velocity = zero_velocity
            self.last_velocity_time = current_time

            # Update odometry with zero velocity
            self._update_odometry()

            self.get_logger().info("Initial zero velocity command sent for safety")

        except Exception as e:
            self.get_logger().warn(f"Failed to send initial zero velocity: {e}")

        # Cancel the timer since we only want to run this once
        if hasattr(self, "_startup_timer"):
            self._startup_timer.cancel()
            self._startup_timer = None

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """
        Convert a yaw angle (in radians) to a geometry_msgs/Quaternion.

        Args:
            yaw: Yaw angle in radians

        Returns:
            Quaternion representation of the yaw rotation
        """
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def _publish_emergency_stop(self) -> None:
        """
        Immediately publish zero velocity commands for emergency stop.
        Called during signal handling while ROS context is still valid.
        """
        try:
            # Create zero velocity message
            zero_velocity = Twist()
            # All fields are already 0.0 by default

            # Publish multiple times immediately for safety
            for _ in range(5):  # More attempts for emergency stop
                self.cmd_vel_publisher.publish(zero_velocity)

            self.get_logger().info("Emergency stop commands published successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to publish emergency stop: {e}")

    def _publish_shutdown_zero_velocity(self) -> None:
        """
        Publish zero velocity commands on shutdown to ensure the robot stops.
        This is a critical safety feature to prevent runaway motion.
        """
        # Only attempt to publish if ROS context is still valid
        if not rclpy.ok():
            self.get_logger().warn(
                "Cannot publish shutdown zero velocity - ROS context is invalid"
            )
            return

        try:
            # Create zero velocity message
            zero_velocity = Twist()
            # All fields are already 0.0 by default

            # Publish multiple times to ensure message is received
            for _ in range(3):
                self.cmd_vel_publisher.publish(zero_velocity)

            self.get_logger().info(
                "Shutdown zero velocity commands published for safety"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to publish shutdown zero velocity: {e}")

    def destroy_node(self) -> None:
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Cleaning up BaseNodeR2 resources...")

        # Only try to publish shutdown commands if not already done by signal handler
        if not self._shutdown_requested:
            self._publish_shutdown_zero_velocity()

        # Cancel startup timer if it still exists
        if self._startup_timer is not None:
            try:
                self._startup_timer.cancel()
                self._startup_timer = None
            except Exception as e:
                self.get_logger().warn(f"Error cancelling startup timer: {e}")

        # Cancel odom timer if it exists
        if self._odom_timer is not None:
            try:
                self._odom_timer.cancel()
                self._odom_timer = None
            except Exception as e:
                self.get_logger().warn(f"Error cancelling odom timer: {e}")

        try:
            super().destroy_node()
        except Exception as e:
            self.get_logger().warn(f"Error in super().destroy_node(): {e}")


def main(args=None) -> None:
    """
    Main entry point for the BaseNodeR2 node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)

    node = None
    try:
        node = BaseNodeR2()
        node.get_logger().info("BaseNodeR2 node started successfully")
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutdown requested by user")
            # Attempt emergency stop if signal handler didn't run
            if not node._shutdown_requested:
                try:
                    node._publish_emergency_stop()
                except Exception as e:
                    node.get_logger().error(
                        f"Failed to publish emergency stop in main: {e}"
                    )
    except (RuntimeError, ValueError) as e:
        if node:
            node.get_logger().error(f"Node error: {e}")
        else:
            print(f"Failed to initialize BaseNodeR2: {e}")
    finally:
        if node:
            node.get_logger().info("Shutting down BaseNodeR2 node")
            node.destroy_node()
        # Only shutdown if not already done by signal handler
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
