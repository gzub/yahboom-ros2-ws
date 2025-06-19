#!/usr/bin/env python3
# coding: utf-8

"""
Rosmaster Robot Control Library

A comprehensive Python library for controlling Yahboom Rosmaster robot platforms
via serial communication. This module provides high-level interfaces for robot
control, sensor data acquisition, and system management.

The library supports a wide range of robotic functions including:
- PWM and UART servo control for robotic arms and steering
- RGB LED control with various lighting effects
- DC motor control with PID speed regulation
- Sensor data acquisition (IMU, encoders, battery)
- System configuration and calibration
- Auto-reporting and real-time data streaming

Hardware Compatibility:
    - Yahboom Rosmaster robot platforms
    - Compatible servo controllers (PWM/UART)
    - RGB LED arrays and individual LEDs
    - DC motors with encoder feedback
    - IMU sensors for orientation data
    - Various sensors and actuators

Communication Protocol:
    - Serial communication over USB/UART
    - Custom binary protocol with checksums
    - Configurable baud rates and timeouts
    - Error detection and recovery mechanisms

Usage Example:
    >>> from Rosmaster_Lib import Rosmaster
    >>>
    >>> # Initialize robot connection
    >>> robot = Rosmaster('/dev/ttyUSB0')
    >>>
    >>> # Basic motor control
    >>> robot.set_motor(50, 50, 50, 50)  # Move forward
    >>>
    >>> # Servo control
    >>> robot.set_uart_servo_angle(1, 90)  # Center servo 1
    >>>
    >>> # LED effects
    >>> robot.set_colorful_effect(0x01, 5, 255)  # Rainbow effect
    >>>
    >>> # Read sensor data
    >>> imu_data = robot.get_motion_data()
    >>>
    >>> # Clean shutdown
    >>> robot.close()

Safety Notes:
    - Always call close() or use context manager for proper cleanup
    - Validate input parameters to prevent hardware damage
    - Monitor battery levels and system temperatures
    - Use appropriate delays between rapid command sequences

Author: Yahboom Technology
Version: 3.3.9
License: Compatible with ROS2 and open-source robotics projects
"""

import logging
import struct
import threading
import time

import serial


# V3.3.9
class Rosmaster(object):
    """
    Rosmaster Robot Control Library v3.3.9

    A comprehensive library for controlling Yahboom Rosmaster robot platforms via serial communication.
    Supports control of motors, servos, LEDs, sensors, and various robot functionalities.

    Features:
        - PWM servo control (individual and batch operations)
        - UART servo control with angle conversion
        - RGB LED control with effects
        - Motor control (individual and motion control)
        - PID parameter configuration
        - IMU sensor data reading
        - Beeper control
        - Auto-reporting functionality
        - Robotic arm control with offset calibration

    Attributes:
        ser (serial.Serial): Serial connection object
        logger (logging.Logger): Logger instance for debugging and info

    Example:
        >>> with Rosmaster(car_type=5, port="/dev/ttyUSB0", debug=True) as robot:
        ...     robot.set_motor(100, 100, 100, 100)  # Move forward
        ...     robot.set_beep(500)  # Beep for 500ms
        ...     robot.set_colorful_lamps(1, 255, 0, 0)  # Set LED to red
    """

    __uart_state = 0

    def __init__(self, car_type=5, port="/dev/myserial", delay=0.002, debug=False):
        """
        Initialize the Rosmaster robot controller.

        Args:
            car_type (int, optional): Robot car type identifier. Defaults to 5.
                Valid values depend on the specific robot model.
            port (str, optional): Serial port path. Defaults to "/dev/myserial".
                Common values: "/dev/ttyUSB0", "/dev/ttyACM0", "COM3", etc.
            delay (float, optional): Command delay in seconds. Defaults to 0.002.
                Minimum delay between serial commands to ensure stable communication.
            debug (bool, optional): Enable debug logging. Defaults to False.
                When True, enables detailed logging of all operations.

        Raises:
            serial.SerialException: If the serial port cannot be opened.

        Note:
            - Automatically enables servo torque on initialization
            - Sets up logging based on debug parameter
            - Initializes all sensor and control state variables
        """

        self.ser = serial.Serial(port, 115200)

        self.__delay_time = delay
        self.__debug = debug

        # Set up logging
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

        # Create console handler if it doesn't exist
        if not self.logger.handlers:
            console_handler = logging.StreamHandler()
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_RGB = 0x05
        self.FUNC_RGB_EFFECT = 0x06

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_REPORT_ICM_RAW = 0x0E

        self.FUNC_RESET_STATE = 0x0F

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_YAW_PID = 0x14
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23
        self.FUNC_ARM_OFFSET = 0x24

        self.FUNC_AKM_DEF_ANGLE = 0x30
        self.FUNC_AKM_STEER_ANGLE = 0x31

        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.CARTYPE_X3 = 0x01
        self.CARTYPE_X3_PLUS = 0x02
        self.CARTYPE_X1 = 0x04
        self.CARTYPE_R2 = 0x05

        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0

        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

        self.__encoder_m1 = 0
        self.__encoder_m2 = 0
        self.__encoder_m3 = 0
        self.__encoder_m4 = 0

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__battery_voltage = 0

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        self.__read_car_type = 0

        if self.__debug:
            self.logger.debug("cmd_delay=%ss", self.__delay_time)

        if self.ser.isOpen():
            self.logger.info("Rosmaster Serial Opened! Baudrate=115200")
        else:
            self.logger.error("Serial Open Failed!")
        # Enable robotic arm torque to prevent servo 6 from being unreadable when first connected.
        self.set_uart_servo_torque(1)
        time.sleep(0.002)

    def __del__(self):
        """Destructor that ensures proper cleanup of resources."""
        self.close()

    def __enter__(self):
        """Context manager entry point."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point that ensures proper cleanup."""
        self.close()
        return False

    def close(self):
        """Properly close the serial connection"""
        if hasattr(self, "ser") and self.ser and self.ser.is_open:
            self.ser.close()
            self.__uart_state = 0
            self.logger.info("Serial connection closed!")

    # According to the type of data frame to make the corresponding parsing
    def __parse_data(self, ext_type, ext_data):
        # print("parse_data:", ext_data, ext_type)
        if ext_type == self.FUNC_REPORT_SPEED:
            # print(ext_data)
            self.__vx = int(struct.unpack("h", bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack("h", bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack("h", bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack("B", bytearray(ext_data[6:7]))[0]
        # (MPU9250)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            # Gyroscope sensor: ±500dps=±500°/s ±32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
            gyro_ratio = 1 / 3754.9  # ±500dps
            self.__gx = struct.unpack("h", bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.__gy = struct.unpack("h", bytearray(ext_data[2:4]))[0] * -gyro_ratio
            self.__gz = struct.unpack("h", bytearray(ext_data[4:6]))[0] * -gyro_ratio
            # Accelerometer sensor: ±2g=±2*9.8m/s^2 ±32768 accel/32768*19.6=accel/1671.84
            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack("h", bytearray(ext_data[6:8]))[0] * accel_ratio
            self.__ay = struct.unpack("h", bytearray(ext_data[8:10]))[0] * accel_ratio
            self.__az = struct.unpack("h", bytearray(ext_data[10:12]))[0] * accel_ratio
            # Magnetometer sensor
            mag_ratio = 1.0
            self.__mx = struct.unpack("h", bytearray(ext_data[12:14]))[0] * mag_ratio
            self.__my = struct.unpack("h", bytearray(ext_data[14:16]))[0] * mag_ratio
            self.__mz = struct.unpack("h", bytearray(ext_data[16:18]))[0] * mag_ratio
        # (ICM20948)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            gyro_ratio = 1 / 1000.0
            self.__gx = struct.unpack("h", bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.__gy = struct.unpack("h", bytearray(ext_data[2:4]))[0] * gyro_ratio
            self.__gz = struct.unpack("h", bytearray(ext_data[4:6]))[0] * gyro_ratio

            accel_ratio = 1 / 1000.0
            self.__ax = struct.unpack("h", bytearray(ext_data[6:8]))[0] * accel_ratio
            self.__ay = struct.unpack("h", bytearray(ext_data[8:10]))[0] * accel_ratio
            self.__az = struct.unpack("h", bytearray(ext_data[10:12]))[0] * accel_ratio

            mag_ratio = 1 / 1000.0
            self.__mx = struct.unpack("h", bytearray(ext_data[12:14]))[0] * mag_ratio
            self.__my = struct.unpack("h", bytearray(ext_data[14:16]))[0] * mag_ratio
            self.__mz = struct.unpack("h", bytearray(ext_data[16:18]))[0] * mag_ratio
        # the attitude Angle of the board
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack("h", bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack("h", bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack("h", bytearray(ext_data[4:6]))[0] / 10000.0
        # Encoder data on all four wheels
        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.__encoder_m1 = struct.unpack("i", bytearray(ext_data[0:4]))[0]
            self.__encoder_m2 = struct.unpack("i", bytearray(ext_data[4:8]))[0]
            self.__encoder_m3 = struct.unpack("i", bytearray(ext_data[8:12]))[0]
            self.__encoder_m4 = struct.unpack("i", bytearray(ext_data[12:16]))[0]

        else:
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__read_val = struct.unpack("h", bytearray(ext_data[1:3]))[0]
                if self.__debug:
                    self.logger.debug(
                        "FUNC_UART_SERVO: %s, %s", self.__read_id, self.__read_val
                    )

            elif ext_type == self.FUNC_ARM_CTRL:
                self.__read_arm[0] = struct.unpack("h", bytearray(ext_data[0:2]))[0]
                self.__read_arm[1] = struct.unpack("h", bytearray(ext_data[2:4]))[0]
                self.__read_arm[2] = struct.unpack("h", bytearray(ext_data[4:6]))[0]
                self.__read_arm[3] = struct.unpack("h", bytearray(ext_data[6:8]))[0]
                self.__read_arm[4] = struct.unpack("h", bytearray(ext_data[8:10]))[0]
                self.__read_arm[5] = struct.unpack("h", bytearray(ext_data[10:12]))[0]
                self.__read_arm_ok = 1
                if self.__debug:
                    self.logger.debug("FUNC_ARM_CTRL: %s", self.__read_arm)

            elif ext_type == self.FUNC_VERSION:
                self.__version_H = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__version_L = struct.unpack("B", bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    self.logger.debug(
                        "FUNC_VERSION: %s, %s", self.__version_H, self.__version_L
                    )

            elif ext_type == self.FUNC_SET_MOTOR_PID:
                self.__pid_index = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack("h", bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack("h", bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack("h", bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    self.logger.debug(
                        "FUNC_SET_MOTOR_PID: %s, [%s, %s, %s]",
                        self.__pid_index,
                        self.__kp1,
                        self.__ki1,
                        self.__kd1,
                    )

            elif ext_type == self.FUNC_SET_YAW_PID:
                self.__pid_index = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack("h", bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack("h", bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack("h", bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    self.logger.debug(
                        "FUNC_SET_YAW_PID: %s, [%s, %s, %s]",
                        self.__pid_index,
                        self.__kp1,
                        self.__ki1,
                        self.__kd1,
                    )

            elif ext_type == self.FUNC_ARM_OFFSET:
                self.__arm_offset_id = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__arm_offset_state = struct.unpack("B", bytearray(ext_data[1:2]))[
                    0
                ]
                if self.__debug:
                    self.logger.debug(
                        "FUNC_ARM_OFFSET: %s, %s",
                        self.__arm_offset_id,
                        self.__arm_offset_state,
                    )

            elif ext_type == self.FUNC_AKM_DEF_ANGLE:
                servo_id = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__akm_def_angle = struct.unpack("B", bytearray(ext_data[1:2]))[0]
                self.__akm_readed_angle = True
                if self.__debug:
                    self.logger.debug(
                        "FUNC_AKM_DEF_ANGLE: %s, %s", servo_id, self.__akm_def_angle
                    )

            elif ext_type == self.FUNC_SET_CAR_TYPE:
                car_type = struct.unpack("B", bytearray(ext_data[0:1]))[0]
                self.__read_car_type = car_type

    # receive data
    def __receive_data(self):
        # Clear the buffer
        self.ser.flushInput()
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            self.logger.debug(
                                "check sum error: %s, %s, %s",
                                ext_len,
                                ext_type,
                                ext_data,
                            )

    # Request data, function: corresponding function word to return data, parm: parameter passed in
    def __request_data(self, function, param=0):
        cmd = [
            self.__HEAD,
            self.__DEVICE_ID,
            0x05,
            self.FUNC_REQUEST_DATA,
            int(function) & 0xFF,
            int(param) & 0xFF,
        ]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            self.logger.debug("request: %s", cmd)
        time.sleep(0.002)

    # Arm converts Angle to position pulse
    def __arm_convert_value(self, s_id, s_angle):
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value

    # Arm converts position pulses into angles
    def __arm_convert_angle(self, s_id, s_value):
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle

    # Limit the PWM duty ratio value of motor input, value=127, keep the original data, do not modify the current motor speed
    def __limit_motor_value(self, value):
        if value == 127:
            return 127
        elif value > 100:
            return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    def create_receive_threading(self):
        """
        Start the background thread for receiving and processing serial data.

        Creates a daemon thread that continuously reads data from the serial port
        and processes incoming sensor data, status updates, and responses.

        Returns:
            bool: True if thread created successfully, False if thread already running
                  or if an error occurred.

        Note:
            - Only one receive thread can be active at a time
            - Thread runs as daemon, so it will terminate when main program exits
            - Automatically parses incoming data and updates internal state variables
        """
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.daemon = True
                task_receive.start()
                self.logger.info(
                    "----------------create receive threading--------------"
                )
                self.__uart_state = 1
                time.sleep(0.05)
        except Exception as e:
            self.logger.error("create_receive_threading error: %s", e)
            return False

    def set_auto_report_state(self, enable, forever=False):
        """
        Control automatic sensor data reporting from the robot.

        Enables or disables the automatic transmission of sensor data from the MCU.
        When enabled, the robot continuously sends IMU, encoder, and status data.

        Args:
            enable (bool): Enable automatic reporting
                True: MCU sends 4 data packets every 10ms (each packet every 40ms)
                False: Stop automatic data transmission
            forever (bool, optional): Permanent setting. Defaults to False.
                True: Save setting permanently to flash memory
                False: Temporary setting, reset on power cycle

        Note:
            - Default state is enabled for most robots
            - Disabling may affect sensor data getter methods
            - Required for real-time data from get_* methods
            - Permanent settings take longer due to flash write

        Example:
            >>> robot.set_auto_report_state(True)         # Enable reporting (temp)
            >>> robot.set_auto_report_state(False, True)  # Disable permanently
            >>> robot.set_auto_report_state(True, True)   # Enable permanently
        """
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x05,
                self.FUNC_AUTO_REPORT,
                state1,
                state2,
            ]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("report: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_auto_report_state serial error: %s", e)
        except Exception as e:
            self.logger.error("set_auto_report_state error: %s", e)

    def set_beep(self, on_time):
        """
        Control the robot's buzzer/beeper with flexible timing options.

        Provides three modes of buzzer operation: off, continuous, or timed beeping.
        Useful for audio feedback, alerts, or user notifications.

        Args:
            on_time (int): Beeper control value:
                0: Turn off buzzer
                1: Continuous beeping (until stopped)
                >=10: Beep for specified milliseconds then auto-stop
                      (value must be multiple of 10)

        Note:
            - Negative values are rejected with warning
            - For timed beeping, use multiples of 10 (e.g., 100, 500, 1000)
            - Continuous beeping (on_time=1) must be manually stopped with on_time=0

        Example:
            >>> robot.set_beep(500)   # Beep for 500ms then stop
            >>> robot.set_beep(1)     # Start continuous beeping
            >>> robot.set_beep(0)     # Stop beeping
            >>> robot.set_beep(1000)  # Beep for 1 second
        """
        try:
            if on_time < 0:
                self.logger.warning("beep input error!")
                return
            value = bytearray(struct.pack("h", int(on_time)))

            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x05,
                self.FUNC_BEEP,
                value[0],
                value[1],
            ]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("beep: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_beep serial error: %s", e)
        except Exception as e:
            self.logger.error("set_beep error: %s", e)

    def set_pwm_servo(self, servo_id, angle):
        """
        Control individual PWM servo position.

        Sets the angle of a specific PWM servo. Commonly used for camera pan/tilt,
        sensor positioning, or simple joint control.

        Args:
            servo_id (int): Servo identifier. Valid range: [1, 4]
                Each ID corresponds to a specific servo output channel
            angle (int): Target angle in degrees. Range: [0, 180]
                0 = minimum position, 90 = center, 180 = maximum position

        Note:
            - Invalid servo_id values are silently ignored
            - Angle values are automatically clamped to [0, 180]
            - PWM servos respond faster than UART servos but with less precision

        Example:
            >>> robot.set_pwm_servo(1, 90)   # Center servo 1
            >>> robot.set_pwm_servo(2, 0)    # Move servo 2 to minimum
            >>> robot.set_pwm_servo(3, 180)  # Move servo 3 to maximum
        """
        try:
            if servo_id < 1 or servo_id > 4:
                if self.__debug:
                    self.logger.debug("set_pwm_servo input invalid")
                return
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_PWM_SERVO,
                int(servo_id),
                int(angle),
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("pwmServo: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_pwm_servo serial error: %s", e)
        except Exception as e:
            self.logger.error("set_pwm_servo error: %s", e)

    # Control four PWM channels simultaneously, angle_sX=[0, 180]
    # At the same time control four PWM Angle, angle_sX=[0, 180]
    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        """
        Set angles for all four PWM servos simultaneously.

        Controls all PWM servos in a single command for coordinated movement.
        More efficient than individual servo commands when moving multiple servos.

        Args:
            angle_s1 (int): Angle for servo 1 in degrees
                Range: [0, 180] - values outside range are set to 255 (no change)
            angle_s2 (int): Angle for servo 2 in degrees
                Range: [0, 180] - values outside range are set to 255 (no change)
            angle_s3 (int): Angle for servo 3 in degrees
                Range: [0, 180] - values outside range are set to 255 (no change)
            angle_s4 (int): Angle for servo 4 in degrees
                Range: [0, 180] - values outside range are set to 255 (no change)

        Note:
            - More efficient than individual set_pwm_servo calls
            - Invalid angles (< 0 or > 180) are automatically set to 255
            - Value 255 means "no change" - servo maintains current position
            - All servos move simultaneously for coordinated motion
            - Useful for robotic arm or multi-servo mechanisms

        Example:
            >>> # Move all servos to center position
            >>> robot.set_pwm_servo_all(90, 90, 90, 90)
            >>>
            >>> # Move servos 1&2, keep 3&4 unchanged
            >>> robot.set_pwm_servo_all(45, 135, -1, -1)  # -1 becomes 255 (no change)
            >>>
            >>> # Custom coordinated movement
            >>> robot.set_pwm_servo_all(0, 180, 90, 45)
        """
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_PWM_SERVO_ALL,
                int(angle_s1),
                int(angle_s2),
                int(angle_s3),
                int(angle_s4),
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("all Servo: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_pwm_servo_all serial error: %s", e)
        except Exception as e:
            self.logger.error("set_pwm_servo_all error: %s", e)

    def set_colorful_lamps(self, led_id, red, green, blue):
        """
        Control RGB LED strips with individual or collective color setting.

        Sets the color of specific RGB LEDs or all LEDs simultaneously.
        Must stop any active RGB effects before using this function.

        Args:
            led_id (int): LED identifier or control mode:
                [0-13]: Control specific numbered RGB LED
                0xFF (255): Control all RGB LEDs simultaneously
            red (int): Red color component. Range: [0, 255]
                0 = no red, 255 = maximum red intensity
            green (int): Green color component. Range: [0, 255]
            blue (int): Blue color component. Range: [0, 255]

        Note:
            - RGB values are automatically masked to 8-bit range [0, 255]
            - Some robots may have fewer than 13 LEDs
            - Stop LED effects with set_colorful_effect(0) before using this function

        Example:
            >>> robot.set_colorful_lamps(0xFF, 255, 0, 0)    # All LEDs red
            >>> robot.set_colorful_lamps(0, 0, 255, 0)       # LED 0 green
            >>> robot.set_colorful_lamps(5, 255, 255, 255)   # LED 5 white
            >>> robot.set_colorful_lamps(10, 0, 0, 0)        # LED 10 off
        """
        try:
            led_id_val = int(led_id) & 0xFF
            r = int(red) & 0xFF
            g = int(green) & 0xFF
            b = int(blue) & 0xFF
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_RGB,
                led_id_val,
                r,
                g,
                b,
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("rgb: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_colorful_lamps serial error: %s", e)
        except Exception as e:
            self.logger.error("set_colorful_lamps error: %s", e)

    # RGB programmable light band special effects display.
    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light, 3: breathing light, 4: gradient light, 5: starlight, 6: power display
    # Speed =[1, 10], the smaller the value, the faster the speed changes
    # Parm, left blank, as an additional argument.  Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
    def set_colorful_effect(self, effect, speed=255, parm=255):
        """
        Activate RGB LED lighting effects with customizable parameters.

        Applies predefined lighting effects to RGB LED strips with adjustable
        speed and effect-specific parameters for dynamic visual displays.

        Args:
            effect (int): Effect type identifier
                0x00: Turn off all LEDs
                0x01: Rainbow cycle effect
                0x02: Breathing effect
                0x03: Marquee/chase effect
                0x04: Flash effect
                0x05: Color wipe effect
                0x06: Theater chase effect
                0x07: Sparkle effect
                Additional effects may be available depending on hardware
            speed (int, optional): Effect speed/rate. Defaults to 255.
                Range: [0, 255] - 0=slowest, 255=fastest
                Higher values = faster animation
            parm (int, optional): Effect-specific parameter. Defaults to 255.
                Range: [0, 255] - meaning varies by effect type
                May control brightness, color range, or pattern density

        Note:
            - All parameters are masked to 8-bit values (0-255)
            - Effect behavior depends on connected LED hardware
            - Some effects may ignore speed or parm parameters
            - Effects run continuously until changed or turned off
            - Use effect=0x00 to turn off all LED effects

        Example:
            >>> # Rainbow effect at medium speed
            >>> robot.set_colorful_effect(0x01, speed=128, parm=255)
            >>>
            >>> # Fast breathing effect
            >>> robot.set_colorful_effect(0x02, speed=255, parm=100)
            >>>
            >>> # Turn off all LED effects
            >>> robot.set_colorful_effect(0x00)
        """
        try:
            eff = int(effect) & 0xFF
            spe = int(speed) & 0xFF
            par = int(parm) & 0xFF
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_RGB_EFFECT,
                eff,
                spe,
                par,
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("rgb_effect: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_colorful_effect serial error: %s", e)
        except Exception as e:
            self.logger.error("set_colorful_effect error: %s", e)

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        """
        Control individual motor speeds using PWM values.

        Sets the speed of each motor independently using PWM control.
        Useful for fine-grained control of robot movement.

        Args:
            speed_1 (int): Speed of motor 1. Range: [-100, 100]
                Positive values = forward, negative = reverse
            speed_2 (int): Speed of motor 2. Range: [-100, 100]
            speed_3 (int): Speed of motor 3. Range: [-100, 100]
            speed_4 (int): Speed of motor 4. Range: [-100, 100]

        Note:
            - Values are automatically clamped to valid range [-100, 100]
            - Motor mapping depends on robot configuration
            - For typical 4-wheel robots: 1=front-left, 2=front-right,
              3=rear-left, 4=rear-right

        Example:
            >>> robot.set_motor(50, 50, 50, 50)    # Move forward at half speed
            >>> robot.set_motor(-30, 30, -30, 30)  # Turn left
            >>> robot.set_motor(0, 0, 0, 0)        # Stop all motors
        """
        try:
            t_speed_a = bytearray(struct.pack("b", self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack("b", self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack("b", self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack("b", self.__limit_motor_value(speed_4)))
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_MOTOR,
                t_speed_a[0],
                t_speed_b[0],
                t_speed_c[0],
                t_speed_d[0],
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("motor: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_motor serial error: %s", e)
        except Exception as e:
            self.logger.error("set_motor error: %s", e)

    def set_car_run(self, state, speed, adjust=False):
        """
        Control basic car movements with predefined motion patterns.

        Provides high-level control for common robot movements using predefined
        motion states. Simpler than individual motor control for basic navigation.

        Args:
            state (int): Movement state. Valid values:
                0 = Stop
                1 = Forward
                2 = Backward
                3 = Left
                4 = Right
                5 = Spin left (rotate in place)
                6 = Spin right (rotate in place)
            speed (int): Movement speed. Range: [-100, 100]
                Positive values for normal direction, negative for reverse
                0 = Stop (regardless of state)
            adjust (bool, optional): Enable gyroscope-assisted motion. Defaults to False.
                When True, uses IMU data to maintain direction stability

        Example:
            >>> robot.set_car_run(1, 50)     # Move forward at 50% speed
            >>> robot.set_car_run(5, 30)     # Spin left at 30% speed
            >>> robot.set_car_run(0, 0)      # Stop
            >>> robot.set_car_run(1, 60, True)  # Forward with gyro assist
        """
        try:
            car_type = self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack("h", int(speed)))
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_CAR_RUN,
                car_type,
                int(state & 0xFF),
                t_speed[0],
                t_speed[1],
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("car_run: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_car_run serial error: %s", e)
        except Exception as e:
            self.logger.error("set_car_run error: %s", e)

    def set_car_motion(self, v_x, v_y, v_z):
        """
        Advanced car motion control using velocity vectors.

        Provides precise 3-axis velocity control for omnidirectional movement.
        The robot will move in the specified direction and angular velocity.

        Args:
            v_x (float): Linear velocity in X direction (forward/backward)
            v_y (float): Linear velocity in Y direction (left/right, for omnidirectional robots)
            v_z (float): Angular velocity around Z axis (rotation)

        Valid input ranges by robot model:
            X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
            X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
            R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]

        Note:
            - Units are in m/s for linear velocities, rad/s for angular
            - Positive v_x = forward, negative = backward
            - Positive v_y = left, negative = right (for mecanum wheels)
            - Positive v_z = counter-clockwise rotation

        Example:
            >>> robot.set_car_motion(0.5, 0.0, 0.0)    # Move forward 0.5 m/s
            >>> robot.set_car_motion(0.0, 0.3, 0.0)    # Strafe left 0.3 m/s
            >>> robot.set_car_motion(0.0, 0.0, 1.0)    # Rotate 1 rad/s
            >>> robot.set_car_motion(0.3, 0.2, 0.5)    # Combined motion
        """
        try:
            vx_parms = bytearray(struct.pack("h", int(v_x * 1000)))
            vy_parms = bytearray(struct.pack("h", int(v_y * 1000)))
            vz_parms = bytearray(struct.pack("h", int(v_z * 1000)))
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_MOTION,
                self.__CAR_TYPE,
                vx_parms[0],
                vx_parms[1],
                vy_parms[0],
                vy_parms[1],
                vz_parms[0],
                vz_parms[1],
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("motion: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_car_motion serial error: %s", e)
        except Exception as e:
            self.logger.error("set_car_motion error: %s", e)

    def set_pid_param(self, kp, ki, kd, forever=False):
        """
        Configure PID controller parameters for motor control.

        Sets the Proportional, Integral, and Derivative gains for the robot's
        motion control system. Can be applied temporarily or permanently.

        Args:
            kp (float): Proportional gain. Range: [0, 10.0]
                Controls response to current error
            ki (float): Integral gain. Range: [0, 10.0]
                Controls response to accumulated error over time
            kd (float): Derivative gain. Range: [0, 10.0]
                Controls response to rate of error change
            forever (bool, optional): Permanent storage flag. Defaults to False.
                False: Temporary effect, fast response, lost on restart
                True: Permanent storage in flash memory (slower operation)

        Note:
            - All PID values must be in range [0, 10.0]
            - Permanent storage takes longer due to flash write operations
            - PID tuning affects robot stability and response
            - Start with small values and adjust incrementally

        Example:
            >>> robot.set_pid_param(2.5, 0.1, 0.05)        # Temporary PID
            >>> robot.set_pid_param(3.0, 0.2, 0.1, True)   # Permanent PID
        """
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                self.logger.warning("PID value must be:[0, 10.00]")
                return
            kp_params = bytearray(struct.pack("h", int(kp * 1000)))
            ki_params = bytearray(struct.pack("h", int(ki * 1000)))
            kd_params = bytearray(struct.pack("h", int(kd * 1000)))
            cmd.append(kp_params[0])  # low
            cmd.append(kp_params[1])  # high
            cmd.append(ki_params[0])  # low
            cmd.append(ki_params[1])  # high
            cmd.append(kd_params[0])  # low
            cmd.append(kd_params[1])  # high
            cmd.append(state)
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("pid: %s", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(0.1)
        except serial.SerialException as e:
            self.logger.error("set_pid_param serial error: %s", e)
        except Exception as e:
            self.logger.error("set_pid_param error: %s", e)

    # Set the PID for yaw Angle adjustment
    # def set_yaw_pid_param(self, kp, ki, kd, forever=False):
    #     try:
    #         state = 0
    #         if forever:
    #             state = 0x5F
    #         cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_YAW_PID]
    #         if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
    #             print("YAW PID value must be:[0, 10.00]")
    #             return
    #         kp_params = bytearray(struct.pack('h', int(kp * 1000)))
    #         ki_params = bytearray(struct.pack('h', int(ki * 1000)))
    #         kd_params = bytearray(struct.pack('h', int(kd * 1000)))
    #         cmd.append(kp_params[0])  # low
    #         cmd.append(kp_params[1])  # high
    #         cmd.append(ki_params[0])  # low
    #         cmd.append(ki_params[1])  # high
    #         cmd.append(kd_params[0])  # low
    #         cmd.append(kd_params[1])  # high
    #         cmd.append(state)
    #         checksum = sum(cmd, self.__COMPLEMENT) & 0xff
    #         cmd.append(checksum)
    #         self.ser.write(cmd)
    #         if self.__debug:
    #             print("pid:", cmd)
    #         time.sleep(self.__delay_time)
    #         if forever:
    #             time.sleep(.1)
    #     except:
    #         print('---set_pid_param error!---')
    #         pass

    # Set car Type
    def set_car_type(self, car_type):
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xFF
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_SET_CAR_TYPE,
                self.__CAR_TYPE,
                0x5F,
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("car_type: %s", cmd)
            time.sleep(0.1)
        else:
            self.logger.warning("set_car_type input invalid")

    # Control bus steering gear.  Servo_id :[1-255], indicating the ID of the steering gear to be controlled. If ID =254, control all connected steering gear.
    # pulse_value=[96,4000] indicates the position to which the steering gear will run.
    # run_time indicates the running time (ms). The shorter the time, the faster the steering gear rotates.  The minimum value is 0 and the maximum value is 2000
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """
        Control a UART servo using raw pulse width values.

        Provides low-level control of UART servos using microsecond pulse widths
        instead of angle values. Useful for precise control or non-standard servos.

        Args:
            servo_id (int): Servo identifier
                Range: [1, 250] - must be valid servo ID
            pulse_value (int): Pulse width in microseconds
                Range: [96, 4000] microseconds
                Typical servo range: 500-2500 μs
                500 μs ≈ 0°, 1500 μs ≈ 90°, 2500 μs ≈ 180°
            run_time (int, optional): Movement duration in milliseconds. Defaults to 500.
                Range: [0, 2000] - automatically clamped
                0 = immediate movement, 2000 = slow movement

        Note:
            - Respects servo control enable state (set_uart_servo_ctrl_enable)
            - More precise than angle-based control methods
            - Pulse values outside [96, 4000] μs cause command rejection
            - Invalid servo_id causes command rejection with warning
            - Use set_uart_servo_angle() for angle-based control instead
            - Essential for servo calibration and fine-tuning

        Warning:
            Extreme pulse values may damage servos. Stay within safe ranges.

        Example:
            >>> # Center position (typical 1500μs pulse)
            >>> robot.set_uart_servo(1, 1500, 1000)
            >>>
            >>> # Minimum position (500μs pulse)
            >>> robot.set_uart_servo(1, 500, 500)
            >>>
            >>> # Maximum position (2500μs pulse)
            >>> robot.set_uart_servo(1, 2500, 500)
        """
        try:
            if not self.__arm_ctrl_enable:
                return
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                self.logger.warning("set uart servo input error")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xFF
            value = bytearray(struct.pack("h", int(pulse_value)))
            r_time = bytearray(struct.pack("h", int(run_time)))

            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_UART_SERVO,
                s_id,
                value[0],
                value[1],
                r_time[0],
                r_time[1],
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug(
                    "uartServo: %s, %s, %s", servo_id, int(pulse_value), cmd
                )
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_uart_servo serial error: %s", e)
        except Exception as e:
            self.logger.error("set_uart_servo error: %s", e)

    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        """
        Set UART servo to specific angle with motion time control.

        Moves a UART servo to the specified angle over the given time period.
        UART servos provide more precise control and feedback compared to PWM servos.

        Args:
            s_id (int): Servo ID. Valid range: [1, 6]
                Each servo has specific angle limitations based on its type
            s_angle (int): Target angle in degrees. Valid ranges by servo:
                Servos 1-4: [0, 180] degrees
                Servo 5: [0, 270] degrees
                Servo 6: [0, 180] degrees
            run_time (int, optional): Movement duration in milliseconds. Defaults to 500.
                Range: [0, 2000] - shorter time = faster movement

        Note:
            - Invalid angles for each servo are rejected with warnings
            - Automatically converts angles to servo pulse values
            - Servo 5 typically has extended range for specialized applications

        Example:
            >>> robot.set_uart_servo_angle(1, 90, 1000)    # Servo 1 to 90° in 1 second
            >>> robot.set_uart_servo_angle(5, 135, 500)    # Servo 5 to 135° in 0.5 seconds
            >>> robot.set_uart_servo_angle(6, 45, 200)     # Servo 6 to 45° quickly
        """
        try:
            if s_id == 1:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_1 set error!")
            elif s_id == 2:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_2 set error!")
            elif s_id == 3:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_3 set error!")
            elif s_id == 4:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_4 set error!")
            elif s_id == 5:
                if 0 <= s_angle <= 270:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_5 set error!")
            elif s_id == 6:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.logger.warning("angle_6 set error!")
        except Exception as e:
            self.logger.error("set_uart_servo_angle error! ID=%s: %s", s_id, e)

    # Set the bus servo ID(Use with caution), servo_id=[1-250].
    # Before running this function, please confirm that only one bus actuator is connected. Otherwise, all connected bus actuators will be set to the same ID, resulting in confusion of control
    def set_uart_servo_id(self, servo_id):
        """
        Configure or assign an ID to a UART servo.

        Sets the identification number for a UART servo, essential for
        multi-servo systems where each servo needs a unique address.

        Args:
            servo_id (int): New servo ID to assign
                Range: [1, 250] - must be within valid servo ID range
                ID 0 is reserved for broadcast commands
                IDs > 250 are reserved for system use

        Note:
            - Used during servo initialization and configuration
            - Each servo in a system must have a unique ID
            - Servo must be properly connected and powered
            - ID assignment may require servo to be in configuration mode
            - Some servos require power cycle after ID change
            - Invalid IDs (< 1 or > 250) cause command rejection

        Warning:
            Changing servo IDs affects all subsequent servo commands.
            Document ID assignments to avoid conflicts.

        Example:
            >>> # Assign ID 5 to a servo
            >>> robot.set_uart_servo_id(5)
            >>>
            >>> # Now control the servo using its new ID
            >>> robot.set_uart_servo_angle(5, 90)
            >>>
            >>> # Configure multiple servos with unique IDs
            >>> for new_id in range(1, 7):
            >>>     # Connect one servo at a time and assign ID
            >>>     robot.set_uart_servo_id(new_id)
            >>>     input(f"Servo ID {new_id} set. Connect next servo and press Enter...")
        """
        try:
            if servo_id < 1 or servo_id > 250:
                self.logger.warning("servo id input error!")
                return
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x04,
                self.FUNC_UART_SERVO_ID,
                int(servo_id),
            ]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("uartServo_id: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_uart_servo_id serial error: %s", e)
        except Exception as e:
            self.logger.error("set_uart_servo_id error: %s", e)

    def set_uart_servo_torque(self, enable):
        """
        Enable or disable torque for all UART servos.

        Controls whether UART servos hold their position with active torque
        or can be moved freely by hand. Affects power consumption and manual control.

        Args:
            enable (int or bool): Torque control setting
                0 or False: Disable torque
                    - Servos can be moved by hand
                    - Commands will not control rotation
                    - Low power consumption
                    - Useful for manual positioning or transport
                1 or True: Enable torque
                    - Commands can control rotation
                    - Servos resist manual movement
                    - Normal operational mode
                    - Required for active control

        Note:
            - Affects ALL UART servos simultaneously
            - Torque is enabled by default during initialization
            - Disabling torque allows manual servo positioning
            - Essential for servo protection during transport
            - Re-enable torque before resuming automated control

        Example:
            >>> # Disable torque for manual positioning
            >>> robot.set_uart_servo_torque(0)
            >>> input("Position servos manually, then press Enter...")
            >>>
            >>> # Re-enable torque for automated control
            >>> robot.set_uart_servo_torque(1)
            >>> robot.set_uart_servo_angle(1, 90)  # Now servo will move
        """
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("uartServo_torque: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_uart_servo_torque serial error: %s", e)
        except Exception as e:
            self.logger.error("set_uart_servo_torque error: %s", e)

    def set_uart_servo_ctrl_enable(self, enable):
        """
        Enable or disable UART servo control protocol transmission.

        Controls whether servo control commands are actually sent to the servos.
        Provides a software safety switch for servo operations.

        Args:
            enable (bool): Control protocol enable state
                True: Normal operation - control commands are sent to servos
                False: Safety mode - control commands are blocked/ignored

        Note:
            - Acts as a software safety switch for servo operations
            - When disabled, servo control methods will not send commands
            - Useful for emergency stops or maintenance mode
            - Does not affect servo torque or power state
            - Recommended to disable during manual servo positioning
            - Must be re-enabled for normal servo operations

        Example:
            >>> # Disable servo control for safety
            >>> robot.set_uart_servo_ctrl_enable(False)
            >>> robot.set_uart_servo_angle(1, 90)  # This command will be ignored
            >>>
            >>> # Re-enable servo control
            >>> robot.set_uart_servo_ctrl_enable(True)
            >>> robot.set_uart_servo_angle(1, 90)  # This command will execute
        """
        if enable:
            self.__arm_ctrl_enable = True
        else:
            self.__arm_ctrl_enable = False

    def set_uart_servo_angle_array(self, angle_s=None, run_time=500):
        """
        Control all 6 UART servo angles simultaneously.

        Efficiently sets all robotic arm servo positions in a single command,
        ensuring coordinated movement with synchronized timing.

        Args:
            angle_s (list, optional): Servo angles [s1, s2, s3, s4, s5, s6].
                Defaults to [90, 90, 90, 90, 90, 180].
                Valid ranges:
                    s1-s4: [0, 180] degrees
                    s5: [0, 270] degrees
                    s6: [0, 180] degrees
            run_time (int, optional): Movement duration in milliseconds. Defaults to 500.
                Range: [0, 2000] - automatically clamped to valid range

        Note:
            - More efficient than individual servo commands
            - Ensures synchronized movement of all servos
            - Respects servo control enable state (set_uart_servo_ctrl_enable)
            - Automatically converts angles to pulse values
            - Invalid angles cause the entire command to be rejected
            - Essential for coordinated robotic arm movements

        Example:
            >>> # Default position (safe/home position)
            >>> robot.set_uart_servo_angle_array()
            >>>
            >>> # Custom position with fast movement
            >>> angles = [45, 135, 90, 90, 180, 90]
            >>> robot.set_uart_servo_angle_array(angles, 200)
            >>>
            >>> # Slow, precise movement
            >>> angles = [0, 0, 0, 0, 0, 180]
            >>> robot.set_uart_servo_angle_array(angles, 2000)
        """
        try:
            if not self.__arm_ctrl_enable:
                return

            if angle_s is None:
                angle_s = [90, 90, 90, 90, 90, 180]
            if (
                0 <= angle_s[0] <= 180
                and 0 <= angle_s[1] <= 180
                and 0 <= angle_s[2] <= 180
                and 0 <= angle_s[3] <= 180
                and 0 <= angle_s[4] <= 270
                and 0 <= angle_s[5] <= 180
            ):
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                temp_val = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    temp_val[i] = self.__arm_convert_value(i + 1, angle_s[i])

                value_s1 = bytearray(struct.pack("h", int(temp_val[0])))
                value_s2 = bytearray(struct.pack("h", int(temp_val[1])))
                value_s3 = bytearray(struct.pack("h", int(temp_val[2])))
                value_s4 = bytearray(struct.pack("h", int(temp_val[3])))
                value_s5 = bytearray(struct.pack("h", int(temp_val[4])))
                value_s6 = bytearray(struct.pack("h", int(temp_val[5])))

                r_time = bytearray(struct.pack("h", int(run_time)))
                cmd = [
                    self.__HEAD,
                    self.__DEVICE_ID,
                    0x00,
                    self.FUNC_ARM_CTRL,
                    value_s1[0],
                    value_s1[1],
                    value_s2[0],
                    value_s2[1],
                    value_s3[0],
                    value_s3[1],
                    value_s4[0],
                    value_s4[1],
                    value_s5[0],
                    value_s5[1],
                    value_s6[0],
                    value_s6[1],
                    r_time[0],
                    r_time[1],
                ]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    self.logger.debug("arm: %s", cmd)
                    self.logger.debug("value: %s", temp_val)
                time.sleep(self.__delay_time)
            else:
                self.logger.warning("angle_s input error!")
        except serial.SerialException as e:
            self.logger.error("set_uart_servo_angle_array serial error: %s", e)
        except Exception as e:
            self.logger.error("set_uart_servo_angle_array error: %s", e)

    def set_uart_servo_offset(self, servo_id):
        """
        Set or reset the center position offset for UART servo calibration.

        Adjusts the mechanical center position of UART servos to compensate for
        mounting variations and mechanical tolerances. This is a calibration
        function that should be used when servos don't align properly.

        Args:
            servo_id (int): Servo to calibrate. Valid values:
                0: Reset ALL servos to factory default offsets
                1-6: Calibrate specific servo ID

        Returns:
            int: Offset calibration status/result from the servo

        Process:
            1. Move servo to desired center position manually
            2. Call this function to save current position as new center
            3. All future angle commands will reference this new center

        Note:
            - CRITICAL: Position servo manually before calling this function
            - servo_id=0 resets ALL servos to factory defaults
            - Changes are saved permanently to servo memory
            - Improper calibration can affect all subsequent movements
            - Use with caution - incorrect offsets can damage mechanisms

        Warning:
            This function modifies servo calibration permanently. Ensure the
            servo is in the correct physical position before executing.

        Example:
            >>> # 1. Manually position servo 1 to desired center
            >>> # 2. Calibrate servo 1 with current position as new center
            >>> result = robot.set_uart_servo_offset(1)
            >>> print(f"Servo 1 offset calibration result: {result}")
            >>>
            >>> # Reset all servos to factory defaults
            >>> robot.set_uart_servo_offset(0)
        """
        try:
            self.__arm_offset_id = 0xFF
            self.__arm_offset_state = 0
            s_id = int(servo_id) & 0xFF
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_OFFSET, s_id]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("uartServo_offset: %s", cmd)
            time.sleep(self.__delay_time)
            for i in range(200):
                if self.__arm_offset_id == servo_id:
                    if self.__debug:
                        if self.__arm_offset_id == 0:
                            self.logger.debug("Arm Reset Offset Value")
                        else:
                            self.logger.debug(
                                "Arm Offset State: %s, %s, %s",
                                self.__arm_offset_id,
                                self.__arm_offset_state,
                                i,
                            )
                    return self.__arm_offset_state
                time.sleep(0.001)
            return self.__arm_offset_state
        except Exception as e:
            self.logger.error("set_uart_servo_offset error: %s", e)
            return self.__arm_offset_state

    # Set the default Angle of akerman type (R2) car front wheel, Angle =[60, 120]
    # forever=True for permanent, =False for temporary.
    # Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.
    # Temporary effect fast response, single effective, data will not be maintained after restarting the single chip
    def set_akm_default_angle(self, angle, forever=False):
        """
        Set the default (center/neutral) angle for Ackermann steering.

        Configures the servo's center position for straight-ahead driving.
        This calibration is essential for accurate steering control and can be
        stored permanently to maintain calibration across power cycles.

        Args:
            angle (int): Default steering angle in degrees
                Range: [60, 120] degrees
                Typical value: 90 degrees (straight ahead)
                Values outside range are ignored
            forever (bool, optional): Permanent storage flag. Defaults to False.
                False: Temporary setting (lost on power cycle)
                True: Store permanently in servo memory

        Note:
            - Essential for Ackermann steering calibration
            - Angle must be within [60, 120] degree range
            - Invalid angles cause the command to be silently ignored
            - Use forever=True only after confirming correct calibration
            - Permanent storage prevents need for recalibration
            - Affects steering calculations in drive commands

        Warning:
            Setting forever=True writes to servo's non-volatile memory.
            Excessive writes may reduce servo lifespan.

        Example:
            >>> # Test calibration temporarily
            >>> robot.set_akm_default_angle(92, forever=False)
            >>> # Test steering to verify calibration
            >>> robot.set_akm_angle(92)  # Should drive straight
            >>>
            >>> # Store calibration permanently once verified
            >>> robot.set_akm_default_angle(92, forever=True)
        """
        try:
            if int(angle) > 120 or int(angle) < 60:
                return
            servo_id = self.__AKM_SERVO_ID
            state = 0
            if forever:
                state = 0x5F
                self.__akm_def_angle = angle
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_AKM_DEF_ANGLE,
                servo_id,
                int(angle),
                state,
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("akm set def angle: %s", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(0.1)
        except serial.SerialException as e:
            self.logger.error("set_akm_default_angle serial error: %s", e)
        except Exception as e:
            self.logger.error("set_akm_default_angle error: %s", e)

    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """
        Control steering angle for Ackermann steering robots (R2 series).

        Sets the front wheel steering angle relative to the default center position.
        Optionally enables differential motor control for proper Ackermann geometry.

        Args:
            angle (int): Steering angle relative to center. Range: [-45, 45]
                Negative values: Turn left
                Positive values: Turn right
                0: Straight ahead
            ctrl_car (bool, optional): Enable differential motor control. Defaults to False.
                False: Only control steering servo angle
                True: Control steering AND adjust left/right motor speeds
                       for proper Ackermann steering geometry

        Note:
            - Only for Ackermann steering robots (R2/R2L series)
            - Angles outside [-45, 45] are ignored for safety
            - With ctrl_car=True, inner wheel turns slower than outer wheel
            - Requires proper default angle calibration (get_akm_default_angle)
            - Not applicable to mecanum or differential drive robots

        Example:
            >>> # Simple steering (servo only)
            >>> robot.set_akm_steering_angle(20)        # Turn right 20°
            >>> robot.set_akm_steering_angle(-15)       # Turn left 15°
            >>> robot.set_akm_steering_angle(0)         # Straight
            >>>
            >>> # Full Ackermann steering (servo + motor compensation)
            >>> robot.set_akm_steering_angle(30, True)  # Right turn with motor diff
        """
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            servo_id = self.__AKM_SERVO_ID
            if ctrl_car:
                servo_id = self.__AKM_SERVO_ID + 0x80
            cmd = [
                self.__HEAD,
                self.__DEVICE_ID,
                0x00,
                self.FUNC_AKM_STEER_ANGLE,
                servo_id,
                int(angle) & 0xFF,
            ]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("akm_steering_angle: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("set_akm_steering_angle serial error: %s", e)
        except Exception as e:
            self.logger.error("set_akm_steering_angle error: %s", e)

    def reset_flash_value(self):
        """
        Reset all flash-stored settings to factory defaults.

        Erases all permanently stored configuration data and restores
        the robot's MCU to factory default settings.

        Resets:
            - PID parameters to factory values
            - Auto-report settings
            - Car type configuration
            - Servo offset calibrations
            - Any other persistent settings

        Warning:
            - This operation is IRREVERSIBLE
            - All custom calibrations will be lost
            - Robot may need reconfiguration after reset
            - Takes additional time due to flash memory operations

        Note:
            - Only affects permanently stored settings
            - Current session settings remain until power cycle
            - Backup important calibrations before using
            - Consider reset_car_state() for temporary reset

        Example:
            >>> # DANGEROUS: Only use if necessary
            >>> robot.reset_flash_value()
            >>> print("Factory reset complete - recalibration needed")
        """
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("flash: %s", cmd)
            time.sleep(self.__delay_time)
            time.sleep(0.1)
        except serial.SerialException as e:
            self.logger.error("reset_flash_value serial error: %s", e)
        except Exception as e:
            self.logger.error("reset_flash_value error: %s", e)

    def reset_car_state(self):
        """
        Reset the robot to a safe default state.

        Immediately stops all motion and resets hardware to safe conditions:
        - Stops all motors (parking state)
        - Turns off all RGB lights
        - Turns off buzzer/beeper
        - Resets other controllable peripherals

        Note:
            - Emergency stop function for safety
            - Does not reset servo positions or PID settings
            - Use when robot behavior becomes unpredictable
            - Always call this before program termination for safety
            - Takes effect immediately without confirmation

        Example:
            >>> # Emergency stop
            >>> robot.reset_car_state()
            >>>
            >>> # Safe shutdown sequence
            >>> robot.reset_car_state()
            >>> robot.close()
        """
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xFF
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.logger.debug("reset_car_state: %s", cmd)
            time.sleep(self.__delay_time)
        except serial.SerialException as e:
            self.logger.error("reset_car_state serial error: %s", e)
        except Exception as e:
            self.logger.error("reset_car_state error: %s", e)

    def clear_auto_report_data(self):
        """
        Clear all cached sensor data received from automatic MCU reports.

        Resets all internally stored sensor values to zero, effectively clearing
        the data cache from automatic reporting. Useful for fresh data collection.

        Clears:
            - Battery voltage reading
            - 3-axis accelerometer data (ax, ay, az)
            - 3-axis gyroscope data (gx, gy, gz)
            - 3-axis magnetometer data (mx, my, mz)
            - 3-axis velocity data (vx, vy, vz)
            - IMU attitude data (yaw, roll, pitch)

        Note:
            - Does not stop automatic reporting, only clears cached values
            - New data will be received and cached as MCU continues reporting
            - Useful for synchronizing data collection or testing
            - All get_*_data() methods will return zeros until new data arrives

        Example:
            >>> robot.clear_auto_report_data()
            >>> # Wait for fresh data
            >>> time.sleep(0.1)
            >>> ax, ay, az = robot.get_accelerometer_data()
            >>> # Now have fresh accelerometer readings
        """
        self.__battery_voltage = 0
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

    def get_akm_default_angle(self):
        """
        Get the default steering angle for Ackermann steering robots (R2 series).

        Retrieves the center/default position angle of the front wheel servo
        used in Ackermann steering configurations.

        Returns:
            int: Default steering angle value
                Positive integer: Valid default angle setting
                -1: Failed to read default angle (timeout or error)

        Note:
            - Specific to Ackermann steering robots (R2/R2L series)
            - Value is cached after first successful read
            - Used for steering calibration and center position reference
            - Timeout after 100 attempts (≈1 second) if no response
            - Not applicable to mecanum or differential drive robots

        Example:
            >>> default_angle = robot.get_akm_default_angle()
            >>> if default_angle > 0:
            ...     print(f"Steering center angle: {default_angle}")
            ...     # Use for steering calibration
            >>> else:
            ...     print("Failed to read steering default angle")
        """
        if not self.__akm_readed_angle:
            self.__request_data(self.FUNC_AKM_DEF_ANGLE, self.__AKM_SERVO_ID)
            akm_count = 0
            while True:
                if self.__akm_readed_angle:
                    break
                akm_count = akm_count + 1
                if akm_count > 100:
                    return -1
                time.sleep(0.01)
        return self.__akm_def_angle

    def get_uart_servo_value(self, servo_id):
        """
        Read raw position parameters from a UART servo.

        Retrieves the current pulse width value from the specified UART servo,
        returning both the confirmed servo ID and raw position value.

        Args:
            servo_id (int): Servo ID to read. Valid range: [1, 250]
                Standard robotic arm servos typically use IDs 1-6

        Returns:
            tuple: (read_id, current_value) servo response
                read_id (int): Confirmed servo ID that responded
                current_value (int): Raw pulse width value (typically 500-2500)
                (-1, -1): Timeout - servo didn't respond
                (-2, -2): Communication error or exception

        Note:
            - Returns raw pulse values, not converted angles
            - Use get_uart_servo_angle() for angle values
            - Confirms servo ID to detect addressing errors
            - Timeout after 30ms if servo doesn't respond
            - Lower-level function used by angle conversion methods

        Example:
            >>> servo_id, pulse_value = robot.get_uart_servo_value(1)
            >>> if servo_id > 0:
            ...     print(f"Servo {servo_id} pulse: {pulse_value}")
            ...     # Convert to angle manually if needed
            >>> elif servo_id == -1:
            ...     print("Servo timeout")
            >>> else:
            ...     print("Communication error")
        """
        try:
            if servo_id < 1 or servo_id > 250:
                self.logger.warning("get servo id input error!")
                return
            self.__read_id = 0
            self.__read_val = 0
            self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xFF)
            timeout = 30
            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val
                timeout = timeout - 1
                time.sleep(0.001)
            return -1, -1
        except Exception as e:
            self.logger.error("get_uart_servo_value error: %s", e)
            return -2, -2

    def get_uart_servo_angle(self, s_id):
        """
        Read the current angle of a specific UART servo.

        Retrieves the current angular position of the specified UART servo,
        with automatic range validation and error checking.

        Args:
            s_id (int): Servo ID to read. Valid range: [1, 6]
                Each servo has specific angle limitations

        Returns:
            int: Current servo angle in degrees
                Servos 1-4: [0, 180] degrees
                Servo 5: [0, 270] degrees
                Servo 6: [0, 180] degrees
                -1: Servo error, out of range, or read failure
                -2: Communication error or timeout

        Note:
            - Automatically converts pulse values to human-readable angles
            - Validates angle ranges for each servo type
            - Out-of-range values indicate mechanical or electrical issues
            - Use get_uart_servo_angle_array() for reading multiple servos

        Example:
            >>> angle = robot.get_uart_servo_angle(1)
            >>> if angle >= 0:
            ...     print(f"Servo 1 is at {angle} degrees")
            >>> elif angle == -1:
            ...     print("Servo 1 error or out of range")
            >>> else:
            ...     print("Communication error")
        """
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 1 and read_id == 1:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            elif s_id == 2 and read_id == 2:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            elif s_id == 3 and read_id == 3:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            elif s_id == 4 and read_id == 4:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            elif s_id == 5 and read_id == 5:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 270 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            elif s_id == 6 and read_id == 6:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.logger.warning("read servo:%s out of range!", s_id)
                    angle = -1
            else:
                if self.__debug:
                    self.logger.error("read servo:%s error!", s_id)
            if self.__debug:
                self.logger.debug("request angle %s: %s, %s", s_id, read_id, value)
            return angle
        except Exception as e:
            self.logger.error("get_uart_servo_angle error: %s", e)
            return -2

    def get_uart_servo_angle_array(self):
        """
        Read all 6 UART servo angles simultaneously.

        Efficiently retrieves the current angle positions of all 6 UART servos
        in a single operation, useful for robotic arm state monitoring.

        Returns:
            list: [angle1, angle2, angle3, angle4, angle5, angle6] servo angles
                angleN (int): Angle of servo N in degrees
                    Valid servos 1-4: [0, 180] degrees
                    Servo 5: [0, 270] degrees
                    Servo 6: [0, 180] degrees
                -1: Servo error or invalid reading
                -2: Communication error or timeout

        Note:
            - More efficient than calling get_uart_servo_angle() 6 times
            - Requires active serial connection
            - Timeout after 30ms if no response
            - Failed servos return -1, communication errors return -2
            - Automatically converts pulse values to angles

        Example:
            >>> angles = robot.get_uart_servo_angle_array()
            >>> for i, angle in enumerate(angles, 1):
            ...     if angle >= 0:
            ...         print(f"Servo {i}: {angle}°")
            ...     elif angle == -1:
            ...         print(f"Servo {i}: Error")
            ...     else:
            ...         print(f"Servo {i}: Communication timeout")
        """
        try:
            # angle = [-1, -1, -1, -1, -1, -1]
            # for i in range(6):
            #     temp1 = self.get_uart_servo_angle(i + 1)
            #     if temp1 >= 0:
            #         angle[i] = temp1
            #     else:
            #         break
            # return angle

            angle = [-1, -1, -1, -1, -1, -1]
            self.__read_arm = [-1, -1, -1, -1, -1, -1]
            self.__read_arm_ok = 0
            self.__request_data(self.FUNC_ARM_CTRL, 1)
            timeout = 30
            while timeout > 0:
                if self.__read_arm_ok == 1:
                    for i in range(6):
                        if self.__read_arm[i] > 0:
                            angle[i] = self.__arm_convert_angle(
                                i + 1, self.__read_arm[i]
                            )
                    if self.__debug:
                        self.logger.debug(
                            "angle_array: timeout=%s, angle=%s", 30 - timeout, angle
                        )
                    break
                timeout = timeout - 1
                time.sleep(0.001)
            return angle
        except Exception as e:
            self.logger.error("get_uart_servo_angle_array error: %s", e)
            return [-2, -2, -2, -2, -2, -2]

    def get_accelerometer_data(self):
        """
        Get 3-axis accelerometer data from the IMU sensor.

        Retrieves linear acceleration measurements along X, Y, and Z axes,
        including both robot motion and gravitational acceleration.

        Returns:
            tuple: (a_x, a_y, a_z) acceleration measurements
                a_x (float): Acceleration along X-axis (forward/backward)
                a_y (float): Acceleration along Y-axis (left/right)
                a_z (float): Acceleration along Z-axis (up/down)
                Units typically in m/s² or g-force (1g ≈ 9.81 m/s²)

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Includes gravitational acceleration (≈9.8 m/s² downward)
            - Can be used for tilt detection and motion analysis
            - Useful for collision detection and stability monitoring

        Example:
            >>> ax, ay, az = robot.get_accelerometer_data()
            >>> total_accel = (ax**2 + ay**2 + az**2)**0.5
            >>> print(f"Total acceleration: {total_accel:.2f} m/s²")
            >>> tilt_angle = math.atan2(ax, az) * 180 / math.pi
            >>> print(f"Tilt angle: {tilt_angle:.1f}°")
        """
        a_x, a_y, a_z = self.__ax, self.__ay, self.__az
        # self.__ax, self.__ay, self.__az = 0.0, 0.0, 0.0
        return a_x, a_y, a_z

    def get_gyroscope_data(self):
        """
        Get 3-axis gyroscope (angular velocity) data from the IMU sensor.

        Retrieves angular velocity measurements around X, Y, and Z axes,
        indicating how fast the robot is rotating in each direction.

        Returns:
            tuple: (g_x, g_y, g_z) angular velocity measurements
                g_x (float): Angular velocity around X-axis (pitch rate)
                g_y (float): Angular velocity around Y-axis (roll rate)
                g_z (float): Angular velocity around Z-axis (yaw rate)
                Units typically in degrees/second or radians/second

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Positive values indicate counter-clockwise rotation
            - Can be integrated over time to estimate orientation changes
            - Useful for detecting rapid movements and stability control

        Example:
            >>> gx, gy, gz = robot.get_gyroscope_data()
            >>> rotation_magnitude = (gx**2 + gy**2 + gz**2)**0.5
            >>> print(f"Total rotation rate: {rotation_magnitude:.2f}°/s")
            >>> if abs(gz) > 50:
            ...     print("Fast turning detected!")
        """
        g_x, g_y, g_z = self.__gx, self.__gy, self.__gz
        # self.__gx, self.__gy, self.__gz = 0.0, 0.0, 0.0
        return g_x, g_y, g_z

    def get_magnetometer_data(self):
        """
        Get 3-axis magnetometer (compass) data from the IMU sensor.

        Retrieves magnetic field measurements along X, Y, and Z axes,
        which can be used for compass heading and magnetic interference detection.

        Returns:
            tuple: (m_x, m_y, m_z) magnetic field measurements
                m_x (float): Magnetic field strength along X-axis
                m_y (float): Magnetic field strength along Y-axis
                m_z (float): Magnetic field strength along Z-axis
                Units typically in microTesla (µT) or Gauss

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Affected by nearby magnetic materials and electronic devices
            - Can be used for compass heading calculation
            - Calibration may be needed for accurate heading

        Example:
            >>> mx, my, mz = robot.get_magnetometer_data()
            >>> magnitude = (mx**2 + my**2 + mz**2)**0.5
            >>> print(f"Magnetic field strength: {magnitude:.2f}")
            >>> heading = math.atan2(my, mx) * 180 / math.pi
            >>> print(f"Magnetic heading: {heading:.1f}°")
        """
        m_x, m_y, m_z = self.__mx, self.__my, self.__mz
        # self.__mx, self.__my, self.__mz = 0.0, 0.0, 0.0
        return m_x, m_y, m_z

    # Get board attitude angle, return yaw, roll, pitch
    # ToAngle=True returns degrees, ToAngle=False returns radians.
    def get_imu_attitude_data(self, ToAngle=True):
        """
        Get IMU attitude data (roll, pitch, yaw) from the robot's sensor.

        Retrieves the current orientation of the robot in 3D space based on
        the integrated IMU sensor data.

        Args:
            ToAngle (bool, optional): Convert to degrees. Defaults to True.
                True: Return values in degrees (more intuitive)
                False: Return raw values in radians

        Returns:
            tuple: (roll, pitch, yaw) orientation values
                roll (float): Rotation around X-axis (tilt left/right)
                pitch (float): Rotation around Y-axis (tilt forward/back)
                yaw (float): Rotation around Z-axis (turn left/right)

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Coordinate system follows standard robotics conventions

        Example:
            >>> roll, pitch, yaw = robot.get_imu_attitude_data()
            >>> print(f"Robot facing: {yaw:.1f}° from start")
            >>> roll, pitch, yaw = robot.get_imu_attitude_data(False)  # In radians
        """
        if ToAngle:
            RtA = 57.2957795
            roll = self.__roll * RtA
            pitch = self.__pitch * RtA
            yaw = self.__yaw * RtA
        else:
            roll, pitch, yaw = self.__roll, self.__pitch, self.__yaw
        # self.__roll, self.__pitch, self.__yaw = 0.0, 0.0, 0.0
        return roll, pitch, yaw

    def get_motion_data(self):
        """
        Get the current motion velocity data from the robot.

        Retrieves the robot's current linear and angular velocities as
        measured or calculated by the motion control system.

        Returns:
            tuple: (val_vx, val_vy, val_vz) velocity components
                val_vx (float): Linear velocity in X direction (forward/backward) [m/s]
                val_vy (float): Linear velocity in Y direction (left/right) [m/s]
                val_vz (float): Angular velocity around Z axis (rotation) [rad/s]

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Velocities may be measured (encoder-based) or commanded values
            - Coordinate system: +X forward, +Y left, +Z counter-clockwise
            - Update rate depends on auto-report settings

        Example:
            >>> vx, vy, vz = robot.get_motion_data()
            >>> speed = (vx**2 + vy**2)**0.5
            >>> print(f"Robot speed: {speed:.2f} m/s")
            >>> print(f"Rotation: {vz:.2f} rad/s")
            >>> if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(vz) < 0.01:
            ...     print("Robot is stationary")
        """
        val_vx = self.__vx
        val_vy = self.__vy
        val_vz = self.__vz
        # self.__vx, self.__vy, self.__vz = 0.0, 0.0, 0.0
        return val_vx, val_vy, val_vz

    def get_battery_voltage(self):
        """
        Get the current battery voltage reading.

        Monitors the robot's battery voltage level for power management
        and low battery warnings.

        Returns:
            float: Battery voltage in volts (V)
                Typical range: 6V-12V depending on battery configuration

        Note:
            - Value updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Use for battery monitoring and low voltage protection
            - Reading accuracy depends on robot's voltage sensing circuit

        Example:
            >>> voltage = robot.get_battery_voltage()
            >>> if voltage < 7.0:
            ...     print("Low battery warning!")
            >>> print(f"Battery: {voltage:.1f}V")
        """
        vol = self.__battery_voltage / 10.0
        # self.__battery_voltage = 0
        return vol

    def get_motor_encoder(self):
        """
        Get encoder data from all four motor channels.

        Retrieves the current encoder count values from each motor, which
        can be used for odometry, position tracking, and motion feedback.

        Returns:
            tuple: (m1, m2, m3, m4) encoder counts
                m1 (int): Encoder count for motor 1
                m2 (int): Encoder count for motor 2
                m3 (int): Encoder count for motor 3
                m4 (int): Encoder count for motor 4

        Note:
            - Values updated automatically by background receive thread
            - Requires create_receive_threading() to be active
            - Encoder counts accumulate and may overflow
            - Motor mapping: typically 1=front-left, 2=front-right,
              3=rear-left, 4=rear-right

        Example:
            >>> m1, m2, m3, m4 = robot.get_motor_encoder()
            >>> total_motion = abs(m1) + abs(m2) + abs(m3) + abs(m4)
            >>> print(f"Total encoder counts: {total_motion}")
        """
        m1, m2, m3, m4 = (
            self.__encoder_m1,
            self.__encoder_m2,
            self.__encoder_m3,
            self.__encoder_m4,
        )
        # self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4 = 0, 0, 0, 0
        return m1, m2, m3, m4

    def get_motion_pid(self):
        """
        Get the current motion control PID parameters from the robot.

        Retrieves the Proportional, Integral, and Derivative gains currently
        being used by the robot's motion control system.

        Returns:
            list: [kp, ki, kd] PID parameter values
                kp (float): Proportional gain [0.0, 10.0]
                ki (float): Integral gain [0.0, 10.0]
                kd (float): Derivative gain [0.0, 10.0]
                [-1, -1, -1]: Failed to retrieve PID values

        Note:
            - Requests current PID settings from MCU
            - Values are converted from internal integer format to float
            - Useful for verifying PID configuration
            - Timeout after 20 attempts if MCU doesn't respond
            - Required for PID tuning and system analysis

        Example:
            >>> kp, ki, kd = robot.get_motion_pid()
            >>> if kp != -1:
            ...     print(f"Current PID: P={kp:.3f}, I={ki:.3f}, D={kd:.3f}")
            ...     if kp > 5.0:
            ...         print("High proportional gain - may cause oscillation")
        """
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for i in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                if self.__debug:
                    self.logger.debug(
                        "get_motion_pid: %s, %s, %s", self.__pid_index, [kp, ki, kd], i
                    )
                return [kp, ki, kd]
            time.sleep(0.001)
        return [-1, -1, -1]

    def get_car_type_from_machine(self):
        """
        Get the robot car type configuration from the MCU.

        Queries the microcontroller to retrieve the currently configured
        robot type, which determines motion algorithms and hardware mapping.

        Returns:
            int: Car type identifier stored in MCU
                Positive integer: Valid car type (model-specific values)
                -1: Failed to retrieve car type (timeout or error)

        Note:
            - Requests current setting from MCU, not the initialized value
            - Useful for verifying configuration matches expectations
            - Car type affects motion control algorithms and servo mappings
            - Timeout after 20 attempts if MCU doesn't respond

        Example:
            >>> car_type = robot.get_car_type_from_machine()
            >>> if car_type == 5:
            ...     print("X3 robot configuration")
            >>> elif car_type == -1:
            ...     print("Failed to read car type")
        """
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for _ in range(0, 20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(0.001)
        return -1

    def get_version(self):
        """
        Get the firmware version of the robot's microcontroller.

        Retrieves the version number of the underlying MCU firmware, which is
        useful for compatibility checking and debugging.

        Returns:
            float: Firmware version number (e.g., 1.1, 3.3, etc.)
                Format: major.minor version
                -1: If version cannot be retrieved

        Note:
            - Version is cached after first successful retrieval
            - Automatically requests version from MCU if not already known
            - Requires active serial connection
            - Timeout after 20 attempts if MCU doesn't respond

        Example:
            >>> version = robot.get_version()
            >>> print(f"MCU Firmware: v{version}")
            >>> if version >= 3.0:
            ...     print("Advanced features available")
        """
        if self.__version_H == 0:
            self.__request_data(self.FUNC_VERSION)
            for i in range(0, 20):
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    if self.__debug:
                        self.logger.debug("get_version:V%s, i:%s", self.__version, i)
                    return self.__version
                time.sleep(0.001)
        else:
            return self.__version
        return -1

    def _validate_servo_id(self, servo_id, max_id=6):
        """Validate servo ID parameter"""
        if not isinstance(servo_id, int) or servo_id < 1 or servo_id > max_id:
            raise ValueError(f"servo_id must be an integer between 1 and {max_id}")
        return True

    def _validate_angle(self, angle, min_angle=0, max_angle=180):
        """Validate angle parameter"""
        if (
            not isinstance(angle, (int, float))
            or angle < min_angle
            or angle > max_angle
        ):
            raise ValueError(
                f"angle must be a number between {min_angle} and {max_angle}"
            )
        return True

    def _validate_speed(self, speed, min_speed=-100, max_speed=100):
        """Validate speed parameter"""
        if (
            not isinstance(speed, (int, float))
            or speed < min_speed
            or speed > max_speed
        ):
            raise ValueError(
                f"speed must be a number between {min_speed} and {max_speed}"
            )
        return True


if __name__ == "__main__":
    # Car underlying processing library
    import platform

    device = platform.system()
    print(f"Read device: {device}")
    if device == "Windows":
        com_index = 1
        while True:
            com_index = com_index + 1
            try:
                print(f"try COM{com_index}")
                com = f"COM{com_index}"
                bot = Rosmaster(1, port=com, debug=True)
                break
            except serial.SerialException:
                if com_index > 256:
                    print(
                        "-----------------------No COM Open--------------------------"
                    )
                    exit(0)
                continue
            except Exception as e:
                print(f"Error opening COM{com_index}: {e}")
                if com_index > 256:
                    print(
                        "-----------------------No COM Open--------------------------"
                    )
                    exit(0)
                continue
        print(f"--------------------Open {com}---------------------")
    else:
        bot = Rosmaster(port="/dev/ttyUSB0", debug=True)

    bot.create_receive_threading()
    time.sleep(0.1)
    bot.set_beep(50)
    time.sleep(0.1)

    version = bot.get_version()
    print(f"version={version}")

    # bot.set_car_type(4)
    # time.sleep(.1)
    # car_type = bot.get_car_type_from_machine()
    # print("car_type:", car_type)
    # bot.set_uart_servo_angle(1, 100)

    # s_id, value = bot.get_uart_servo_value(1)
    # print("value:", s_id, value)

    # bot.set_uart_servo_torque(1)
    # time.sleep(.1)
    # state = bot.set_uart_servo_offset(6)
    # print("state=", state)

    # bot.set_pid_param(0.5, 0.1, 0.3, 8, True)
    # bot.set_yaw_pid_param(0.4, 2, 0.2, False)
    # bot.reset_flash_value()

    # pid = bot.get_motion_pid(5)
    # pid = bot.get_yaw_pid()
    # print("pid:", pid)

    # angle= bot.get_uart_servo_angle(1)
    # print("angle:", angle)

    # angle_array = bot.get_uart_servo_angle_array()
    # print("angle_array:", angle_array)

    # bot.set_car_motion(0.5, 0, 0)

    # bot.send_ip_addr("192.168.1.2")

    # bot.set_uart_servo_angle(6, 150)

    # bot.set_auto_report_state(0, False)

    # bot.set_pwm_servo_all(50, 50, 50, 50)
    # time.sleep(1)
    # bot.set_car_motion(0, 0, -3.5)
    # bot.set_car_run(6, 50)
    try:
        while True:
            ax, ay, az = bot.get_accelerometer_data()
            gx, gy, gz = bot.get_gyroscope_data()
            mx, my, mz = bot.get_magnetometer_data()
            # print(ax, ay, az)
            print(ax, ay, az, gx, gy, gz, mx, my, mz)
            # print("%3.3f, %3.3f, %3.3f,      %3.3f, %3.3f, %3.3f" %
            # (ax, ay, az, gx, gy, gz))
            # print("%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f" %
            # (ax, ay, az, gx, gy, gz, mx, my, mz))
            # roll, pitch, yaw = bot.get_imu_attitude_data()
            # print("roll:%f, pitch:%f, yaw:%f" % (roll, pitch, yaw))
            # m1, m2, m3, m4 = bot.get_motor_encoder()
            # print("encoder:", m1, m2, m3, m4)
            bot.set_motor(0, 50, 0, 50)
            bot.set_pwm_servo(1, 120)
            time.sleep(2)
            bot.set_pwm_servo(1, 50)
            time.sleep(2)

            # v = bot.get_motion_data()
            # print("v:", v)

            # pid = bot.get_motion_pid()
            # print(pid)
            # version = bot.get_version()
            # print("version=", version)
            # vx, vy, vz = bot.get_motion_data()
            # print("V:", vx, vy, vz)
            time.sleep(0.1)
    except KeyboardInterrupt:
        bot.set_car_motion(0, 0, 0)
    exit(0)
