# Yahboom Ackermann ROS2 Workspace

This workspace contains ROS2 packages and configuration for a Yahboom Ackermann steering robot, running on a Raspberry Pi 5 with a Yahboom Robot Expansion Board v3.0 and a Raspberry Pi AI Camera (IMX500). The project targets ROS2 Kilted Kaiju (or Rolling) and follows ROS2 best practices.

## Features

- Ackermann steering robot control
- Integration with Yahboom Robot Expansion Board v3.0
- Camera support for object detection, tracking, visual odometry, and SLAM
- Sensor fusion with the onboard IMU
- Navigation using the ROS2 Navigation2 stack

## Hardware

- **Computer:** Raspberry Pi 5
- **Robot Base:** Yahboom Robot Expansion Board v3.0
- **Camera:** Raspberry Pi AI Camera (IMX500)
- **IMU:** Built-in on Yahboom board

## Getting Started

### Prerequisites

- Raspberry Pi OS (64-bit recommended)
- ROS2 Kilted Kaiju (or Rolling)
- Colcon build system
- Python 3.10+
- [Yahboom ROS Driver Board Documentation](http://www.yahboom.net/study/ROS-Driver-Board)
- [ROS2 Kilted Kaiju Documentation](https://docs.ros.org/en/kilted/index.html)
- [Raspberry Pi AI Camera Documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)

### Workspace Setup


### Running the Robot


## Project Structure

- `scripts/` - Utility scripts

## Contributing

- Follow PEP8 for Python code.
- Use descriptive names and add docstrings to all public functions/classes.
- Prefer explicit error handling and avoid magic numbers.
- Add comments for complex logic.

## References

- [Yahboom ROS Driver Board Documentation](http://www.yahboom.net/study/ROS-Driver-Board)
- [ROS2 Kilted Kaiju Documentation](https://docs.ros.org/en/kilted/index.html)
- [Raspberry Pi AI Camera Documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)
