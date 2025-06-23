#!/bin/bash

###############################################################################
# setup-ros.sh
#
# This script automates the setup of a ROS 2 workspace for the robot project.
# It installs required dependencies, clones and updates necessary repositories,
# configures rosdep, and builds the workspace using colcon.
#
# Usage:
#   ./scripts/setup-ros.sh [--ros-distro <distro>] [--no-skip-build-finished]
#
# Options:
#   --ros-distro <distro>           Specify the ROS 2 distribution (default: kilted)
#   --no-skip-build-finished        Do not use --packages-skip-build-finished for colcon build
#
# The script is idempotent and can be run multiple times to update sources.
#
# Key Features:
#   - Installs system and Python dependencies
#   - Clones and updates all required ROS 2 and third-party repositories
#   - Handles missing binary packages by skipping them in rosdep
#   - Applies workaround for Fast-DDS build issues with GCC 12+
#   - Builds the workspace with colcon
#
# NOTE: This script is intended for Raspberry Pi OS (tested on Pi 5, Bookworm).
###############################################################################

# Exit on error and treat unset variables as errors
set -euo pipefail

# Default ROS distribution and build options
ROS_DISTRO="kilted"
SKIP_BUILD_FINISHED=1
EXTERNAL_SOURCE_DIR=""
RUN_GIT_PULL=0
DRYRUN=0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
    --ros-distro)
        if [[ -z "${2:-}" ]]; then
            echo "Error: --ros-distro requires a value."
            exit 1
        fi
        ROS_DISTRO="$2"
        shift 2
        ;;
    --no-skip-build-finished)
        SKIP_BUILD_FINISHED=0
        shift
        ;;
    --external-source-dir)
        if [[ -z "${2:-}" ]]; then
            echo "Error: --external-source-dir requires a value."
            exit 1
        fi
        EXTERNAL_SOURCE_DIR="$2"
        shift 2
        ;;
    --run-git-pull)
        RUN_GIT_PULL=1
        shift
        ;;
    --dryrun)
        DRYRUN=1
        shift
        ;;
    *)
        echo "Unknown option: $1"
        echo "Usage: $0 [--ros-distro <distro>] [--no-skip-build-finished] [--external-source-dir <dir>] [--run-git-pull] [--dryrun]"
        exit 1
        ;;
    esac

done

# Function to run or echo commands depending on DRYRUN
run_cmd() {
    if [ "$DRYRUN" -eq 1 ]; then
        echo "[DRYRUN] $*"
        return 0
    else
        eval "$@"
        return $?
    fi
}

# Set default if not provided
WORKSPACE_DIR=$(pwd)
if [[ -z "$EXTERNAL_SOURCE_DIR" ]]; then
    EXTERNAL_SOURCE_DIR="$WORKSPACE_DIR/ext_src"
fi

# Function to check if a command exists
check_command() {
    if ! command -v "$1" &>/dev/null; then
        echo "Error: $1 is not installed. Please install it and try again."
        exit 1
    fi
}

# Function to clone a repository if it doesn't already exist
clone_repo() {
    local repo_url="$1"
    local branch="$2"
    local target_dir="$3"

    if [ -d "$target_dir" ]; then
        echo "$target_dir already exists. Updating..."
        pushd "$target_dir"
        run_cmd git fetch --all
        # Warn if there are local changes
        if ! git diff-index --quiet HEAD --; then
            echo "Warning: Local changes detected in $target_dir."
        else
            echo "No local changes detected in $target_dir."
        fi
        run_cmd git checkout -B "$branch" "origin/$branch"
        if [ "$RUN_GIT_PULL" -eq 1 ]; then
            run_cmd git pull
        fi
        popd
    else
        run_cmd git clone -b "$branch" "$repo_url" "$target_dir"
    fi
}

# Check for required tools
check_command git
check_command vcs
check_command colcon
check_command rosdep

# Install required dependencies
echo "Installing dependencies..."
run_cmd "sudo apt update"
run_cmd "sudo apt install -y \
    ament-cmake-core \
    colcon \
    git \
    imx500-all \
    imx500-tools \
    libasio-dev \
    libboost1.74-all-dev \
    libmessage-filters-dev \
    libtinyxml2-dev \
    libx11-dev \
    libxrandr-dev \
    libyaml-cpp-dev \
    pigpio \
    pigpio-tools \
    pigpiod \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-munkres \
    python3-opencv \
    python3-pigpio \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep2 \
    python3-transforms3d \
    python3-vcstools \
    software-properties-common \
    vcstool \
    wget"

# Prepare external source directory
run_cmd "mkdir -p \"$EXTERNAL_SOURCE_DIR\""

# Clone picamera2 to external source dir
clone_repo https://github.com/raspberrypi/picamera2.git next "$EXTERNAL_SOURCE_DIR/picamera2"
run_cmd "pip install -e \"$EXTERNAL_SOURCE_DIR/picamera2\" --break-system-packages"

#sudo pip install --break-system-packages sparkfun-qwiic-icm20948

# Clone ROS2 repositories
cd "$EXTERNAL_SOURCE_DIR"
echo "Importing ROS2 repositories..."
echo "Using ROS2 distribution: ${ROS_DISTRO}"
echo "Importing repositories"
run_cmd "vcs import --input 'https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos' ."

# Configure rosdep
echo "Configuring rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    run_cmd "sudo rosdep init"
fi
run_cmd "rosdep update"

echo "Importing ros-controls repositories"
run_cmd "vcs import --input 'https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos' ."

# Optionally run find/git pull on all repos in external source dir
if [ "$RUN_GIT_PULL" -eq 1 ]; then
    run_cmd "find \"$EXTERNAL_SOURCE_DIR\" -type d -name '.git' -execdir git pull \;"
fi

# Clone repositories to external source dir
clone_repo https://github.com/jbeder/yaml-cpp.git master "$EXTERNAL_SOURCE_DIR/yaml-cpp"
clone_repo https://github.com/pal-robotics/backward_ros.git foxy-devel "$EXTERNAL_SOURCE_DIR/backward_ros"
clone_repo https://github.com/pal-robotics/pal_statistics.git humble-devel "$EXTERNAL_SOURCE_DIR/pal_statistics"
clone_repo https://github.com/PickNikRobotics/cpp_polyfills.git main "$EXTERNAL_SOURCE_DIR/cpp_polyfills"
clone_repo https://github.com/PickNikRobotics/generate_parameter_library.git main "$EXTERNAL_SOURCE_DIR/generate_parameter_library"
clone_repo https://github.com/PickNikRobotics/RSL.git main "$EXTERNAL_SOURCE_DIR/RSL"
clone_repo https://github.com/ros-drivers/ackermann_msgs.git ros2 "$EXTERNAL_SOURCE_DIR/ackermann_msgs"
clone_repo https://github.com/ros/diagnostics.git ros2-kilted "$EXTERNAL_SOURCE_DIR/diagnostics"
clone_repo https://github.com/ros/filters.git ros2 "$EXTERNAL_SOURCE_DIR/filters"
clone_repo https://github.com/ros/joint_state_publisher.git ros2 "$EXTERNAL_SOURCE_DIR/joint_state_publisher"
clone_repo https://github.com/ros/xacro.git ros2 "$EXTERNAL_SOURCE_DIR/xacro"

# Raspberry Pi AI Camera ROS2 packages
#clone_repo https://github.com/mzahana/raspberrypi_ai_camera_ros2.git camerainfo "$EXTERNAL_SOURCE_DIR/raspberrypi_ai_camera_ros2"
clone_repo https://github.com/foonathan/memory.git main "$EXTERNAL_SOURCE_DIR/memory"
clone_repo https://github.com/gzub/raspberrypi_ai_camera_ros2.git main "$EXTERNAL_SOURCE_DIR/raspberrypi_ai_camera_ros2"
clone_repo https://github.com/Kukanani/vision_msgs.git ros2 "$EXTERNAL_SOURCE_DIR/vision_msgs"
clone_repo https://github.com/ros-perception/image_transport_plugins.git rolling "$EXTERNAL_SOURCE_DIR/image_transport_plugins"
clone_repo https://github.com/ros-perception/pcl_msgs.git ros2 "$EXTERNAL_SOURCE_DIR/pcl_msgs"
clone_repo https://github.com/ros-perception/perception_pcl.git ros2 "$EXTERNAL_SOURCE_DIR/perception_pcl"
clone_repo https://github.com/ros-perception/vision_opencv.git rolling "$EXTERNAL_SOURCE_DIR/vision_opencv"

# Joystick ROS2 packages
echo Installing Joystick
clone_repo https://github.com/ros-drivers/joystick_drivers.git ros2 "$EXTERNAL_SOURCE_DIR/joystick_drivers"

# Web video server ROS2 packages
echo Installing Web Video Server
clone_repo https://github.com/fkie/async_web_server_cpp.git "ros2-develop" "$EXTERNAL_SOURCE_DIR/async_web_server_cpp"
clone_repo https://github.com/RobotWebTools/web_video_server.git ros2 "$EXTERNAL_SOURCE_DIR/web_video_server"

# Topic Based ROS2 Control
#echo Installing Topic Based Control
#clone_repo https://github.com/gzub/topic_based_ros2_control.git main "$EXTERNAL_SOURCE_DIR/topic_based_ros2_control"

# Nav2
echo Installing Navigation2
clone_repo https://github.com/BehaviorTree/BehaviorTree.CPP.git master "$EXTERNAL_SOURCE_DIR/BehaviorTree.CPP"
clone_repo https://github.com/cra-ros-pkg/robot_localization.git kilted-devel "$EXTERNAL_SOURCE_DIR/robot_localization"
clone_repo https://github.com/DLu/tf_transformations.git main "$EXTERNAL_SOURCE_DIR/tf_transformations"
clone_repo https://github.com/ros-geographic-info/geographic_info.git ros2 "$EXTERNAL_SOURCE_DIR/geographic_info"
clone_repo https://github.com/ros-navigation/navigation2.git ${ROS_DISTRO} "$EXTERNAL_SOURCE_DIR/navigation2"
clone_repo https://github.com/ros/bond_core.git ros2 "$EXTERNAL_SOURCE_DIR/bond_core"
clone_repo https://github.com/SteveMacenski/slam_toolbox.git ros2 "$EXTERNAL_SOURCE_DIR/slam_toolbox"

# IMUTools
# echo Installing IMUTools
clone_repo https://github.com/CCNYRoboticsLab/imu_tools.git ${ROS_DISTRO} "$EXTERNAL_SOURCE_DIR/imu_tools"
clone_repo https://github.com/ros-perception/imu_pipeline.git ros2 "$EXTERNAL_SOURCE_DIR/imu_pipeline"

# rqt_tf_tree
#echo Installing rqt_tf_tree
#clone_repo https://github.com/ros-visualization/rqt_tf_tree.git humble src/rqt_tf_tree

cd ${WORKSPACE_DIR}
# Install dependencies
echo "Installing ROS2 dependencies..."
run_cmd "rosdep install -r --from-paths \"$WORKSPACE_DIR\" --ignore-src --rosdistro \"${ROS_DISTRO}\" -y --skip-keys 'nav2_system_tests fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz-sim rti-connext-dds-7.3.0-ros ros-${ROS_DISTRO}-gz-sim-vendor ros-${ROS_DISTRO}-gz-plugin-vendor ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-sdformat-urdf' || true"

# Build the workspace (second build)
echo "Building the workspace..."
export CMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized -Wno-error=unused-parameter -Wno-error=deprecated-declarations -Wno-error=unused-result"

COLCON_ARGS=(
    --symlink-install
    --continue-on-error
    --packages-ignore nav2_system_tests gz_ros2_control gz_ros2_control_demos rosbag2_examples_cpp rosbag2_tests rosbag2_tests test_tracetools
    --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized -Wno-error=unused-parameter -Wno-error=deprecated-declarations -Wno-error=unused-result"
)
if [ "$SKIP_BUILD_FINISHED" -eq 1 ]; then
    COLCON_ARGS+=(--packages-skip-build-finished)
fi
export MAKEFLAGS="-j 2"
echo "Running: colcon build"
run_cmd colcon build --parallel-workers 2 --paths "$WORKSPACE_DIR" "${COLCON_ARGS[@]}"

echo "ROS2 setup completed successfully!"
