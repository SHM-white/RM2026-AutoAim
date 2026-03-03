#!/usr/bin/env bash

CURRENT_DIR=$(pwd)
set -euo pipefail
# Install core dependencies (add unzip, lsb-release, python3-pip)
sudo apt update || true
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen \
    googletest \
    libceres-dev \
    curl \
    unzip \
    lsb-release \
    python3-pip

# Get user Downloads directory
DOWNLOADS_DIR="$HOME/Downloads"

if [ ! -d "$DOWNLOADS_DIR" ]; then
    mkdir -p "$DOWNLOADS_DIR"
fi

if [ ! -d "/opt/intel/openvino_2024" ]; then
    sudo mkdir -p /opt/intel
    cd "$DOWNLOADS_DIR"
    # Get Ubuntu major version
    ubuntu_version=$(lsb_release -rs | cut -d. -f1)
    curl -L "https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu${ubuntu_version}_2024.6.0.17404.4c0f47d2335_x86_64.tgz" --output openvino_2024.6.0.tgz
    tar -xf openvino_2024.6.0.tgz
    sudo mv l_openvino_toolkit_ubuntu${ubuntu_version}_2024.6.0.17404.4c0f47d2335_x86_64 /opt/intel/openvino_2024.6.0
    cd /opt/intel/openvino_2024.6.0
    sudo -E ./install_dependencies/install_openvino_dependencies.sh
    python3 -m pip install -r ./python/requirements.txt
    cd /opt/intel
    sudo ln -s openvino_2024.6.0 openvino_2024 || true
fi

cd "$DOWNLOADS_DIR"
if [ ! -d "/opt/MVS" ]; then
    sdk_zip="MvCamCtrlSDK_STD_V4.7.0_251113.zip"
    sdk_url="https://www.hikrobotics.com/cn2/source/support/software/MvCamCtrlSDK_STD_V4.7.0_251113.zip"
    
    max_retries=3
    attempt=0
    success=false

    while [ $attempt -lt $max_retries ]; do
        attempt=$((attempt+1))
        echo "Downloading MVS SDK from: $sdk_url (Attempt $attempt/$max_retries)..."
        
        # Clean up previous attempt
        rm -f "$sdk_zip"

        if curl -fSL --retry 3 --retry-delay 5 "$sdk_url" -o "$sdk_zip"; then
            if unzip -t "$sdk_zip" >/dev/null 2>&1; then
                success=true
                break
            else
                echo "Warning: Downloaded file corrupted (unzip check failed)."
            fi
        else
            echo "Warning: Download failed."
        fi
    done

    if [ "$success" = false ]; then
        echo "Error: Failed to download valid SDK after $max_retries attempts"
        exit 1
    fi

    unzip "$sdk_zip" -d MvCamCtrlSDK_STD
    cd MvCamCtrlSDK_STD
    # Get system architecture
    arch=$(uname -m)
    case "$arch" in
        x86_64)
            sudo apt install ./MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113.deb -y
            ;;
        i386|i686)
            sudo apt install ./MvCamCtrlSDK_Runtime-4.7.0_i386_20251113.deb -y
            ;;
        aarch64)
            sudo apt install ./MvCamCtrlSDK_Runtime-4.7.0_arm64_20251113.deb -y
            ;;
        armv7l)
            sudo apt install ./MvCamCtrlSDK_Runtime-4.7.0_armhf_20251113.deb -y
            ;;
        *)
            echo "Unsupported architecture: $arch"
            ;;
    esac
    ls /opt/MVS || true
fi

if [ ! -d "/opt/ros/humble" ]; then
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    sudo curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb || true
    sudo apt update & sudo apt upgrade -y
    sudo apt install ros-humble-desktop-full -y
    sudo apt install ros-dev-tools -y

    # WRITE source /opt/ros/humble/setup.bash to ~/.bashrc if not already present
    if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi

    source /opt/ros/humble/setup.bash
    sudo apt install ros-humble-plotjuggler-ros -y
fi

mkdir "$HOME/ros_ws/src" || true
cp ${CURRENT_DIR}/sp_msgs "$HOME/ros_ws/src/" -r
cd "$HOME/ros_ws"
source /opt/ros/humble/setup.bash || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install