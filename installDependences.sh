apt install -y \
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
    libceres-dev

if [ ! -d "~/Downloads" ]; then
        mkdir ~/Downloads
    fi

if [ ! -d "/opt/intel/openvino_2024" ]; then
    mkdir -p /opt/intel
    if [ ! -d "~/Downloads" ]; then
        mkdir ~/Downloads
    fi
    cd ~/Downloads
    # Get Ubuntu major version
    ubuntu_version=$(lsb_release -rs | cut -d. -f1)
    curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu${ubuntu_version}_2024.6.0.17404.4c0f47d2335_x86_64.tgz --output openvino_2024.6.0.tgz
    tar -xf openvino_2024.6.0.tgz
    sudo mv l_openvino_toolkit_ubuntu${ubuntu_version}_2024.6.0.17404.4c0f47d2335_x86_64 /opt/intel/openvino_2024.6.0
    cd /opt/intel/openvino_2024.6.0
    sudo -E ./install_dependencies/install_openvino_dependencies.sh
    cd /opt/intel/openvino_2024.6.0
    python3 -m pip install -r ./python/requirements.txt --break-system-packages
    cd /opt/intel
    sudo ln -s openvino_2024.6.0 openvino_2024
fi

cd ~/Downloads
if [ ! -d "/opt/MVS"]; then
    curl -L https://www.hikrobotics.com/cn2/source/support/software/MvCamCtrlSDK_STD_V4.7.0_251113.zip --output MvCamCtrlSDK_STD_V4.7.0_251113.zip
    unzip MvCamCtrlSDK_STD_V4.7.0_251113.zip -d MVS