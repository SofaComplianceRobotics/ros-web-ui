# igf ROS2 not installed, install it
if [ ! -d "/opt/ros/jazzy" ]; then
    # Set Locale
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

    # Set up sources
    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo apt install /tmp/ros2-apt-source.deb

    # install ros2 jazzy
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
fi

# install turtlesim
sudo apt install ros-jazzy-turtlesim

# install rosbridge_suite
sudo apt-get install ros-jazzy-rosbridge-suite

source /opt/ros/jazzy/setup.bash

# install yaml python module and other pytohn packages that might be missing
sudo apt install python3-yaml
python -m pip install pyyaml==6.0.1 numpy==1.26.4 psutil argcomplete pymongo pillow tornado
