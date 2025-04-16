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
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-jazzy-desktop
fi

# install turtlesim
sudo apt install ros-jazzy-turtlesim

# install rosbridge_suite
sudo apt-get install ros-jazzy-rosbridge-suite

source /opt/ros/jazzy/setup.bash