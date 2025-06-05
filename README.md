# ROS2 Web UI
This is a simple web interface to control a ROS2 robot using rosbridge and a web browser. It allows you to send commands via ROS2 using a web page.

## Requirements
You need to have Python on a Ubuntu machine. 

You can use the [back/install_demo_tools.sh](./back/install_demo_tools.sh) script to install the requirements for you. It will install the following:
- ros2
- ros-turtlesim
- rosbridge_suite

You also need a Sofa Robotics version that is compatible with ROS2. The easiest way to set up this demo is with EmioLabs.

## Set Up
To run this demo you need to run a rosbridge server and a web server.

You need one computer that runs rosbridge and another machine (computer, smartphone, etc) with an Internet browser to open the `index.html` file. It can be the same machine.
Both computers need to be on the same network.

### rosbridge server
#### 1. ⚠️ Don't forget to source ROS2!

The script `install_demo_tools.sh` will do it for you, but if you run it manually, you need to source the ROS2 installation. You can do this by running the following command in your terminal:
```bash
source /opt/ros/jazzy/setup.bash
```

#### 2. Run the rosbridge server
You can run one of hte following commands to run the rosbridge server:

```bash
ros2 launch ./back/rosbridge_secure_websocket_launch.xml
```
or
```bash
ros2 launch ./backros_server_launch.py
```

These commands launch a rosbridge server that listens on port 9090 with SSL support. 
The server will be running on the machine where you run the command and should be on the same network as the machine where you will run Sofa Robotics.

### Webpage
You can use the built-in web server in Python or any other web server you prefer or use the github page that hosts the `index.html` file in the `front` folder.


Or you can use the online page [https://sofacompliancerobotics.github.io/ros-web-ui/](https://sofacompliancerobotics.github.io/ros-web-ui/).

### Sofa Robotics
Open Sofa robotics with the right simulation.
The easist way is to open EmioLabs and launch SofA Robotics from the sandbox with the right configuration of Emio.

__⚠️ Don't forget to source ROS2!__

__⚠️ If running on Unix system, don't forget to run `sudo chmod 777 /dev/ttyUSB0` in a terminal to be able to see the robot__


#### 1. Run the simulation
- Click on the play button.
- Enable the communication with the robot if needed
  
#### 2. Enable ROS communication
- In the `View` menu in the top bar, click on `Input/Output` to open the IOWindow.
- In the `Output` section, select or create the topic you want to publish on, for example `TCPFrame`
- Click on the `Publish` switch to start publishing
- In the `Intput` section, check the `TCPTarget/Frame` and `Gripper` topics
- Click on the `Listening` switch  to start listening to the topics 

## Usage
- Get the IP address of the rosbridge running machine
    - this should be something in the form of `192.168.x.x` or `127.0.0.1` if you are running it on the same machine
- Open `index.html` either by double clicking on the file or directly go online to [https://sofacompliancerobotics.github.io/ros-web-ui/](https://sofacompliancerobotics.github.io/ros-web-ui/)
- Int the `Settings` section of the page
    - Enter the IP address in the field on the webpage
    - Make sure `SSL` is enabled if you are using a secure connection (https://) or disabled if you are using an insecure connection (http://) (default is enabled)
    - Enable the `Publish` switch to start publishing the topics
- Use the sliders to send the commands
