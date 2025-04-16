# ROS Web UI
This is a simple web interface to control a ROS2 robot using rosbridge and a web browser. It allows you to send commands via ROS2 using a web page.

## Requirements
You need to have Python on a Ubuntu machine. 

You can use the [back/install_demo_tools.sh](./back/install_demo_tools.sh) script to install the requirements for you. It will install the following:
- ros2
- ros-turtlesim
- rosbridge_suite

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
ros2 launch rosbridge_server ./back/rosbridge_secure_websocket_launch.xml
```
or
```bash
python ./backros_server_launch.py
```

These commands launch a rosbridge server that listens on port 9090 with SSL support. The server will be running on the machine where you run the command and shold be on the same network as the machine where you will run Sofa Robotics.

### Webpage
You can use the built-in web server in Python or any other web server you prefer or use the github page that hosts the `index.html` file in the `front` folder.


## Usage
- Get the IP address of the rosbridge running machine
    - this should be something in the form of `192.168.x.x` or `127.0.0.1` if you are running it on the same machine) 
- Open `index.html` either by double clicking on the file or directly go online to [https://sofacompliancerobotics.github.io/ros-web-ui/](https://sofacompliancerobotics.github.io/ros-web-ui/)
- Int the `Settings` section of the page
    - Enter the IP address in the field on the webpage
    - Make sure `SSL` is enabled if you are using a secure connection (https://) or disabled if you are using an insecure connection (http://) (default is enabled)
    - Enable the `Publish` switch to start publishing the topics
- Use the sliders to send the commands
