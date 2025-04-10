# ROS Web UI
You need one computer that runs rosbridge and another machine (computer, smartphone, etc) with an Internet browser to open the `index.html` file. It can be the same machine.
Both computers need to be on the same network.

## Set Up
- Install `ros2` on the computer that will run the rosbridge
- Install `rosbridge_suite`

## Usage
- Don't forget to source ROS!
- Run `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Get the IP address of the rosbridge running machine
- Open `index.html`
- Enter the IP address in the field on the webpage
- Toggle the topics you want to publish on, e.g., /TCPTarget/Frame
- Use the input to send the commands
