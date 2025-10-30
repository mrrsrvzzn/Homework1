# HomeworkRL1
This repository contains ROS packages that implement the visualisation and control of the ‘Armando’ robot in simulation, using Rviz and Gazebo.

Before running this code, ensure you have added the following lines to your Dockerfile and rebuilded your ros2 docker image:
``` bash
RUN apt-get install ros-humble-ros-ign-bridge -y && \
apt-get install ros-humble-ros-gz -y && \
apt-get install ros-humble-controller-manager -y && \
apt-get install ros-humble-ros2-control -y && \
apt-get install ros-humble-ros2-controllers -y && \
apt-get install ros-humble-ign-ros2-control -y && \
apt-get install ros-humble-joint-state-publisher -y && \
apt-get install ros-humble-joint-state-publisher-gui -y && \
apt-get install ros-humble-xacro -y && \
apt-get install ros-humble-urdf-launch -y && \
apt-get install ros-humble-urdf-tutorial -y
```
If you're not using ros2 as a docker image, simply install those packages. Eventually replace the humble voice with your ros2 version.

To check the visualization of the Armando robot in Rviz and Gazebo, execute the following command:
``` bash
ros2 launch armando_gazebo armando_world.launch.py
```
By doing so, in Rviz, you should also see the images from the camera placed on the base_link of Armando, which displays a bottom-up point of view of the robot.
You could also check the image of the camera by executing the following line from another terminal:
``` bash
rqt run rqt_image_view rqt_image_view
```
Through Rviz, you can also check the robot's collision meshes by ticking the corresponding item in the left sidebar and see how they differ from the visual meshes.

To control the Armando robot in a desired configuration of the joints, open another terminal and paste the following command:
``` bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.9, -0.6, 0.3, 0.0]"
```
Replace the four numbers of the data with the configuration you desire for the j0, j1, j2, j3 joints. 

You can control the Armando robot through the arm_controller_node node. Once the simulation has started, open another terminal and execute one of the following commands:
``` bash
ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=position
```
if you want to control the position,
``` bash
ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=trajectory
```
if you want to control the trajectory.

These controllers will move the robot's joints to a predefined position, characterised by the following vector of the positions of Armando's individual movable joints: [0.9, -0.6, 0.3, 0.1], defined in the arm_controller_node. 
