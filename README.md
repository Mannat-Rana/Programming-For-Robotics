# Completed Work for Programming For Robotics at ETH Zurich ![SHIELD](https://img.shields.io/badge/Project%20Status%3A-Complete-green?style=for-the-badge) ![ros](https://camo.githubusercontent.com/4c117e738ecff5825b1031d601ac04bc70cc817805ba6ce936c0c556ba8e14f0/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d524f5326636f6c6f723d323233313445266c6f676f3d524f53266c6f676f436f6c6f723d464646464646266c6162656c3d) ![c++](https://camo.githubusercontent.com/6301a47e098ea0b84260920a75b5a71f121c5a0b55965dff8ad80bd60db208c7/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d4325324225324226636f6c6f723d303035393943266c6f676f3d43253242253242266c6f676f436f6c6f723d464646464646266c6162656c3d) ![ROBOTICS](https://camo.githubusercontent.com/b8e2732eda54a502cb34a56c1ea83747134ce98754e6c49a3177cd89f411bc97/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d526f626f742b4672616d65776f726b26636f6c6f723d303030303030266c6f676f3d526f626f742b4672616d65776f726b266c6f676f436f6c6f723d464646464646266c6162656c3d)

## About
This repository contains my final ROS package developed as part of my completion of the Programming for Robotics course taught at ETH Zurich. The main function of the package is to provide a SMB robot with an emergency stop triggered by either detecting a nearby obstacle with a laser scanner, or crashing into an obstacle detected via IMU. Furthermore, the robot publishes the location of the pillar on RViz on the odom frame using the tf package to convert the pillar location from the rslidar frame to the odom frame. 

## Demonstration

### Emergency Stop
![smb_obstacle_detection_demo](https://user-images.githubusercontent.com/82643627/155904216-915a3572-43ed-4c8b-be95-edf5d02811ac.gif)

### Pillar Marker
![pillar_detection](https://user-images.githubusercontent.com/82643627/155904170-e5d9f43e-605f-47d0-b55a-04faf5ac5ae6.png)


## To Install
```bash
cd ~/catkin_ws/src
git clone https://github.com/Mannat-Rana/Programming-For-Robotics.git
catkin build smb_highlevel_controller
cd ~/catkwin_ws
source devel/setup.bash
```
## To Run
```bash
roslaunch smb_highlevel_controller smb_gazebo.launch
```
