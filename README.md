# URDF
Let's build a model for a diffrential drive mobile robot using a Unified Robot Description Format (URDF) file. First, let's create such a model for **visualization** purpose only. We will use this model file for other applications (simulation, slam, navigation) later, which are powered by Robot Operating System (ROS). If you don't know what ROS/ROS2 is, check their official [documentation](https://docs.ros.org/en/humble/index.html) to get started. 

## Create a ROS Workspace and a ROS Package
To visualize a URDF model does not require ROS. However, this model will be employed by ROS eventually. So, let's give it a home using ROS standards.
1. Create a ROS workspace
```bash
mkdir -p ~/demobot_ws/src/ros2_demo_robot/
```
2. Create a package 
```bash
cd ~/demobot_ws/src/ros2_demo_robot
ros2 pkg create --build-type ament_python demobot_description
```