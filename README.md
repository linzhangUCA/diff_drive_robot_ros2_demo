# 10-Navigation
Use [Navigation2](https://github.com/ros-planning/navigation2/tree/main) to ease mapping and localization with more autonomous.
Install following packages to get ready for the navigation:
```console
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 
```

## Try Out Navigation2
[navigation2](https://github.com/ros-planning/navigation2/tree/main) is a collection of remarkable ROS packages that allows robot to navigate autonomously. To try it out, we'll need to:
1. Launch a gazebo simulation. 
```console
source <demobot_workspace>/install/local_setup.bash
ros2 launch robot_gazebo simulate.launch.py
```
2. (*In a new terminal window*) Launch a SLAM module from [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).
```console
ros2 launch slam_toolbox online_async_launch.py 
```
3. (*In a new terminal window*) Launch a navigation module from [nav2_bringup](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup)
```console
ros2 launch nav2_bringup navigation_launch.py  
```
4. (*In a new terminal window*) Start a [rviz](https://github.com/ros2/rviz) session.
```console
rviz2
``` 
> You may need to create a rviz config from scratch.

