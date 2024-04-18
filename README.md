# 10-Navigation
Use [Navigation2](https://github.com/ros-planning/navigation2/tree/main) to ease mapping and localization with more autonomous.
Install following packages to get ready for the navigation:
```console
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 
```

## 10.1 Branch Updates

### 10.1.1 [mapping_nav.launch.py](demobot_navigation/launch/mapping_nav.launch.py)
Includes [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/a3442d2f6824ff058fab0cb0a635e7a454294855/launch/online_async_launch.py) from [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [navigation_launch.py](https://github.com/ros-planning/navigation2/blob/12c786f8b8c533b0c4adc1a38c08021c90bfcba3/nav2_bringup/launch/navigation_launch.py) from [nav2_bringup](https://github.com/ros-planning/navigation2/tree/12c786f8b8c533b0c4adc1a38c08021c90bfcba3/nav2_bringup) package. 

### 10.1.2 [localize_nav.launch.py](demobot_navigation/launch/localize_nav.launch.py)
Includes [localization_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/a3442d2f6824ff058fab0cb0a635e7a454294855/launch/localization_launch.py) from [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [navigation_launch.py](https://github.com/ros-planning/navigation2/blob/12c786f8b8c533b0c4adc1a38c08021c90bfcba3/nav2_bringup/launch/navigation_launch.py) from [nav2_bringup](https://github.com/ros-planning/navigation2/tree/12c786f8b8c533b0c4adc1a38c08021c90bfcba3/nav2_bringup) package.

### 10.1.3 Configurations
- [slam_mapping_params.yaml](demobot_navigation/config/slam_mapping_params.yaml) contains configuration parameters for [online_async_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/a3442d2f6824ff058fab0cb0a635e7a454294855/launch/online_async_launch.py). `max_laser_range` is set to `6.0`.
- [slam_localization_params.yaml](demobot_navigation/config/slam_localization_params.yaml) contains configuration parameters for [localization_launch.py](https://github.com/SteveMacenski/slam_toolbox/blob/a3442d2f6824ff058fab0cb0a635e7a454294855/launch/localization_launch.py). `max_laser_range` is set to `6.0`. `map_file_name` is set according to the map files saved in [map/](demobot_navigation/map/) directory.
- [nav2_params.yaml](demobot_navigation/config/nav2_params.yaml) contains configuration parameters for [navigation_launch.py](https://github.com/ros-planning/navigation2/blob/12c786f8b8c533b0c4adc1a38c08021c90bfcba3/nav2_bringup/launch/navigation_launch.py). `robot_radius` under `global_costmap` is set to `0.6`. `robot_radius` under `local_costmap` is set to `0.55`.

## Tryout Navigation2
[navigation2](https://github.com/ros-planning/navigation2/tree/main) is a collection of remarkable ROS packages that allows robot to navigate autonomously. To try it out, you'll need to:
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
4. (*In a new terminal window*) Start a [rviz](https://github.com/ros2/rviz) session to visualize the slam process.
```console
rviz2
``` 

> You may need to create a rviz config from scratch.

