# 1 - URDF
Let's build a model for a diffrential drive mobile robot using a Unified Robot Description Format (URDF) file. First, let's create such a model for **visualization** purpose only. We will use this model file for other applications (simulation, slam, navigation) later, which are powered by Robot Operating System (ROS). If you don't know what ROS/ROS2 is, check out their official [documentation](https://docs.ros.org/en/humble/index.html).

## 1.1 Create a ROS Workspace and a ROS Package
To visualize a URDF model does not require ROS. However, this model will live in the big family of ROS, eventually. So, let's give it a home using ROS standards.
1. Create a ROS workspace
```bash
mkdir -p ~/demobot_ws/src/ros2_demo_robot/
```
2. Create a package 
```bash
cd ~/demobot_ws/src/ros2_demo_robot
ros2 pkg create --build-type ament_python demobot_description
```

## 1.2 URDF robot model
We can model a wide range of robots using URDF. Two key elements are **link** and **joint**. A robot model can be configured with a set of links and joints.
![urdf_config](https://wiki.ros.org/urdf/XML/model?action=AttachFile&do=get&target=link.png)
1. Create a URDF file.
```bash
cd ~/demobot_ws/src/ros2_demo_robot/demobot_description
mkdir urdf
cd urdf
touch demobot.urdf
```
2. Initiate the URDF file.

Place the following contents into the `demobot.urdf` file we just created.
```xml
<?xml version="1.0"?>
<robot name="demobot">
</robot>
```
We gave the robot model a name: `demobot`, but the model now is empty. Despite that, we can still visualize this model using an online urdf visualization tool: **[mymodelrobot](https://mymodelrobot.appspot.com/)**.

3. Add the `base_link` to the URDF file.

The `base_link` is always the first link for a robot's model. We'll use a box to represent the main body of our robot. And the dimension of this box will be set to 0.8m x 0.4m x 1.0m (W x H x D). A fixed cartesian reference frame (`base_link` frame) comes with the first link. We'll set the box's width edges parallel to the y axis, height edges to parallel to the z axis and depth edges to parallel to the x axis of the frame. Now your URDF file should look like below, and you can visualize the model by updating the code block on **[mymodelrobot](https://mymodelrobot.appspot.com/)** page.
```xml
<?xml version="1.0"?>
<robot name="demobot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 0.8 0.4"/>
            </geometry>
        </visual>
    </link>
</robot>
```

4. Add the `caster_joint` and `caster_ball` to the URDF file.
In order to specify next link's location and orientation. We need to set up another cartesian reference frame, so that we can attach the next link to it. We would like to model the caster wheel next, and we would like to place the `caster_ball` (next link) under the `base_link` and tailing at the back of the robot's body. 
```xml
<?xml version="1.0"?>
<robot name="demobot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 0.8 0.4"/>
            </geometry>
        </visual>
    </link>

    <joint name="caster_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_ball"/>
        <origin xyz="-0.4 0 -0.2"/>
    </joint>

    <link name="caster_ball">
        <visual>
            <geometry>
                <sphere radius="0.1"/>
            </geometry>
        </visual>
    </link>
</robot>
```