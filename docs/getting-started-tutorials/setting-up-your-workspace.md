# How to set up your workspace

In this tutorial, you will set up a directory on your ROS 2 enabled PC as your workspace for development and install the competition ROS 2 packages. Please follow the instructions below carefully.

!!! note
    This can ONLY be completed after you have set up your PC (by following the tutorial here: [Setting up your PC](../getting-started-tutorials/setting-up-your-pc.md)).

<!-- uncommment once we have docker setup -->
<!-- !!! note -->
<!--     If you are using a Docker container, you can skip this tutorial and follow the instructions in [Setting up your PC using Docker](../getting-started-tutorials/setting-up-with-docker.md) instead. -->

### Step 1: Setup ROS 2 workspace

Open a new terminal on your PC, then copy and paste the following one line at a time:
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone the repository

In the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/PARC2024-Engineers-League.git .
```

### Step 3: Install dependencies

In the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

### Step 4: Compile packages

The next step is to compile the installed packages using `colcon build`:
```sh
cd ~/ros2_ws
colcon build
```

### Step 5: Set up ROS 2 environment
The following command needs to be run in every new terminal you open to get access to ROS 2 commands:

```sh
source /opt/ros/humble/setup.bash
```

To avoid sourcing the ROS setup file every time you launch a new terminal, you can add the command to your shell startup script by executing these lines:

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The `ros2_ws` workspace is an **overlay** on top of the ROS installation, which is known as the **underlay**, and similarly to use the package executables or libraries
in `ros2_ws`, the workspace will need to be sourced in every new terminal opened with this command:


```sh
source ~/ros2_ws/install/setup.bash
```

Likewise to avoid manually sourcing the workspace in each newly launched terminal, the command can also be added to shell startup script:

```sh
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

!!! note
    As you develop, it is good to set the environment variables whenever you run a `colcon build` command to compile changes to your packages. You can do that by:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

### Step 6: Gazebo Classic installation and setup

Gazebo Classic, version 11, is the robot simulator used in the competition and can be installed [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install){target=_blank}.

The tomato field is made up of custom models, the tomato plants for instance, that are not available in Gazebo. These models will need to be copied to the appropriate directory to completely view the tomato field.

Navigate to the models folder in the `parc_robot_bringup` package, then copy its directory contents to the Gazebo `models` directory on your PC:

```sh
cd ~/ros2_ws/src/parc_robot_bringup/models
cp -R . ~/.gazebo/models
```

!!! Note 
    The 3D visualizer for ROS, [`RViz`](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html){target=_blank}, is automatically installed when ROS 2 Humble was installed on your PC in the [setting up your PC](../getting-started-tutorials/setting-up-your-pc.md) tutorial.


### Step 7: Test installation

If you completed the preceding tasks successfully, you should be able to run this ROS 2 launch command and see the Gazebo Classic simulator and RViz simulator open with the following display:

```sh
ros2 launch parc_robot_bringup task_1_launch.py
```
![Gazebo Simulator window](assets/gazebo.png)
Gazebo Classic Simulator window


![RViz window](assets/rviz.png)
RViz window


If you run the following command in a new terminal,
```
rqt_graph
```
You will see a screen like this:

![RQT Graph](assets/rosgraph.png)

You need to publish/write to the `topic` `robot_base_controller/cmd_vel_unstamped` to move the robot.
The following guide will help you control the robot using keyboard. Once you have tested that, you can follow the [getting-started-with-ros](../getting-started-tutorials/getting-started-with-ros.md) guide to write a python program to control the robot.

### Step 8: Controlling the robot using keyboard

First of all, the `teleop_twist_keyboard` ROS 2 package is installed which will enable us to use the keyboard to control the robot in a terminal as follows,

```sh
sudo apt install ros-humble-teleop-twist-keyboard
```

Then run the following command in a new terminal,

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap \
/cmd_vel:=/robot_base_controller/cmd_vel_unstamped
```

Now keeping this second terminal active (or on top) press `i` to move the robot forward, you can see the robot moving in the "RViz" and "Gazebo" windows.
You can use the keys shown below to move the robot and `k` key to stop the movement.

```sh
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```
