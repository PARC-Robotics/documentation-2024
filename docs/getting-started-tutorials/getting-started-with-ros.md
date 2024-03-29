# Getting started with ROS

The [Robot Operating System](https://www.ros.org/about-ros/) (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

We have planned this competition around ROS because of its features as well as its widespread use in robotics research and industry. The version that will be used for this year's competition is **ROS 2 Humble**.

![ROS and APIs](assets/ros-apis.png)

## ROS 2 OnRamp!

Whether you are a beginner or a more advanced ROS 2 developer, we recommend you take some time to review the following ROS 2 Humble tutorials.

* [ROS 2 Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html){target=_blank}
* [ROS 2 Tutorial Youtube Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy){target=_blank}  (**Strongly recommended for beginners**, the content is excellent!)
!!! note
    Your overall learning experience in this competition is **strongly dependent** on how much of the fundamental concepts of ROS 2 you can grasp early on. Hence, we **strongly recommend** that you put in the time to use these resources.

<!-- - [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials){target=_blank}  - Official ROS tutorials
- [ROS Tutorial YouTube Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q){target=_blank}  - YouTube playlist of ROS tutorials. This is a good resource if you prefer to learn by watching videos

* [Chapter 5](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) (ROS Nodes): *"This tutorial introduces ROS graph concepts and discusses the use of roscore, rosnode, and rosrun commandline tools"*
* [Chapter 6](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics) (ROS Topics): *"This tutorial introduces ROS topics as well as using the rostopic and rqt_plot commandline tools."*
* [Chapter 12](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) (Writing simple publisher and subscriber in Python)
* Understand the [core tools provided by ROS](https://www.ros.org/core-components/), including RViz, rqt_graph, Gazebo, etc. -->


## Writing your First ROS 2 Package

After you complete the required tutorials listed above, you can start [setting up the workspace](../setting-up-your-workspace).

Assuming the workspace at `~/ros2_ws/` as completed from the steps done in [setting up your workspace](../setting-up-your-workspace),
This should be your folder structure till now.

```
~/ros2_ws/
├── build/
│   ├── .
│   └── .
├── install/
│   ├── .
│   └── .
├── log/
│   ├── .
│   └── .
└── src/
    ├── CMakeLists.txt
    └── PARC2024-Engineers-League/
        ├── parc_robot/
        │   ├── .
        │   ├── .
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── .
        └── .
```

First step is to create your solution folder in `~/ros2_ws/src/`, we can call it `parc_solutions` for instance.
```shell
mkdir ~/ros2_ws/src/parc_solutions
```
Go inside the folder,
```shell
cd ~/ros2_ws/src/parc_solutions
```

TODO: Mention Python. Create a python package, ie `ament_python`, instead

And here you can create a new ROS package called `test_publisher` (for example) by running the command below,
```shell
ros2 pkg create test_publisher --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs
```

### Moving the Robot Programmatically

[Setting up your workspace](../setting-up-your-workspace) guide has already shown how to control the robot with keyboard using `teleoperation`

But this guide will help you to move the robot by publishing commands to `/cmd_vel` topic programmically using Python and C++. In the competition, you would have to choose one of these languages/platforms to interact with ROS.


To do this, create a file, `robot_publisher.py` inside `scripts` folder in your ROS package (for example `test_publisher`) and make it executable.

```shell
mkdir test_publisher/scripts
touch test_publisher/scripts/robot_publisher.py
chmod +x test_publisher/scripts/robot_publisher.py
```

!!! note "Note"
    You need to change the permission of the file to executable to be able to run (as done in the last command shown above).

Now open the file and copy and paste the following code inside:

```python
#!/usr/bin/env python3
"""
Script to move Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        move_cmd = Twist()

        print("Moving Straight")
        move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 4 seconds publish cmd_vel move commands
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publish to robot

        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0 # Assigning both to 0.0 stops the robot

        now = time.time()
        # For the next 5 seconds publish cmd_vel move commands
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # For the next 10 seconds publish cmd_vel move commands
        while time.time() - now < 10:
            self.pub.publish(move_cmd)

        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```


This code will make the robot move straight for 4 seconds, stop for 5 seconcds, rotate counterclockwise for 10 seconds and then stop.

### Compile and Run

USE PYTHON

!!! Note 
    For C++, we need to update the `CMakeLists.txt` file to include our new program. Add the following line to the `CMakeLists.txt` file:

    ```cmake
    add_executable(robot_publisher src/robot_publisher.cpp)
    target_link_libraries(robot_publisher ${catkin_LIBRARIES})
    ```

Run the following command to compile the code:

```shell
cd ~/ros2_ws
colcon_build
```

To see it working, first run the robot in simulation by running the following command in one terminal

```shell
source ~/ros2_ws/install/setup.bash
ros2 launch parc_robot_bringup task1_launch.py
```

And run the following command in another terminal to run this new program:

```shell
source ~/ros2_ws/install/setup.bash
ros2 run test_publisher robot_publisher.py
```

If you have set up everything well, you should see the robot moving in Gazebo as below:

![publisher demo](assets/getting_started_demo.gif)

<!-- ## Extra Resources

If you want to learn more about ROS, you can check out the following resources:

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials){target=_blank}  - Official ROS tutorials
- [ROS Tutorial YouTube Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q){target=_blank}  - YouTube playlist of ROS tutorials. This is a good resource if you prefer to learn by watching videos. -->
