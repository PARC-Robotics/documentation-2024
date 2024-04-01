# Getting started with ROS

The [Robot Operating System](https://www.ros.org/about-ros/) (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

We have planned this competition around ROS because of its features as well as its widespread use in robotics research and industry. The version that will be used for this year's competition is **ROS 2 Humble** and utilizing Python.

![ROS and APIs](assets/ros-apis.png)

## ROS 2 OnRamp!

Whether you are a beginner or a more advanced ROS 2 developer, we recommend you take some time to review the following ROS 2 Humble tutorials, particularly the beginner section.

* [ROS 2 Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html){target=_blank}
* [ROS 2 Tutorial Youtube Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy){target=_blank}  (**Strongly recommended for beginners**, the content is excellent!)
!!! note
    Your overall learning experience in this competition is **strongly dependent** on how much of the fundamental concepts of ROS 2 you can grasp early on. Hence, we **strongly recommend** that you put in the time to use these resources.

## Writing your First ROS 2 Package

After you complete the required tutorials listed above, you can start [setting up the workspace](../getting-started-tutorials/setting-up-your-workspace.md).

Assuming the workspace at `~/ros2_ws/` as completed from the steps done in [setting up your workspace](../getting-started-tutorials/setting-up-your-workspace.md),
this should be your folder structure:

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

<!-- First step is to create your solution folder in `~/ros2_ws/src/`, we can call it `parc_solutions` for instance. -->
<!-- ```shell -->
<!-- mkdir ~/ros2_ws/src/parc_solutions -->
<!-- ``` -->
<!-- Go inside the folder, -->
<!-- ```shell -->
<!-- cd ~/ros2_ws/src/parc_solutions -->
<!-- ``` -->

First navigate to the source folder in your workspace,
```shell
cd ~/ros2_ws/src
```

Then create a new ROS 2 Python package called `test_publisher` (for example) by running the command below,
```shell
ros2 pkg create test_publisher --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs
```

Change directory into the newly created ROS 2 Python package,

```shell
cd test_publisher/
```

The `test_publisher` package file structure is as follows,

```
├── package.xml
├── resource
│   └── test_publisher
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── test_publisher
    └── __init__.py
```

### Moving the Robot Programmatically

[Setting up your workspace](../getting-started-tutorials/setting-up-your-workspace.md) guide has already shown how to control the robot with keyboard using `teleop_twist_keyboard`.

This guide will help you to move the robot by publishing commands to the `/robot_base_controller/cmd_vel_unstamped` topic programmically using Python, which is the language that will be used in the competition to interact with ROS 2.

To do this, create a file, `robot_publisher.py` inside the `test_publisher` directory with the `__init__` file your ROS 2 package (`test_publisher` in this instance) and make it executable.

```shell
cd ~/ros2_ws/src/test_publisher/test_publisher
touch robot_publisher.py
chmod +x robot_publisher.py
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
        # Create a publisher which can "talk" to Robot and tell it to move
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        # Create a Twist message and add linear x and angular z values
        move_cmd = Twist()

        ######## Move Straight ########
        print("Moving Straight")
        move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 4 seconds publish cmd_vel move commands
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publish to robot
            
        ######## Stop ########
        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0 # Assigning both to 0.0 stops the robot

        now = time.time()
        # For the next 5 seconds publish cmd_vel move commands
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotating Counterclockwise ########
        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # For the next 15 seconds publish cmd_vel move commands
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Stop ########
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


This code will make the robot move straight for 4 seconds, stop for 5 seconcds, rotate counterclockwise for 15 seconds and then stop.

### Compile and Run

!!! Note 
    We need to update the `setup.py` file in the ROS 2 package to include our new program. Add the following line in the`console_scripts` section of the `setup.py` file:

    ```python
    entry_points={
            'console_scripts': [
                    'move_robot = test_publisher.robot_publisher:main',
            ],
    },
    ```

Run the following commands to compile the code,

```shell
cd ~/ros2_ws
colcon_build
```

To see it working, first run the robot in simulation by running the following command in one terminal,

```shell
source ~/ros2_ws/install/setup.bash
ros2 launch parc_robot_bringup task1_launch.py
```

And run the following commands in another terminal to run this new program,

```shell
source ~/ros2_ws/install/setup.bash
ros2 run test_publisher robot_publisher.py
```

If you have set up everything well, you should see the robot moving in Gazebo as below,

![publisher demo](assets/getting_started_demo.gif)

<!-- ## Extra Resources

If you want to learn more about ROS, you can check out the following resources:

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials){target=_blank}  - Official ROS tutorials
- [ROS Tutorial YouTube Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q){target=_blank}  - YouTube playlist of ROS tutorials. This is a good resource if you prefer to learn by watching videos. -->
