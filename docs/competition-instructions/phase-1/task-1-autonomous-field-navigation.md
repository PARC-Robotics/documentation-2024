# Task 1: Autonomous Field Navigation

## General Description

![task1_simulation](../assets/task1_sim.gif)
Agricultural robots must be able to navigate through crops and farmland, which includes autonomously moving through rows of tomato plants on rough terrain. This task involves reaching the end of a row, making a turn, and returning in adjacent rows until the goal location is reached. Teams must develop software to guide the robot through a [pre-defined path](#exploring-multiple-routes) within the crop rows, from its starting position to the goal location.

## Task Guidelines

### Launching the Task
In a new terminal, run the following launch file to bring up the robot in Gazebo and RViz:

```sh
ros2 launch parc_robot_bringup task1_launch.py
```

You should see the display below in Gazebo and RViz respectively. To the right, there's the robot and to the left is the green circle which represents the goal location.

=== "Gazebo"
    ![task1_gazebo](../assets/task1_gazebo.jpg)

=== "RViz"
    ![task1_rviz](../assets/task1_rviz.png)

### Exploring Multiple Routes
* We have prepared three pre-defined routes you can use as you develop your solution with each route having different goal location.

=== "Route 1"
    ![task1_route1](../assets/Task1Route1.jpg)

=== "Route 2"
    ![task1_route2](../assets/Task1Route2.jpg)

=== "Route 3"
    ![task1_route3](../assets/Task1Route3.jpg)


The default route is `route1`, but you can select the second and third route option (`route2` and `route3`) by passing the argument in the `ros2 launch` command as follows:

```sh
## route2
ros2 launch parc_robot_bringup task1_launch.py route:=route2

## route3
ros2 launch parc_robot_bringup task1_launch.py route:=route3
```

* We recommend you play around with at least these three routes to ensure your solution is robust to different start locations.

* To obtain the GPS goal location for this task, regardless of the route option, you use ROS [parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html){target=_blank} in a node. 
    Here is an example of how to obtain the goal location as a ROS parameter:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GoalLocation(Node):
    def __init__(self):
        super().__init__("goal_location")

        # Declare goal latitude and longitude parameters
        self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        # Get goal location from world coordinates yaml file
        goal_lat = self.get_parameter("goal_latitude")
        goal_long = self.get_parameter("goal_longitude")

        # Print goal location
        self.get_logger().info(
            "goal location: %f %f"
            % (
                goal_lat.value,
                goal_long.value,
            )
        )


def main(args=None):
    rclpy.init(args=args)

    goal_location = GoalLocation()
    rclpy.spin(goal_location)

    goal_location.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

The goal location parameter values are obtained from provided `.yaml` files, which vary depending on the route chosen, and these files are passed as arguments when the node is run as follows:

```sh
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

For instance, choosing `route1` for the navigation task, the following sample command will be executed:

```sh

ros2 run parc_solutions task1_solution.py --ros-args --params-file ~/ros2_ws/src/parc_robot_bringup/config/route1_world_coordinates.yaml
```

!!! note
    An [absolute file path](https://www.redhat.com/sysadmin/linux-path-absolute-relative){target=_blank} was used to locate the parameter file. A relative file path can also be used
    if the command is called inside the `config` directory where the coordinate files are located. However, specifying the absolute file path is recommended to avoid path errors.

The `route1_world_coordinates.yaml` file contents are as follows:

```
/**:
  ros__parameters:

    origin_latitude: 49.90000010022057
    origin_longitude: 8.900000304717647
    
    goal_latitude: 49.89995550730833
    goal_longitude: 8.900078504470644
    
    peg_a_latitude: 49.90003227516326
    peg_a_longitude: 8.899956219604325

    peg_b_latitude: 49.90000470401652
    peg_b_longitude: 8.89994553616028
  
    peg_c_latitude: 49.89998381891777
    peg_c_longitude: 8.899978564376983

    peg_d_latitude: 49.90001471436836
    peg_d_longitude: 8.899990252377338
  
    peg_e_latitude: 49.900026830087526
    peg_e_longitude: 8.900024049357661

    peg_f_latitude: 49.89999732019855
    peg_f_longitude: 8.900012822919749

```

Similarly, the GPS coordinates of the pegs on the farmland can be obtained as a parameter if you need it for localization. Here is an example of how to obtain the GPS coordinate of **peg A**:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class PegALocation(Node):
    def __init__(self):
        super().__init__("peg_a_coordinates")

        # Declare peg A latitude and longitude parameters
        self.declare_parameter("peg_a_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_a_longitude", rclpy.Parameter.Type.DOUBLE)

        # Get peg A location from world coordinates yaml file
        peg_a_lat = self.get_parameter("peg_a_latitude")
        peg_a_long = self.get_parameter("peg_a_longitude")

        # Print peg A location
        self.get_logger().info(
            "Peg A location: %f %f"
            % (
                peg_a_lat.value,
                peg_a_long.value,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    
    peg_a_coordinates = PegALocation()
    rclpy.spin(peg_a_coordinates)

    peg_a_coordinates.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

!!! warning
    Please **DO NOT** use the cartesian coordinates of the goal location and pegs provided by Gazebo or the world file in any way. You will be penalized if you do.

### Converting GPS to Cartesian
Our module, **gps2cartesian**, provides a convenient way to convert GPS locations to x-y Cartesian coordinates. By using the Gazebo world origin as the GPS reference origin (0, 0) in Cartesian coordinates, the **gps_to_cartesian()** function calculates the Cartesian coordinates of any desired GPS location passed as a parameter to the function. Here is an example of how to use the module to get the cartesian coordinate of the robot with respect to the reference origin:

```python
#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition tasks must be running for this code to work.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class GPSGoal(Node):
    def __init__(self):
        super().__init__("gps_goal")

        # Subscribe to the gps topic once
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )

    def gps_callback(self, gps):

        # Get the cartesian coordinates from the GPS coordinates
        x, y = gps_to_cartesian(gps.latitude, gps.longitude)

        # Print cartesian coordinates
        self.get_logger().info(
            "The translation from the origin (0, 0) to the gps location provided: %.3f %.3f"
            % (x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    gps_goal = GPSGoal()
    rclpy.spin(gps_goal)

    gps_goal.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Preparing your Solution
* Your solution should be prepared as ROS packages to be saved in your solution folder. Create a node executable file in your ROS package which runs ALL the code you need in your solution. Name this node file: `task1_solution.py`.

* Hence, your solution to Task 1 should be run by calling the following commands:

In one terminal:

```sh
ros2 launch parc_robot_bringup task1_launch.py
```

Or 

```sh
ros2 launch parc_robot_bringup task1_launch.py route:=route2
```

Or

```sh
ros2 launch parc_robot_bringup task1_launch.py route:=route3
```

!!! note "Note"
    Please wait until both the world and robot models have finished spawning. This process may take longer than usual, especially when running the program for the first time.

In another terminal:

```sh
ros2 run <your-package-name> task1_solution.py --ros-args --params-file <absolute-path-to-route-world-coordinates-yaml-file>
```

## Task Rules

* The time limit to complete the task is **7 minutes (420 seconds)**.

* The task is ONLY complete when ANY part of the robot is inside the green circle (goal location marker) after following the pre-defined path as shown above.

!!! note "Note"
    Ensure you DO NOT provide a solution with hard-coded positions for the robot to move to because in evaluation, the robot's initial position would be randomized. 

Scoring for this task would be based on the following criteria:

| S/N      | Criteria/Metric | Description |
| ----------- | ----------- | ------- |
| 1  | **Pre-defined path** | Every route launched has a pre-defined path that **must** be followed as explained at [Route Description](#exploring-multiple-routes). |
| 2  | **Plant and peg avoidance**  | The robot should avoid making contact with the tomato plants and/or pegs. **(Less contact is better)** |
| 3 | **Final travel distance to goal** | Shortest travel distance from robot (measured from robot center) through the crop rows to the goal which is calculated at the time limit [6 minutes] **(Smaller is better)**
| 4  | **Completion time** | Time from launching the solution to task completion **(Smaller is better)** |
