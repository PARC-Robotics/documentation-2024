# Task 2: Crop Yield Estimation

![Task 2 Demo](../assets/task2.gif)

## General Description

According to the United Nations, the world population is predicted to rise to 8.5 billion by 2030, with Africa being the fastest growing continent. This projected population growth necessitates the urgency for food security. Accurately estimating crop yields are an essential facet in the pursuit for food security
and autonomous agricultural robots are employed in this undertaking.

For this task, the PARC robot is autonomously driven through two rows of tomato plants and teams are to develop software to estimate the yield of the tomato field by making use of the robot's RGB cameras and computer vision.

## Task Guidelines

### Launching the Task

In a new terminal, run the following launch file to bring up the robot in Gazebo and RViz:

```bash
ros2 launch parc_robot_bringup task2_launch.py
```

You should see the display below in Gazebo and RViz respectively.

![task2_world](../assets/gazebo_on_start.png)
![task2_rviz](../assets/task2rviz.png)

There are three worlds for this task, with each world varying in the number of fruit producing tomato plants. The default world is `world1` and similar to task 1, the second and third world options, `world2` and `world3`, can be selected by passing the argument in the `ros2 launch` command below:

```bash
# world2
ros2 launch parc_robot_bringup task2_launch.py world:=world2

# world3
ros2 launch parc_robot_bringup task2_launch.py world:=world3
```

The robot starts moving once the nodes called by the launch file have been successfully loaded.

To publish the number of fruits in a chosen world, you should use the topic `/parc_robot/crop_yield` that has the message type `/std_msgs/String`.


A new topic called `/parc_robot/robot_status` has been added to publish the current status of the robot. The message type for this topic is `/std_msgs/String`, which indicates whether the robot has started moving along the route or has finished the designated route. The robot status has two possible values: **"started"** and **"finished"**.


### Moving at different speeds

The robot can move at different speeds. The default speed is 0.1 m/s, but you can change the speed by passing the argument in the `ros2 launch` command as follows:

```bash
## 0.5 m/s
ros2 launch parc_robot_bringup task2_launch.py speed:=0.5
```

We recommend you play around with different speeds to ensure your solution is robust to different speeds.

### Task Expectations

The objective of the task is to identify the number of tomato fruits as the robot moves between the two rows of tomato plants. in a world 


Each world will be tested

drive the robot through a row of crops to identify and communicate the locations of any weeds in the field. When the robot reaches the end of the row, it will come to a stop, and you should publish the weed locations to the `/parc_robot/weed_detection` topic.

It's important to note that real-time publication of counted tomato fruits is not necessary. You can publish the number of tomato fruits after the robot has stopped moving, which you can monitor through the `/parc_robot/robot_status` topic.

They need to count for the three worlds

### Preparing your Solution
* Your solution should be prepared as ROS packages to be saved in your solution folder. Create a node executable file in your ROS package which runs ALL the code you need in your solution. Name this node file: `task2_solution.py`.

* Hence, your solution to Task 2 should be run by calling the following commands:

In one terminal:

```sh
ros2 launch parc_robot_bringup task2_launch.py
```

Or 

```sh
ros2 launch parc_robot_bringup task2_launch.py world:=world2
```

Or

```sh
ros2 launch parc_robot_bringup task2_launch.py world:=world3
```

!!! note "Note"
    Please wait until both the world and robot models have finished spawning. This process may take longer than usual, especially when running the program for the first time.

In another terminal:

```sh
ros2 run <your-package-name> task2_solution.py --ros-args --params-file <absolute-path-to-route-world-coordinates-yaml-file>
```

## Task Rules

* Be sure to publish just ONCE to the `/parc_robot/crop_yield` topic, AND at the END of the run. The run ends when the robot sends `finished` on the `/parc_robot/robot_status` topic.
* You are not allowed to publish to the `/robot_base_controller/cmd_vel_unstamped` topic. The robot will be driven through the field by the Gazebo simulation. You are only allowed to publish to the `/parc_robot/crop_yield` topic.
* You should publish the number of tomato fruits in the field to the `/parc_robot/crop_yield` topic not more than 5 seconds after the robot has stopped moving.

## Task Evaluation

Your solution will be evaluated based on the following criteria:

| S/N | Criteria/Metric | Description |
| ----------- | ----------- | ------- |
| 1 | Accuracy | Accuracy is based on how many weeds are correctly detected, within 0.1m of their actual location. Incorrect detections or missed weeds reduce accuracy. Multiple detections within 0.1m of the same actual location count as one accurate detection. |
| 2 | Robustness | We measure the robustness of your solution by evaluating its accuracy across various routes and speeds. The accuracy is given a weight and averaged across different speeds and all three routes to determine the overall robustness of your solution. |


!!! hint "Computing the X and Y values"
    To get the X and Y coordinates of the weeds, you might first get the position of weed in the robot frame, and then transform the position to the Gazbeo world frame. You can find more resources on frame transformations [here](/documentation-2023/resources-and-support/additional-transform-resources/).

