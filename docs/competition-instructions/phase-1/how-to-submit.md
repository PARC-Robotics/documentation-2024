# How to Submit

Teams are expected to develop their solutions (ROS packages) in a 'solutions' folder inside the `~/catkin_ws/src` directory. You may have one or more ROS packages in this folder for all your tasks. 

See figure below of expected directory structure:

```
~/catkin_ws/src
├── CMakeLists.txt
├── parc-engineers-league
│   ├── parc-robot
│   │   ├── .
│   │   ├── .
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── .
│   └── .
└── <YOUR_SOLUTION_FOLDER>        # Zip this folder and submit
    ├── <your_ros_package1>
    │   ├── .
    │   ├── .
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── <your_ros_package2>
    │   ├── .
    │   ├── .
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── .
    ├── .
    ├── .
    ├── .
    └── README.md                   # Required
```

1. Prepare a README.md file following this format and store in solution folder (see [example](https://github.com/PARC-Robotics/PARC-Engineers-League/blob/master/resources/sample-submission-readme.md)):
    * Introduction Section: Briefly describe your approach
    * Dependencies: List all the packages installed and used in your solution
    * Task 1 - 2 description and run command(s)
    * Challenges faced

2. Include all the packages (dependencies) used in your solution in your package's "package.xml" file ([see guide](http://wiki.ros.org/rosdep/Tutorials/How%20to%20add%20a%20system%20dependency))

3. Create simple short video demos of your solutions for Task 1 and 2. This can be done by taking a screen recording of your solution running in Gazebo. Please ensure the videos are less than 200 MB in size.

4. Zip your solution folder and upload the folder and the videos on the [solution submission form](https://forms.gle/GwE7Tzm9FpYzUVQX9)
