# Getting started with MATLAB

MATLAB is a programming language and numerical computing environment used by millions of engineers and scientists worldwide. It provides a powerful set of tools for analyzing data, developing algorithms, and creating models and simulations. 
Participants will need to use MATLAB to complete Task 2 of the competition.

!!! note "MATLAB For Docker Users"
    It's important to note that if you are using the Docker image provided by PARC, you can run MATLAB normally on your local machine. You do not need to install MATLAB inside the Docker container. For the purposes of this competition, you can use MATLAB on your local machine to complete the tasks.

## Overview of MATLAB and its Features

MATLAB is a high-level language and interactive environment that enables you to perform computationally intensive tasks. It includes a programming, visualization, and numerical computing environment that has become the standard for technical computing at leading engineering and science companies and the standard language for mathematics, computing, and data science.

MATLAB offers a number of benefits for engineers and scientists, including:

* Comprehensive numerical analysis tools
* Easy-to-use graphics and visualization capabilities
* A large community of users and support resources
* Compatibility with other programming languages and software tools

For ROS, MATLAB provides a set of tools for working with ROS topics, services, and actions. These tools allow you to publish and subscribe to ROS topics, call ROS services, and send and receive ROS actions. You can also use MATLAB to create and run ROS nodes, and to create and run ROS launch files.

!!! note "ROS Concepts"
    For more information about ROS nodes, topics, services, and actions, see the [Getting Started with ROS](/getting-started-tutorials/getting-started-with-ros/){:target="_blank"} documentation.

## Getting Started

### MATLAB Installation

!!! note "MATLAB Installation"
    As an official sponsor of PARC, MathWorks has provided a free, complimentary licence to all participating teams. To request the software, please visit the [MathWorks PARC page](https://www.mathworks.com/academia/student-competitions/PARC.html){:target="_blank"}.

### Starting MATLAB

To start MATLAB, open a terminal and type `matlab`. This will open the MATLAB desktop application. You can also start MATLAB by clicking the MATLAB icon on your desktop or in your start menu.

### MATLAB Command Window

The MATLAB command window is where you can enter commands and see the results. You can also use the command window to display the value of variables and to view the results of computations.

## Basic syntax and Data Types

### Basic MATLAB Syntax

MATLAB has a simple and intuitive syntax that is easy to learn and use. Here are some of the basic syntax rules:

* Statements are executed line by line.
* A semicolon (;) at the end of a statement suppresses output to the command window.
* Variables are created by assigning a value to them.
* Whitespace is ignored by MATLAB, so indentation is not necessary.

Here is an example of basic MATLAB syntax:

``` matlab
% This is a comment
a = 5; % Assign the value 5 to variable a
b = 2*a; % Assign the value 10 to variable b
disp(b); % Display the value of b to the command window
```

### Data Types
MATLAB supports a variety of data types, including:

* Numeric data types (integers, floating-point numbers, and complex numbers)
* Character and string data types
* Logical data types (true/false values)

Here are some examples of how to create and use these data types in MATLAB:

``` matlab
% Numeric data types
x = 5; % integer
y = 3.14159; % floating-point number
z = 2+3i; % complex number

% Character and string data types
c = 'a'; % character
s = 'Hello, world!'; % string

% Logical data types
p = true; % true value
q = false; % false value
r = (x > y); % logical expression (returns true or false)

% Displaying and manipulating data
disp(x); % display value of x
fprintf('The value of y is %f\n', y); % print formatted string
s2 = strcat(s, ' MATLAB is awesome!'); % concatenate strings
```

## Common Functions and Tools

MATLAB provides a rich collection of built-in functions and tools that enable you to perform various mathematical and engineering tasks. Here are some of the common MATLAB functions and tools that you might find useful:

### 1. Plotting and Visualization

MATLAB provides powerful tools for creating different types of plots, graphs, and charts. You can use the plot function to create 2D line plots, surf function to create 3D surface plots, imagesc function to create color-coded images, and many others. Here is an example of how to use the plot function to create a 2D line plot:

``` matlab
% Example code for creating a simple line plot
x = linspace(0, 10, 100);
y = sin(x);
plot(x, y)
```

### 2. Matrix Operations

MATLAB has built-in support for matrix and vector operations. You can perform element-wise operations, matrix multiplication, matrix inversion, and many others. Here are some examples:

``` matlab
% Example code for matrix operations
A = [1 2; 3 4];
B = [5 6; 7 8];
C = A + B;         % element-wise addition
D = A * B;         % matrix multiplication
E = inv(A);        % matrix inversion
F = A .* B;        % element-wise multiplication
G = A .^ 2;        % element-wise exponentiation
```

### 3. Image Processing

MATLAB provides a set of functions and tools for image processing. You can use the imresize function to resize an image, imrotate function to rotate an image, and many others. Here is an example of how to use the imresize function to resize an image:

``` matlab
% Example code for image processing
I = imread('image.jpg'); % read an image from a file
J = imresize(I, 0.5);    % resize the image by a factor of 0.5
imshow(J);               % display the resized image
```

### 4. ROS Integration

MATLAB provides a set of functions and tools for integrating with ROS. You can use the rossubscriber function to create a ROS subscriber, rospublisher function to create a ROS publisher, and many others. Here is an example of how to use the rossubscriber function to create a ROS subscriber and a ROS publisher:

=== "ROS Publisher Example"

    ``` matlab
    % Example code for ROS integration

    rosshutdown;                                % shutdown ROS
    rosinit;                                    % initialize ROS.
    pub = rospublisher('/chatter', 'std_msgs/String'); % create a ROS publisher
    count = 0;                                  % create a counter

    while true                                      % keep publishing forever
        msg = rosmessage(pub);                      % create a ROS message
        c = num2str(count);
        msg.Data = ['Hello, world!' ' ' c];         % assign a value to the message data
        send(pub, msg);                             % send the message
        count = count + 1;                          % increment counter
    end
    ```
    To test this, run the MATLAB code and then run the following command in a terminal to subscribe to the topic: `rostopic echo /chatter`
    
    You should see a similar output to this in the terminal:

    ``` bash
    data: 'Hello, world! 4230'
    ---
    ```
=== "ROS Subscriber Example"
    ``` matlab
    % Example code for ROS integration

    rosshutdown;                                % restart global node
    rosinit;                                    % initialize ROS
    sub = rossubscriber('/chatter');            % create a ROS subscriber
    msg = receive(sub, 3);                      % receive a message from the subscriber
    disp(msg.Data);                             % display the message data
    rosshutdown;                                % shutdown ROS
    ```
    To test this, run publish a message using the terminal command `rostopic pub /chatter std_msgs/String "data: 'Hello, world!'"` and then run the MATLAB code.

    You should see a similar output in the MATLAB command window:

    ``` matlab
    Hello, world! 213243
    ```

!!! note "Multiple Nodes"
    Kindly note that the MATLAB ROS integration is limited to a single node. This means that you can only create a single ROS publisher or a single ROS subscriber in MATLAB at a time. If you need to create multiple publishers and/or subscribers, you will need to start a separate MATLAB instance for each publisher and subscriber.

## Important MATLAB Resources
There are many resources available for learning MATLAB, including tutorials, online courses, and documentation. The MathWorks website provides a comprehensive set of resources, including:

<!-- * [Getting Started with MATLAB tutorial](https://www.mathworks.com/help/matlab/getting-started-with-matlab.html){:target="_blank"} -->
* [MATLAB OnRamp](https://matlabacademy.mathworks.com/details/matlab-onramp/gettingstarted) (Strongly Recommended)
* [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/){:target="_blank"} - a community of MATLAB users who can help you with your questions
* [MATLAB and Simulink ROS Tutorials](https://github.com/mathworks-robotics/matlab-and-simulink-ros-tutorials)

With these resources, you can quickly get up to speed with MATLAB and start using it for this competition and your own engineering and scientific projects.
