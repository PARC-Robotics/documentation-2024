# Additional MATLAB Resources

## Publishing to the `/parc_robot/weed_detection` topic

The `/parc_robot/weed_detection` topic expects a JSON array string of weed locations. The array should be an `n x 2` array of X and Y values. Here is an example MATLAB script to publish to the `/parc_robot/weed_detection` topic:

```matlab
rosshutdown
rosinit             % Create a ROS node

% Create a publisher to the /parc_robot/weed_detection topic
pub = rospublisher('/parc_robot/weed_detection', 'std_msgs/String');

% Define the weed locations as a nx2 array of X and Y values
weed_locations = [
    0.5, 0.5;
    0.5, 0.6;
    0.5, 0.7;
    0.5, 0.8;
    -1.2, 3.25;  % Note: These are just example values.
    1.1, 4.0;    % Your actual code will need to detect the weeds in the field.
    ];

% Convert the weed locations to a JSON array string
json_str = jsonencode(weed_locations);

% Create a String message
msg = rosmessage('std_msgs/String');

% Set the message data to the JSON array string
msg.Data = json_str;

% Publish the message
send(pub, msg);

% Stop the ROS node
rosshutdown
```

## Subscribing to the `/parc_robot/robot_status` topic

The `/parc_robot/robot_status` topic publishes the current status of the robot. The message type for this topic is `/std_msgs/String`, which indicates whether the robot has started moving along the route or has finished the designated route. The robot status has two possible values: "started" and "finished".

```matlab
rosshutdown
rosinit                % Create a ROS node

% Create a subscriber to the /parc_robot/robot_status topic. The callback function is called when a message is received.
sub = rossubscriber('/parc_robot/robot_status', @callbackFcn, 'DataFormat', 'struct');

% Wait for the robot to stop moving
msg = receive(sub, 10);

% Stop the ROS node
rosshutdown
```
