clear
clc

% Initialize node and publisher
test_publisher = ros2node("/test_vm_ros", 30);
cmdPub = ros2publisher(test_publisher, "/cmd_vel", "geometry_msgs/Twist");

% Define velocity message for straight-line motion
cmdMsg = ros2message(cmdPub);
cmdMsg.linear.x = -0.3;      % Set forward velocity for straight movement
cmdMsg.angular.z = 0.0;     % Set angular velocity to zero for no turning

% Send command in a loop to drive forward
for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end
cmdMsg.linear.x = 0.3;
cmdMsg.angular.z = 0.5;
for cnt = 1:5
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end

cmdMsg.linear.x = -0.3;      % Set forward velocity for straight movement
cmdMsg.angular.z = 0.0;     % Set angular velocity to zero for no turning

for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end

cmdMsg.linear.x = 0.3;
cmdMsg.angular.z = 0.5;
for cnt = 1:5
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end

cmdMsg.linear.x = -0.3;      % Set forward velocity for straight movement
cmdMsg.angular.z = 0.0;     % Set angular velocity to zero for no turning

for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end

cmdMsg.linear.x = 0.3;
cmdMsg.angular.z = 0.5;
for cnt = 1:5
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end

cmdMsg.linear.x = -0.3;      % Set forward velocity for straight movement
cmdMsg.angular.z = 0.0;     % Set angular velocity to zero for no turning

for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);               % Adjust pause based on desired control frequency
end
% Stop the robot after the loop
cmdMsg.linear.x = 0.0;
cmdMsg.angular.z = 0.0;  
send(cmdPub, cmdMsg);
