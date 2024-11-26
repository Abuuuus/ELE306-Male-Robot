clear
clc

robotNode = ros2node("/ArmTrajectoryController", 30);

jointTrajectoryPub = ros2publisher(robotNode, '/robot_manipulator_controller/commands', 'std_msgs/Float64MultiArray');

pause(1); 

% Define start and end positions
startPositions1 = [0.05, 0, -pi/2, pi, -4*pi/5.5, -pi, 0];
endPositions1 = [1, -0.5, 0, pi/4, pi/12, pi/12, 0];

numSteps = 75;  % Increase this for smoother, slower transitions
pauseTime = 0.05;  % Adjust this for faster or slower movements

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions1(i), endPositions1(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions2 = endPositions1;
endPositions2 = [0.05, -0.5, pi/4, pi/2, pi/12, pi/12, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions2(i), endPositions2(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions3 = endPositions2;
endPositions3 = [1, -0.5, pi/2, pi/4, pi/12, pi/12, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions3(i), endPositions3(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions4 = endPositions3;
endPositions4 = [0.05, -0.5, pi/4, pi/2, pi/12, pi/12, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions4(i), endPositions4(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions5 = endPositions4;
endPositions5 = [1, 0, pi/2, pi/4, pi/12, pi/12, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions5(i), endPositions5(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions6 = endPositions5;
endPositions6 = [0.05, 0, pi/4, pi/2, pi/12, pi/12, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions6(i), endPositions6(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end

pause(1)

startPositions7 = endPositions6;
endPositions7 = [0.05, 0, -pi/2, pi, -4*pi/5.5, -pi, 0];

% Create interpolated trajectory
interpolatedPositions = zeros(7, numSteps);
for i = 1:7
    interpolatedPositions(i, :) = linspace(startPositions7(i), endPositions7(i), numSteps);
end

% Publish each interpolated position
for step = 1:numSteps
    % Create and populate the message
    trajectoryMsg = ros2message(jointTrajectoryPub);
    trajectoryMsg.layout.dim = ros2message("std_msgs/MultiArrayDimension"); % Clear layout dimensions
    trajectoryMsg.data = interpolatedPositions(:, step);  % Set interpolated position
    
    % Send the position command
    send(jointTrajectoryPub, trajectoryMsg);
    
    % Pause between commands to control movement speed
    pause(pauseTime);
end