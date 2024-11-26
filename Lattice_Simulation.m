clear
clc
% Define Vehicle
R = 0.1;    % Wheel radius [m]
L = 0.5;    % Wheelbase (distance between the wheels) [m]
dd = DifferentialDrive(R, L);  % Create a differential drive vehicle model

% Define start position and checkpoints
startposition = [0 0 0];
checkpoint_1 = [10 0 0];
checkpoint_2 = [10 0 pi/2];
checkpoint_3 = [10 10 pi/2];
checkpoint_4 = [10 10 pi];
checkpoint_5 = [0 10 pi];


map = binaryOccupancyMap(20, 20, 1);  % Create a 20x20 map with 1m resolution
setOccupancy(map, [5, 5], 1); 

% Add walls to the map
% Wall from (12, 0) to (12, 12)
for y = 0:12
    setOccupancy(map, [12, y], 1);  % Set occupancy for vertical wall
end
% Wall from (0, 12) to (12, 12)
for x = 0:12
    setOccupancy(map, [x, 12], 1);  % Set occupancy for horizontal wall
end

lp = Lattice(map); 
lp.plan('iterations', 20);  % Create roadmaps
viz = Visualizer2D;
viz.hasWaypoints = true;

% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:50;         % Adjust time array for navigation
pose = zeros(3, numel(tVec));   % Pose matrix
pose(:,1) = startposition';     % Initialize pose at start

controller = controllerPurePursuit;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1000000;

% First stage: Navigate to checkpoint 1
disp('Navigating to checkpoint 1...');
path_1 = lp.query(startposition, checkpoint_1);  % Find path to checkpoint 1
controller.Waypoints = path_1(:, [1, 2]);        % Set the waypoints

% Plot the map and the path
figure;
hold on;
show(map);  % Display the occupancy grid map
plot(path_1(:,1), path_1(:,2), 'r--', 'LineWidth', 2);  % Plot path to checkpoint 1

% Execute path following for first stage
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    plot(pose(1,idx), pose(2,idx), 'bo');  % Plot robot's current position

    % Check if reached checkpoint 1
    if norm(pose(1:2, idx) - checkpoint_1(1:2)') < 0.1
        disp('Reached checkpoint 1.');
        break;  % Exit the loop once checkpoint 1 is reached
    end

    waitfor(r);
end

% Update the initial pose to checkpoint 1 for the next leg
pose(:,1) = checkpoint_1';

% Second stage: Navigate to checkpoint 2
disp('Navigating to checkpoint 2...');
path_2 = lp.query(checkpoint_1, checkpoint_2);  % Find path to checkpoint 2
controller.Waypoints = path_2(:, [1, 2]);       % Set the waypoints for next leg

% Plot the path to checkpoint 2
plot(path_2(:,1), path_2(:,2), 'g--', 'LineWidth', 2);  % Plot path to checkpoint 2

% Execute path following for second stage
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    % Compute the velocities
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1));  % Convert from body to world

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    plot(pose(1,idx), pose(2,idx), 'bo');  % Plot robot's current position

    % Check if reached checkpoint 2
    if norm(pose(1:2, idx) - checkpoint_2(1:2)') < 0.1
        disp('Reached checkpoint 2.');
        break;  % Exit the loop once checkpoint 2 is reached
    end

    waitfor(r);
end

% Update the initial pose to checkpoint 1 for the next leg
pose(:,1) = checkpoint_2';

% Second stage: Navigate to checkpoint 2
disp('Navigating to checkpoint 3...');
path_3 = lp.query(checkpoint_2, checkpoint_3);  % Find path to checkpoint 3
controller.Waypoints = path_3(:, [1, 2]);       % Set the waypoints for next leg

% Plot the path to checkpoint 3
plot(path_3(:,1), path_3(:,2), 'g--', 'LineWidth', 2);  % Plot path to checkpoint 3

% Execute path following for second stage
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    % Compute the velocities
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1));  % Convert from body to world

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    % Update plot with the current pose
    plot(pose(1,idx), pose(2,idx), 'bo');  % Plot robot's current position

    % Check if reached checkpoint 3
    if norm(pose(1:2, idx) - checkpoint_3(1:2)') < 0.1
        disp('Reached checkpoint 3.');
        break;  % Exit the loop once checkpoint 3 is reached
    end

    waitfor(r);
end

% Update the initial pose to checkpoint 3 for the next leg
pose(:,1) = checkpoint_3';

% Second stage: Navigate to checkpoint 4
disp('Navigating to checkpoint 4...');
path_4 = lp.query(checkpoint_3, checkpoint_4);  % Find path to checkpoint 4
controller.Waypoints = path_4(:, [1, 2]);       % Set the waypoints for next leg

% Plot the path to checkpoint 4
plot(path_4(:,1), path_4(:,2), 'g--', 'LineWidth', 2);  % Plot path to checkpoint 4

% Execute path following for second stage
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    % Compute the velocities
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1));  % Convert from body to world

    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    % Update plot with the current pose
    plot(pose(1,idx), pose(2,idx), 'bo');  % Plot robot's current position

    % Check if reached checkpoint 4
    if norm(pose(1:2, idx) - checkpoint_4(1:2)') < 0.1
        disp('Reached checkpoint 4.');
        break;  % Exit the loop once checkpoint 4 is reached
    end

    waitfor(r);
end

% Update the initial pose to checkpoint 4 for the next leg
pose(:,1) = checkpoint_4';

% Second stage: Navigate to checkpoint 5
disp('Navigating to checkpoint 5...');
path_5 = lp.query(checkpoint_4, checkpoint_5);  % Find path to checkpoint 5
controller.Waypoints = path_5(:, [1, 2]);       % Set the waypoints for next leg

% Plot the path to checkpoint 5
plot(path_5(:,1), path_5(:,2), 'g--', 'LineWidth', 2);  % Plot path to checkpoint 5

% Execute path following for second stage
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    % Compute the velocities
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1));  % Convert from body to world

    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    % Update plot with the current pose
    plot(pose(1,idx), pose(2,idx), 'bo');  % Plot robot's current position

    % Check if reached checkpoint 5
    if norm(pose(1:2, idx) - checkpoint_5(1:2)') < 0.1
        disp('Reached checkpoint 5.');
        break;  % Exit the loop once checkpoint 5 is reached
    end

    waitfor(r);
end



hold off;
