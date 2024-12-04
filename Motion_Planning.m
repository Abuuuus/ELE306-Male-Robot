clear
L1 = Link([0 0 0 pi/2], 'standard'); 
L2 = Link([0 0.15 0.50 0], 'standard');
L3 = Link([0 0.15 0 -pi/2], 'standard');
L4 = Link([0 0 0.50 0], 'standard');
L5 = Link([0 0.15 0 pi/2], 'standard');
L6 = Link([0 0 0.15 0], 'standard');

% Definer roboten
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'RobotArm');

% Sett startposisjon - for eksempel [0, 0, 0]'
%q_StartPos = [0 1.068 -4*pi/5 -pi/2 pi/2 1.4451]
%q_StartPos = [pi/2 0.5655 -1.6336 0 0 1.0681];
%T3 = robot.fkine(q_StartPos)
qn_SleepPos = [pi/2 -pi/2 pi/2 0 -pi/2 0]
T2_SleepPos = robot.fkine(qn_SleepPos);

T3_StartPos = [1 0 0 0.7; 0 0 -1 -0.5; 0 1 0 -0.1; 0 0 0 1];
qi_StartPos = robot.ikine(T3_StartPos)

% Definer ønsket sluttposisjon [x, y, z]
T4_StartPaintPos = [1 0 0 0.65; 0 0 -1 -0.5; 0 1 0 -0.7; 0 0 0 1];
T5_PaintPos2 = [1 0 0 0.65; 0 0 -1 -0.5; 0 1 0 0.5; 0 0 0 1];
T6_PaintPos3 = [1 0 0 0.65; 0 0 -1 0; 0 1 0 0.5; 0 0 0 1];
T7_PaintPos4 = [1 0 0 0.65; 0 0 -1 0; 0 1 0 -0.7; 0 0 0 1];

% Posisjoner for venstre side av maling
% T8_PaintPos5 = [1 0 0 0.65; 0 0 -1 0.5; 0 1 0 -0.7; 0 0 0 1];
% T9_StopPaintPos = [1 0 0 0.65; 0 0 -1 0.5; 0 1 0 0.5; 0 0 0 1];

qi2_StartPaintPos = robot.ikine(T4_StartPaintPos);
qi3_PaintPos2 = robot.ikine(T5_PaintPos2);
qi4_PaintPos3 = robot.ikine(T6_PaintPos3);
qi5_PaintPos4 = robot.ikine(T7_PaintPos4);
% qi6_PaintPos5 = robot.ikine(T8_PaintPos5);
% qi7_StopPaintPos6 = robot.ikine(T9_StopPaintPos);

% Safety pos til male pos
Kurs = jtraj(qn_SleepPos, qi_StartPos, 50);

% Male posisjoner 
Kurs2 = ctraj(T3_StartPos, T4_StartPaintPos, 50);
Kurs3 = ctraj(T4_StartPaintPos, T5_PaintPos2, 50);
% Knekker starter her under maling
Kurs4 = ctraj(T5_PaintPos2, T6_PaintPos3, 50);
Kurs5 = ctraj(T6_PaintPos3, T7_PaintPos4, 50);

% Venstre side av malingen som ikke fungerer
%Kurs6 = ctraj(T7_PaintPos4, T8_PaintPos5, 50);
%Kurs7 = ctraj(T8_PaintPos5, T9_StopPaintPos, 50);

% Initialize a matrix to store joint configurations
num_points = size(Kurs2, 3);
joint_trajectory = zeros(num_points, length(robot.links));
num_points2 = size(Kurs3, 3);
joint2_trajectory = zeros(num_points2, length(robot.links));
num_points3 = size(Kurs4, 3);
joint3_trajectory = zeros(num_points3, length(robot.links));
num_points4 = size(Kurs5, 3);
joint4_trajectory = zeros(num_points4, length(robot.links));

% Tilhører venstre side av malingen
% num_points5 = size(Kurs6, 3);
% joint5_trajectory = zeros(num_points5, length(robot.links));
% num_points6 = size(Kurs7, 3);
% joint6_trajectory = zeros(num_points6, length(robot.links));

% ikcon	inverse kinematics using optimisation with joint limits?

% Metode for plot av alle stegene
for i = 1:50
    robot.plot(Kurs(i, :));
end
for i = 1:num_points
    T = Kurs2(:, :, i);                    % Get the i-th pose
    joint_trajectory(i, :) = robot.ikine(T,qi2_StartPaintPos); % Solve IK for each pose
end
robot.plot(joint_trajectory)
for i = 1:num_points2
    T2 = Kurs3(:, :, i);                    
    joint2_trajectory(i, :) = robot.ikine(T2,qi3_PaintPos2); 
end
robot.plot(joint2_trajectory)
for i = 1:num_points3
    T3 = Kurs4(:, :, i);                    
    joint3_trajectory(i, :) = robot.ikine(T3,qi4_PaintPos3); 
end
robot.plot(joint3_trajectory)
for i = 1:num_points4
    T4 = Kurs5(:, :, i);                   
    joint4_trajectory(i, :) = robot.ikine(T4,qi5_PaintPos4); 
end
robot.plot(joint4_trajectory)

% Plot for venstre side som ikke fungerer
% for i = 1:num_points5
%     T5 = Kurs6(:, :, i);                
%     joint5_trajectory(i, :) = robot.ikine(T5); 
% end
% robot.plot(joint5_trajectory)
% for i = 1:num_points6
%     T6 = Kurs7(:, :, i);                
%     joint6_trajectory(i, :) = robot.ikine(T6); 
% end
% robot.plot(joint6_trajectory)
