clear
clc

% Kinematics
L1 = Link([0 0 0 pi/2], 'standard'); 
L2 = Link([0 0.15 0.50 0], 'standard');
L3 = Link([0 0.15 0 -pi/2], 'standard');
L4 = Link([0 0 0.50 0], 'standard');
L5 = Link([0 0.15 0 pi/2], 'standard');
L6 = Link([0 0 0.15 0], 'standard');

% Definerer roboten
PaintRobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'RobotArm');

% Forwards kinematics med alle start vinkler for leddene lik 0, utstrakt arm
T = PaintRobot.fkine([0 0 0 0 0 0])
% Forwards kinematics for "kjøre" posisjon, med robotarmen "brettet" til siden for trygg kjøring
T2 = PaintRobot.fkine([pi/2 -pi/2 pi/2 0 -pi/2 0])

%leddposisjonsvektor
qn = [0 0.5655 -1.6336 0 0 1.0681]
PaintRobot.plot(qn);

T3 = PaintRobot.fkine(qn)
qi = PaintRobot.ikine(T3)
PaintRobot.plot(qi);

% Differential kinematics
J = PaintRobot.jacob0(qn)

joint_velocity = [0.1; 0.2; -0.1; 0.1; -0.05; 0.05];  

end_effector_velocity = J * joint_velocity
