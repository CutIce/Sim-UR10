close all;
clear;
clc;
addpath('./connectCoppeliaSim/');
addpath('./planning/');
addpath('./controlCoppeliaSim/');
%%  Get  points and  prepare points
[robot, para, axis]= model();
gap_x = 200;
gap_y = 0;
start_x = -900;
start_y = 400;
format short
% loose place
T0 = double(robot.fkine([0 0 0 0 0 0]));
T_suck = [
    -1      0       0       650;
     0      1       0       400;
     0      0       -1      810;
     0      0       0         1;
];

T_suck_pre = [
    -1      0       0       650;
     0      1       0       400;
     0      0       -1      1000;
     0      0       0         1;
];

for i = 0:7
    px = start_x + gap_x * floor(i/2);
    py = start_y + gap_y * mod(i, 2);
    pz = 180 + 150 * mod(i, 2);
    pz_pre = pz + 50;
    R = [
        0   -1   0;
        -1   0   0;
        0    0   -1;
    ];
    
    T_loose(:, :, i+1) = [
        R [px py pz]';
        0 0 0 1;
    ];

    T_loose_pre(:, :, i+1) = [
        R [px py pz_pre]';
        0 0 0 1;
    ];
    q_loose_pre(i+1, :) = robot.ikunc(T_loose_pre(:, :, i+1));
    q_loose(i+1, :) = robot.ikunc(T_loose(:, :, i+1));
end



%% enable remote API
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot);
%% check connection 
if id < -1
    fprintf("Failed to connect to remote API server\n");
    vrep.delete();
    return;
end
fprintf("Successfully connect to remote API server\n");

%% Get UR10 Joints' Handle
ur10 = struct('id', id);
jointNames={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4','UR10_joint5','UR10_joint6'};
jointHandle = -ones(1, 6);
for i = 1:6
    [res, jointHandle(i)] = vrep.simxGetObjectHandle(id, jointNames{i}, vrep.simx_opmode_blocking);
%     vrchk(vrep, res);
end
ur10.JointHandle = jointHandle;

%% Get Conveyor, Ray Sensor data and SunctionPad
[res, suction_pad] = vrep.simxGetObjectHandle(id, 'suctionPad', vrep.simx_opmode_blocking);
[res, conveyor_sensor]  = vrep.simxGetObjectHandle(id, 'conveyor_sensor', vrep.simx_opmode_blocking);

%% Simulation 


q0 = [0 0 0 0 0 0];
q_suck_pre = [0.335164944401056   0.361596942813198   0.445077830622957   0.764121541604784  -1.570796305367679   0.335164837572886];
q_suck     = [0.335165861538509   0.137551720081628   1.127653467370393   0.305591217970581  -1.570797279014585   0.335165484016708];
q3 = [-pi/2 -pi/3 -pi/6 0 0 0];
q4 = [-pi/2 -pi/3 -pi/6 0 pi/2 0];

workstate = 0;
placed_boxex = 0;

q_seq = [q0; q_suck_pre; q_suck; q4; q0];

n1 = 200;
n2 = 100;
while placed_boxex < 8
    [res, detect_state, detected_point, ~, ~] = vrep.simxReadProximitySensor(id, conveyor_sensor, vrep.simx_opmode_blocking);
    if detect_state
        workstate = 1;
    else
        workstate = 0;
    end
    
    workstate
    
    moveLine(id, T0, T_suck_pre, ur10, 30, 0);
    moveLine(id, T_suck_pre, T_suck, ur10, 10, 0);
    controlSuction(id, 1);pause(0.1);
    moveLine(id, T_suck, T_loose_pre(:, :, placed_boxex+1), ur10, 30, 0);
    moveLine(id, T_loose_pre(:, :, placed_boxex+1), T_loose(:, :, placed_boxex+1), ur10, 10, 0);
    controlSuction(id, 0);pause(0.1);
    
    [res, detect_state, detected_point, ~, ~] = vrep.simxReadProximitySensor(id, conveyor_sensor, vrep.simx_opmode_blocking);
    if ~detect_state
        moveLine(id, T_loose(:, :, placed_boxex+1), T0, ur10, 30, 0);
    end
    
    
    
    
% %     if workstate
%         fprintf("enter! prepare to move!\n");
%         traj = jtraj(q0, q_suck_pre, n1);
%         for x = 1:n1
%             for k = 1:6
%                 vrep.simxSetJointPosition(id, ur10.JointHandle(k), traj(x, k), vrep.simx_opmode_blocking);
%             end
%         end
%         
%         traj = jtraj(q_suck_pre, q_suck, n2);
%         for x = 1:n2
%             for k = 1:6
%                 vrep.simxSetJointPosition(id, ur10.JointHandle(k), traj(x, k), vrep.simx_opmode_blocking);
%             end
%         end
%         controlSuction(id, 1);
% 
%         traj = jtraj(q_suck, q_loose_pre(placed_boxex+1, :), n1);
%         for x = 1:n1
%             for k = 1:6
%                 vrep.simxSetJointPosition(id, ur10.JointHandle(k), traj(x, k), vrep.simx_opmode_blocking);
%             end
%         end
% 
%         traj = jtraj(q_loose_pre(placed_boxex+1, :), q_loose(placed_boxex+1, :), n2);
%         for x = 1:n2
%             for k = 1:6
%                 vrep.simxSetJointPosition(id, ur10.JointHandle(k), traj(x, k), vrep.simx_opmode_blocking);
%             end
%         end
%         controlSuction(id, 0);
%         
%         traj = jtraj(q_loose(placed_boxex+1, :), q0, n1);
%         for x = 1:n1
%             for k = 1:6
%                 vrep.simxSetJointPosition(id, ur10.JointHandle(k), traj(x, k), vrep.simx_opmode_blocking);
%             end
%         end
%         
%         placed_boxex = placed_boxex + 1;
% %     end
    
end
fprintf("Finish\n")
vrep.simxFinish(id);
vrep.delete();