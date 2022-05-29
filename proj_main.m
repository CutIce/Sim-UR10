close all;
clear;
clc;
addpath('./CoppeliaSimConnection');

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
    vrchk(vrep, res);
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
max_placed_boxes = 8;

q_seq = [q0; q_suck_pre; q_suck; q4; q0];

n = 50;
while max_placed_boxes > 0
    [res, detect_state, detected_point, ~, ~] = vrep.simxReadProximitySensor(id, conveyor_sensor, vrep.simx_opmode_blocking);
    if detect_state
        workstate = 1;
    else
        workstate = 0;
    end
    
    if workstate
        for k = 1:6
            vrep.simxSetJointTargetPosition(id, ur10.JointHandle(k), q_suck_pre(k), vrep.simx_opmode_blocking);
        end 
        pause(2);
        
        for k = 1:6
            vrep.simxSetJointTargetPosition(id, ur10.JointHandle(k), q_suck(k), vrep.simx_opmode_blocking);
        end
        pause(1.2);
        controlSuction(id, 1);

        
        for k = 1:6
            vrep.simxSetJointTargetPosition(id, ur10.JointHandle(k), q4(k), vrep.simx_opmode_blocking);
        end 
        pause(2.5);
        controlSuction(id, 0);
        
        for k = 1:6
            vrep.simxSetJointTargetPosition(id, ur10.JointHandle(k), q0(k), vrep.simx_opmode_blocking);
        end 
        pause(2);
    end
    
    max_placed_boxes = max_placed_boxes - 1;
%     traj = jtraj(q_seq(1, :), q_seq(2, :), n);
%     for s = 1:n
%         for k = 1:6
%             vrep.simxSetJointTargetPosition(id, ur10.JointHandle()
%         end
%     end
    
%     workstate
%     if workstate == 1
%         for k = 1:4
%             traj = jtraj(q_seq(k, :), q_seq(k+1, :), n);
%             for s = 1:n
%                 for j = 1:6
%                     vrep.simxSetJointTargetPosition(id, ur10.JointHandle(j), traj(s, j), vrep.simx_opmode_blocking);
%                 end
%             end
%         end
%     end
    
%     for i = 1:6
%         vrep.simxSetJointPosition(id, ur10.JointHandle(i), q_suck_pre(i), vrep.simx_opmode_blocking);
%     end
%     pause(3);
% 
%     for i = 1:6
%         vrep.simxSetJointPosition(id, ur10.JointHandle(i), q_suck(i), vrep.simx_opmode_blocking);
%     end
%     pause(3);
%     
%     for i = 1:6
%         vrep.simxSetJointPosition(id, ur10.JointHandle(i), q4(i), vrep.simx_opmode_blocking);
%     end
%     pause(3);
%     
%     [res, detect_state, detected_point, ~, ~] = vrep.simxReadProximitySensor(id, conveyor_sensor, vrep.simx_opmode_blocking);
%     
%     for i = 1:6
%         if detect_state 
%             target = q_suck_pre;
%         else
%             target = q0;
%         end
%             
%         vrep.simxSetJointPosition(id, ur10.JointHandle(i), target(i), vrep.simx_opmode_blocking);
%     end
%     pause(3);
%     
%     [traj, ~, ~] = jtraj(q0, q_suck_pre, n);
%     for k = 1:n
%         for i = 1:6
%             vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), traj(k, i), vrep.simx_opmode_blocking);
%         end
%     end
% %     pause(2);
% 
%     [traj, ~, ~] = jtraj(q_suck_pre, q_suck, n);
%     for k = 1:n
%         for i = 1:6
%             vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), traj(k, i), vrep.simx_opmode_blocking);
%         end
%     end
% 
%     controlSuction(id, 1);
%     pause(0.2);
% 
%     [traj, ~, ~] = jtraj(q_suck, q4, n);
%     for k = 1:n
%         for i = 1:6
%             vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), traj(k, i), vrep.simx_opmode_blocking);
%         end
%     end
% 
% %     pause(10);
%     controlSuction(id, 0);
%     
% 
%     [traj, ~, ~] = jtraj(q4, q0, n);
%     for k = 1:n
%         for i = 1:6
%             vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), traj(k, i), vrep.simx_opmode_blocking);
%         end
%     end
% %     pause(2);
% 
%     max_placed_boxes = max_placed_boxes - 1;
end

vrep.simxFinish(id);
vrep.delete();