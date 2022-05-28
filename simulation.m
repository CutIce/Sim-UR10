close all;
clear;
clc;
addpath('./CoppeliaSimConnection');

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID < -1
    fprintf("Failed to connect to remote API server\n");
    vrep.delete();
    return;
end


fprintf("Successfully connect to remote API server\n");

robot_ur10 = struct('clientID', clientID);
jointNames={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4','UR10_joint5','UR10_joint6'};

% Get Joint Handlers of UR10
jointHandle = -ones(1, 6);
for i = 1:6
    [res, jointHandle(i)] = vrep.simxGetObjectHandle(clientID, jointNames{i}, vrep.simx_opmode_blocking);
    vrchk(vrep, res);
end
robot_ur10.JointHandle = jointHandle;
[~, j1] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', vrep.simx_opmode_blocking);
[~, j2] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', vrep.simx_opmode_blocking);


vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

q0 = [0 0 0 0 0 0];
q1 = pi/18 * [18 0 0 0 0 0];
q2 = pi/18 * [9 1 2 12 4 3];

while true
    for i = 1:6
        vrep.simxSetJointTargetPosition(clientID, robot_ur10.JointHandle(i), q0(i), vrep.simx_opmode_streaming);
    end
    pause(5);
%     gripper(clientID, 0, j1, j2); pause(1.5);
    
%     gripper(clientID, 1, j1, j2); pause(1.5);
    
    
    for i = 1:6
        vrep.simxSetJointTargetPosition(clientID, robot_ur10.JointHandle(i), q1(i), vrep.simx_opmode_streaming);
    end
    
    pause(5);
%     gripper(clientID, 1, j1, j2)
    
    for i = 1:6
        vrep.simxSetJointTargetPosition(clientID, robot_ur10.JointHandle(i), q2(i), vrep.simx_opmode_streaming);
    end
    pause(5);
end


vrep.simxFinish(clientID);
vrep.delete();