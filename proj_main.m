close all;
clear;
clc;
addpath('./CoppeliaSimConnection');

%% enable remote API
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

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

%% Get Conveyor, Ray Sensor data and 

%% Simulation 
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot);

q0 = [0 0 0 0 0 0];
q1 = [-pi/2 0 0 0 0 0];
q2 = [-pi/2 -pi/3 0 0 0 0];
q3 = [-pi/2 -pi/3 -pi/6 0 0 0];
q4 = [-pi/2 -pi/3 -pi/6 0 pi/2 0];


cnt= 0;
while true
    for i = 1:6
        vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), q0(i), vrep.simx_opmode_streaming);
    end
    pause(1);
    
    for i = 1:6
        vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), q1(i), vrep.simx_opmode_streaming);
    end
    
    pause(1);
    
    for i = 1:6
        vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), q2(i), vrep.simx_opmode_streaming);
    end
    pause(1);

    for i = 1:6
        vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), q3(i), vrep.simx_opmode_streaming);
    end
    pause(1);

    for i = 1:6
        vrep.simxSetJointTargetPosition(id, ur10.JointHandle(i), q4(i), vrep.simx_opmode_streaming);
    end
    pause(1);    
    
    controlSuction(id, 1);
end


vrep.simxFinish(id);
vrep.delete();