function gripper(id, isClose, j1, j2)
    addpath('./CoppeliaSimConnection');
    vrep = remApi('remoteApi');

[~, p1] = vrep.simxGetJointPosition(id, j1, vrep.simx_opmode_blocking);
[~, p2] = vrep.simxGetJointPosition(id, j2, vrep.simx_opmode_blocking);


if isClose == 1
    if p1 < (p2-0.008)
        vrep.simxSetJointTargetVelocity(id, j1, -0.01, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(id, j2, -0.04, vrep.simx_opmode_blocking);
    else
        vrep.simxSetJointTargetVelocity(id, j1, -0.04, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(id, j2, -0.04, vrep.simx_opmode_blocking);
    end
else
    if p1 < p2
        vrep.simxSetJointTargetVelocity(id, j1, 0.04, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(id, j2, 0.02, vrep.simx_opmode_blocking);
    else
        vrep.simxSetJointTargetVelocity(id, j1, 0.02, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(id, j2, 0.04, vrep.simx_opmode_blocking);
    end
end

end