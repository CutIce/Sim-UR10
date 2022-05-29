function controlSuction(id, isSuck)
    addpath('./CoppeliaSimConnection');
    vrep = remApi('remoteApi');
    vrep.simxSetIntegerSignal(id, 'suctionPad_active', isSuck, vrep.simx_opmode_oneshot);
end