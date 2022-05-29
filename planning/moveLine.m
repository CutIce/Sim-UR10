function moveLine(id, Ts, Tf, ur10, n, speed)
    addpath("../connectCoppeliaSim/")
    addpath("../func/");
    vrep = remApi('remoteApi');
    
    ps = Ts(1:3, 4); pf = Tf(1:3, 4);
    dist_vec = pf - ps;
    if speed ~= 0
        n = ceil(norm(dist_vec / speed));
    end
    
    Rs = Ts(1:3, 1:3);
    Rf = Tf(1:3, 1:3);
    Rsf = Rs' * Rf;
    thetaf = acos(( trace(Rsf) - 1) / 2);
    rf = 1 / (2*sin(thetaf)) .* [Rsf(3, 2) - Rsf(2, 3); Rsf(1, 3) - Rsf(3, 1); Rsf(2, 1) - Rsf(1, 2)];

    dlt_thetaf = thetaf  / round(n, 3);
    
    [robot, ~, ~] = model();
    for x = 0:1:n
    %% interpolate pos
        p = ps + x / n * dist_vec;
    %% interpolate pose
        R = axis2rotm(thetaf * x / n, rf);
        
        T = [
            R     p;
            0 0 0 1;
        ];
    
        q = robot.ikunc(T);
        for i = 1:6
            vrep.simxSetJointPosition(id, ur10.JointHandle(i), q(i), vrep.simx_opmode_blocking);
        end
    end
end