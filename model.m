function [robot, DHpara, axis] = model()
    theta1 = 0; d1 = 127.3;    a1 = 0;        alpha1 = pi/2;       off1 = pi;
    theta2 = 0; d2 = 0;        a2 = -612;     alpha2 = 0;          off2 = -pi/2;
    theta3 = 0; d3 = 0;        a3 = -572.3;   alpha3 = 0;          off3 = 0;
    theta4 = 0; d4 = 163.941;  a4 = 0;        alpha4 = pi/2;       off4 = -pi/2;
    theta5 = 0; d5 = 115.7;    a5 = 0;        alpha5 = -pi/2;      off5 = 0;
    theta6 = 0; d6 = 092.2;    a6 = 0;        alpha6 = 0;          off6 = pi/2;

    g = [0 -9.81 0]';
    DHpara = [theta1  d1   a1  alpha1 off1;
              theta2  d2   a2  alpha2 off2;
              theta3  d3   a3  alpha3 off3;
              theta4  d4   a4  alpha4 off4;
              theta5  d5   a5  alpha5 off5;
              theta6  d6   a6  alpha6 off6;
    ];

    axis = [
        0 0 1;
        0 1 0;
        0 1 0;
        0 1 0;
        0 0 -1;
        0 1 0;];
    axis = axis';
    

    m = [7.1, 12.7, 4.27, 2, 2, 0.365];
    r = [
        0.021 0.000 0.027;
        0.38  0.000 0.158;
        0.240 0.000 0.068;
        0.000 0.007 0.018;
        0.000 0.007 0.018;
        0     0    -0.026;    ];
    
    
    for i = 1:6
        L(i) = Revolute('d', DHpara(i, 2), 'a', DHpara(i, 3), 'alpha', DHpara(i, 4), 'offset', DHpara(i, 5), 'standard', 'I', eye(3), 'm', m(i), 'r', r(i, :));
        L(i).Jm = 0;
        L(i).nofriction();
    end
    robot = SerialLink(L, 'name', "robot");
    robot.gravity = g;
end