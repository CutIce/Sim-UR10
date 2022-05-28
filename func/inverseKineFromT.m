function q = inverseKineFrom(T)
%     [robot, DHpara, axis] = model();
%     l1 = 0.1273;
%     l2 = 0.612;
%     l3 = 0.5723;
%     l4 = 0.163941;
%     l5 = 0.1157;
%     l6 = 0.0922;
%     
%     p1 = [0 0 0]';
%     p2 = [0 0 l1]';
%     p3 = [0 0 l1 + l2]';
%     p4 = [0 0 l1 + l2 + l3]';
%     p5 = [0 l4 l1+l2+l3]';
%     p6 = [0 l4 l1+l2+l3+l5]';
%     p = [p1 p2 p3 p4 p5 p6];
    q = zeros(1, 6);
    [robot, DHpara, axis] = model();
    [d1; d2; d3; d4; d5; d6] = DHpara(:, 2);
    [a1; a2; a3; a4; a5; a6] = DHpara(:, 3);
    
    [nx; ny; nz] = T(1:3, 1);
    [ox; oy; oz] = T(1:3, 2);
    [ax; ay; az] = T(1:3, 3);
    [px; py; pz] = T(1:3, 4);
    
    %% calc theta1
    m = d6 * ay - py;
    n = ax * d6 - px;
    if m^2 + n^2 - d4^2 < 0
        fprintf("Solve q1 Error\n")
        return
    end
    theta1(1) = atan2(m, n) - atan2(d4, sqrt(m^2 + n^2 - d4^2));
    theta1(2) = atan2(m, n) - atan2(d4, -sqrt(m^2 + n^2 - d4^2));
    
    %% calc theta5
    num = 1;
    for i = 1:length(theta1)
        th1 = theat1(i);
        if abs(ax * sin(th1) - ay * cos(th1)) <= 1
            theta5(num) =  acos(ax * sin(th1) - ay * cos(th1));
            num = num + 1;
            theta5(num) = -acos(ax * sin(th1) - ay * cos(th1));
            num = num + 1;
        end
    end
    
    %% calc theta6
    num = 1;

    
end