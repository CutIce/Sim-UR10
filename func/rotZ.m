function T = rotZ(q, theta)
    d = q(2); a = q(3); alpha = q(4); offset = q(5);
    theta = theta + offset;
    T1 = [
        cos(theta)  -sin(theta)     0    0;
        sin(theta)   cos(theta)     0    0;
        0            0              1    d;
        0            0              0    1;    ];
    
    T2 = [
        1            0              0              a;
        0            cos(alpha)    -sin(alpha)     0;
        0            sin(alpha)     cos(alpha)     0;
        0            0              0              1;];
    
    T = T1 * T2;
end