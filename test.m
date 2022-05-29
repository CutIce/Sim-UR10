addpath('./func');
[robot, para, axis]= model();

robot.plot([0 0 0 0 0 0]);

Tt(1:3, 1:3) = [
    -1 0 0;
    0  1 0;
    0 0 -1;
];
Tt(1:3, 4)   = [0 1 0.5]';
Tt(4, 1:4)   = [0 0 0 1];
% q1 = robot.ikine(Tt)
format long
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



q1 = pi/18 * [1 2 3 4 5 6];
q2 = pi/18 * [6 5 4 3 2 1];
q3 = pi/18 * [9 6 2 4 1 8];

q = [q1; q2; q3];

%% test Forward Kine
for i = 1:3
    fprintf(strcat("测试正向运动学 样例", num2str(i), "误差：  "));
    r1 = double(robot.fkine(q(i, :)));
    r2 = forwardKine(q(i, :));
    fprintf(strcat(num2str(norm(r1 - r2)), "\n"));
end


%% test Inverse Kine
q_suck = robot.ikunc(T_suck);
q_suck_pre = robot.ikunc(T_suck_pre);


disp(q_suck_pre)
disp(q_suck)