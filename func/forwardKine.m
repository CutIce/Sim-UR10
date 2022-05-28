function T = forwardKine(q)
    addpath('./func');
    [robot, para] = model();
    T = eye(4);
    for i = 1:6
        T = T * rotZ(para(i, :), q(i));
    end
end