function T = axis2rotm(theta, r)
        rx = r(1, :);
        ry = r(2, :);
        rz = r(3, :);

        st = sin(theta);
        ct = cos(theta);

        R(1, 1, :) = rx.^2   .* (1 - ct) + ct;
        R(1, 2, :) = rx.* ry .* (1 - ct) - rz .* st;
        R(1, 3, :) = rx.* rz .* (1 - ct) + ry .* st;

        R(2, 1, :) = rx .* ry .* (1 - ct) + rz .* st;
        R(2, 2, :) = ry.^2 .* (1 - ct) + ct;
        R(2, 3, :) = ry .* rz .* (1 - ct) - rx .* st;

        R(3, 1, :) = rx .* rz .* (1 - ct) - ry .* st;
        R(3, 2, :) = ry .* rz .* (1 - ct) + rx .* st;
        R(3, 3, :) = rz.^2 .* (1 - ct) + ct;

        T(1:3, 1:3, :) = R;
end