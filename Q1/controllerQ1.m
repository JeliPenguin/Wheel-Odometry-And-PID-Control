% walk in a straight line, for Q1
e = y(1) - y(2);
de = (e - e_prev) / dt;
ie = ie + e * dt;
e_prev = e;
ctrl = Kp * e + Ki * ie + Kd * de;
u = [2 - ctrl; 2 + ctrl];

% collect odometry reading from start point to end point
% this is for wheel radius calculation
if (~start_calc) && ((y(3) <= st && y(4) <= st) && (~end_calc))
    y1s = y(1);
    y2s = y(2);
    start_calc = true;
end
if (start_calc) && ((y(3) <= en && y(4) <= en) && (~end_calc))
    y1e = y(1);
    y2e = y(2);
    end_calc = true;
end

% Turn full circle for wheel distance calculation
if end_calc && (~calc2)
    current_state = state(1);
    calc2 = true;
end

switch current_state
    case state(1)
        u = [2, 0];
        spin_timer = spin_timer + 1;
        dis1 = abs(y(3) - en);
        dis2 = abs(y(4) - en);
        if (((dis1 + dis2)/2) <= 0.005) && (spin_timer >= 30) && (abs(y(3) - en) <= 0.004)
            current_state = state(2);
        end
    case state(2)
        % stop the robot, get odometry readings
        u = [0 ,0];
        circle_diff = y(1) - y1e;
    otherwise
end
