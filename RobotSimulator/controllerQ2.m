% Odometer reading
odo_left  = y(1);
odo_right = y(2);

% Sensor reading
front_distance1 = y(3);
front_distance2 = y(4);
left_distance1 = y(5);
left_distance2 = y(6);
right_distance1 = y(7);
right_distanc2 = y(7);

% Reference errors
odo_error  = odo_left-odo_right; % mode 1
left_distance_error1 = distance_from_obstacle - left_distance1; % mode 2
left_distance_error2 = distance_from_obstacle - left_distance1; % mode 3

% Dynamics
dTickL = odo_left - lastTickL;
dTickR = odo_right - lastTickR;
lastTickL = odo_left;
lastTickR = odo_right;
dL = 2 * pi * r * (dTickL/nTicks);
dR = 2 * pi * r * (dTickR/nTicks);

% Forward Kinematics
sR = dR / dt;
sL = dL / dt;
sm = (sL + sR)/2;
phiDot = (sR-sL)/lw;
if phiDot ~= 0 
    px = px + sm * (sin(phiDot * dt+phi)-sin(phi))/phiDot;
    py = py - sm * (cos(phiDot * dt+phi)-cos(phi))/phiDot;
else
    px = px + sm*dt*cos(phi);
    py = py + sm*dt*sin(phi);
end
phi = phi + phiDot * dt;
if phi > pi
    phi = phi - 2*pi;
elseif phi < -pi
    phi=phi+2*pi;
end

if round(py,1) ~= 0 && halfCycleStart == 0 && init_turn == 0
    halfCycleStart = 1;
elseif round(py,1) == 0 && halfCycleStart == 1
    halfCycleStart = 0;
    halfCycles = halfCycles + 1;
end

if halfCycles < wantedCycles * 2
    % While robot hasn't finished desired number of cycles
    if init_walkup == 1
        % Drive in straight line up to obstacle
        if (front_distance1>=(distance_from_obstacle+lw/2-0.1))
            e = odo_error;
            current_mode = 1;
        else
            init_walkup = 0;
        end

    elseif init_turn == 1
        % Turning the robot clockwise
        emptySensor = round(left_distance1,1)==1.1 && round(left_distance2,1) ==1.1;
        if (emptySensor || (~emptySensor && round(left_distance1,3) ~= round(left_distance2,3))) 
            % Turning robot clockwise to be parrallel to the side of obstacle at its current position
            u = [u_m;-u_m];
            current_mode = 0;
        else
            init_turn = 0;
            % Slight offset to ensure cycle counting works
            sy = py+10e-3;
        end
    else
        % disp([left_distance1,left_distance2,left_distance_error])
        if (round(left_distance1,3) ~= distance_from_obstacle)
            e = left_distance_error1;
            current_mode = 2;
        elseif (round(left_distance2,3) ~= distance_from_obstacle)
            % Following the edge of obstacle
            e = left_distance_error2;
            current_mode = 3;
        end
    end
else
    % Return back to start location
    if round(px,2) == round(rx,2) && round(py,2) == round(ry,2)
        u = [0;0];
        current_mode = 0;
    else
        desiredPhi = atan2((ry-py),(rx-px));
        e = desiredPhi-phi;
        if e > pi
            e = e - 2*pi;
        elseif e < -pi
            e=e+2*pi;
        end
        current_mode = 4;
    end
end

if mode ~= current_mode
    % Resetting PID parameters when changing mode
    mode = current_mode;
    e_prev = 0;
    ie = 0;
end

if mode ~= 0
    % If not manual control, then do PID
    de = (e-e_prev)/dt;
    ie = ie + e*dt;
    u_turn = Kp(mode)*e + Ki(mode)*ie + Kd(mode)*de;
    u_l = u_m-u_turn;
    u_r = u_m+u_turn;
    u = [min(max(u_l,-6),6); ...
         min(max(u_r,-6),6)];
    e_prev = e;
end

disp(u)