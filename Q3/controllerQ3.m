% Q3
% forwardKinematic
dl = 2*pi*Rw*(y(1)-y1_p)/64;
dr = 2*pi*Rw*(y(2)-y2_p)/64;
sl = dl/dt;
sr = dr/dt;
sm = 0.5*(sl+sr);
alpha = (dr-dl)/lw;
theta_p = alpha/dt;

if theta_p ~= 0 
    x_e = x_e + (sm/theta_p)*(sin(theta + theta_p*dt)-sin(theta));
    y_e = y_e - (sm/theta_p)*(cos(theta + theta_p*dt)-cos(theta));

else
    x_e = x_e + sm*cos(theta)*dt;
    y_e = y_e + sm*sin(theta)*dt;

end
theta = theta + theta_p*dt;

y1_p = y(1);
y2_p = y(2);

% make sure the range of theta is between [-pi,pi]
if theta > pi
    theta = theta-2*pi;
end
if theta < -pi
    theta = theta+2*pi;
end

if label < 100
    if abs(x_e-x_d) < 0.005 && abs(y_e-y_d) < 0.005
        label = label+1;
        x_d = lst(label,1);
        y_d = lst(label,2);
        e_i = 0;
        e_prev = 0;
        u = [0,0];
    else
        e = atan2((y_d-y_e),(x_d-x_e)) - theta;

        % make sure the range of e is between [-pi,pi]
        if e > pi
            e = e-2*pi;
        end
        if e < -pi
            e = e+2*pi;
        end

        ed = (e-e_prev)/dt;
        ei = ei + e*dt;

        u_e = Kp*e + Ki*ei + Kd*ed; 
        e_prev = e;

        %inverseKinematic
        u_m = 2;

        ul = max(min(u_m - u_e,6),-6);
        ur = max(min(u_m + u_e,6),-6);
        u =  [ul,ur];
    end

elseif label == 100
    if abs(theta) < 0.001
        e_i = 0;
        e_prev = 0;
        u = [0,0];

    else
        u = [2*theta,-2*theta];
    end
end