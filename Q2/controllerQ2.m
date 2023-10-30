
%Q2 version2
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

if theta > pi
    theta = theta-2*pi;
end
if theta < -pi
    theta = theta+2*pi;
end

% PID control
if label == -1
    % Used to simulate cases where obstacles have been rotated at certain angle 
    if abs(theta-angle)>0.001 % In general angle=0
        u = [0.5*(theta-angle),0.5*(-theta+angle)];
    else
        y_ll = y(1);
        y_rr = y(2);
        label = 0;
    end

elseif label == 0 % go forward in straight segment
    if y(3)-0.5 < 0 || y(4)-0.5 < 0
        % reach the end of straight segment
        label = 1;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        before_tick = y(1);

        ref_x = x_e;
        ref_y = y_e;
    else
        % input error is ticks' difference
        e = (y(1)-y_ll)-(y(2)-y_rr);
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        u_e = 5*e + 0.25*ei + 0*ed; 
        u_m = 4;
        u=[u_m-u_e;u_m+u_e];

    end

elseif label == 1 % Rotate to parallel to obstacle side
    if abs(y(5)-y(6)) < 0.001 && abs(y(5)-0.5)<0.1
        label = 2;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        after_tick = y(1); % mark down start ticks
        if abs(theta+pi/2)>pi/15
            % slightly modify the ntick by 1
            % disp(theta);
            ntick = 1813;
        end
    else
        % keep on ratation
        u = [0.5;-0.5];
    end

elseif label == 2 % start to move counter clock-wise
    if y(1) < ntick + after_tick % until 2 rounds
        e = y(5)-0.5;
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        u_e = Kp*e + Ki*ei + Kd*ed; 
        u_m = 2;
        u=[u_m-u_e;u_m+u_e];

    else 
        label = 3; % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
    end


elseif label == 3 % then rotate to 0°
    if (theta-angle)^2 < 0.0001
        label = 4;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        tick_1 = y(1);
        tick_2 = y(2);

        x_e = (ref_x+x_e)/2;
        y_e = (ref_y+y_e)/2;
    else
        u = [0.5*(theta-angle),-0.5*(theta-angle)];
    end
  
elseif label == 4 % go backward in straight segment
    if abs(x_e-1.1) < 0.02 && abs(y_e-1.1/cos(angle)*sin(angle)) < 0.02
        label = 5;
        e = 0;
        ed = 0;
        ei = 0;
        e_u = 0;
        u = [0,0];
    else
        e = atan2(y_e-1.1/cos(angle)*sin(angle),x_e-1.1) - theta;
    
        % make sure the range of e is between [-pi,pi]
        if e > pi
            e = e-2*pi;
        end
        if e < -pi
            e = e+2*pi;
        end
        
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        
        u_e = 0.5*e + 0.01*ei + 0*ed; 

        e_prev = e;
        
        %inverseKinematic
        u_m = -0.3;
        
        ul = max(min(u_m - u_e,6),-6);
        ur = max(min(u_m + u_e,6),-6);
        u =  [ul,ur];
    end

elseif label == 5 % go backward in straight segment
    if abs(x_e-0) < 0.01 && abs(y_e-0) < 0.01
        label = 6;
        e = 0;
        ed = 0;
        ei = 0;
        e_u = 0;
        u = [0,0];
    else
        e = atan2(y_e,x_e) - theta;
    
        % make sure the range of e is between [-pi,pi]
        if e > pi
            e = e-2*pi;
        end
        if e < -pi
            e = e+2*pi;
        end
        
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        
        u_e = 0.5*e + 0.01*ei + 0*ed; 
        e_prev = e;
        
        %inverseKinematic
        u_m = -0.3;
        
        ul = max(min(u_m - u_e,6),-6);
        ur = max(min(u_m + u_e,6),-6);
        u =  [ul,ur];
    end


elseif label == 6
    % Minor fix of the angle
    if abs(theta-angle) < 0.005
        e_i = 0;
        e_prev = 0;
        u = [0,0];
    else
        if theta > 0
            u = [0.5*(theta-angle),-0.5*(theta-angle)];
        elseif theta < 0
            u = [-0.5*(theta-angle),0.5*(theta-angle)];
        end
    end

end