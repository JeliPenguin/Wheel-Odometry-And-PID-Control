% % Q1(1)
% if y(1)<180
%     e = y(1)-y(2);
%     ed = (e-e_prev)/dt;
%     ei = ei + e*dt;
%     
%     u_e = Kp*e + Ki*ei + Kd*ed; 
%     
%     u_m = 4;
%     
%     u=[u_m-u_e;u_m+u_e];
% else
%     u = [0,0];
% end



% % Q1(2)
% u = [4,-4];

% %Q2
% % forwardKinematic
% dl = 2*pi*Rw*(y(1)-y1_p)/64;
% dr = 2*pi*Rw*(y(2)-y2_p)/64;
% sl = dl/dt;
% sr = dr/dt;
% sm = 0.5*(sl+sr);
% alpha = (dr-dl)/lw;
% theta_p = alpha/dt;
% 
% if theta_p ~= 0 
%     x_e = x_e + (sm/theta_p)*(sin(theta + theta_p*dt)-sin(theta));
%     y_e = y_e - (sm/theta_p)*(cos(theta + theta_p*dt)-cos(theta));
% 
% else
%     x_e = x_e + sm*cos(theta)*dt;
%     y_e = y_e + sm*sin(theta)*dt;
% 
% end
% theta = theta + theta_p*dt;
% 
% y1_p = y(1);
% y2_p = y(2);
% 
% if theta > pi
%     theta = theta-2*pi;
% end
% if theta < -pi
%     theta = theta+2*pi;
% end
% 
% 
% % PID control
% if label == 0 % go forward in straight segment
%     if y(3) < 0.5 || y(4) < 0.5 
%         % reach the end of straight segment
%         label = 1;  % change to next state
%         ed = 0;
%         ei = 0;
%         e_prev = 0;
%         u = [0,0];
% 
% %         % start rounding point;
% %         label4_odo_x = x_e;
% %         label4_odo_y = y_e;
% 
%     else
%         % input error is ticks' difference
%         e = y(1)-y(2);
%         ed = (e-e_prev)/dt;
%         ei = ei + e*dt;
%         u_e = 5*e + 0.25*ei + 0*ed; 
%         u_m = 3;
%         u=[u_m-u_e;u_m+u_e];
%     end
% 
% elseif label == 1 % rotate 90° such that we could take use of y(5)&y(6)
%     if y(5) < 0.5  || y(6) < 0.5 
%         label = 2;  % change to next state
%         ed = 0;
%         ei = 0;
%         e_prev = 0;
%         u = [0,0];
%         start = y(1); % mark down start ticks 
%     else
%         % keep on ratation
%         u = [4;-4];
%     end
% 
% elseif label == 2 % start to move counter clock-wise
%     if y(1) < ntick + start % until 2 rounds
%         e = y(5)-0.5;
%         ed = (e-e_prev)/dt;
%         ei = ei + e*dt;
%         u_e = Kp*e + Ki*ei + Kd*ed; 
%         u_m = 3;
%         u=[u_m-u_e;u_m+u_e];
%     else 
%         label = 3; % change to next state
%         ed = 0;
%         ei = 0;
%         e_prev = 0;
%         u = [0,0];
%     end
% 
% elseif label == 3 % rotate to 0° angle
%     disp(abs(y(3)-abs(y(4))))
%     if (y(3)-0.5)^2 < 0.001 && (y(4)-0.5)^2 < 0.001 && abs(y(3)-y(4)) < 0.00001
%         label = 4;  % change to next state
%         ed = 0;
%         ei = 0;
%         e_prev = 0;
%         u = [0,0];
%         tick_1 = y(1);
%         tick_2 = y(2);
%     else
%         % keep on ratation
%         if (y(3)-0.5)^2 < 0.001 && (y(4)-0.5)^2 < 0.001
%             u = [-0.01;0.01];
%         else
%             u = [-4,4];
%         end
%     end
%   
% elseif label == 4 % go backward in straight segment
%     if x_e > 0
%         e = (tick_1-y(1))-(tick_2-y(2));
%         ed = (e-e_prev)/dt;
%         ei = ei + e*dt;
%         
%         u_e = 5*e + 0.25*ei + 0*ed; 
%         u_m = -3;
%         
%         u=[u_m+u_e;u_m-u_e];
% 
%     else 
%         ei = 0;
%         ed = 0;
%         u = [0,0];
%     end
% 
% end


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
if label == 0 % go forward in straight segment
    if abs(y(3)-0.5) < 0.002 || abs(y(4)-0.5) < 0.002
        % reach the end of straight segment
        label = 1;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        before_tick = y(1);

    else
        % input error is ticks' difference
        e = y(1)-y(2);
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        u_e = 1.5*e + 0.1*ei + 0*ed; 
        u_m = 2;
        u=[u_m-u_e;u_m+u_e];

    end

elseif label == 1 % rotate 90° such that we could take use of y(5)&y(6)
    if abs(y(5)-y(6)) < 0.001 && abs(y(5)-0.5)<0.1
        label = 2;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        after_tick = y(1); % mark down start ticks
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
        u_m = 3;
        u=[u_m-u_e;u_m+u_e];

    else 
        label = 3; % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
    end


elseif label == 3 % then rotate to 0° angle
    if (theta)^2 < 0.0001
        label = 4;  % change to next state
        ed = 0;
        ei = 0;
        e_prev = 0;
        u = [0,0];
        tick_1 = y(1);
        tick_2 = y(2);
    else
        u = [0.5*theta,-0.5*theta];
    end
  
elseif label == 4 % go backward in straight segment
    if x_e > 0
        e = (tick_1-y(1))-(tick_2-y(2));
        ed = (e-e_prev)/dt;
        ei = ei + e*dt;
        
        u_e = 1.5*e + 0.1*ei + 0*ed; 
        u_m = -2.5;
        
        u=[u_m+u_e;u_m-u_e];

    else 
        ei = 0;
        ed = 0;
        u = [0,0];
    end

end



% % Q3
% % forwardKinematic
% dl = 2*pi*Rw*(y(1)-y1_p)/64;
% dr = 2*pi*Rw*(y(2)-y2_p)/64;
% sl = dl/dt;
% sr = dr/dt;
% sm = 0.5*(sl+sr);
% alpha = (dr-dl)/lw;
% theta_p = alpha/dt;
% 
% if theta_p ~= 0 
%     x_e = x_e + (sm/theta_p)*(sin(theta + theta_p*dt)-sin(theta));
%     y_e = y_e - (sm/theta_p)*(cos(theta + theta_p*dt)-cos(theta));
% 
% else
%     x_e = x_e + sm*cos(theta)*dt;
%     y_e = y_e + sm*sin(theta)*dt;
% 
% end
% theta = theta + theta_p*dt;
% 
% y1_p = y(1);
% y2_p = y(2);
% 
% % make sure the range of thera is between [-pi,pi]
% if theta > pi
%     theta = theta-2*pi;
% end
% if theta < -pi
%     theta = theta+2*pi;
% end
% 
% if label < 100
%     if abs(x_e-x_d) < 0.005 && abs(y_e-y_d) < 0.005
%         label = label+1;
%         x_d = lst(label,1);
%         y_d = lst(label,2);
%         e_i = 0;
%         e_prev = 0;
%         u = [0,0];
%     else
%         e = atan2((y_d-y_e),(x_d-x_e)) - theta;
%     
%         % make sure the range of e is between [-pi,pi]
%         if e > pi
%             e = e-2*pi;
%         end
%         if e < -pi
%             e = e+2*pi;
%         end
%         
%         ed = (e-e_prev)/dt;
%         ei = ei + e*dt;
%         
%         u_e = Kp*e + Ki*ei + Kd*ed; 
%         e_prev = e;
%         
%         %inverseKinematic
%         u_m = 2;
%         
%         ul = max(min(u_m - u_e,6),-6);
%         ur = max(min(u_m + u_e,6),-6);
%         u =  [ul,ur];
%     end
% 
% elseif label == 100
%     if abs(theta) < 0.001
%         e_i = 0;
%         e_prev = 0;
%         u = [0,0];
% 
%     else
%         u = [2*theta,-2*theta];
%     end
% end


% % Q4
% e = y(1)-y(2);
% ed = (e-e_prev)/dt;
% ei = ei + e*dt;
% 
% u_e = Kp*e + Ki*ei + Kd*ed; 
% 
% u_m = 4;
% 
% u=[u_m-u_e;u_m+u_e];
% 
% 
% if mod(i,100) == 0 && i>100
%     disp((csim.Log.Output(2,i-1)-csim.Log.Output(2,i-101))/(0.02*100));
% end