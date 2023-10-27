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

% %Q3
% %forwardKinematic
% if i > 2
%     dl = 2*pi*Rw*(y(1)-y1_p)/64;
%     dr = 2*pi*Rw*(y(2)-y2_p)/64;
%     sl = dl/dt;
%     sr = dr/dt;
%     sm = 0.5*(sl+sr);
%     alpha = (dr-dl)/lw;
%     theta_p = alpha/dt;
%     
%     if theta_p ~= 0 
%         x_e = x_e + (sm/theta_p)*(sin(theta + theta_p*dt)-sin(theta));
%         y_e = y_e - (sm/theta_p)*(cos(theta + theta_p*dt)-cos(theta));
% 
%     else
%         x_e = x_e + sm*cos(theta)*dt;
%         y_e = y_e + sm*sin(theta)*dt;
% 
%     end
%     theta = theta + theta_p*dt;
% end
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
% %inverseKinematic
% if y(3)>0.6 && label == 0
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
%     label = 1;
%     e_i1 = 0;
%     e_prev1 = 0;
%     e_i2 = 0;
%     e_prev2 = 0;
% end
% 
% if label == 1
%     Kp = 100;
%     Ki = 2;
%     Kd = 0;
%     e1 = (y(5)-0.5);
%     e2 = (y(6)-0.5);
%     ed1 = (e1-e_prev1)/dt;
%     ed2 = (e2-e_prev2)/dt;
%     e_i1 = e_i1 + e1*dt;    
%     e_i2 = e_i2 + e2*dt;
%     u_e1 = Kp*e1 + Ki*e_i1 + Kd*ed1; 
%     u_e2 = Kp*e2 + Ki*e_i2 + Kd*ed2; 
%     u_m = 4;
%     u=[u_m-u_e1;u_m+u_e2];
% end


% Q3
%forwardKinematic
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

if label < 76
    if (x_e-x_d)^2+(y_e-y_d)^2 < 0.0005
        label = label+1;
        x_d = lst(label,1);
        y_d = lst(label,2);
        e_i = 0;
        e_prev = 0;
    end

    e = atan2((y_d-y_e),(x_d-x_e)) - theta;

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
    u_m = 4;
    
    ul = max(min(u_m - u_e,6),-6);
    ur = max(min(u_m + u_e,6),-6);
    u =  [ul,ur];

elseif label == 76
    if (theta)^2 < 0.001
        u = [0,0];
        e_i = 0;
        e_prev = 0;
    else
        u = [theta,-theta];
    end
end


% % Q4
% e = y(1)-y(2);
% ed = (e-e_prev)/dt;
% ei = ei + .e*dt;
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