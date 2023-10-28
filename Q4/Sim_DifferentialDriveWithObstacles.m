% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 50;
dt          = 0.01;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)
sampSize = 200; %Set when to calculate delta ticks
r = 0.1;
nTick = 64;
Kss = 0;
for u_m =1:6
    % Initialise plot
    figure;
    ax1 = axes;
    hold(ax1,'on');
    axis('equal')
    axis([-0.4 10 5 15])
    axis('manual')
    xlabel('x');
    ylabel('y');
    ax1.Toolbar.Visible = 'off';
    ax1.Interactions = [];
    
    % Initialise Simulation
    robot = DifferentialDriveWithObstacles(ax1);
    robot.setState([0,0,0,0,0,0,10,0,0]);
    robot.setInput([0;0]);
    robot.updateOutput;
    csim = ControlSimulator(robot,TOTAL_TIME,dt);
    
    trajplot = plot(csim.Log.Output(2,:),csim.Log.Output(3,:),'linewidth',1,'Color','k');
    robot.plot;

    lastTick = 0;
    currentTick = 0; % For calculating difference in ticks after sampleSize steps
    counts = 0;
    speeds = 0; % To do averaging across calculated speeds
    
    time = nan(1,csim.TotalSteps-1);
    % Run Simulation
    for i = 2:csim.TotalSteps
        tic
        
        % Read odometry and range sensors
        % y(1:2) - odometry
        % y(3:11) - range sensors
        
        y = robot.Output(4:11);
        currentTick = (y(1)+y(2))/2;
        % disp([currentTick,lastTick])
        % Control: determine dc motor inputs. assume controller only has access
        % to the odometry measurements variable 'y' and
        
        u = [u_m;u_m];
        
        % Simulation
        csim.step(u);
        set(trajplot,'XData',csim.Log.Output(2,:));
        set(trajplot,'YData',csim.Log.Output(3,:));
        robot.plot;
        drawnow nocallbacks limitrate
        if mod(i,sampSize) == 0 && i>csim.TotalSteps/2
            if lastTick ~= 0
                angSpeed = ((currentTick-lastTick)*2*pi/nTick)/(dt*sampSize);
                speed = angSpeed*r;
                speeds = speeds + speed;
                counts = counts + 1;
            end
            lastTick = currentTick;
        end
        time(i-1) = toc;
        pause(TIME_SCALE*dt-toc); 
    end
    
    Ks = speeds/counts/u_m; %Formula obtained from limit value theorem assuming constant v
    Kss = Kss + Ks;
    fprintf("For u=%i Ks = %f\n",u_m,Ks)
    close all
end

fprintf("On average, Ks=%f\n",Kss/6)