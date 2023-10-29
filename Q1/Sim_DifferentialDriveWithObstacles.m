% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 40;
dt          = 0.02;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
axis('equal')
axis([-0.4 4.6 -2 2])
axis('manual')
xlabel('x');
ylabel('y');
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
robot = DifferentialDriveWithObstacles(ax1);
robot.setState(zeros(9,1));
robot.setInput([0;0]);
robot.updateOutput;
csim = ControlSimulator(robot,TOTAL_TIME,dt);

trajplot = plot(csim.Log.Output(2,:),csim.Log.Output(3,:),'linewidth',1,'Color','k');
robot.plot;


%%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
initQ1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = nan(1,csim.TotalSteps-1);
% Run Simulation
for i = 2:csim.TotalSteps
    tic
    
    % Read odometry and range sensors
    % y(1:2) - odometry
    % y(3:11) - range sensors
    
    y = robot.Output(4:11);
    
    % Control: determine dc motor inputs. assume controller only has access
    % to the odometry measurements variable 'y' and
    
    %%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
    controllerQ1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulation
    csim.step(u);
    set(trajplot,'XData',csim.Log.Output(2,:));
    set(trajplot,'YData',csim.Log.Output(3,:));
    robot.plot;
    drawnow nocallbacks limitrate
    time(i-1) = toc;
    pause(TIME_SCALE*dt-toc); 
    
end

% Below is the data we collected throughout 10 runs. These are used for
% the report.
wrs = [0.099862, 0.099942, 0.099862, 0.099862, 0.099862, 0.099862, 0.099862, 0.097942, 0.099862, 0.099862];
wds = [0.24965, 0.24932, 0.24809, 0.25122, 0.24809, 0.24965, 0.24965, 0.24332, 0.24965, 0.24809];
T = table(wrs', wds', 'VariableNames', {'wheel_radius', 'wheel_distance'});
disp(T)
disp("The average radius is: " + mean(wrs));
disp("The average distance is: " + mean(wds));

% Q1 Calculate wheel radius
tpr = 64; % ticks per wheel revolution
wheel_rev = ((y1e - y1s + y2e - y2s)/2)/tpr;
wheel_rad = (dist / wheel_rev) / (2*pi);
disp("In this run, wheel radius is: " + wheel_rad);

% Q1 Calculate distance between wheels
revo_count = circle_diff / 64;
wheel_dist = revo_count * wheel_rad;
disp("In this run, wheel distance is: " + wheel_dist);



