% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 200;
dt          = 0.02;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)
% TIME_SCALE = 2;


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
initQ3;
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
    controllerQ3;
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