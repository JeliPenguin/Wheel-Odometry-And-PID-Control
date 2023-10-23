e_prev = 0;
ie = 0; %integral error

% PID parameters for different modes
Kp = [1.5,50,30,10];
Ki = [0.5,0.5,5,0.5];
Kd = [0,0,5,0];

% Constants
distance_from_obstacle = 0.5;
lw = 0.25;
r = 0.1;
nTicks = 64;
u_m = 4;
sx = -10; % Starting point of circulating precedure
sy = -10; % Starting point of circulating precedure
ry = 0;
rx = 0;
wantedCycles = 1;

% Variables
init_walkup = 1;
init_turn = 1;
mode = 1; % 0 for manual u, 1 for odo_error, 2 for left_distance_error, 3 for desired_distance_error, 4 for angular_error
current_mode = 1;
px = 0;
py = 0;
lastTickL = 0;
lastTickR = 0;
phi = 0;
halfCycles = 0;
halfCycleStart = 0;



