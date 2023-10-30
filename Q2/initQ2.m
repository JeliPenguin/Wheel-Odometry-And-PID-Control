% Q2
% pid factors
Kp = 50;
Ki = 5;
Kd = 0;

% integral error
ei = 0;

% previous error (for derivative)
e_prev = 0;

%initial voltage
u=[0;0];

% previous tick counts
y1_p=0;
y2_p=0;

% defferential driver parmeters
lw = 0.250;
Rw = 0.100;
x_e = 0;
y_e = 0;
theta = 0;

% label of states
label = -1;

start = 0; % start ticks
ntick = 1814; % two round ticks

% refenrence point when coming backward
ref_x = 0;
ref_y = 0;

% Initial orientation of robot
angle = 0;
