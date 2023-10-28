% Q1 Calculate wheel radius and distance between wheels
e_prev = 0;
de = 0;
ie = 0;

% some key point variables for calculating wheel radius & distance
st = 1.0;
en = 0.5;
dist = st - en;
start_calc = false;
end_calc = false;
y1s = 0;
y2s = 0;
y1e = 0;
y2e = 0;

Kp = 10;
Ki = 3.5;
Kd = 3;

% these are for controlling different states for calculations
calc2 = false;
state = ["turn", "circle"];
current_state = "none";
spin_timer = 0;
circle_diff = 0;

