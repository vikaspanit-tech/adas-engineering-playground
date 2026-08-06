%% Kinematic Bicycle Model
clear; clc; close all;

% Vehicle parameters
L = 2.8;                 % Wheelbase (m)
%v = 10;                  % Constant longitudinal speed (m/s)
%delta = deg2rad(20);     % Front-wheel steering angle (rad)

% Simulation parameters
Ts = 0.01;               % Sample time (s)
Tend = 10;               % Simulation duration (s)
time = 0:Ts:Tend;

v = zeros(size(time));
v(1) = 10;              % Initial speed (m/s)

a = zeros(size(time));
a(time >= 0 & time < 3) = 2;     % Accelerate for 3 s
a(time >= 7 & time < 9) = -2;    % Brake for 2 s


delta = zeros(size(time));

% Turn left from 2 s to 5 s
delta(time >= 2 & time < 5) = deg2rad(12);

% Turn right from 5 s to 8 s
delta(time >= 5 & time < 8) = deg2rad(-12);

% Initial state: [x; y; yaw]
x = zeros(size(time));
y = zeros(size(time));
yaw = zeros(size(time));

% Euler integration
for k = 1:numel(time)-1
    v(k+1) = max(0, v(k) + Ts * a(k));
    xDot = v(k) * cos(yaw(k));
    yDot = v(k) * sin(yaw(k));
    %yawDot = (v / L) * tan(delta);
    yawDot = (v(k) / L) * tan(delta(k));

    x(k+1) = x(k) + Ts * xDot;
    y(k+1) = y(k) + Ts * yDot;
    yaw(k+1) = yaw(k) + Ts * yawDot;
end

% Plot the vehicle path
figure('Color','w');
plot(x, y, 'b-', 'LineWidth', 2);
grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Kinematic Bicycle Model: Time-Varying Steering Trajectory');

hold on;

% Draw the vehicle at selected path positions
vehicleLength = 4.7;
vehicleWidth = 1.8;
indices = round(linspace(1, numel(time), 8));

for k = indices
    drawVehicle(x(k), y(k), yaw(k), vehicleLength, vehicleWidth);
end

hold off;

figure('Color', 'w');
plot(time, v, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Longitudinal Speed Profile');

function drawVehicle(x, y, yaw, length, width)

    % Rectangle starts at the rear axle and extends forward
    vehicle = [0,       -width/2;
               length,  -width/2;
               length,   width/2;
               0,        width/2;
               0,       -width/2];

    % Rotate and translate into world coordinates
    rotation = [cos(yaw), -sin(yaw);
                sin(yaw),  cos(yaw)];

    vehicleWorld = (rotation * vehicle')';
    vehicleWorld(:,1) = vehicleWorld(:,1) + x;
    vehicleWorld(:,2) = vehicleWorld(:,2) + y;

    fill(vehicleWorld(:,1), vehicleWorld(:,2), ...
         [0.2 0.7 0.9], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
end