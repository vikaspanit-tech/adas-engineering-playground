%% Pure Pursuit Lane-Following Controller
clear; clc; close all;

% Vehicle and simulation parameters
L = 2.8;                 % Wheelbase (m)
v = 12;                  % Constant longitudinal speed (m/s)
Ld = 5;                 % Look-ahead distance (m)
Ts = 0.05;               % Sample time (s)
Tend = 20;               % Simulation duration (s)
time = 0:Ts:Tend;

% Reference lane centreline: gentle sinusoidal curve
xRef = 0:0.2:250;
yRef = 3 * sin(0.04 * xRef);

% Initial ego-vehicle state
x = zeros(size(time));
y = zeros(size(time));
yaw = zeros(size(time));
steering = zeros(size(time));

% Start slightly away from the lane centre
x(1) = 0;
y(1) = -1.5;
yaw(1) = deg2rad(5);

for k = 1:numel(time)-1

    % Find nearest reference-path point
    distanceToPath = hypot(xRef - x(k), yRef - y(k));
    [~, nearestIndex] = min(distanceToPath);

    % Find the first path point at least Ld ahead of the ego vehicle
    lookAheadDistances = hypot( ...
        xRef(nearestIndex:end) - x(k), ...
        yRef(nearestIndex:end) - y(k));

    localTargetIndex = find(lookAheadDistances >= Ld, 1);

    if isempty(localTargetIndex)
        targetIndex = numel(xRef);
    else
        targetIndex = nearestIndex + localTargetIndex - 1;
    end

    targetX = xRef(targetIndex);
    targetY = yRef(targetIndex);

    % Angle from ego vehicle to target point
    targetHeading = atan2(targetY - y(k), targetX - x(k));
    alpha = wrapToPi(targetHeading - yaw(k));

    % Pure Pursuit steering law
    steering(k) = atan2(2 * L * sin(alpha), Ld);

    % Kinematic bicycle-model update
    xDot = v * cos(yaw(k));
    yDot = v * sin(yaw(k));
    yawDot = v / L * tan(steering(k));

    x(k+1) = x(k) + Ts * xDot;
    y(k+1) = y(k) + Ts * yDot;
    yaw(k+1) = wrapToPi(yaw(k) + Ts * yawDot);
end

steering(end) = steering(end-1);

% Lateral tracking error: distance to closest reference point
lateralError = zeros(size(time));

for k = 1:numel(time)
    lateralError(k) = min(hypot(xRef - x(k), yRef - y(k)));
end

% Path-following visualization
figure('Color', 'w');
plot(xRef, yRef, 'k--', 'LineWidth', 1.8); hold on;
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');
grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Pure Pursuit Lane Following');
legend('Reference lane centre', 'Ego vehicle path', ...
       'Start', 'End', 'Location', 'best');

% Controller response
figure('Color', 'w');

subplot(2,1,1);
plot(time, rad2deg(steering), 'LineWidth', 1.8);
grid on;
ylabel('Steering angle (deg)');
title('Pure Pursuit Controller Response');

subplot(2,1,2);
plot(time, lateralError, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Tracking error (m)');