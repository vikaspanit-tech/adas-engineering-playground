%% Speed-Adaptive Pure Pursuit Lane Following
clear; clc; close all;

% Vehicle parameters
L = 2.8;                         % Wheelbase (m)

% Simulation parameters
Ts = 0.05;                       % Sample time (s)
Tend = 20;                       % Simulation duration (s)
time = 0:Ts:Tend;

% Speed profile
v = zeros(size(time));
v(time < 7) = 8;                 % Low-speed section (m/s)
v(time >= 7 & time < 14) = 20;   % High-speed section (m/s)
v(time >= 14) = 12;              % Medium-speed section (m/s)

% Speed-adaptive look-ahead parameters
LdMin = 5;                        % Minimum look-ahead distance (m)
LdMax = 20;                       % Maximum look-ahead distance (m)
KvLookAhead = 0.6;                % Look-ahead gain (s)
Ld = zeros(size(time));           % Adaptive look-ahead distance (m)

% Steering actuator limits
maxSteering = deg2rad(12);        % Maximum road-wheel angle (rad)
maxSteeringRate = deg2rad(30);    % Maximum steering rate (rad/s)

% Reference lane centreline
xRef = 0:0.2:300;
yRef = 3 * sin(0.04 * xRef);

% Ego-vehicle state: rear-axle reference point
x = zeros(size(time));
y = zeros(size(time));
yaw = zeros(size(time));

steeringRequest = zeros(size(time));
steering = zeros(size(time));

% Initial ego pose
x(1) = 0;
y(1) = -1.5;                     % Initial lateral offset (m)
yaw(1) = deg2rad(5);             % Initial heading error (rad)

for k = 1:numel(time)-1

    % Adapt look-ahead distance using current speed
    Ld(k) = min(LdMax, ...
            max(LdMin, LdMin + KvLookAhead * v(k)));

    % Find the reference-path point closest to the ego vehicle
    distanceToPath = hypot(xRef - x(k), yRef - y(k));
    [~, nearestIndex] = min(distanceToPath);

    % Select first reference point at least Ld(k) away
    lookAheadDistances = hypot( ...
        xRef(nearestIndex:end) - x(k), ...
        yRef(nearestIndex:end) - y(k));

    localTargetIndex = find(lookAheadDistances >= Ld(k), 1);

    if isempty(localTargetIndex)
        targetIndex = numel(xRef);
    else
        targetIndex = nearestIndex + localTargetIndex - 1;
    end

    % Target point on the lane-centre reference path
    targetX = xRef(targetIndex);
    targetY = yRef(targetIndex);

    % Target angle relative to ego heading
    targetHeading = atan2(targetY - y(k), targetX - x(k));
    alpha = wrapToPi(targetHeading - yaw(k));

    % Pure Pursuit steering request
    steeringRequest(k) = atan2(2 * L * sin(alpha), Ld(k));

    % Steering-angle saturation
    steeringLimited = max(-maxSteering, ...
                      min(maxSteering, steeringRequest(k)));

    % Steering-rate limiting
    maxSteeringChange = maxSteeringRate * Ts;
    steeringChange = steeringLimited - steering(k);

    steeringChange = max(-maxSteeringChange, ...
                     min(maxSteeringChange, steeringChange));

    % Actual actuator steering output
    steering(k+1) = steering(k) + steeringChange;

    % Kinematic bicycle-model update
    xDot = v(k) * cos(yaw(k));
    yDot = v(k) * sin(yaw(k));
    yawDot = v(k) / L * tan(steering(k+1));

    x(k+1) = x(k) + Ts * xDot;
    y(k+1) = y(k) + Ts * yDot;
    yaw(k+1) = wrapToPi(yaw(k) + Ts * yawDot);
end

% Fill final values for plotting
Ld(end) = min(LdMax, ...
          max(LdMin, LdMin + KvLookAhead * v(end)));

steeringRequest(end) = steeringRequest(end-1);

% Calculate closest-point lane-centre tracking error
trackingError = zeros(size(time));

for k = 1:numel(time)
    trackingError(k) = min(hypot(xRef - x(k), yRef - y(k)));
end

% Plot vehicle path and reference lane
figure('Color', 'w');

plot(xRef, yRef, 'k--', 'LineWidth', 1.8); hold on;
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');

grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Speed-Adaptive Pure Pursuit Lane Following');
legend('Reference lane centre', 'Ego vehicle path', ...
       'Start', 'End', 'Location', 'best');

% Plot steering request, actuator response, and tracking error
figure('Color', 'w');

subplot(3,1,1);
plot(time, rad2deg(steeringRequest), '--', ...
     'Color', [0 0.45 0.74], 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(steering), ...
     'Color', [0.85 0.33 0.10], 'LineWidth', 2);
yline(rad2deg(maxSteering), 'k:', 'Maximum steering');
yline(-rad2deg(maxSteering), 'k:');
grid on;
ylabel('Steering (deg)');
title('Pure Pursuit Steering Request and Actuator Output');
legend('Controller request', 'Actuator output', ...
       'Location', 'best');

subplot(3,1,2);
steeringRate = [0 diff(steering)] / Ts;

plot(time, rad2deg(steeringRate), ...
     'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
hold on;
yline(rad2deg(maxSteeringRate), 'k:', 'Maximum rate');
yline(-rad2deg(maxSteeringRate), 'k:');
grid on;
ylabel('Steering rate (deg/s)');
title('Steering-Rate Limiting');

subplot(3,1,3);
plot(time, trackingError, ...
     'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Tracking error (m)');
title('Lane-Centre Tracking Error');

% Plot speed profile and resulting adaptive look-ahead distance
figure('Color', 'w');

subplot(2,1,1);
plot(time, v, 'b', 'LineWidth', 1.8);
grid on;
ylabel('Speed (m/s)');
title('Speed-Adaptive Pure Pursuit Inputs');

subplot(2,1,2);
plot(time, Ld, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Look-ahead distance (m)');
title('Adaptive Look-Ahead Distance');