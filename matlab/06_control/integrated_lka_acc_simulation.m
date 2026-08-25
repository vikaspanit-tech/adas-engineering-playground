%% Integrated Lane Keeping Assist and Adaptive Cruise Control
% Lateral control: Speed-adaptive Pure Pursuit
% Longitudinal control: ACC finite-state machine

clear; clc; close all;

%% Simulation settings
Ts = 0.05;
Tend = 20;
time = 0:Ts:Tend;
N = numel(time);

%% Vehicle and lateral-controller parameters
L = 2.8;                         % Wheelbase (m)

LdMin = 5;                       % Minimum look-ahead distance (m)
LdMax = 20;                      % Maximum look-ahead distance (m)
KvLookAhead = 0.6;               % Speed-to-look-ahead gain (s)

maxSteering = deg2rad(12);       % Maximum road-wheel angle (rad)
maxSteeringRate = deg2rad(30);   % Maximum steering rate (rad/s)

%% Reference lane-centre path
xRef = 0:0.2:400;
yRef = 3 * sin(0.035 * xRef);

%% ACC finite-state machine
CRUISE = 1;
FOLLOW = 2;
EMERGENCY_BRAKE = 3;
RECOVER = 4;

state = zeros(1, N);
state(1) = CRUISE;
stateTime = zeros(1, N);

% ACC parameters
setSpeed = 25;                   % Desired cruise speed (m/s)
minimumGap = 5;                  % Standstill gap (m)
desiredTimeGap = 2;              % Desired following time gap (s)

followEntryTimeGap = 2.2;        % Enter Follow below this value
followExitTimeGap = 2.8;         % Return to Cruise above this value

warningTTC = 7.0;                % Emergency Brake entry threshold (s)
safeTTC = 8.0;                   % Emergency Brake exit threshold (s)
minimumEmergencyTime = 0.8;      % Minimum emergency hold time (s)

KpGap = 0.18;                    % Gap-error gain
KvRelative = 0.8;                % Relative-velocity gain
KpCruise = 0.6;                  % Cruise-speed gain

normalMinAcceleration = -3;      % Normal ACC braking limit (m/s^2)
normalMaxAcceleration = 2;       % Normal ACC acceleration limit (m/s^2)
emergencyAcceleration = -6;      % Emergency braking command (m/s^2)
maxJerk = 3;                     % Normal acceleration-change limit (m/s^3)

%% Lead-vehicle behaviour
leadAcceleration = zeros(1, N);
leadAcceleration(time >= 6 & time < 8) = -5;

leadSpeed = zeros(1, N);
leadSpeed(1) = 20;

%% Ego-vehicle initial conditions
x = zeros(1, N);
y = zeros(1, N);
yaw = zeros(1, N);

x(1) = 0;
y(1) = -1.5;                     % Initial lateral offset (m)
yaw(1) = deg2rad(5);             % Initial heading error

egoSpeed = zeros(1, N);
egoSpeed(1) = 25;

gap = zeros(1, N);
gap(1) = 59;                     % Initial lead-vehicle distance (m)

%% Output signals
steeringRequest = zeros(1, N);
steering = zeros(1, N);
lookAheadDistance = zeros(1, N);
trackingError = zeros(1, N);

relativeVelocity = zeros(1, N);
desiredGap = zeros(1, N);
timeGap = zeros(1, N);
ttc = inf(1, N);

accelerationRequest = zeros(1, N);
egoAcceleration = zeros(1, N);

%% Main simulation loop
for k = 1:N-1

    %% Longitudinal perception quantities
    relativeVelocity(k) = leadSpeed(k) - egoSpeed(k);
    desiredGap(k) = minimumGap + desiredTimeGap * egoSpeed(k);
    timeGap(k) = gap(k) / max(egoSpeed(k), 0.1);

    if relativeVelocity(k) < 0
        ttc(k) = -gap(k) / relativeVelocity(k);
    else
        ttc(k) = inf;
    end

    %% ACC state transitions
    switch state(k)

        case CRUISE
            if timeGap(k) < followEntryTimeGap
                state(k+1) = FOLLOW;
            else
                state(k+1) = CRUISE;
            end

        case FOLLOW
            if ttc(k) < warningTTC
                state(k+1) = EMERGENCY_BRAKE;
            elseif timeGap(k) > followExitTimeGap && ...
                    relativeVelocity(k) >= 0
                state(k+1) = CRUISE;
            else
                state(k+1) = FOLLOW;
            end

        case EMERGENCY_BRAKE
            emergencyTimeComplete = ...
                stateTime(k) >= minimumEmergencyTime;

            safetyRecovered = ...
                ttc(k) > safeTTC || relativeVelocity(k) >= 0;

            if emergencyTimeComplete && safetyRecovered
                state(k+1) = RECOVER;
            else
                state(k+1) = EMERGENCY_BRAKE;
            end

        case RECOVER
            if ttc(k) < warningTTC
                state(k+1) = EMERGENCY_BRAKE;
            elseif timeGap(k) < followEntryTimeGap
                state(k+1) = FOLLOW;
            elseif timeGap(k) > followExitTimeGap && ...
                    relativeVelocity(k) >= 0
                state(k+1) = CRUISE;
            else
                state(k+1) = RECOVER;
            end
    end

    % Track time spent in the active ACC state
    if state(k+1) == state(k)
        stateTime(k+1) = stateTime(k) + Ts;
    else
        stateTime(k+1) = 0;
    end

    %% ACC acceleration command
    gapError = gap(k) - desiredGap(k);

    switch state(k)

        case CRUISE
            accelerationRequest(k) = ...
                KpCruise * (setSpeed - egoSpeed(k));

        case FOLLOW
            accelerationRequest(k) = ...
                KpGap * gapError + ...
                KvRelative * relativeVelocity(k);

        case EMERGENCY_BRAKE
            accelerationRequest(k) = emergencyAcceleration;

        case RECOVER
            accelerationRequest(k) = ...
                0.10 * gapError + ...
                0.45 * relativeVelocity(k);
    end

    % Normal-state acceleration saturation
    if state(k) ~= EMERGENCY_BRAKE
        accelerationRequest(k) = max(normalMinAcceleration, ...
                                 min(normalMaxAcceleration, ...
                                     accelerationRequest(k)));
    end

    % Jerk-limited acceleration actuator
    if state(k) == EMERGENCY_BRAKE
        egoAcceleration(k) = emergencyAcceleration;
    else
        maxAccelerationChange = maxJerk * Ts;
        accelerationChange = ...
            accelerationRequest(k) - egoAcceleration(k);

        accelerationChange = max(-maxAccelerationChange, ...
                             min(maxAccelerationChange, ...
                                 accelerationChange));

        egoAcceleration(k+1) = ...
            egoAcceleration(k) + accelerationChange;
    end

    %% Speed-adaptive Pure Pursuit lateral controller
    lookAheadDistance(k) = min(LdMax, ...
        max(LdMin, LdMin + KvLookAhead * egoSpeed(k)));

    % Find closest lane-centre reference point
    distanceToPath = hypot(xRef - x(k), yRef - y(k));
    [~, nearestIndex] = min(distanceToPath);

    % Select target point at adaptive look-ahead distance
    targetDistances = hypot( ...
        xRef(nearestIndex:end) - x(k), ...
        yRef(nearestIndex:end) - y(k));

    localTargetIndex = find( ...
        targetDistances >= lookAheadDistance(k), 1);

    if isempty(localTargetIndex)
        targetIndex = numel(xRef);
    else
        targetIndex = nearestIndex + localTargetIndex - 1;
    end

    targetX = xRef(targetIndex);
    targetY = yRef(targetIndex);

    targetHeading = atan2(targetY - y(k), targetX - x(k));
    alpha = wrapToPi(targetHeading - yaw(k));

    % Pure Pursuit steering request
    steeringRequest(k) = atan2( ...
        2 * L * sin(alpha), lookAheadDistance(k));

    % Steering-angle saturation
    steeringLimited = max(-maxSteering, ...
                      min(maxSteering, steeringRequest(k)));

    % Steering-rate limit
    maxSteeringChange = maxSteeringRate * Ts;
    steeringChange = steeringLimited - steering(k);

    steeringChange = max(-maxSteeringChange, ...
                     min(maxSteeringChange, steeringChange));

    steering(k+1) = steering(k) + steeringChange;

    %% Update lead vehicle and ego longitudinal motion
    leadSpeed(k+1) = max(0, leadSpeed(k) + ...
                         leadAcceleration(k) * Ts);

    egoSpeed(k+1) = max(0, egoSpeed(k) + ...
                        egoAcceleration(k) * Ts);

    gap(k+1) = max(0, gap(k) + relativeVelocity(k) * Ts);

    %% Update ego lateral motion using kinematic bicycle model
    xDot = egoSpeed(k) * cos(yaw(k));
    yDot = egoSpeed(k) * sin(yaw(k));
    yawDot = egoSpeed(k) / L * tan(steering(k+1));

    x(k+1) = x(k) + Ts * xDot;
    y(k+1) = y(k) + Ts * yDot;
    yaw(k+1) = wrapToPi(yaw(k) + Ts * yawDot);
end

%% Final values for plots
relativeVelocity(end) = leadSpeed(end) - egoSpeed(end);
desiredGap(end) = minimumGap + desiredTimeGap * egoSpeed(end);
timeGap(end) = gap(end) / max(egoSpeed(end), 0.1);

if relativeVelocity(end) < 0
    ttc(end) = -gap(end) / relativeVelocity(end);
end

lookAheadDistance(end) = min(LdMax, ...
    max(LdMin, LdMin + KvLookAhead * egoSpeed(end)));

steeringRequest(end) = steeringRequest(end-1);
egoAcceleration(end) = egoAcceleration(end-1);
accelerationRequest(end) = accelerationRequest(end-1);

for k = 1:N
    trackingError(k) = min(hypot(xRef - x(k), yRef - y(k)));
end

% Hide TTC when ego is not closing on lead vehicle
ttcPlot = ttc;
ttcPlot(isinf(ttcPlot)) = NaN;

%% Plot 1: Lane-following path
figure('Color', 'w');

plot(xRef, yRef, 'k--', 'LineWidth', 1.8); hold on;
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');

grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Integrated LKA + ACC: Lane Following');
legend('Reference lane centre', 'Ego vehicle path', ...
       'Start', 'End', 'Location', 'best');

%% Plot 2: Lateral-control response
figure('Color', 'w');

subplot(3,1,1);
plot(time, rad2deg(steeringRequest), '--', ...
     'Color', [0 0.45 0.74], 'LineWidth', 1.5); hold on;
plot(time, rad2deg(steering), ...
     'Color', [0.85 0.33 0.10], 'LineWidth', 2);
yline(rad2deg(maxSteering), 'k:', 'Maximum steering');
yline(-rad2deg(maxSteering), 'k:');
grid on;
ylabel('Steering (deg)');
title('Lateral Controller Response');
legend('Controller request', 'Actuator output', ...
       'Location', 'best');

subplot(3,1,2);
plot(time, trackingError, ...
     'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
grid on;
ylabel('Tracking error (m)');
title('Lane-Centre Tracking Error');

subplot(3,1,3);
plot(time, lookAheadDistance, ...
     'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Look-ahead (m)');
title('Speed-Adaptive Look-Ahead Distance');

%% Plot 3: Longitudinal ACC response
figure('Color', 'w');

subplot(4,1,1);
plot(time, gap, 'b', 'LineWidth', 2); hold on;
plot(time, desiredGap, 'k--', 'LineWidth', 1.5);
grid on;
ylabel('Distance (m)');
title('Longitudinal ACC Response');
legend('Actual gap', 'Desired gap', 'Location', 'best');

subplot(4,1,2);
plot(time, egoSpeed, 'b', 'LineWidth', 2); hold on;
plot(time, leadSpeed, 'r', 'LineWidth', 2);
yline(setSpeed, 'k--', 'Set speed');
grid on;
ylabel('Speed (m/s)');
legend('Ego vehicle', 'Lead vehicle', 'Location', 'best');

subplot(4,1,3);
plot(time, ttcPlot, 'r', 'LineWidth', 2); hold on;
yline(warningTTC, 'k--', 'TTC warning threshold');
ylim([0 20]);
grid on;
ylabel('TTC (s)');

subplot(4,1,4);
stairs(time, state, 'LineWidth', 2);
yticks([CRUISE FOLLOW EMERGENCY_BRAKE RECOVER]);
yticklabels({'Cruise', 'Follow', 'Emergency brake', 'Recover'});
ylim([0.5 4.5]);
grid on;
xlabel('Time (s)');
ylabel('ACC state');
title('Active ACC State');