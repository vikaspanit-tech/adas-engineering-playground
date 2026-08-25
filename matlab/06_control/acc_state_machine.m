%% ACC Finite-State Machine with Emergency-Braking Hold Time
clear; clc; close all;

% Simulation
Ts = 0.05;
Tend = 20;
time = 0:Ts:Tend;
N = numel(time);

% ACC states
CRUISE = 1;
FOLLOW = 2;
EMERGENCY_BRAKE = 3;
RECOVER = 4;

state = zeros(1, N);
state(1) = CRUISE;

% Time spent in the active state
stateTime = zeros(1, N);

% Vehicle and controller parameters
setSpeed = 25;             % Desired cruise speed (m/s)
minimumGap = 5;            % Standstill gap (m)
desiredTimeGap = 2;        % Desired time gap (s)

% Hysteresis thresholds
followEntryTimeGap = 2.2;  % Enter Follow below this value
followExitTimeGap = 2.8;   % Leave Follow above this value

% TTC thresholds
warningTTC = 7.0;          % Enter Emergency Brake below this value (s)
safeTTC = 8.0;             % Safe TTC to leave Emergency Brake (s)

% Emergency-state protection
minimumEmergencyTime = 0.8; % Minimum Emergency Brake duration (s)

% Controller gains
KpGap = 0.18;              % Gap-error gain
KvRelative = 0.8;          % Relative-velocity gain
KpCruise = 0.6;            % Set-speed gain

% Acceleration and comfort limits
normalMinAcceleration = -3;
normalMaxAcceleration = 2;
emergencyAcceleration = -6;

maxJerk = 3;               % Maximum acceleration change (m/s^3)

% Lead-vehicle motion
leadAcceleration = zeros(1, N);
leadAcceleration(time >= 6 & time < 8) = -5;

leadSpeed = zeros(1, N);
leadSpeed(1) = 20;

% Ego-vehicle initial conditions
egoSpeed = zeros(1, N);
egoSpeed(1) = 25;

gap = zeros(1, N);
gap(1) = 59;

% Signals for analysis and plotting
relativeVelocity = zeros(1, N);
desiredGap = zeros(1, N);
timeGap = zeros(1, N);
ttc = inf(1, N);

accelerationRequest = zeros(1, N);
egoAcceleration = zeros(1, N);

for k = 1:N-1

    % Longitudinal quantities supplied by perception/fusion in a full system
    relativeVelocity(k) = leadSpeed(k) - egoSpeed(k);
    desiredGap(k) = minimumGap + desiredTimeGap * egoSpeed(k);
    timeGap(k) = gap(k) / max(egoSpeed(k), 0.1);

    % TTC is valid only while ego is closing on the lead vehicle
    if relativeVelocity(k) < 0
        ttc(k) = -gap(k) / relativeVelocity(k);
    else
        ttc(k) = inf;
    end

    % State transitions
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

    % Track state duration
    if state(k+1) == state(k)
        stateTime(k+1) = stateTime(k) + Ts;
    else
        stateTime(k+1) = 0;
    end

    % Controller action for the active state
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
            % Gentler command after emergency braking
            accelerationRequest(k) = ...
                0.10 * gapError + ...
                0.45 * relativeVelocity(k);
    end

    % Normal acceleration limits
    if state(k) ~= EMERGENCY_BRAKE
        accelerationRequest(k) = max(normalMinAcceleration, ...
                                 min(normalMaxAcceleration, ...
                                     accelerationRequest(k)));
    end

    % Jerk-limited actuator response.
    % Emergency Brake deliberately overrides this comfort limit.
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

    % Update lead-vehicle speed
    leadSpeed(k+1) = max(0, leadSpeed(k) + ...
                         leadAcceleration(k) * Ts);

    % Update ego-vehicle speed
    egoSpeed(k+1) = max(0, egoSpeed(k) + ...
                        egoAcceleration(k) * Ts);

    % Update relative distance
    gap(k+1) = max(0, gap(k) + relativeVelocity(k) * Ts);
end

% Fill final values for plotting
relativeVelocity(end) = leadSpeed(end) - egoSpeed(end);
desiredGap(end) = minimumGap + desiredTimeGap * egoSpeed(end);
timeGap(end) = gap(end) / max(egoSpeed(end), 0.1);

if relativeVelocity(end) < 0
    ttc(end) = -gap(end) / relativeVelocity(end);
end

accelerationRequest(end) = accelerationRequest(end-1);
egoAcceleration(end) = egoAcceleration(end-1);

% Show TTC only while it has a finite physical meaning
ttcPlot = ttc;
ttcPlot(isinf(ttcPlot)) = NaN;

% Main ACC behaviour figure
figure('Color', 'w');

subplot(4,1,1);
plot(time, gap, 'b', 'LineWidth', 2); hold on;
plot(time, desiredGap, 'k--', 'LineWidth', 1.5);
grid on;
ylabel('Distance (m)');
title('ACC Finite-State Machine with Emergency Hold Time');
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

% Acceleration and time-gap inputs figure
figure('Color', 'w');

subplot(2,1,1);
plot(time, accelerationRequest, 'm--', 'LineWidth', 1.5); hold on;
plot(time, egoAcceleration, 'm', 'LineWidth', 2);
plot(time, leadAcceleration, 'r--', 'LineWidth', 1.5);
grid on;
ylabel('Acceleration (m/s^2)');
title('Longitudinal Acceleration Commands');
legend('Ego request', 'Ego applied', 'Lead acceleration', ...
       'Location', 'best');

subplot(2,1,2);
plot(time, timeGap, 'b', 'LineWidth', 2); hold on;
yline(desiredTimeGap, 'k--', 'Desired time gap');
yline(followEntryTimeGap, 'r:', 'Follow entry');
grid on;
xlabel('Time (s)');
ylabel('Time gap (s)');
title('Time-Gap Decision Input');

% Emergency-state duration figure
figure('Color', 'w');
plot(time, stateTime, 'LineWidth', 1.8); hold on;
yline(minimumEmergencyTime, 'r--', ...
      'Minimum emergency hold time');
grid on;
xlabel('Time (s)');
ylabel('Time in current state (s)');
title('ACC State Duration');