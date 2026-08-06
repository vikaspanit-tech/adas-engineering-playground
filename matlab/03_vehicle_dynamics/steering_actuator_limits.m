%% Steering Actuator Limits
clear; clc; close all;

% Simulation settings
Ts = 0.01;
Tend = 8;
time = 0:Ts:Tend;

% Physical steering limits
maxAngle = deg2rad(30);      % Maximum road-wheel angle (rad)
maxRate = deg2rad(60);       % Maximum steering rate (rad/s)

% Controller's requested road-wheel angle
deltaDesired = zeros(size(time));
%deltaDesired(time >= 1 & time < 4) = deg2rad(25);
deltaDesired(time >= 1 & time < 4) = deg2rad(40);
deltaDesired(time >= 4 & time < 6) = deg2rad(-20);

% Actual actuator output
deltaActual = zeros(size(time));

for k = 1:numel(time)-1

    % Saturate requested angle
    requestedAngle = min(max(deltaDesired(k), -maxAngle), maxAngle);

    % Limit the change per sample
    maxStep = maxRate * Ts;
    angleError = requestedAngle - deltaActual(k);

    limitedStep = min(max(angleError, -maxStep), maxStep);
    deltaActual(k+1) = deltaActual(k) + limitedStep;
end

% Plot command and actuator response
figure('Color', 'w');
plot(time, rad2deg(deltaDesired), '--', 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(deltaActual), 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Road-wheel angle (deg)');
title('Steering Angle Saturation and Rate Limiting');
legend('Controller request', 'Actuator output', 'Location', 'best');