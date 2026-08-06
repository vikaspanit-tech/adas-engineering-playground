%% Ackermann Steering Geometry
clear; clc; close all;

% Vehicle geometry
L = 2.8;         % Wheelbase (m)
track = 1.6;     % Track width (m)
R = 15;          % Turning radius at vehicle centre (m)

% Ackermann steering angles
deltaInner = atan(L / (R - track/2));
deltaOuter = atan(L / (R + track/2));

% Equivalent bicycle-model steering angle
deltaBicycle = atan(L / R);

% Display results
fprintf('Turning radius: %.2f m\n\n', R);
fprintf('Inner-wheel angle:  %.2f deg\n', rad2deg(deltaInner));
fprintf('Outer-wheel angle:  %.2f deg\n', rad2deg(deltaOuter));
fprintf('Bicycle-model angle: %.2f deg\n', rad2deg(deltaBicycle));

% Visual comparison
figure('Color', 'w');
bar(rad2deg([deltaInner, deltaBicycle, deltaOuter]));
grid on;
set(gca, 'XTickLabel', ...
    {'Inner wheel', 'Bicycle equivalent', 'Outer wheel'});
ylabel('Steering angle (deg)');
title('Ackermann Steering Angle Comparison');