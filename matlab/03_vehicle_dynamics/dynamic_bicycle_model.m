%% Dynamic Bicycle Model
clear; clc; close all;

% Vehicle parameters
m = 1500;                % Vehicle mass (kg)
Iz = 3000;               % Yaw moment of inertia (kg*m^2)
lf = 1.2;                % CG to front axle (m)
lr = 1.6;                % CG to rear axle (m)
Cf = 80000;              % Front cornering stiffness (N/rad)
Cr = 65000;              % Rear cornering stiffness (N/rad)

% Constant longitudinal speed
u = 15;                  % Forward speed (m/s)

% Simulation parameters
Ts = 0.001;              % Sample time (s)
Tend = 10;               % Duration (s)
time = 0:Ts:Tend;

% Steering command: turn left, then return wheels to centre
delta = zeros(size(time));
delta(time >= 1 & time < 6) = deg2rad(5);

% States
X = zeros(size(time));       % World X position (m)
Y = zeros(size(time));       % World Y position (m)
psi = zeros(size(time));     % Heading angle (rad)
v = zeros(size(time));       % Lateral velocity (m/s)
r = zeros(size(time));       % Yaw rate (rad/s)

alphaFHistory = zeros(size(time));
alphaRHistory = zeros(size(time));
FyfHistory = zeros(size(time));
FyrHistory = zeros(size(time));

% Dynamic bicycle model
for k = 1:numel(time)-1

    % Tire slip angles
    alphaF = (v(k) + lf * r(k)) / u - delta(k);
    alphaR = (v(k) - lr * r(k)) / u;

    % Linear tire model
    Fyf = -Cf * alphaF;
    Fyr = -Cr * alphaR;

    % Vehicle dynamics
    vDot = (Fyf + Fyr) / m - u * r(k);
    rDot = (lf * Fyf - lr * Fyr) / Iz;

    alphaFHistory(k) = alphaF;
    alphaRHistory(k) = alphaR;
    FyfHistory(k) = Fyf;
    FyrHistory(k) = Fyr;

    % Vehicle pose in world frame
    XDot = u * cos(psi(k)) - v(k) * sin(psi(k));
    YDot = u * sin(psi(k)) + v(k) * cos(psi(k));

    % Euler integration
    v(k+1) = v(k) + Ts * vDot;
    r(k+1) = r(k) + Ts * rDot;
    psi(k+1) = psi(k) + Ts * r(k);
    X(k+1) = X(k) + Ts * XDot;
    Y(k+1) = Y(k) + Ts * YDot;
end

alphaFHistory(end) = alphaFHistory(end-1);
alphaRHistory(end) = alphaRHistory(end-1);
FyfHistory(end) = FyfHistory(end-1);
FyrHistory(end) = FyrHistory(end-1);

figure('Color', 'w');

subplot(2,1,1);
plot(time, rad2deg(alphaFHistory), 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(alphaRHistory), 'LineWidth', 1.5);
grid on;
ylabel('Slip angle (deg)');
legend('Front tire', 'Rear tire', 'Location', 'best');
title('Tire Slip Angles');

subplot(2,1,2);
plot(time, FyfHistory, 'LineWidth', 1.5);
hold on;
plot(time, FyrHistory, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Lateral force (N)');
legend('Front tire', 'Rear tire', 'Location', 'best');
title('Tire Lateral Forces');


beta = atan2(v, u);
% Plot trajectory
figure('Color','w');
plot(X, Y, 'b-', 'LineWidth', 2);
grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Dynamic Bicycle Model Trajectory');

% Plot lateral response
figure('Color','w');

subplot(3,1,1);
plot(time, rad2deg(delta), 'LineWidth', 1.5);
grid on;
ylabel('Steering angle (deg)');
title('Steering Input and Vehicle Response');

subplot(3,1,2);
plot(time, rad2deg(r), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');

subplot(3,1,3);
plot(time, rad2deg(beta), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Side-slip angle (deg)');
title('Vehicle Side-Slip Angle');