%% Kinematic vs Dynamic Bicycle Model
clear; clc; close all;

% Shared parameters
L = 2.8;                 % Wheelbase (m)
u = 15;                  % Constant longitudinal speed (m/s)
Ts = 0.001;
Tend = 10;
time = 0:Ts:Tend;

% Same steering input for both models
delta = zeros(size(time));
delta(time >= 1 & time < 6) = deg2rad(5);

%% Kinematic bicycle model
XKin = zeros(size(time));
YKin = zeros(size(time));
psiKin = zeros(size(time));

for k = 1:numel(time)-1
    psiDot = (u / L) * tan(delta(k));

    XKin(k+1) = XKin(k) + Ts * u * cos(psiKin(k));
    YKin(k+1) = YKin(k) + Ts * u * sin(psiKin(k));
    psiKin(k+1) = psiKin(k) + Ts * psiDot;
end

%% Dynamic bicycle model parameters
m = 1500;
Iz = 3000;
lf = 1.2;
lr = 1.6;
Cf = 80000;
Cr = 80000;

XDyn = zeros(size(time));
YDyn = zeros(size(time));
psiDyn = zeros(size(time));
vLat = zeros(size(time));
yawRate = zeros(size(time));

for k = 1:numel(time)-1
    alphaF = (vLat(k) + lf * yawRate(k)) / u - delta(k);
    alphaR = (vLat(k) - lr * yawRate(k)) / u;

    Fyf = -Cf * alphaF;
    Fyr = -Cr * alphaR;

    vLatDot = (Fyf + Fyr) / m - u * yawRate(k);
    yawRateDot = (lf * Fyf - lr * Fyr) / Iz;

    XDot = u * cos(psiDyn(k)) - vLat(k) * sin(psiDyn(k));
    YDot = u * sin(psiDyn(k)) + vLat(k) * cos(psiDyn(k));

    vLat(k+1) = vLat(k) + Ts * vLatDot;
    yawRate(k+1) = yawRate(k) + Ts * yawRateDot;
    psiDyn(k+1) = psiDyn(k) + Ts * yawRate(k);
    XDyn(k+1) = XDyn(k) + Ts * XDot;
    YDyn(k+1) = YDyn(k) + Ts * YDot;
end

%% Compare trajectories
figure('Color', 'w');
plot(XKin, YKin, '--', 'LineWidth', 2);
hold on;
plot(XDyn, YDyn, '-', 'LineWidth', 2);
grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Kinematic vs Dynamic Bicycle Model');
legend('Kinematic model', 'Dynamic model', 'Location', 'best');

%% Compare heading response
figure('Color', 'w');
plot(time, rad2deg(psiKin), '--', 'LineWidth', 2);
hold on;
plot(time, rad2deg(psiDyn), '-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Heading angle (deg)');
title('Heading Response Comparison');
legend('Kinematic model', 'Dynamic model', 'Location', 'best');