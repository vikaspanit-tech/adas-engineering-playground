%% Lead-Vehicle TTC and Longitudinal Driving Decision
% Run camera_radar_lead_vehicle_fusion.m first. This script uses the
% resulting eventTime, fusedRange, and fusedVelocity variables.

clearvars -except eventTime fusedRange fusedVelocity;
clc;

requiredVariables = {'eventTime','fusedRange','fusedVelocity'};
for k = 1:numel(requiredVariables)
    if ~exist(requiredVariables{k}, 'var')
        error(['Run camera_radar_lead_vehicle_fusion.m first. ' ...
            'Missing variable: ' requiredVariables{k}]);
    end
end

egoSpeed = 25;             % m/s, scenario ego speed
desiredTimeGap = 2.0;      % s
warningTTC = 3.0;          % s

timeGap = fusedRange / egoSpeed;
ttc = inf(size(eventTime));
closing = fusedVelocity < 0;
ttc(closing) = -fusedRange(closing) ./ fusedVelocity(closing);

% Numerical form makes the decision easy to plot.
% 1 = Follow lead vehicle, 2 = Maintain set speed, 3 = Brake.
decisionValue = 2 * ones(size(eventTime));
decisionValue(timeGap < desiredTimeGap) = 1;
decisionValue(ttc < warningTTC) = 3;

decisionLabel = strings(size(eventTime));
decisionLabel(decisionValue == 1) = "Follow lead vehicle";
decisionLabel(decisionValue == 2) = "Maintain set speed";
decisionLabel(decisionValue == 3) = "Brake";

firstFollowIndex = find(decisionValue == 1, 1);
if ~isempty(firstFollowIndex)
    fprintf('Follow-lead decision begins at %.2f s.\n', ...
        eventTime(firstFollowIndex));
end

fprintf('Minimum TTC: %.2f s\n', min(ttc(isfinite(ttc))));

figure('Color', 'w');

subplot(2,1,1)
plot(eventTime, timeGap, 'b', 'LineWidth', 1.8); hold on;
yline(desiredTimeGap, '--k', 'Desired gap: 2 s');
grid on;
ylabel('Time gap (s)');
title('Longitudinal Driving Decision Inputs');

subplot(2,1,2)
stairs(eventTime, decisionValue, 'LineWidth', 1.8);
ylim([0.5 3.5]);
yticks([1 2 3]);
yticklabels({'Follow lead vehicle', 'Maintain set speed', 'Brake'});
grid on;
xlabel('Time (s)');
ylabel('Decision');
