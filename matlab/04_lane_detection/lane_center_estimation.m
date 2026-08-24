%% Lane-Centre Estimation from Exported Camera Detections
% Requires allSensorData exported from Driving Scenario Designer.
clearvars -except allSensorData; clc;

if ~exist('allSensorData','var')
    error('Export sensor data first; the workspace variable must be allSensorData.');
end

hasLane = arrayfun(@(frame) ~isempty(frame.LaneDetections), allSensorData);
frameIndices = find(hasLane);
if isempty(frameIndices), error('No lane detections found.'); end

laneTime = zeros(numel(frameIndices),1);
lateralError = zeros(numel(frameIndices),1);
headingError = zeros(numel(frameIndices),1);

for k = 1:numel(frameIndices)
    frame = allSensorData(frameIndices(k));
    lanes = frame.LaneDetections.LaneBoundaries;
    offsets = arrayfun(@(lane) lane.LateralOffset,lanes);
    lateralError(k) = (max(offsets)+min(offsets))/2;
    headingError(k) = mean([lanes.HeadingAngle]);
    laneTime(k) = frame.Time;
end

% Inspect geometry in the first valid frame.
lanes = allSensorData(frameIndices(1)).LaneDetections.LaneBoundaries;
offsets = arrayfun(@(lane) lane.LateralOffset,lanes);
[~,leftIndex] = max(offsets); [~,rightIndex] = min(offsets);
leftLane = lanes(leftIndex); rightLane = lanes(rightIndex);

fprintf('Left boundary:  %.3f m\n', leftLane.LateralOffset);
fprintf('Right boundary: %.3f m\n', rightLane.LateralOffset);
fprintf('Lane centre:    %.3f m\n', lateralError(1));
fprintf('Heading error:  %.4f rad (%.2f deg)\n', ...
    headingError(1),rad2deg(headingError(1)));

x = linspace(0,60,200);
leftY = evaluateBoundary(leftLane,x);
rightY = evaluateBoundary(rightLane,x);

figure('Color','w');
plot(x,leftY,'b','LineWidth',2); hold on;
plot(x,rightY,'b','LineWidth',2);
plot(x,(leftY+rightY)/2,'--','Color',[0 0.7 0],'LineWidth',2);
plot(0,0,'ko','MarkerFaceColor','k'); grid on;
xlabel('Forward distance, X (m)'); ylabel('Lateral offset, Y (m)');
title('Camera-Based Lane Boundaries and Estimated Lane Centre');
legend('Left boundary','Right boundary','Lane centre','Ego vehicle reference', ...
    'Location','best');

figure('Color','w');
subplot(2,1,1)
plot(laneTime,lateralError,':','LineWidth',2.5); grid on;
ylabel('Lateral error (m)'); title('Camera-Based Lane-Following Errors');
subplot(2,1,2)
plot(laneTime,rad2deg(headingError),':','LineWidth',2.5); grid on;
xlabel('Time (s)'); ylabel('Heading error (deg)');

function y = evaluateBoundary(lane,x)
y = lane.LateralOffset + x.*tan(lane.HeadingAngle) + ...
    0.5*lane.Curvature*x.^2 + (1/6)*lane.CurvatureDerivative*x.^3;
end
