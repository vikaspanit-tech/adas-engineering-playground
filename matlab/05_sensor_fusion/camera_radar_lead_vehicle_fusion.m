%% Camera-Radar Kalman Fusion for the Simulated Lead Vehicle
% Requires allSensorData from Driving Scenario Designer.
% TargetIndex is simulation ground truth, used only to select this target.
clearvars -except allSensorData; clc;

if ~exist('allSensorData','var')
    error('Export sensor data first; the workspace variable must be allSensorData.');
end

leadTargetIndex = 2; cameraSensorIndex = 1; radarSensorIndex = 2;
cameraTime=[]; cameraRange=[]; cameraVelocity=[];
radarTime=[]; radarRange=[]; radarVelocity=[];

for k = 1:numel(allSensorData)
    detections = allSensorData(k).ObjectDetections;
    for n = 1:numel(detections)
        detection = detections{n};
        if isempty(detection.ObjectAttributes) || ...
                detection.ObjectAttributes{1}.TargetIndex ~= leadTargetIndex
            continue
        end
        z = detection.Measurement;
        if detection.SensorIndex == cameraSensorIndex
            cameraTime(end+1,1)=allSensorData(k).Time;
            cameraRange(end+1,1)=z(1); cameraVelocity(end+1,1)=z(4);
        elseif detection.SensorIndex == radarSensorIndex
            radarTime(end+1,1)=allSensorData(k).Time;
            radarRange(end+1,1)=z(1); radarVelocity(end+1,1)=z(4);
        end
    end
end

if isempty(cameraTime) || isempty(radarTime)
    error('Camera or radar lead-vehicle measurements were not found.');
end

% Asynchronous measurement events. Source 1 is radar; source 2 is camera.
eventTime=[radarTime;cameraTime];
eventSource=[ones(numel(radarTime),1);2*ones(numel(cameraTime),1)];
eventRange=[radarRange;cameraRange];
eventVelocity=[radarVelocity;cameraVelocity];
[eventTime,order]=sort(eventTime);
eventSource=eventSource(order); eventRange=eventRange(order);
eventVelocity=eventVelocity(order);

% State: [forward range; relative velocity].
x=[radarRange(1);radarVelocity(1)]; P=diag([1,0.5^2]); H=eye(2);
Rradar=diag([0.25^2,0.05^2]); Rcamera=diag([1.5^2,2.0^2]);
accelerationStd=0.5;
fusedRange=zeros(size(eventTime)); fusedVelocity=zeros(size(eventTime));
previousTime=eventTime(1);

for k = 1:numel(eventTime)
    dt=eventTime(k)-previousTime;
    F=[1 dt;0 1]; G=[0.5*dt^2;dt]; Q=accelerationStd^2*(G*G.');
    xPrediction=F*x; PPrediction=F*P*F.'+Q;
    if eventSource(k)==1, R=Rradar; else, R=Rcamera; end
    innovation=[eventRange(k);eventVelocity(k)]-H*xPrediction;
    S=H*PPrediction*H.'+R; K=PPrediction*H.'/S;
    x=xPrediction+K*innovation; P=(eye(2)-K*H)*PPrediction;
    fusedRange(k)=x(1); fusedVelocity(k)=x(2); previousTime=eventTime(k);
end

figure('Color','w');
subplot(2,1,1)
plot(cameraTime,cameraRange,'bo'); hold on; plot(radarTime,radarRange,'rx');
plot(eventTime,fusedRange,'k','LineWidth',1.8); grid on;
ylabel('Forward distance (m)'); title('Camera-Radar Kalman Fusion for Lead Vehicle');
legend('Camera','Radar','Fused track','Location','best');
subplot(2,1,2)
plot(cameraTime,cameraVelocity,'bo'); hold on; plot(radarTime,radarVelocity,'rx');
plot(eventTime,fusedVelocity,'k','LineWidth',1.8); grid on;
xlabel('Time (s)'); ylabel('Relative velocity (m/s)');
legend('Camera','Radar','Fused track','Location','best');
