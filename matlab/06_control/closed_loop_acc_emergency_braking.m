%% Closed-Loop ACC: Sudden Lead-Vehicle Braking Test
clear; clc; close all;

Ts=0.05; Tend=20; time=0:Ts:Tend; N=numel(time);
setSpeed=25; initialLeadSpeed=20; initialGap=59;
minimumGap=5; desiredTimeGap=2.0; warningTTC=5.5;
emergencyAcceleration=-6;
Kp=0.15; Kv=0.60; minimumAcceleration=-3; maximumAcceleration=2;
maximumJerk=1.5;

leadSpeed=zeros(1,N); egoSpeed=zeros(1,N); gap=zeros(1,N);
leadAcceleration=zeros(1,N); relativeVelocity=zeros(1,N);
desiredDistance=zeros(1,N); ttc=inf(1,N);
accelerationCommand=zeros(1,N); emergencyBrakeActive=false(1,N);
leadSpeed(1)=initialLeadSpeed; egoSpeed(1)=setSpeed; gap(1)=initialGap;

for k=1:N-1
    if time(k)>=6 && time(k)<8, leadAcceleration(k)=-5; end
    leadSpeed(k+1)=max(0,leadSpeed(k)+Ts*leadAcceleration(k));
    relativeVelocity(k)=leadSpeed(k)-egoSpeed(k);
    desiredDistance(k)=minimumGap+desiredTimeGap*egoSpeed(k);
    if relativeVelocity(k)<0, ttc(k)=-gap(k)/relativeVelocity(k); end

    distanceError=gap(k)-desiredDistance(k);
    accelerationTarget=Kp*distanceError+Kv*relativeVelocity(k);
    accelerationTarget=min(max(accelerationTarget,minimumAcceleration), ...
        maximumAcceleration);

    if ttc(k)<warningTTC
        emergencyBrakeActive(k)=true;
        accelerationCommand(k)=emergencyAcceleration;
    else
        previousAcceleration=accelerationCommand(max(k-1,1));
        maximumChange=maximumJerk*Ts;
        accelerationCommand(k)=previousAcceleration+ ...
            min(max(accelerationTarget-previousAcceleration, ...
            -maximumChange),maximumChange);
    end
    egoSpeed(k+1)=max(0,egoSpeed(k)+Ts*accelerationCommand(k));
    gap(k+1)=max(0,gap(k)+Ts*relativeVelocity(k));
end

relativeVelocity(end)=leadSpeed(end)-egoSpeed(end);
desiredDistance(end)=minimumGap+desiredTimeGap*egoSpeed(end);
leadAcceleration(end)=leadAcceleration(end-1);
accelerationCommand(end)=accelerationCommand(end-1);
if relativeVelocity(end)<0, ttc(end)=-gap(end)/relativeVelocity(end); end

% Large TTC means speed matching, not a near-term collision.
ttcForPlot=ttc; ttcForPlot(ttcForPlot>20)=NaN;

figure('Color','w');
subplot(4,1,1)
plot(time,gap,'b','LineWidth',1.8); hold on;
plot(time,desiredDistance,'--k','LineWidth',1.5); grid on;
ylabel('Distance (m)'); title('ACC Response to Sudden Lead-Vehicle Braking');
legend('Actual gap','Desired gap','Location','best');
subplot(4,1,2)
plot(time,egoSpeed,'b','LineWidth',1.8); hold on;
plot(time,leadSpeed,'r','LineWidth',1.8); grid on; ylabel('Speed (m/s)');
legend('Ego vehicle','Lead vehicle','Location','best');
subplot(4,1,3)
plot(time,ttcForPlot,'r','LineWidth',1.8); hold on;
yline(warningTTC,'--k','TTC warning threshold'); ylim([0 20]); grid on;
ylabel('TTC (s)');
subplot(4,1,4)
plot(time,accelerationCommand,'m','LineWidth',1.8); hold on;
plot(time,leadAcceleration,'--r','LineWidth',1.3); yline(0,'--k'); grid on;
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
legend('Ego command','Lead acceleration','Location','best');
