# Perception to Adaptive Cruise Control Prototype

## 1. Purpose and Scope

This chapter documents the first end-to-end ADAS prototype built from exported virtual-sensor data in the straight-highway lead-vehicle scenario. It connects sensor observations to a longitudinal driving response.

~~~text
camera and radar data
        -> perception and tracking
        -> safety metrics
        -> driving decision
        -> acceleration command
        -> closed-loop vehicle response
~~~

This is a learning prototype, not a production ACC system. The camera-radar fusion exercise uses the simulation-only TargetIndex to select the known lead vehicle. A real vehicle must associate detections to tracks without that ground-truth identifier.

## 2. System Architecture

~~~mermaid
flowchart LR
    C[Forward camera] --> LP[Lane perception]
    C --> CD[Camera lead-vehicle detections]
    R[Forward radar] --> RD[Radar lead-vehicle detections]
    LP --> LE[Lane centre, lateral error, heading error]
    CD --> KF[Kalman fusion]
    RD --> KF
    KF --> LS[Lead state: range and relative velocity]
    LS --> SM[Time gap and TTC]
    SM --> DL[Longitudinal decision]
    DL --> ACC[ACC controller]
    ACC --> PLANT[Vehicle and gap model]
    PLANT --> LS
~~~

### Data and Coordinate Conventions

| Quantity | Meaning | Sign convention used here |
|---|---|---|
| x | Forward distance in the ego frame | Positive in front of the ego vehicle |
| y | Lateral distance in the ego frame | Positive to the left, negative to the right |
| range | Longitudinal distance to the lead vehicle | Positive while the lead vehicle is ahead |
| relativeVelocity | Lead speed minus ego speed | Negative while the ego vehicle is closing in |
| TTC | Time to collision if current motion continues | Calculated only while closing in |

The exported allSensorData variable contains sensor data at every scenario time step. This chapter uses its LaneDetections plus camera and radar ObjectDetections.

## 3. Lane Perception and Lane-Centre Estimation

### Input and Boundary Model

The forward camera supplies two clothoid lane-boundary models. Each detected boundary contains its lateral offset, heading angle, curvature, and curvature derivative in ego coordinates.

At a forward coordinate x, the local boundary geometry is:

~~~text
y(x) = y0 + x * tan(headingAngle)
       + 0.5 * curvature * x^2
       + (1/6) * curvatureDerivative * x^3
~~~

Mathematically, the same clothoid approximation is:

$$
y(x)=y_0+x\tan(\psi)+\frac{1}{2}\kappa x^2+
\frac{1}{6}\dot{\kappa}x^3
$$

| Value | Meaning | Unit |
|---|---|---|
| y0 or LateralOffset | Boundary position at the ego reference point | m |
| headingAngle | Boundary direction relative to ego heading | rad |
| curvature | Change in heading per metre | 1/m |
| curvatureDerivative | Change in curvature per metre | 1/m^2 |

### Lane Centre and Errors

The lane centre is the average of the detected left and right boundaries:

~~~text
laneCentreOffset = (leftBoundaryOffset + rightBoundaryOffset) / 2
laneHeadingAngle = mean(leftHeadingAngle, rightHeadingAngle)
~~~

Using \(y_L\) and \(y_R\) for the left and right boundaries, the lateral
and heading errors at the ego reference point are:

$$
e_y=\frac{y_L+y_R}{2}
$$

$$
e_\psi=\frac{\psi_L+\psi_R}{2}
$$

For the first valid straight-highway frame:

~~~text
Left boundary   = +1.863 m
Right boundary  = -1.705 m
Lane centre     = +0.079 m
Heading error   = -0.0208 rad = -1.19 deg
~~~

The positive centre offset indicates that the ego vehicle is estimated to be about 7.9 cm to the right of lane centre. This is a small, plausible perception error for a vehicle that is almost centred in its lane.

![Camera lane-following errors](../../assets/images/perception_lane_following_errors.png)

Reproduce with [lane_center_estimation.m](../../matlab/04_lane_detection/lane_center_estimation.m).

## 4. Lead-Vehicle Perception and Camera-Radar Fusion

### Why Fusion Is Necessary

| Sensor | Strength in this scenario | Limitation |
|---|---|---|
| Camera | Observes scene structure, vehicles, and lanes | Range and velocity estimates are visibly noisy |
| Radar | Accurate range and range rate | Does not provide lane boundaries or image context |

Fusion produces a single, stable target estimate for downstream decision making.

### Kalman Filter State and Prediction

The track state contains the two longitudinal quantities required for car following:

~~~text
x = [range; relativeVelocity]
~~~

$$
\mathbf{x}_k=
\begin{bmatrix}
d_k \\
v_{\mathrm{rel},k}
\end{bmatrix}
$$

The constant-relative-velocity prediction model is:

~~~text
range(k+1)            = range(k) + relativeVelocity(k) * dt
relativeVelocity(k+1) = relativeVelocity(k)

F = [1  dt
     0   1]
~~~

$$
\mathbf{x}_{k|k-1}=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\mathbf{x}_{k-1|k-1}
$$

The process-noise term allows the lead vehicle to accelerate or decelerate instead of assuming perfect constant motion.

### Measurement Update

For every asynchronous camera or radar event, the filter predicts then corrects:

~~~text
innovation = measurement - predictedMeasurement
KalmanGain = predictedCovariance * H-transpose / innovationCovariance
updatedState = predictedState + KalmanGain * innovation
~~~

The standard Kalman update is:

$$
\mathbf{K}_k=\mathbf{P}_{k|k-1}\mathbf{H}^{T}
\left(\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^{T}+\mathbf{R}\right)^{-1}
$$

$$
\mathbf{x}_{k|k}=\mathbf{x}_{k|k-1}+
\mathbf{K}_k\left(\mathbf{z}_k-\mathbf{H}\mathbf{x}_{k|k-1}\right)
$$

Radar is assigned lower measurement uncertainty than the camera. It therefore has greater influence on the fused estimate. The black fused track remains close to the accurate radar data and does not chase the camera velocity outliers.

![Camera-radar Kalman fusion](../../assets/images/perception_camera_radar_kalman_fusion.png)

Reproduce with [camera_radar_lead_vehicle_fusion.m](../../matlab/05_sensor_fusion/camera_radar_lead_vehicle_fusion.m).

## 5. Time Gap, TTC, and Longitudinal Decision

### Time Gap

Time gap is the time required for the ego vehicle to reach the lead vehicle's current position at the current ego speed:

~~~text
timeGap = range / egoSpeed
~~~

$$
t_{\mathrm{gap}}=\frac{d}{v_{\mathrm{ego}}}
$$

The prototype uses a desired time gap of 2 seconds. At an ego speed of 25 m/s, this is a nominal 50 m following gap before adding any standstill-distance term.

### Time to Collision

TTC predicts the collision time if range and relative velocity remained unchanged:

~~~text
TTC = -range / relativeVelocity
~~~

$$
\mathrm{TTC}=-\frac{d}{v_{\mathrm{rel}}},\qquad v_{\mathrm{rel}}<0
$$

This is meaningful only when relativeVelocity is negative. If the ego vehicle is not closing in, collision is not predicted and TTC is treated as infinity.

Example:

~~~text
range = 50 m
relativeVelocity = -5 m/s
TTC = -50 / -5 = 10 s
~~~

### Decision Logic

~~~mermaid
flowchart TD
    A[Receive fused range and relative velocity] --> B[Calculate time gap and TTC]
    B --> C{TTC below warning threshold?}
    C -- Yes --> D[Brake]
    C -- No --> E{Time gap below desired gap?}
    E -- Yes --> F[Follow lead vehicle]
    E -- No --> G[Maintain set speed]
~~~

| Condition | Decision | Purpose |
|---|---|---|
| TTC below warning threshold | Brake | Collision-risk protection |
| Time gap below 2 s | Follow lead vehicle | Restore a comfortable gap |
| Otherwise | Maintain set speed | Normal cruise behaviour |

In the exported scenario, the time gap crosses below 2 seconds around 1.8 seconds. TTC stays above the 3-second warning threshold. Therefore the appropriate action is Follow lead vehicle, not emergency braking.

![TTC and longitudinal decision](../../assets/images/control_acc_ttc_decision.png)

Reproduce with [lead_vehicle_ttc_decision.m](../../matlab/06_control/lead_vehicle_ttc_decision.m). Run the fusion script first because this script uses its fused range and velocity outputs.

## 6. Closed-Loop Adaptive Cruise Control

### Desired Following Distance

The desired distance combines a minimum standstill gap with a speed-dependent time gap:

~~~text
desiredDistance = minimumGap + desiredTimeGap * egoSpeed
~~~

$$
d_{\mathrm{desired}}=d_{\min}+T_{\mathrm{gap}}v_{\mathrm{ego}}
$$

At the final lead speed of 20 m/s:

~~~text
desiredDistance = 5 m + 2 s * 20 m/s = 45 m
~~~

### ACC Control Law

The controller combines spacing error with relative-velocity error:

~~~text
distanceError = actualGap - desiredDistance
accelerationTarget = Kp * distanceError + Kv * relativeVelocity
~~~

$$
e_d=d-d_{\mathrm{desired}}
$$

$$
a^*=K_p e_d+K_vv_{\mathrm{rel}}
$$

| Element | Role |
|---|---|
| Kp times distance error | Corrects a gap that is too large or too small |
| Kv times relative velocity | Reacts to how quickly the gap is changing |
| Acceleration saturation | Keeps normal response comfortable |
| Jerk limit | Avoids sudden acceleration changes |

The simple vehicle model updates with forward-Euler integration:

~~~text
egoSpeed(k+1) = egoSpeed(k) + accelerationCommand(k) * dt
gap(k+1) = gap(k) + relativeVelocity(k) * dt
relativeVelocity = leadSpeed - egoSpeed
~~~

$$
v_{\mathrm{ego},k+1}=v_{\mathrm{ego},k}+a_k\Delta t
$$

$$
d_{k+1}=d_k+v_{\mathrm{rel},k}\Delta t
$$

The jerk limit constrains acceleration changes:

$$
\left|a_k-a_{k-1}\right|\leq j_{\max}\Delta t
$$

The normal-following response is stable: ego speed falls from 25 m/s to the 20 m/s lead speed and the actual gap settles near 45 m.

![Normal closed-loop ACC response](../../assets/images/control_acc_closed_loop_normal.png)

## 7. Sudden Lead-Vehicle Braking and Emergency Override

The stress test makes the lead vehicle brake at -5 m/s^2 from 6 s to 8 s, reducing its speed from 20 m/s to 10 m/s. A TTC threshold of 5.5 seconds deliberately activates the emergency path.

~~~mermaid
stateDiagram-v2
    [*] --> NormalACC
    NormalACC --> EmergencyBrake: TTC below warning threshold
    EmergencyBrake --> NormalACC: simplified prototype returns immediately
~~~

When TTC falls below the threshold, normal comfort limits are bypassed:

~~~text
accelerationCommand = -6 m/s^2
~~~

![Emergency-braking response](../../assets/images/control_acc_emergency_braking.png)

### Result and Limitation

The test proves that the TTC override works. It also exposes a limitation: after the TTC condition clears, the prototype returns immediately to normal ACC. It can consequently over-brake and then accelerate too strongly while recovering.

This is a useful engineering result because it identifies the missing design feature: controller modes with hysteresis.

## 8. Production Limitations and Next Step

The prototype deliberately omits several production requirements:

- TargetIndex is simulation-only; real systems need association and track management.
- Only one lead vehicle is considered.
- The vehicle model is longitudinal and simplified.
- Emergency-brake recovery has no state retention or hysteresis.
- Sensor validation, fault handling, and actuator dynamics are simplified.

The next version should use three explicit modes:

~~~mermaid
stateDiagram-v2
    [*] --> Cruise
    Cruise --> Follow: time gap below follow threshold
    Follow --> Cruise: time gap above release threshold
    Follow --> EmergencyBrake: TTC below emergency threshold
    Cruise --> EmergencyBrake: TTC below emergency threshold
    EmergencyBrake --> Follow: speed and safety margin restored
~~~

The separate entry and exit thresholds prevent rapid switching. This behavior is called hysteresis.

## 9. Reproducibility Checklist

1. Export the straight-highway camera/radar scenario into MATLAB as allSensorData.
2. Run [lane_center_estimation.m](../../matlab/04_lane_detection/lane_center_estimation.m).
3. Run [camera_radar_lead_vehicle_fusion.m](../../matlab/05_sensor_fusion/camera_radar_lead_vehicle_fusion.m).
4. Without clearing the workspace, run [lead_vehicle_ttc_decision.m](../../matlab/06_control/lead_vehicle_ttc_decision.m).
5. Run [closed_loop_acc_emergency_braking.m](../../matlab/06_control/closed_loop_acc_emergency_braking.m) for the independent ACC stress test.
