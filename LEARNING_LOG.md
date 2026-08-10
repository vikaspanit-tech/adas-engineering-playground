# Learning Log

## 2026-08-02 — Highway Lane Following architecture phase complete

Completed an end-to-end architecture study of the MathWorks Highway Lane Following test bench.

### Covered

- ADAS pipeline, ego vehicle, and closed-loop system
- Simulation 3D Scenario: roads, actors, virtual sensors, and ground truth
- Lane Marker Detector and Vision Vehicle Detector interfaces
- Forward Vehicle Sensor Fusion interface and confirmed tracks
- Lane Following Decision Logic: lane centre and most important object (MIO) outputs
- Lane Following Controller: steering and acceleration interfaces
- Vehicle Dynamics: pose and velocity feedback
- Metrics Assessment: comparison of algorithm outputs with simulation ground truth

### Outcome

Consolidated the architecture study into one system-level document:

- `docs/02_architecture/Highway_Lane_Following_Test_Bench_Architecture.md`

### Next Focus

Begin Phase 2: Driving Scenarios, starting with reusable highway-road and traffic scenarios.
## 2026-08-04 — Driving Scenarios phase started

Created and simulated the first reusable Automated Driving Toolbox scenario.

### Scenario 01 — Straight Highway Lead Vehicle

- Created a 250 m, two-lane straight highway in Driving Scenario Designer.
- Defined `EgoVehicle` in the right lane at 25 m/s (90 km/h).
- Defined `LeadVehicle` ahead in the same lane at 20 m/s (72 km/h).
- Created a 60 m initial gap and a 5 m/s closing-speed condition.
- Verified the relative motion in the Designer simulation.
- Exported the scenario as a MATLAB function and saved the Designer project file.
- Added GitHub-ready scenario documentation and a simulation image.

### Files

- `matlab/02_driving_scenarios/scenario01_straight_highway_lead_vehicle.m`
- `matlab/02_driving_scenarios/scenario01_straight_highway_lead_vehicle.mat`
- `matlab/02_driving_scenarios/README.md`

### Next Focus

Create a curved-highway scenario, then build an intersection scenario before beginning virtual sensor simulation.
## 2026-08-06 — Curved highway scenario complete

Created, simulated, exported, and documented Scenario 02: Curved Highway Lead Vehicle.

### Covered

- Curved road creation using multiple road-centre points
- Two-lane road layout and lane markings
- Ego-vehicle trajectory design through changing road geometry
- Slower lead-vehicle trajectory in the same lane
- Relative-motion observation in an ego-centric simulation view
- Export of the Designer scenario to a clean MATLAB function

### Files

- `matlab/02_driving_scenarios/scenario02_curved_highway_lead_vehicle.m`
- `matlab/02_driving_scenarios/scenario02_curved_highway_lead_vehicle.mat`
- `assets/images/scenario02_curved_highway_lead_vehicle_simulation.png`

### Next Focus

Create an intersection scenario with crossing actors, then begin virtual-sensor simulation.
## 2026-08-06 — Intersection crossing scenario complete

Created, simulated, exported, and documented Scenario 03: Intersection Crossing Vehicle.

### Covered

- Multi-road intersection geometry
- Straight ego-vehicle motion through an intersection
- Perpendicular crossing-vehicle trajectory
- Deterministic timing of a cross-traffic conflict
- Ego-centric visual inspection of the unmitigated collision condition

### Files

- `matlab/02_driving_scenarios/scenario03_intersection_crossing_vehicle.m`
- `matlab/02_driving_scenarios/scenario03_intersection_crossing_vehicle.mat`
- `assets/images/scenario03_intersection_crossing_vehicle_conflict.png`

### Next Focus

Begin virtual sensor simulation with a forward-facing ego camera, its field of view, and synthetic detections.
## 2026-08-06 — Vehicle Dynamics fundamentals complete

Completed a foundational Vehicle Dynamics study using MATLAB implementations of the kinematic and dynamic bicycle models.

### Covered

- Vehicle world and body coordinate systems
- Kinematic bicycle model: position, heading, turning radius, Euler integration, time-varying steering, and longitudinal acceleration
- Dynamic bicycle model: lateral velocity, yaw rate, tyre slip angles, linear tyre forces, and side-slip angle
- Handling balance: baseline, understeer tendency, and oversteer tendency through front/rear cornering-stiffness changes
- Ackermann steering geometry and the bicycle-model steering approximation
- Steering actuator angle saturation and steering-rate limiting
- Direct kinematic-versus-dynamic trajectory and heading comparison

### Files

- `docs/03_vehicle_dynamics/Vehicle_Dynamics.md`
- `matlab/03_vehicle_dynamics/kinematic_bicycle_model.m`
- `matlab/03_vehicle_dynamics/dynamic_bicycle_model.m`
- `matlab/03_vehicle_dynamics/compare_kinematic_dynamic_bicycle_models.m`
- `matlab/03_vehicle_dynamics/ackermann_steering_geometry.m`
- `matlab/03_vehicle_dynamics/steering_actuator_limits.m`
- `assets/images/vehicle_dynamics_*.png`

### Outcome

Created a GitHub-ready Vehicle Dynamics chapter with equations, block diagrams, MATLAB-result screenshots, and links to each reproducible exercise.

### Next Focus

Begin Phase 4: Virtual Sensors — start with a forward-facing camera, field of view, and synthetic detections in a driving scenario.

## 2026-08-10 — Virtual Sensors phase complete

Configured, simulated, exported, and inspected a complete ego-vehicle virtual sensor suite in the straight-highway lead-vehicle scenario.

### Covered

- Forward camera: field of view, object detections, lane-boundary detections, and Ego Cartesian measurements
- Forward radar: range, relative velocity, field of view, measurement noise, and false alarms
- Roof LiDAR: 360 degree point-cloud coverage, top-view inspection, and lead-vehicle returns
- INS/GNSS: ego position, velocity, orientation, acceleration, angular velocity, and measurement noise
- Sensor-data export and inspection using `SensorIndex`, `TargetIndex`, `PointClouds`, and `INSMeasurements`
- Comparison of camera and radar measurements for the same lead vehicle

### Files

- `docs/04_sensors/Virtual_Sensors.md`
- `matlab/02_driving_scenarios/scenario04_straight_highway_front_camera.mat`
- `assets/images/virtual_sensors_*.png`

### Outcome

Completed a four-sensor simulation suite and validated each sensor's output against the known straight-highway scenario.

### Next Focus

Begin Phase 5: Perception — start with lane detection using the front-camera output.

