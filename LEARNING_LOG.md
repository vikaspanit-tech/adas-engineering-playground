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