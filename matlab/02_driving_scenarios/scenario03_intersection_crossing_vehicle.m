function [scenario, egoVehicle] = scenario03_intersection_crossing_vehicle()
%SCENARIO03_INTERSECTION_CROSSING_VEHICLE Create a cross-traffic conflict scenario.
%
% This scenario contains a two-road intersection, an ego vehicle travelling
% through the main road, and a crossing vehicle timed to enter the
% intersection at the same time. It provides a repeatable unmitigated
% conflict case for future perception, warning, planning, and braking work.

% Create the scenario container.
scenario = drivingScenario;

% Define the two-lane main road used by the ego vehicle.
mainRoadCenters = [52.3 2.7 0;
                    4.4 2.0 0];
mainLaneSpecification = lanespec(2);
road(scenario, mainRoadCenters, ...
    'Lanes', mainLaneSpecification, ...
    'Name', 'MainRoad');

% Define the two-lane road crossing the main road.
crossRoadCenters = [29.6  27.2 0;
                    30.1 -24.1 0];
crossLaneSpecification = lanespec(2);
road(scenario, crossRoadCenters, ...
    'Lanes', crossLaneSpecification, ...
    'Name', 'CrossRoad');

% Create the ego vehicle and its path through the intersection.
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [6 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'EgoVehicle');

egoWaypoints = [ 6 0 0;
                24 0 0;
                36 0 0;
                48 0 0];
egoSpeed = 10 * ones(size(egoWaypoints,1),1);
trajectory(egoVehicle, egoWaypoints, egoSpeed);

% Create the crossing vehicle and time its entry into the intersection.
crossingVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [31.1 21.6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'CrossingVehicle');

crossingWaypoints = [31.1  21.6 0;
                     31.0   0.0 0;
                     31.8 -21.7 0];
crossingSpeed = 9 * ones(size(crossingWaypoints,1),1);
trajectory(crossingVehicle, crossingWaypoints, crossingSpeed);
end