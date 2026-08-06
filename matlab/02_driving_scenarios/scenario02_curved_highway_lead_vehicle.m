function [scenario, egoVehicle] = scenario02_curved_highway_lead_vehicle()
%SCENARIO02_CURVED_HIGHWAY_LEAD_VEHICLE Create a curved-road lead-vehicle scenario.
%
% This scenario contains a two-lane curved highway, an ego vehicle, and a
% slower lead vehicle travelling in the same lane. It is intended for
% developing and validating lane-following behaviour on curved roads.

% Create the scenario container.
scenario = drivingScenario;

% Define the curved road centreline and its two-lane layout.
roadCenters = [43.4  8.8   0;
               18.5  8.7   0;
               18.4 -22.6  0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, ...
    'Lanes', laneSpecification, ...
    'Name', 'CurvedHighway');

% Create the ego vehicle and define its trajectory through the curve.
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [38.3 8.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'EgoVehicle');

egoWaypoints = [38.3   8.2   0;
                28.9  10.2   0;
                20.6   8.0   0;
                15.4   0.4   0;
                15.5 -11.8   0;
                18.9 -21.0   0];
egoSpeed = 15 * ones(size(egoWaypoints,1),1);
trajectory(egoVehicle, egoWaypoints, egoSpeed);

% Create the slower lead vehicle on the same curved route.
leadVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [23.8 9.7 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'LeadVehicle');

leadWaypoints = [23.8   9.7   0;
                 17.0   4.5   0;
                 14.4  -6.3   0;
                 18.7 -21.1   0];
leadSpeed = 12 * ones(size(leadWaypoints,1),1);
trajectory(leadVehicle, leadWaypoints, leadSpeed);
end