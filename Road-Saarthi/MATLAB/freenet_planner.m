%% STEP 3: Collision-free path planner (real Frenet planner)
% RUN AFTER step2_sensor_config.m.
% This is the "upgrade" from the original script's closing note: it uses
% your installed Navigation Toolbox to build a real reference path and
% Frenet trajectory generator, replacing the plain lateral-offset
% heuristic. Requires Navigation Toolbox.

clear; clc; close all;

load('sensor_data.mat');

dangerZoneAhead = 15;   % m, look-ahead distance that triggers avoidance
dangerZoneWidth = 3;    % m, lateral width considered "in the way"
avoidOffset     = 3;    % m, how far to swerve laterally
replanHorizon   = 1.0;  % s, how far ahead each Frenet replan looks

% Straight-road reference path along the centerline (x from 0 to 100)
refWaypoints = [0 0; 25 0; 50 0; 75 0; 100 0];

% Build once here just to confirm the toolbox call works and to report
% basic path info. step4 rebuilds these objects fresh (objects aren't
% saved to the .mat file - only the plain waypoint data is).
refPath = referencePathFrenet(refWaypoints); %#ok<NASGU>
connector = trajectoryGeneratorFrenet(refPath, 'TimeResolution', dt); %#ok<NASGU>

fprintf('STEP 3 ready: referencePathFrenet + trajectoryGeneratorFrenet configured.\n');
fprintf('  dangerZoneAhead=%.1fm  dangerZoneWidth=%.1fm  avoidOffset=%.1fm  replanHorizon=%.1fs\n', ...
    dangerZoneAhead, dangerZoneWidth, avoidOffset, replanHorizon);

save('planner_data.mat', 'dt', 'simTime', 'steps', 'egoSpeed', 'egoStart', 'obstacles', ...
    'sensorRange', 'sensorFOV', 'noiseStd', 'dangerZoneAhead', 'dangerZoneWidth', ...
    'avoidOffset', 'replanHorizon', 'refWaypoints');
fprintf('Saved planner_data.mat -> run step4_closed_loop_sim.m next.\n');