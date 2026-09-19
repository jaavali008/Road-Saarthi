%% STEP 2: Sensor detection (simulated perception)
% RUN AFTER step1_scenario_setup.m.
% Loads scenario_data.mat, configures the simulated sensor cone (range +
% field-of-view + measurement noise), and prints a single-frame
% detection check so you can confirm it's wired up correctly before
% moving on to planning.

clear; clc; close all;

load('scenario_data.mat', 'dt', 'simTime', 'steps', 'egoSpeed', 'egoStart', 'obstacles');

sensorRange = 60;      % m
sensorFOV   = 60;      % degrees half-angle (total cone = 2x this)
noiseStd    = 0.3;     % m, measurement noise

fprintf('STEP 2 ready: Simulated sensor cone (range=%dm, FOV=+/-%ddeg) configured.\n', ...
    sensorRange, sensorFOV);

% Sanity check: which obstacles are visible from the ego start pose?
egoPoseTest = [egoStart 0];
fprintf('\nDetection check at t=1 (ego at [%.1f %.1f]):\n', egoPoseTest(1), egoPoseTest(2));
for k = 1:numel(obstacles)
    dx = obstacles(k).pos(1,1) - egoPoseTest(1);
    dy = obstacles(k).pos(1,2) - egoPoseTest(2);
    dist = hypot(dx, dy);
    angle = abs(atan2d(dy, dx) - rad2deg(egoPoseTest(3)));
    isDetected = dist <= sensorRange && angle <= sensorFOV;
    fprintf('  %-15s dist=%6.2fm  angle=%6.2fdeg  detected=%d\n', ...
        obstacles(k).name, dist, angle, isDetected);
end

save('sensor_data.mat', 'dt', 'simTime', 'steps', 'egoSpeed', 'egoStart', 'obstacles', ...
    'sensorRange', 'sensorFOV', 'noiseStd');
fprintf('\nSaved sensor_data.mat -> run step3_frenet_planner_setup.m next.\n');