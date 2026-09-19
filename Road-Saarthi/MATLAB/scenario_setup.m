%% STEP 1: Scenario with ego vehicle + 4 dynamic obstacles
% RUN THIS FIRST.
% Builds the Indian-road scenario (ego + 4 obstacles), precomputes each
% obstacle's trajectory, plots the initial layout, and saves everything
% to scenario_data.mat for step2_sensor_config.m to load.
%
% Toolbox-free: plain MATLAB only.

clear; clc; close all;

dt = 0.1;            % timestep (s)
simTime = 12;         % total sim duration (s)
steps = simTime/dt;

egoSpeed = 10;        % m/s
egoStart = [0 0];

% --- Obstacle 1: Pedestrian - sudden crossing mid-road ---
obstacles(1).name = 'Pedestrian';
obstacles(1).waypoints = [45 -6; 45 6];
obstacles(1).speed = 1.5;
obstacles(1).marker = 'ko'; obstacles(1).size = 60;

% --- Obstacle 2: Auto-rickshaw - merges without signaling ---
obstacles(2).name = 'Auto-rickshaw';
obstacles(2).waypoints = [30 5; 55 -1; 80 0];
obstacles(2).speed = 6;
obstacles(2).marker = 'y^'; obstacles(2).size = 90;

% --- Obstacle 3: Cow - slow, unpredictable crossing ---
obstacles(3).name = 'Cow';
obstacles(3).waypoints = [65 5; 65 -5];
obstacles(3).speed = 0.8;
obstacles(3).marker = 'ms'; obstacles(3).size = 80;

% --- Obstacle 4: Static obstruction (pothole/debris) ---
obstacles(4).name = 'Pothole';
obstacles(4).waypoints = [20 1.5; 20 1.5]; % stays put
obstacles(4).speed = 0;
obstacles(4).marker = 'kx'; obstacles(4).size = 100;

% Precompute obstacle positions at every timestep
for k = 1:numel(obstacles)
    obstacles(k).pos = computeTrajectory(obstacles(k).waypoints, obstacles(k).speed, dt, steps);
end

% Quick visualization to confirm scenario looks right
figure('Name', 'Step 1: Scenario Setup');
hold on; grid on; axis equal;
xlim([-5 105]); ylim([-15 15]);
plot([0 100], [0 0], 'k--'); % road centerline
for k = 1:numel(obstacles)
    plot(obstacles(k).pos(1,1), obstacles(k).pos(1,2), obstacles(k).marker, ...
        'MarkerSize', 10, 'MarkerFaceColor', 'auto', 'DisplayName', obstacles(k).name);
end
plot(egoStart(1), egoStart(2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Ego');
legend show;
title('Ego Vehicle + Dynamic Obstacles (Indian Road Scenario) - Initial Positions');

fprintf('STEP 1 complete: Scenario built with ego + %d obstacles (no toolbox needed).\n', numel(obstacles));

save('scenario_data.mat', 'dt', 'simTime', 'steps', 'egoSpeed', 'egoStart', 'obstacles');
fprintf('Saved scenario_data.mat -> run step2_sensor_config.m next.\n');

%% Helper function - computes obstacle position at each timestep
function pos = computeTrajectory(waypoints, speed, dt, steps)
pos = zeros(steps, 2);
if speed == 0 || size(waypoints,1) == 1
    pos(:,1) = waypoints(1,1);
    pos(:,2) = waypoints(1,2);
    return;
end
segLengths = sqrt(sum(diff(waypoints).^2, 2));
totalLength = sum(segLengths);
totalTime = totalLength / speed;
for t = 1:steps
    currTime = min((t-1)*dt, totalTime);
    distTravelled = speed * currTime;
    cumLen = [0; cumsum(segLengths)];
    segIdx = find(cumLen <= distTravelled, 1, 'last');
    segIdx = min(segIdx, size(waypoints,1)-1);
    if segIdx < 1, segIdx = 1; end
    segStart = waypoints(segIdx,:);
    segEnd = waypoints(segIdx+1,:);
    segDist = distTravelled - cumLen(segIdx);
    segFrac = segDist / max(segLengths(segIdx), eps);
    segFrac = min(max(segFrac, 0), 1);
    pos(t,:) = segStart + segFrac * (segEnd - segStart);
end
end