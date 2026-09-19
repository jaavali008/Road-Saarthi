%% STEP 4: Closed-loop simulation - detection -> Frenet planning -> motion
% RUN AFTER step3_frenet_planner_setup.m.
% Loads planner_data.mat, rebuilds the Frenet path/connector, then runs
% the live detection -> decision -> Frenet replan -> motion loop.
% Requires Navigation Toolbox (for referencePathFrenet /
% trajectoryGeneratorFrenet / global2frenet / connect).

clear; clc; close all;

load('planner_data.mat', 'dt','simTime','steps','egoSpeed','egoStart','obstacles', ...
     'sensorRange','sensorFOV','noiseStd','dangerZoneAhead','dangerZoneWidth', ...
     'avoidOffset','replanHorizon','refWaypoints');

refPath = referencePathFrenet(refWaypoints);
connector = trajectoryGeneratorFrenet(refPath, 'TimeResolution', dt);

egoPose = [egoStart(1) egoStart(2) 0]; % x, y, heading(rad)
poseHistory = zeros(steps, 3);
replanTimes = zeros(steps, 1);
detectionLog = cell(steps, 1);

figure('Name', 'Step 4: Closed-Loop Simulation');
hold on; grid on; axis equal;
xlim([-5 105]); ylim([-15 15]);
xlabel('X (m)'); ylabel('Y (m)');
title('Live Path Planning & Obstacle Avoidance - Indian Road Scenario (Frenet)');
plot([0 100], [0 0], 'k--', 'HandleVisibility', 'off'); % road centerline

egoPlot = plot(0, 0, 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Ego Vehicle');
pathPlot = plot(NaN, NaN, 'g-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
obsPlots = gobjects(numel(obstacles),1);
for k = 1:numel(obstacles)
    obsPlots(k) = plot(NaN, NaN, obstacles(k).marker, 'MarkerSize', 10, ...
        'DisplayName', obstacles(k).name);
end
detCirclePlot = plot(NaN, NaN, 'r:', 'LineWidth', 1, 'DisplayName', 'Sensor Range');
legend('Location', 'northeastoutside');
statusText = text(2, 12, '', 'FontSize', 12, 'FontWeight', 'bold');

for t = 1:steps
    % --- Get current obstacle positions ---
    obsX = zeros(1, numel(obstacles));
    obsY = zeros(1, numel(obstacles));
    for k = 1:numel(obstacles)
        obsX(k) = obstacles(k).pos(t,1);
        obsY(k) = obstacles(k).pos(t,2);
    end

    % --- Perception: simulate detections within sensor cone ---
    detected = false(1, numel(obstacles));
    for k = 1:numel(obstacles)
        dx = obsX(k) - egoPose(1);
        dy = obsY(k) - egoPose(2);
        dist = hypot(dx, dy);
        angle = abs(atan2d(dy, dx) - rad2deg(egoPose(3)));
        if dist <= sensorRange && angle <= sensorFOV
            detected(k) = true;
            obsX(k) = obsX(k) + noiseStd*randn();
            obsY(k) = obsY(k) + noiseStd*randn();
        end
    end
    detectionLog{t} = {obstacles(detected).name};

    % --- Decision: anything in the danger zone ahead? ---
    dangerFlag = false;
    for k = 1:numel(obstacles)
        if ~detected(k), continue; end
        dx = obsX(k) - egoPose(1);
        dy = obsY(k) - egoPose(2);
        if dx > 0 && dx < dangerZoneAhead && abs(dy) < dangerZoneWidth
            dangerFlag = true;
        end
    end
    targetLateral = 0;
    if dangerFlag
        targetLateral = avoidOffset;
        status = 'AVOIDING';
    else
        status = 'CRUISING';
    end

    % --- Real-time Frenet replanning ---
    tPlanStart = tic;
    currGlobalState = [egoPose(1) egoPose(2) egoPose(3) 0 egoSpeed 0]; % x y theta kappa v a
    currFrenetState = global2frenet(refPath, currGlobalState);

    termFrenetState = currFrenetState;
    termFrenetState(1) = currFrenetState(1) + egoSpeed*replanHorizon; % S
    termFrenetState(2) = egoSpeed;                                    % dS
    termFrenetState(3) = 0;                                           % ddS
    termFrenetState(4) = targetLateral;                               % L
    termFrenetState(5) = 0;                                           % dL
    termFrenetState(6) = 0;                                           % ddL

    [~, globalTraj] = connect(connector, currFrenetState, termFrenetState, replanHorizon);
    replanTimes(t) = toc(tPlanStart);

    % Step the ego pose one dt along the freshly-planned trajectory
    rowIdx = min(2, size(globalTraj.Trajectory, 1));
    nextGlobal = globalTraj.Trajectory(rowIdx, :); % x y theta kappa v a
    egoPose = [nextGlobal(1) nextGlobal(2) nextGlobal(3)];

    poseHistory(t, :) = egoPose;

    % --- Live plot update ---
    set(egoPlot, 'XData', egoPose(1), 'YData', egoPose(2));
    set(pathPlot, 'XData', poseHistory(1:t,1), 'YData', poseHistory(1:t,2));
    for k = 1:numel(obstacles)
        set(obsPlots(k), 'XData', obsX(k), 'YData', obsY(k));
    end
    th = linspace(0, 2*pi, 40);
    set(detCirclePlot, 'XData', egoPose(1) + sensorRange*cos(th), ...
                        'YData', egoPose(2) + sensorRange*sin(th));
    set(statusText, 'Position', [egoPose(1)-5, 12], 'String', status);
    drawnow limitrate;

    if egoPose(1) >= 100
        poseHistory = poseHistory(1:t, :);
        replanTimes = replanTimes(1:t);
        break;
    end
end

fprintf('\n=== RESULTS ===\n');
fprintf('STEP 4 complete: Closed-loop run finished (%d steps).\n', size(poseHistory,1));
fprintf('Avg replanning latency: %.6f sec\n', mean(replanTimes));
fprintf('Max lateral deviation: %.2f m\n', max(abs(poseHistory(:,2))));
fprintf('Scenario completed (reached end of road): %s\n', string(poseHistory(end,1) >= 99));