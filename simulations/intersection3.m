% intersection3.m
% VANET V2V Offloading Simulation (4 vehicles at an intersection)

%% Clear and globals
clear; clc; close all;
global SIM_TIME STEP_SIZE ROAD_LENGTH INTERSECTION_LOC NUM_VEHICLES COMM_RANGE BANDWIDTH TASK_ARRIVAL_RATE

SIM_TIME         = 100;   % seconds
STEP_SIZE        = 0.1;   % seconds
ROAD_LENGTH      = 500;   % meters
INTERSECTION_LOC = 250;   % meters
NUM_VEHICLES     = 4;

COMM_RANGE       = 300;   % meters
BANDWIDTH        = 50;    % MB/s
TASK_ARRIVAL_RATE = 0.5;  % tasks per second

%% Initialization
vehicles      = initializeVehicles(NUM_VEHICLES, ROAD_LENGTH, INTERSECTION_LOC);
trafficSignal = struct( ...
    'position',[INTERSECTION_LOC INTERSECTION_LOC], ...
    'state','RED_NS', ...
    'timer',0, ...
    'cycle',[30 5 30 5] ...
);

taskQueues = cell(1,NUM_VEHICLES);
for v = 1:NUM_VEHICLES
    taskQueues{v} = [];
end

results = struct('completedTasks',0,'offloadedTasks',0,'localTasks',0,'failedTasks',0);

timeSteps        = 0:STEP_SIZE:SIM_TIME;
nSteps           = length(timeSteps);
vehiclePositions = zeros(nSteps,NUM_VEHICLES,2);
taskMetrics      = zeros(nSteps,4);

%% Main loop
for step = 1:nSteps
    t = timeSteps(step);

    % 1) Traffic signal update
    trafficSignal = updateTrafficSignal(trafficSignal, STEP_SIZE);

    % 2) New tasks
    for v = 1:NUM_VEHICLES
        if rand < TASK_ARRIVAL_RATE * STEP_SIZE
            taskQueues{v} = [taskQueues{v}; generateTask(t, vehicles(v))];
        end
    end

    % 3) Move vehicles & compute V2V links
    [vehicles, V2V] = updateVehicles(vehicles, trafficSignal, STEP_SIZE);

    % 4) Offloading decision
    [taskQueues, taskDec] = optimizeTaskOffloading(vehicles, taskQueues, V2V);

    % 5) Execute tasks
    [taskQueues, taskRes] = executeTasks(vehicles, taskQueues, taskDec, STEP_SIZE);

    % 6) Update results
    results = updateResults(results, taskRes);

    % 7) Log positions & metrics
    for v = 1:NUM_VEHICLES
        vehiclePositions(step,v,:) = vehicles(v).position;
    end
    taskMetrics(step,:) = [taskRes.completed, taskRes.offloaded, taskRes.local, taskRes.failed];
end

%% Visualize & display
visualizeResults(vehiclePositions, timeSteps, taskMetrics, NUM_VEHICLES);
displayResults(results);


%% ===== Local Functions BELOW =====

function vehicles = initializeVehicles(n, roadLen, ix)
    dist = 100;
    routes = [ix-dist, ix, 1, 0;
              ix+dist, ix, -1, 0;
              ix, ix-dist, 0, 1;
              ix, ix+dist, 0, -1];
    for i = 1:n
        vehicles(i) = struct( ...
            'id', i, ...
            'position', [routes(i,1) routes(i,2)], ...
            'velocity', 10 + 5*rand(), ...
            'direction', [routes(i,3) routes(i,4)], ...
            'processingPower', 10 + 20*rand(), ...
            'energyLevel', 70 + 30*rand(), ...
            'taskExecutionRate', 5 + 5*rand() ...
        );
    end
end

function sig = updateTrafficSignal(sig, dt)
    sig.timer = sig.timer + dt;
    cycleSum = sum(sig.cycle);
    m = mod(sig.timer, cycleSum);
    if m < sig.cycle(1)
        sig.state = 'GREEN_EW';
    elseif m < sum(sig.cycle(1:2))
        sig.state = 'YELLOW_EW';
    elseif m < sum(sig.cycle(1:3))
        sig.state = 'GREEN_NS';
    else
        sig.state = 'YELLOW_NS';
    end
end

function task = generateTask(now, veh)
    types = {'PERCEPTION','PLANNING','CONTROL'};
    tp = types{randi(3)};
    switch tp
      case 'PERCEPTION'
        cr = 5 + 5*rand(); ds = 20 + 10*rand(); dl = now + 1; pr = 3;
      case 'PLANNING'
        cr = 2 + 3*rand(); ds = 5 + 5*rand();  dl = now + 1.5; pr = 2;
      case 'CONTROL'
        cr = 1 + 1*rand(); ds = 1 + 1*rand();  dl = now + 0.5; pr = 3;
    end
    task = struct( ...
      'id', sprintf('V%d_T%d',veh.id,round(now*100)), ...
      'type', tp, ...
      'computationalRequirement', cr, ...
      'dataSize', ds, ...
      'generationTime', now, ...
      'deadline', dl, ...
      'priority', pr, ...
      'status', 'PENDING', ...
      'vehicleID', veh.id ...
    );
end

function [vehOut, V2V] = updateVehicles(vehIn, sig, dt)
    global COMM_RANGE
    n = numel(vehIn);
    V2V = zeros(n);
    for i = 1:n
        % stop at red
        atInt = norm(vehIn(i).position - sig.position) < 50 && ...
                ((vehIn(i).direction(1)~=0 && ~strcmp(sig.state,'GREEN_EW')) || ...
                 (vehIn(i).direction(2)~=0 && ~strcmp(sig.state,'GREEN_NS')));
        if ~atInt
            vehIn(i).position = vehIn(i).position + vehIn(i).direction * vehIn(i).velocity * dt;
        end
        % V2V link quality
        for j = 1:n
            if i~=j
                d = norm(vehIn(i).position - vehIn(j).position);
                if d <= COMM_RANGE
                    V2V(i,j) = max(0,1 - d/COMM_RANGE);
                end
            end
        end
    end
    vehOut = vehIn;
end

function [queues, dec] = optimizeTaskOffloading(vehicles, queues, V2V)
    global BANDWIDTH
    n = numel(vehicles);
    dec = struct('offloaded',0,'local',0);
    for i = 1:n
        q = queues{i};
        for k = 1:numel(q)
            if ~strcmp(q(k).status,'PENDING'), continue; end
            % find candidates quality>=0.5
            cand = find(V2V(i,:) >= 0.5 & (1:n)~=i);
            bestT = inf; bestIdx = 0;
            for j = cand
                dt = q(k).dataSize / BANDWIDTH;
                rt = q(k).computationalRequirement / vehicles(j).processingPower;
                tot = dt + rt;
                if tot < bestT
                    bestT = tot;
                    bestIdx = j;
                end
            end
            timeRem = q(k).deadline - q(k).generationTime;
            if bestIdx>0 && bestT < timeRem
                q(k).status = 'OFFLOADED';
                q(k).vehicleID = bestIdx;
                queues{bestIdx} = [queues{bestIdx}; q(k)];
                dec.offloaded = dec.offloaded + 1;
            else
                q(k).status = 'LOCAL';
                dec.local = dec.local + 1;
            end
        end
        queues{i} = q;
    end
end

function [queues, res] = executeTasks(vehicles, queues, dec, dt)
    res = struct('completed',0,'offloaded',dec.offloaded,'local',dec.local,'failed',0);
    n = numel(vehicles);
    for i = 1:n
        q = queues{i};
        maxN = vehicles(i).taskExecutionRate * dt;
        cnt = 0;
        for k = 1:numel(q)
            if cnt >= maxN, break; end
            if any(strcmp(q(k).status,{'LOCAL','OFFLOADED'}))
                q(k).status = 'COMPLETED';
                res.completed = res.completed + 1;
                cnt = cnt + 1;
            end
        end
        queues{i} = q;
    end
end

function out = updateResults(in, tr)
    in.completedTasks  = in.completedTasks  + tr.completed;
    in.offloadedTasks  = in.offloadedTasks  + tr.offloaded;
    in.localTasks      = in.localTasks      + tr.local;
    in.failedTasks     = in.failedTasks     + tr.failed;
    out = in;
end

function visualizeResults(vehiclePositions, timeSteps, taskMetrics, numVehicles)
    % Bring in the global intersection location
    global INTERSECTION_LOC

    % 1) Vehicle paths
    figure('Name','V2V Simulation Results','Position',[100 100 1000 600]);
    subplot(2,1,1);
    hold on;
    cols = ['r','g','b','k'];
    marks = ['o','s','^','d'];
    for i = 1:numVehicles
        plot( ...
          squeeze(vehiclePositions(:,i,1)), ...
          squeeze(vehiclePositions(:,i,2)), ...
          'Color',cols(i),'LineWidth',1.5);
        plot( ...
          vehiclePositions(end,i,1), ...
          vehiclePositions(end,i,2), ...
          marks(i),'MarkerFaceColor',cols(i),'MarkerSize',8);
    end
    % now INTERSECTION_LOC is known
    plot(INTERSECTION_LOC, INTERSECTION_LOC, 'kx','MarkerSize',12,'LineWidth',2);
    title('Vehicle Paths During Simulation','FontSize',12);
    xlabel('X Position (m)','FontSize',10);
    ylabel('Y Position (m)','FontSize',10);
    legend(...
      [arrayfun(@(i) sprintf('Vehicle %d',i),1:numVehicles,'UniformOutput',false), {'Intersection'}], ...
      'Location','bestoutside');
    grid on; axis equal;
    hold off;

    % 2) Task metrics
    subplot(2,1,2);
    hold on;
    plot(timeSteps, taskMetrics(:,1), 'b-','LineWidth',1.5);   % Completed
    plot(timeSteps, taskMetrics(:,2), 'g-','LineWidth',1.5);   % Offloaded
    plot(timeSteps, taskMetrics(:,3), 'r-','LineWidth',1.5);   % Local
    plot(timeSteps, taskMetrics(:,4), 'k-','LineWidth',1.5);   % Failed
    title('Task Counts Over Time','FontSize',12);
    xlabel('Simulation Time (s)','FontSize',10);
    ylabel('Number of Tasks','FontSize',10);
    legend({'Completed','Offloaded','Local','Failed'},'Location','bestoutside');
    grid on;
    hold off;
end

function displayResults(res)
    fprintf('\n=== Final Simulation Results ===\n');
    fprintf('Completed Tasks: %d\n', res.completedTasks);
    fprintf('Offloaded Tasks: %d\n', res.offloadedTasks);
    fprintf('Local Tasks:     %d\n', res.localTasks);
    fprintf('Failed Tasks:    %d\n', res.failedTasks);
end
