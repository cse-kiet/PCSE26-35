% intersection_multi_policy.m
% -------------------------------------------------------------------------
% Runs a 4-vehicle V2V offloading sim under five different heuristics and
% plots a bar chart comparing Completion Rate and Offloading Rate.
% FIXED: executeTasks now checks deadlines before marking tasks complete.

%% Main Script
clear; clc; close all;
global SIM_TIME STEP_SIZE ROAD_LENGTH INTERSECTION_LOC NUM_VEHICLES ...
       COMM_RANGE BANDWIDTH TASK_ARRIVAL_RATE

% Simulation parameters
SIM_TIME          = 100;   % seconds
STEP_SIZE         = 0.1;   % seconds
ROAD_LENGTH       = 500;   % meters
INTERSECTION_LOC  = 250;   % meters
NUM_VEHICLES      = 12;

COMM_RANGE        = 300;   % meters
BANDWIDTH         = 50;    % MB/s
TASK_ARRIVAL_RATE = 2.0;   % tasks/sec

% Define the five policies
policies = { ...
    @greedy_policy, ...
    @speedcheck_policy, ...
    @threshold_policy, ...
    @gametheoretic_policy, ...
    @loadbalance_policy ...
    };
names = {'Greedy','SpeedCheck','Threshold','GameTh','LoadBal'};
nP = numel(policies);

% Preallocate rate matrix: [CompletionRate, OffloadingRate]
rates = zeros(nP, 2);

% Run the sim under each policy
for pi = 1:nP
    [c, o, l, f] = runSim(policies{pi});
    total = c + f;
    rates(pi, 1) = c / max(total, 1);       % completion rate
    rates(pi, 2) = o / max(o + l, 1);       % offload rate
end

% Print exact results
fprintf('\n===== EXACT SIMULATION RESULTS =====\n');
for pi = 1:nP
    fprintf('%s: Completion=%.4f, Offloading=%.4f\n', ...
        names{pi}, rates(pi,1), rates(pi,2));
end

% Plot results
figure('Name','Policy Comparison','Position',[300 200 700 450]);
b = bar(rates);
b(1).FaceColor = [0.2 0.4 0.8];
b(2).FaceColor = [0.85 0.33 0.1];
set(gca, 'XTick', 1:nP, 'XTickLabel', names, 'FontSize', 11);
legend('Completion Rate', 'Offloading Rate', 'Location', 'northwest');
ylabel('Rate');
title('V2V Offloading Heuristics Comparison');
ylim([0 1.1]);
grid on;


%% runSim: execute one full sim with the given policy function
function [totalComp, totalOffl, totalLoc, totalFail] = runSim(policyFcn)
    global SIM_TIME STEP_SIZE ROAD_LENGTH INTERSECTION_LOC ...
           NUM_VEHICLES COMM_RANGE BANDWIDTH TASK_ARRIVAL_RATE

    % --- Initialization ---
    vehicles      = initializeVehicles(NUM_VEHICLES, ROAD_LENGTH, INTERSECTION_LOC);
    trafficSignal = struct('position', [INTERSECTION_LOC, INTERSECTION_LOC], ...
                           'state', 'RED_NS', 'timer', 0, 'cycle', [30 5 30 5]);
    taskQueues = cell(1, NUM_VEHICLES);
    for v = 1:NUM_VEHICLES
        taskQueues{v} = [];
    end

    % Running totals
    results = struct('completedTasks', 0, 'offloadedTasks', 0, ...
                     'localTasks', 0, 'failedTasks', 0);

    timeSteps = 0:STEP_SIZE:SIM_TIME;
    nSteps    = numel(timeSteps);

    % --- Main loop ---
    for step = 1:nSteps
        t = timeSteps(step);

        % 1) Update traffic light
        trafficSignal = updateTrafficSignal(trafficSignal, STEP_SIZE);

        % 2) Generate new tasks
        for v = 1:NUM_VEHICLES
            if rand < TASK_ARRIVAL_RATE * STEP_SIZE
                taskQueues{v} = [taskQueues{v}; generateTask(t, vehicles(v))];
            end
        end

        % 3) Move vehicles & build V2V links
        [vehicles, V2V] = updateVehicles(vehicles, trafficSignal, STEP_SIZE);

        % 4) Offloading decisions
        dec = struct('offloaded', 0, 'local', 0);
        for i = 1:NUM_VEHICLES
            q = taskQueues{i};
            for k = 1:numel(q)
                if ~strcmp(q(k).status, 'PENDING'), continue; end
                cands = find(V2V(i,:) >= 0.5 & (1:NUM_VEHICLES) ~= i);
                [doOff, tgt] = policyFcn(q(k), i, cands, vehicles, taskQueues);
                if doOff
                    q(k).status    = 'OFFLOADED';
                    q(k).vehicleID = tgt;
                    taskQueues{tgt} = [taskQueues{tgt}; q(k)];
                    dec.offloaded  = dec.offloaded + 1;
                else
                    q(k).status = 'LOCAL';
                    dec.local   = dec.local + 1;
                end
            end
            taskQueues{i} = q;
        end

        % 5) Execute tasks with deadline checking
        [taskQueues, tr] = executeTasks(vehicles, taskQueues, dec, STEP_SIZE, t);

        % 6) Update totals
        results = updateResults(results, tr);
    end

    totalComp = results.completedTasks;
    totalOffl = results.offloadedTasks;
    totalLoc  = results.localTasks;
    totalFail = results.failedTasks;
end


%% ——— Policy Functions ———————————————————————————

% 1) Pure greedy: minimize transfer + remote compute time
function [doOff, target] = greedy_policy(task, src, cands, vehicles, ~)
    global BANDWIDTH
    bestT = inf; bestK = 0;
    for j = cands
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        tot = ttx + trt;
        if tot < bestT
            bestT = tot; bestK = j;
        end
    end
    rem   = task.deadline - task.generationTime;
    doOff = (bestK > 0) && (bestT < rem);
    target = bestK * doOff;
end

% 2) Speed-aware greedy: only offload if remote faster than local
function [doOff, target] = speedcheck_policy(task, src, cands, vehicles, ~)
    global BANDWIDTH
    localT = task.computationalRequirement / vehicles(src).processingPower;
    bestT  = inf; bestK = 0;
    for j = cands
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        tot = ttx + trt;
        if tot < bestT
            bestT = tot; bestK = j;
        end
    end
    rem   = task.deadline - task.generationTime;
    doOff = (bestK > 0) && (bestT < rem) && (bestT < localT);
    target = bestK * doOff;
end

% 3) First-fit threshold: offload to first feasible neighbor
function [doOff, target] = threshold_policy(task, ~, cands, vehicles, ~)
    global BANDWIDTH
    rem   = task.deadline - task.generationTime;
    doOff = false; target = 0;
    for j = cands
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        if (ttx + trt) < rem
            doOff = true; target = j; return;
        end
    end
end

% 4) Game-theoretic: maximize (savedTime - alpha*energyCost)
function [doOff, target] = gametheoretic_policy(task, src, cands, vehicles, ~)
    global BANDWIDTH
    alpha  = 0.2;
    localT = task.computationalRequirement / vehicles(src).processingPower;
    bestU  = 0; bestK = 0;
    for j = cands
        ttx   = task.dataSize / BANDWIDTH;
        trt   = task.computationalRequirement / vehicles(j).processingPower;
        tot   = ttx + trt;
        saved = localT - tot;
        ecost = task.dataSize * 0.1;
        U     = saved - alpha * ecost;
        if U > bestU
            bestU = U; bestK = j;
        end
    end
    rem   = task.deadline - task.generationTime;
    doOff = (bestK > 0) && (bestU > 0) && ...
            ((task.dataSize/BANDWIDTH + task.computationalRequirement/vehicles(bestK).processingPower) < rem);
    target = bestK * doOff;
end

% 5) Load-balancing: pick neighbor with smallest queue and feasible deadline
function [doOff, target] = loadbalance_policy(task, src, cands, vehicles, queues)
    global BANDWIDTH
    timeRem = task.deadline - task.generationTime;
    bestLen = inf;
    bestK   = 0;
    for j = cands
        L   = numel(queues{j});
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        tot = ttx + trt;
        if L < bestLen && tot < timeRem
            bestLen = L;
            bestK   = j;
        end
    end
    if bestK > 0
        doOff  = true;
        target = bestK;
    else
        doOff  = false;
        target = 0;
    end
end


%% ——— Helper Functions ——————————————————————————————

function vehicles = initializeVehicles(n, roadLen, ix)
    % Place vehicles evenly across 4 approach roads
    dirs = [1 0; -1 0; 0 1; 0 -1];
    for i = 1:n
        d      = dirs(mod(i-1,4)+1, :);
        offset = 80 + 40*floor((i-1)/4);
        if d(1) ~= 0
            pos = [ix - d(1)*offset, ix];
        else
            pos = [ix, ix - d(2)*offset];
        end
        vehicles(i) = struct( ...
            'id',              i, ...
            'position',        pos, ...
            'velocity',        10 + 5*rand(), ...
            'direction',       d, ...
            'processingPower', 10 + 20*rand(), ...
            'energyLevel',     70 + 30*rand(), ...
            'taskExecutionRate', 2 + 2*rand() ...
        );
    end
end

function sig = updateTrafficSignal(sig, dt)
    sig.timer = sig.timer + dt;
    cyc = sum(sig.cycle);
    m   = mod(sig.timer, cyc);
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
    types = {'PERCEPTION', 'PLANNING', 'CONTROL'};
    tp    = types{randi(3)};
    switch tp
        case 'PERCEPTION'
            cr = 5 + 5*rand();  ds = 20 + 10*rand(); dl = now + 0.6;  pr = 3;
        case 'PLANNING'
            cr = 2 + 3*rand();  ds = 5  + 5*rand();  dl = now + 1.2;  pr = 2;
        case 'CONTROL'
            cr = 1 + 1*rand();  ds = 1  + 1*rand();  dl = now + 0.3;  pr = 3;
    end
    task = struct( ...
        'id',                      sprintf('V%d_T%d', veh.id, round(now*100)), ...
        'type',                    tp, ...
        'computationalRequirement',cr, ...
        'dataSize',                ds, ...
        'generationTime',          now, ...
        'deadline',                dl, ...
        'priority',                pr, ...
        'status',                  'PENDING', ...
        'vehicleID',               veh.id ...
    );
end

function [vehOut, V2V] = updateVehicles(vehIn, sig, dt)
    global COMM_RANGE
    n   = numel(vehIn);
    V2V = zeros(n);
    for i = 1:n
        atInt = norm(vehIn(i).position - sig.position) < 50 && ...
                ((vehIn(i).direction(1) ~= 0 && ~strcmp(sig.state,'GREEN_EW')) || ...
                 (vehIn(i).direction(2) ~= 0 && ~strcmp(sig.state,'GREEN_NS')));
        if ~atInt
            vehIn(i).position = vehIn(i).position + ...
                                 vehIn(i).direction * vehIn(i).velocity * dt;
        end
        for j = 1:n
            if i ~= j
                d = norm(vehIn(i).position - vehIn(j).position);
                if d <= COMM_RANGE
                    V2V(i,j) = max(0, 1 - d/COMM_RANGE);
                end
            end
        end
    end
    vehOut = vehIn;
end

% FIXED: deadline-aware task execution
function [queues, res] = executeTasks(vehicles, queues, dec, dt, currentTime)
    res = struct('completed', 0, 'offloaded', dec.offloaded, ...
                 'local', dec.local, 'failed', 0);
    n = numel(vehicles);
    for i = 1:n
        q    = queues{i};
        maxN = max(1, round(vehicles(i).taskExecutionRate * dt));
        cnt  = 0;
        for k = 1:numel(q)
            if any(strcmp(q(k).status, {'LOCAL','OFFLOADED'}))
                if currentTime > q(k).deadline
                    % Deadline missed — mark as failed
                    q(k).status = 'FAILED';
                    res.failed  = res.failed + 1;
                elseif cnt < maxN
                    % Within deadline and execution slot available
                    q(k).status  = 'COMPLETED';
                    res.completed = res.completed + 1;
                    cnt = cnt + 1;
                end
                % If cnt >= maxN and not expired: stays LOCAL/OFFLOADED,
                % retried next timestep
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
