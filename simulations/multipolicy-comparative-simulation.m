% intersection_multi_policy.m
% -------------------------------------------------------------------------
% Runs a 4‑vehicle V2V offloading sim under five different heuristics and
% plots a bar chart comparing Completion Rate and Offloading Rate.

%% Main Script
clear; clc; close all;
global SIM_TIME STEP_SIZE ROAD_LENGTH INTERSECTION_LOC NUM_VEHICLES ...
       COMM_RANGE BANDWIDTH TASK_ARRIVAL_RATE

% Simulation parameters (copy from your intersection3)
SIM_TIME          = 100;   % seconds
STEP_SIZE         = 0.1;   % seconds
ROAD_LENGTH       = 500;   % meters
INTERSECTION_LOC  = 250;   % meters
NUM_VEHICLES      = 4;

COMM_RANGE        = 300;   % meters
BANDWIDTH         = 50;    % MB/s
TASK_ARRIVAL_RATE = 0.5;   % tasks/sec

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
rates = zeros(nP,2);

% Run the sim under each policy
for pi = 1:nP
    [c,o,l,f] = runSim(policies{pi});
    rates(pi,1) = c/(c+f);    % completion rate
    rates(pi,2) = o/(o+l);    % offload rate
end

% Plot results
figure('Name','Policy Comparison','Position',[300 200 600 400]);
bar(rates);
set(gca,'XTick',1:nP,'XTickLabel',names);
legend('Completion Rate','Offloading Rate','Location','northwest');
ylabel('Rate');
title('V2V Offloading Heuristics Comparison');


%% runSim: execute one full sim with the given policy function
function [totalComp, totalOffl, totalLoc, totalFail] = runSim(policyFcn)
    global SIM_TIME STEP_SIZE ROAD_LENGTH INTERSECTION_LOC ...
           NUM_VEHICLES COMM_RANGE BANDWIDTH TASK_ARRIVAL_RATE

    % --- Initialization (same as intersection3) ---
    vehicles      = initializeVehicles(NUM_VEHICLES, ROAD_LENGTH, INTERSECTION_LOC);
    trafficSignal = struct('position',[INTERSECTION_LOC,INTERSECTION_LOC], ...
                           'state','RED_NS','timer',0,'cycle',[30 5 30 5]);
    taskQueues = cell(1,NUM_VEHICLES);
    for v = 1:NUM_VEHICLES
        taskQueues{v} = [];
    end

    % Running totals
    results = struct('completedTasks',0,'offloadedTasks',0,'localTasks',0,'failedTasks',0);

    timeSteps = 0:STEP_SIZE:SIM_TIME;
    nSteps    = numel(timeSteps);

    % --- Main loop ---
    for step = 1:nSteps
        t = timeSteps(step);

        % 1) traffic light
        trafficSignal = updateTrafficSignal(trafficSignal, STEP_SIZE);

        % 2) new tasks
        for v = 1:NUM_VEHICLES
            if rand < TASK_ARRIVAL_RATE * STEP_SIZE
                taskQueues{v} = [taskQueues{v}; generateTask(t, vehicles(v))];
            end
        end

        % 3) move & build V2V links
        [vehicles, V2V] = updateVehicles(vehicles, trafficSignal, STEP_SIZE);

        % 4) offloading decision
        dec = struct('offloaded',0,'local',0);
        for i = 1:NUM_VEHICLES
            q = taskQueues{i};
            for k = 1:numel(q)
                if ~strcmp(q(k).status,'PENDING'), continue; end
                cands = find(V2V(i,:)>=0.5 & (1:NUM_VEHICLES)~=i);
                [doOff, tgt] = policyFcn(q(k), i, cands, vehicles, taskQueues);
                if doOff
                    q(k).status    = 'OFFLOADED';
                    q(k).vehicleID = tgt;
                    taskQueues{tgt} = [taskQueues{tgt}; q(k)];
                    dec.offloaded = dec.offloaded + 1;
                else
                    q(k).status = 'LOCAL';
                    dec.local   = dec.local + 1;
                end
            end
            taskQueues{i} = q;
        end

        % 5) execute tasks
        [taskQueues, tr] = executeTasks(vehicles, taskQueues, dec, STEP_SIZE);

        % 6) update totals
        results = updateResults(results, tr);
    end

    % return totals
    totalComp = results.completedTasks;
    totalOffl = results.offloadedTasks;
    totalLoc  = results.localTasks;
    totalFail = results.failedTasks;
end


%% ——— Policy Functions ——————————————————————————

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
    rem     = task.deadline - task.generationTime;
    doOff   = (bestK>0) && (bestT < rem);
    target  = bestK * doOff;
end

% 2) Greedy + speed check: only if remote faster than local
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
    rem = task.deadline - task.generationTime;
    doOff  = (bestK>0) && (bestT < rem) && (bestT < localT);
    target = bestK * doOff;
end

% 3) First‑fit threshold: offload to first feasible neighbor
function [doOff, target] = threshold_policy(task, ~, cands, vehicles, ~)
    global BANDWIDTH
    rem = task.deadline - task.generationTime;
    doOff = false; target = 0;
    for j = cands
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        if (ttx + trt) < rem
            doOff = true; target = j; return;
        end
    end
end

% 4) Game‑theoretic: maximize (savedTime – alpha*energyCost)
function [doOff, target] = gametheoretic_policy(task, src, cands, vehicles, ~)
    global BANDWIDTH
    alpha  = 0.2;
    localT = task.computationalRequirement / vehicles(src).processingPower;
    bestU  = 0; bestK = 0;
    for j = cands
        ttx = task.dataSize / BANDWIDTH;
        trt = task.computationalRequirement / vehicles(j).processingPower;
        tot = ttx + trt;
        saved = localT - tot;
        ecost = task.dataSize * 0.1;
        U = saved - alpha*ecost;
        if U > bestU
            bestU = U; bestK = j;
        end
    end
    rem = task.deadline - task.generationTime;
    doOff  = (bestK>0) && (bestU>0) && ((task.dataSize/BANDWIDTH + task.computationalRequirement/vehicles(bestK).processingPower) < rem);
    target = bestK * doOff;
end

% 5) Load‑balancing: pick neighbor with smallest queue
% 5) Load‑balancing: pick neighbor with smallest queue length *and* feasible
function [doOff, target] = loadbalance_policy(task, src, cands, vehicles, queues)
    global BANDWIDTH
    timeRem = task.deadline - task.generationTime;
    bestLen = inf;
    bestK   = 0;
    % Find the neighbor with the smallest queue *and* that can finish before deadline
    for j = cands
        L = numel(queues{j});
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
    dist   = 100;
    routes = [ix-dist, ix,    1,  0;
              ix+dist, ix,   -1,  0;
              ix,      ix-dist,0,  1;
              ix,      ix+dist, 0, -1];
    for i=1:n
        vehicles(i) = struct( ...
            'id', i, ...
            'position',[routes(i,1),routes(i,2)], ...
            'velocity',10+5*rand(), ...
            'direction',[routes(i,3),routes(i,4)], ...
            'processingPower',10+20*rand(), ...
            'energyLevel',70+30*rand(), ...
            'taskExecutionRate',5+5*rand() ...
        );
    end
end

function sig = updateTrafficSignal(sig, dt)
    sig.timer = sig.timer + dt;
    cyc = sum(sig.cycle);
    m   = mod(sig.timer,cyc);
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
    tp    = types{randi(3)};
    switch tp
      case 'PERCEPTION'
        cr = 5+5*rand(); ds = 20+10*rand(); dl = now+1; pr = 3;
      case 'PLANNING'
        cr = 2+3*rand(); ds = 5+5*rand();  dl = now+1.5; pr = 2;
      case 'CONTROL'
        cr = 1+1*rand(); ds = 1+1*rand();  dl = now+0.5; pr = 3;
    end
    task = struct( ...
        'id',                    sprintf('V%d_T%d',veh.id,round(now*100)), ...
        'type',                  tp, ...
        'computationalRequirement',cr, ...
        'dataSize',              ds, ...
        'generationTime',        now, ...
        'deadline',              dl, ...
        'priority',              pr, ...
        'status',                'PENDING', ...
        'vehicleID',             veh.id ...
    );
end

function [vehOut, V2V] = updateVehicles(vehIn, sig, dt)
    global COMM_RANGE
    n  = numel(vehIn);
    V2V = zeros(n);
    for i=1:n
        atInt = norm(vehIn(i).position - sig.position) < 50 && ...
                ((vehIn(i).direction(1)~=0 && ~strcmp(sig.state,'GREEN_EW')) || ...
                 (vehIn(i).direction(2)~=0 && ~strcmp(sig.state,'GREEN_NS')));
        if ~atInt
            vehIn(i).position = vehIn(i).position + vehIn(i).direction * vehIn(i).velocity * dt;
        end
        for j=1:n
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

function [queues, res] = executeTasks(vehicles, queues, dec, dt)
    res = struct('completed',0,'offloaded',dec.offloaded,'local',dec.local,'failed',0);
    n = numel(vehicles);
    for i=1:n
        q = queues{i};
        maxN = vehicles(i).taskExecutionRate * dt;
        cnt = 0;
        for k=1:numel(q)
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
