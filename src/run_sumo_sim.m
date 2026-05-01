function [totalComp, totalOffl, totalLoc, totalFail] = ...
    run_sumo_sim(policyFcn, vehicles, timeSteps, BANDWIDTH, TASK_ARRIVAL_RATE)
% RUN_SUMO_SIM  Run one full V2V offloading simulation over SUMO traces.
%
%   policyFcn        : function handle with signature
%                      [doOff, tgt] = f(task, src, cands, vehicles, queues, BANDWIDTH)
%   vehicles         : 1xN struct array from parse_fcd
%   timeSteps        : sorted time vector from parse_fcd
%   BANDWIDTH        : link bandwidth in MB/s
%   TASK_ARRIVAL_RATE: tasks per second per vehicle

  N         = numel(vehicles);
  STEP_SIZE = timeSteps(2) - timeSteps(1);

  taskQueues = cell(1, N);
  for v = 1:N, taskQueues{v} = []; end

  results = struct('completedTasks',0,'offloadedTasks',0, ...
                   'localTasks',0,'failedTasks',0);

  for step = 1:numel(timeSteps)
    t = timeSteps(step);

    % Collect active vehicles at this timestep
    positions   = zeros(N, 2);
    active_mask = false(N, 1);
    for i = 1:N
      [pos, ~, act] = get_vehicle_state(vehicles(i), t);
      if act
        positions(i,:) = pos;
        active_mask(i) = true;
      end
    end

    active_idx = find(active_mask);
    if numel(active_idx) < 2, continue; end

    % V2V link quality matrix
    V2V = build_v2v_matrix(positions, active_mask);

    % Generate new tasks for active vehicles
    for i = active_idx'
      if rand() < TASK_ARRIVAL_RATE * STEP_SIZE
        taskQueues{i} = [taskQueues{i}; generate_task_sumo(t, vehicles(i))];
      end
    end

    % Offloading decisions
    dec = struct('offloaded',0,'local',0);
    for i = active_idx'
      q = taskQueues{i};
      for k = 1:numel(q)
        if ~strcmp(q(k).status,'PENDING'), continue; end
        cands = find(V2V(i,:) >= 0.5 & active_mask' & (1:N) ~= i);
        [doOff, tgt] = policyFcn(q(k), i, cands, vehicles, taskQueues, BANDWIDTH);
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

    % Execute tasks
    for i = active_idx'
      q    = taskQueues{i};
      maxN = vehicles(i).taskExecutionRate * STEP_SIZE;
      cnt  = 0;
      for k = 1:numel(q)
        if cnt >= maxN, break; end
        if any(strcmp(q(k).status, {'LOCAL','OFFLOADED'}))
          q(k).status            = 'COMPLETED';
          results.completedTasks = results.completedTasks + 1;
          cnt = cnt + 1;
        end
      end
      taskQueues{i} = q;
    end

    results.offloadedTasks = results.offloadedTasks + dec.offloaded;
    results.localTasks     = results.localTasks     + dec.local;
  end

  totalComp = results.completedTasks;
  totalOffl = results.offloadedTasks;
  totalLoc  = results.localTasks;
  totalFail = results.failedTasks;
end