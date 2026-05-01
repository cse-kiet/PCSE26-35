function [doOff, target] = loadbalance_policy(task, ~, cands, vehicles, queues, BANDWIDTH)
% LOADBALANCE_POLICY  Pick neighbor with smallest queue that meets deadline.
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