function [doOff, target] = speedcheck_policy(task, src, cands, vehicles, ~, BANDWIDTH)
% SPEEDCHECK_POLICY  Offload only if remote is faster than local execution.
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
  rem    = task.deadline - task.generationTime;
  doOff  = (bestK > 0) && (bestT < rem) && (bestT < localT);
  target = bestK * doOff;
end