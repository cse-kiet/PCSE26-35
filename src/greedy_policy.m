function [doOff, target] = greedy_policy(task, src, cands, vehicles, ~, BANDWIDTH)
% GREEDY_POLICY  Offload to neighbor that minimises transfer + compute time.
  bestT = inf; bestK = 0;
  for j = cands
    ttx = task.dataSize / BANDWIDTH;
    trt = task.computationalRequirement / vehicles(j).processingPower;
    tot = ttx + trt;
    if tot < bestT
      bestT = tot; bestK = j;
    end
  end
  rem    = task.deadline - task.generationTime;
  doOff  = (bestK > 0) && (bestT < rem);
  target = bestK * doOff;
end