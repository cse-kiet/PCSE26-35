function [doOff, target] = threshold_policy(task, ~, cands, vehicles, ~, BANDWIDTH)
% THRESHOLD_POLICY  First-fit: offload to first neighbor that meets deadline.
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