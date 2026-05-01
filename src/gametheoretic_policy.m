function [doOff, target] = gametheoretic_policy(task, src, cands, vehicles, ~, BANDWIDTH)
% GAMETHEORETIC_POLICY  Maximise utility: saved time minus energy cost.
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
  rem = task.deadline - task.generationTime;
  if bestK > 0
    ttx_best = task.dataSize / BANDWIDTH;
    trt_best = task.computationalRequirement / vehicles(bestK).processingPower;
    doOff  = (bestU > 0) && ((ttx_best + trt_best) < rem);
  else
    doOff = false;
  end
  target = bestK * doOff;
end