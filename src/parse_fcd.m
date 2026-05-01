function [vehicles, timeSteps] = parse_fcd(csv_path)
% PARSE_FCD  Load SUMO FCD traces into a vehicle struct array.
%
%   [vehicles, timeSteps] = parse_fcd('src/sumo/fcd_traces.csv')
%
%   Output:
%     vehicles  — 1xN struct array, one per unique vehicle ID
%       .id                integer vehicle ID
%       .times             Tx1 vector of timesteps this vehicle was active
%       .positions         Tx2 matrix [x, y] at each timestep
%       .speeds            Tx1 vector of speed (m/s) at each timestep
%       .processingPower   scalar, drawn from U(10,30) GHz
%       .taskExecutionRate scalar, drawn from U(5,10) tasks/s
%       .energyLevel       scalar, drawn from U(70,100)
%
%     timeSteps — unique sorted time values across all vehicles

  data = readtable(csv_path, 'VariableNamingRule', 'preserve');

  % Column names: time, vehicle_id, x, y, speed, angle
  times  = data.time;
  vids   = data.vehicle_id;
  xs     = data.x;
  ys     = data.y;
  speeds = data.speed;

  unique_ids = unique(vids);
  N = numel(unique_ids);
  timeSteps = unique(times);

  vehicles = struct();
  for i = 1:N
    idx = (vids == unique_ids(i));
    vehicles(i).id             = unique_ids(i);
    vehicles(i).times          = times(idx);
    vehicles(i).positions      = [xs(idx), ys(idx)];
    vehicles(i).speeds         = speeds(idx);
    vehicles(i).processingPower    = 10 + 20 * rand();
    vehicles(i).taskExecutionRate  = 5  + 5  * rand();
    vehicles(i).energyLevel        = 70 + 30 * rand();
  end

  fprintf('Loaded %d vehicles, %d timesteps from %s\n', N, numel(timeSteps), csv_path);
end