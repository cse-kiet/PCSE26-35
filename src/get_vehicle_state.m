function [pos, spd, active] = get_vehicle_state(vehicle, t)
% GET_VEHICLE_STATE  Return position and speed of a vehicle at time t.
%   active = false if the vehicle is not in the network at time t.
  idx = find(abs(vehicle.times - t) < 1e-9, 1);
  if isempty(idx)
    pos    = [NaN, NaN];
    spd    = NaN;
    active = false;
  else
    pos    = vehicle.positions(idx, :);
    spd    = vehicle.speeds(idx);
    active = true;
  end
end