function V2V = build_v2v_matrix(positions, active_mask)
% BUILD_V2V_MATRIX  Compute link quality matrix for all active vehicles.
%
%   positions   : Nx2 matrix of [x,y] positions
%   active_mask : Nx1 logical, true if vehicle is in network at this step
%
%   V2V(i,j) = channel_model(distance(i,j)) for active pairs, 0 otherwise

  N   = size(positions, 1);
  V2V = zeros(N);
  idx = find(active_mask);
  for ii = 1:numel(idx)
    for jj = 1:numel(idx)
      i = idx(ii); j = idx(jj);
      if i ~= j
        d = norm(positions(i,:) - positions(j,:));
        V2V(i,j) = channel_model(d);
      end
    end
  end
end