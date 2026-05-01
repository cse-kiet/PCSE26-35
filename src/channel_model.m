function Q = channel_model(d)
% CHANNEL_MODEL  IEEE 802.11p log-distance path loss link quality.
%
%   Q = channel_model(d)
%
%   Models the V2V link quality for DSRC (5.9 GHz) using the
%   log-distance path loss model with urban parameters:
%     PL(d) = PL(d0) + 10*n*log10(d/d0) + X_sigma
%
%   Parameters (ITU-R urban V2V, 5.9 GHz):
%     d0        = 10 m        (reference distance)
%     PL(d0)    = 68 dB       (free-space path loss at d0)
%     n         = 2.7         (path loss exponent, urban)
%     sigma     = 4 dB        (shadowing standard deviation)
%     P_tx      = 20 dBm      (transmit power)
%     noise     = -95 dBm     (receiver sensitivity threshold)
%     SINR_min  = 5 dB        (minimum SINR for link to be usable)
%
%   Returns Q in [0, 1]:
%     Q = 1.0  ->  excellent link (close range)
%     Q = 0.5  ->  marginal link (edge of reliable range)
%     Q = 0.0  ->  no link (beyond range or path loss too high)
%
%   Eligibility threshold: Q >= 0.5 (same as original simulation)

  % Parameters
  d0       = 10;        % m
  PL_d0    = 68;        % dB
  n        = 2.7;       % path loss exponent
  sigma    = 4;         % dB shadowing std dev
  P_tx     = 20;        % dBm
  noise    = -95;       % dBm (receiver sensitivity)
  SINR_min = 5;         % dB
  P_rx_min = noise + SINR_min;   % = -90 dBm minimum received power

  % Clamp d to avoid log(0)
  d = max(d, d0);

  % Log-distance path loss (dB)
  PL = PL_d0 + 10 * n * log10(d / d0);

  % Deterministic pseudo-shadowing (avoids randomness per-step)
  X_sigma = sigma * sin(d * 0.1);

  % Received power (dBm)
  P_rx = P_tx - PL - X_sigma;

  % SINR margin above minimum required
  margin = P_rx - P_rx_min;

  % Map margin to Q in [0,1] using a soft sigmoid
  Q = 1 ./ (1 + exp(-margin / 5));

  % Hard cutoff at 300 m
  Q(d > 300) = 0;
end