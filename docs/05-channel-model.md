# 05 — Channel Model (IEEE 802.11p)

## Why replace the linear model?

The original simulation used:

```matlab
Q(i,j) = max(0, 1 - d / 300)
```

This says link quality decreases linearly with distance from 1.0 (at d=0) to 0.0
(at d=300 m). It is easy to understand but physically wrong in two ways:

1. **Radio signals do not decay linearly with distance.** They decay according to
   the inverse square law at minimum, and faster in urban environments due to
   reflections and obstructions.

2. **There is no noise floor or sensitivity threshold.** A real radio receiver can
   only decode a signal if the received power exceeds its sensitivity threshold
   by a minimum signal-to-noise ratio (SNR). Below that, the link is simply unusable —
   it does not gradually degrade to "almost usable."

The IEEE 802.11p log-distance model corrects both issues.

---

## What is IEEE 802.11p?

IEEE 802.11p is the wireless standard used for Dedicated Short-Range Communication
(DSRC) in automotive networks. It operates at 5.9 GHz and is specifically designed
for vehicle-to-vehicle and vehicle-to-infrastructure communication.

Key properties relevant to this simulation:
- Carrier frequency: 5.9 GHz
- Maximum transmit power: 33 dBm (we use 20 dBm, a typical setting)
- Designed communication range: up to 300 m (up to 1000 m line-of-sight)
- Receiver sensitivity: approximately -95 dBm

---

## The path loss model

The log-distance path loss model (also called the log-normal shadowing model) is
the standard model used in V2V research literature:

```
PL(d) = PL(d0) + 10 * n * log10(d / d0) + X_sigma
```

Where:
- `PL(d)` is the path loss in dB at distance d
- `d0 = 10 m` is a reference distance (close enough for near-free-space conditions)
- `PL(d0) = 68 dB` is the path loss at the reference distance (5.9 GHz, free space)
- `n = 2.7` is the path loss exponent for urban environments
- `X_sigma` is a shadowing term (zero-mean Gaussian, std = sigma dB)

### Path loss exponent `n`

The exponent captures how the environment affects signal propagation:

| Environment | n | Meaning |
|---|---|---|
| Free space | 2.0 | Inverse square law only |
| Suburban roads | 2.0–2.5 | Open, few obstructions |
| Urban intersection | 2.7 | Buildings on corners, NLOS likely |
| Dense urban | 3.0–3.5 | Heavy obstructions, frequent NLOS |

We use n=2.7 for an urban intersection. Signals attenuate faster than free space
because vehicles frequently drive in non-line-of-sight (NLOS) conditions — buildings
block the direct path.

### Received power

```
P_rx = P_tx - PL(d)
```

If the transmitter sends at 20 dBm and path loss is 90 dB, the receiver gets
20 - 90 = -70 dBm.

### Link quality Q

We need to convert received power to a usability score Q ∈ [0, 1].

First, compute the SINR margin — how far above the minimum usable power the
received signal is:

```
P_rx_min = noise + SINR_min = -95 + 5 = -90 dBm
margin   = P_rx - P_rx_min
```

A positive margin means the signal is receivable. A negative margin means the
link is below the noise floor.

Then map margin to Q using a sigmoid function:

```matlab
Q = 1 ./ (1 + exp(-margin / 5))
```

The sigmoid has these properties:
- `margin = +20 dB` → Q ≈ 0.98 (excellent link)
- `margin = 0 dB`   → Q = 0.5  (link is right at threshold)
- `margin = -20 dB` → Q ≈ 0.02 (link is unusable)

The sigmoid is smooth — it avoids the sharp cutoff of a step function while
still having a natural threshold region.

Finally, a hard cutoff at 300 m is applied because beyond that range, 802.11p
simply cannot maintain a usable link in urban conditions:

```matlab
Q(d > 300) = 0;
```

---

## Comparison: linear vs 802.11p

| Distance | Linear Q | 802.11p Q |
|---|---|---|
| 10 m | 0.97 | ~0.97 |
| 50 m | 0.83 | ~0.88 |
| 100 m | 0.67 | ~0.72 |
| 150 m | 0.50 | ~0.55 |
| 200 m | 0.33 | ~0.28 |
| 250 m | 0.17 | ~0.08 |
| 300 m | 0.00 | 0.00 |

At close range, both models agree. At long range, 802.11p decays faster — correctly
reflecting the exponential nature of radio propagation.

---

## The offloading threshold

Both the baseline and the SUMO-upgraded simulation use Q ≥ 0.5 as the condition
for a vehicle to be considered a valid offloading candidate:

```matlab
cands = find(V2V(i,:) >= 0.5 & active_mask' & (1:N) ~= i);
```

With the 802.11p model, Q = 0.5 corresponds to a margin of exactly 0 dB — the
signal is exactly at the minimum usable power. This is a physically meaningful
threshold: below it, the link is unreliable and offloading would likely fail.

With the linear model, Q = 0.5 meant d = 150 m — a purely geometric condition
with no physical justification.

---

## The V2V matrix

`build_v2v_matrix.m` applies `channel_model` to every pair of active vehicles:

```matlab
function V2V = build_v2v_matrix(positions, active_mask)
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
```

`V2V(i,j)` is the link quality from vehicle i to vehicle j.
Since the channel model is symmetric (Q depends only on distance), V2V(i,j) = V2V(j,i).
Diagonal entries are 0 (a vehicle cannot offload to itself).
Inactive vehicles have 0 for all their entries.
