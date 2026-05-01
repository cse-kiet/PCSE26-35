# 08 — Parameters Reference

Every tunable parameter in the simulation, where it lives, what it controls,
and what values to try.

---

## SUMO parameters (terminal / Makefile)

### Seed — `SEED=42`

```bash
make sumo-rebuild SEED=123
```

Controls the randomness in both `randomTrips.py` (which routes vehicles take,
when they depart) and SUMO's internal simulation (driver behaviour, lane changes).

Changing the seed gives a completely different set of vehicle trajectories.
Use different seeds to verify that your results hold across different mobility
patterns — not just one lucky arrangement.

Recommended values to try: 42 (default), 99, 123, 777, 2026.

---

### Vehicle injection period — `PERIOD=5`

```bash
make sumo-rebuild PERIOD=3    # ~33 vehicles (dense)
make sumo-rebuild PERIOD=5    # ~20 vehicles (default)
make sumo-rebuild PERIOD=10   # ~10 vehicles (sparse)
```

`randomTrips.py` injects one vehicle approximately every `period` seconds.
Over a 100 s simulation:
- Period 3 → ~33 vehicles
- Period 5 → ~20 vehicles
- Period 10 → ~10 vehicles

Lower period = denser traffic = more offloading opportunities but also more
competition for resources.

---

### Simulation duration — `intersection.sumocfg`

```xml
<end value="100"/>
```

Change to 200 for a longer trace. Also update the MATLAB scripts:

```matlab
SIM_TIME = 200;  % in intersectionmultipolicy_sumo.m
```

And regenerate: `make sumo-rebuild`.

---

### Road speed limit — `intersection.edg.xml`

```xml
speed="13.89"   <!-- 50 km/h -->
```

| Speed (m/s) | Speed (km/h) | Use case |
|---|---|---|
| 8.33 | 30 | Slow urban / school zone |
| 13.89 | 50 | Standard urban (default) |
| 16.67 | 60 | Fast urban arterial |
| 22.22 | 80 | Semi-highway approach |

Higher speed means vehicles spend less time near each other — fewer timesteps
where two vehicles are within 300 m, so fewer offloading opportunities.

After changing: `make sumo-rebuild` to regenerate all traces.

---

## MATLAB parameters

### `BANDWIDTH` — link bandwidth in MB/s

```matlab
BANDWIDTH = 50;   % default, in all three main scripts
```

| Value | Scenario |
|---|---|
| 10 MB/s | Congested channel, many simultaneous transmissions |
| 27 MB/s | IEEE 802.11p theoretical max at 10 MHz channel |
| 50 MB/s | Default (generous, assumes good conditions) |
| 100 MB/s | Future 802.11bd / C-V2X scenario |

Higher bandwidth reduces transfer time `ttx = dataSize / BANDWIDTH`, making
offloading more attractive for all policies. Very high bandwidth makes SpeedCheck
and Game-Theoretic offload more aggressively.

---

### `TASK_ARRIVAL_RATE` — tasks per second per vehicle

```matlab
TASK_ARRIVAL_RATE = 0.5;   % default
```

| Value | Scenario |
|---|---|
| 0.25 | Light load — vehicles rarely generate tasks |
| 0.5 | Moderate load — default |
| 1.0 | Heavy load — vehicles generating tasks frequently |
| 1.5 | Near-saturation — queues start building up |
| 2.0 | Overload — more tasks than can be processed |

The stress test (`stress_test.m`) sweeps this automatically. Change it manually
in `intersectionmultipolicy_sumo.m` to run a specific scenario.

---

### `N_TRIALS` — number of trials in `run_trials.m`

```matlab
N_TRIALS = 30;
```

| Value | Purpose | Approximate runtime |
|---|---|---|
| 5 | Quick sanity check | ~1 min |
| 30 | Paper-quality statistics | ~10 min |
| 100 | High-confidence confidence intervals | ~35 min |

30 trials is the standard minimum for reporting mean ± std in a simulation paper.
With 30 trials, the standard error of the mean is std / sqrt(30) ≈ 0.18 × std.

---

### `vehicle_counts` — density sweep range in `stress_test.m`

```matlab
vehicle_counts = [4, 8, 12, 16, min(20, N_total)];
```

Controls which vehicle counts are tested. You can change this to any subset of
vehicles in the trace. `min(20, N_total)` ensures you don't request more vehicles
than the trace actually has.

---

### `arrival_rates` — load sweep range in `stress_test.m`

```matlab
arrival_rates = [0.25, 0.5, 1.0, 1.5, 2.0];
```

Controls which task arrival rates are tested. Add more points for a finer sweep:

```matlab
arrival_rates = [0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0];
```

---

## Channel model parameters — `src/channel_model.m`

### Path loss exponent `n`

```matlab
n = 2.7;   % urban intersection
```

| Value | Environment |
|---|---|
| 2.0 | Free space / open suburban road |
| 2.5 | Light urban |
| 2.7 | Urban intersection (default) |
| 3.0 | Dense urban with frequent NLOS |
| 3.5 | Heavily obstructed (tunnels, parking garages) |

Increasing `n` shrinks the effective communication range — Q falls to 0.5 at
a shorter distance, so fewer vehicle pairs qualify as offloading candidates.

---

### Shadowing std deviation `sigma`

```matlab
sigma = 4;   % dB
```

| Value | Scenario |
|---|---|
| 0 | No shadowing (deterministic, theoretical) |
| 2 | Light shadowing (wide open road) |
| 4 | Typical urban (default) |
| 8 | Heavy shadowing (buildings, large trucks blocking signal) |

Larger sigma causes the Q value to fluctuate more with distance — some distances
that would normally have Q > 0.5 may fall below threshold, and vice versa.

---

### Transmit power `P_tx`

```matlab
P_tx = 20;   % dBm
```

| Value | Scenario |
|---|---|
| 10 dBm | Low power / battery saving mode |
| 20 dBm | Default DSRC power level |
| 30 dBm | Maximum allowed by FCC/ETSI for DSRC |
| 33 dBm | Absolute maximum (regulations forbid this) |

Higher transmit power increases received signal strength, extending the effective
communication range. At 30 dBm, vehicles can communicate reliably beyond 300 m —
you would need to raise the hard cutoff accordingly.

---

### Communication range cutoff

```matlab
Q(d > 300) = 0;   % in channel_model.m
```

Change 300 to any value in metres. This is a hard limit — regardless of calculated
Q, no offloading is possible beyond this distance. This represents the practical
limit of 802.11p in urban deployments.

---

### Offloading eligibility threshold

```matlab
cands = find(V2V(i,:) >= 0.5 & active_mask' & (1:N) ~= i);
```

Change `0.5` in `run_sumo_sim.m` to adjust when a neighbour qualifies:

| Threshold | Effect |
|---|---|
| 0.3 | More candidates (longer range, weaker links accepted) |
| 0.5 | Default (link is at least at threshold power) |
| 0.7 | Fewer candidates (only strong links accepted) |
| 0.9 | Very few candidates (near-optimal link quality required) |

---

## Reproducibility settings

### Fix MATLAB random seed

```matlab
rng(42);   % add at top of any script
```

This fixes task generation, vehicle capability assignment, and all other MATLAB
randomness. Results will be identical across runs.

### Fix SUMO seed

```bash
make sumo-rebuild SEED=42   % default
```

Same seed = same vehicle routes and SUMO behaviour. Different seed = different
vehicle patterns (use this to test generalisability).

### Fully reproducible run

```bash
make sumo-rebuild SEED=42           # fixed SUMO seed
```

Then in MATLAB:
```matlab
rng(42);
intersectionmultipolicy_sumo        % fixed MATLAB seed
```

`run_trials.m` is already fully reproducible — it sets `rng(trial * 100 + pi)`
per trial, so you get the same table every time without needing an external `rng(42)`.