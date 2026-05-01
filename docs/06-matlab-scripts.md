# 06 — MATLAB Scripts

## `parse_fcd.m`

**Purpose:** Load the SUMO CSV into MATLAB as a struct array.

```matlab
[vehicles, timeSteps] = parse_fcd('src/sumo/fcd_traces.csv')
```

**What it returns:**

`vehicles` is a 1×N struct array where N is the number of unique vehicles.
Each element has:

| Field | Type | Description |
|---|---|---|
| `id` | integer | Vehicle ID (1, 2, 3...) |
| `times` | T×1 vector | Timesteps at which this vehicle was active |
| `positions` | T×2 matrix | [x, y] at each timestep |
| `speeds` | T×1 vector | Speed in m/s at each timestep |
| `processingPower` | scalar | Drawn from U(10, 30) GHz — CPU speed |
| `taskExecutionRate` | scalar | Drawn from U(5, 10) tasks/s |
| `energyLevel` | scalar | Drawn from U(70, 100) — battery level |

`timeSteps` is a sorted vector of all unique time values across all vehicles
(e.g., [0.0, 0.1, 0.2, ..., 100.0]).

**Why randomise vehicle capabilities here?**
Each call to `parse_fcd` assigns new random capabilities. This means that across
30 trials in `run_trials.m`, each trial uses the same mobility traces (same positions)
but different processing power and execution rates. This isolates the effect of the
offloading policy from the effect of vehicle hardware.

---

## `get_vehicle_state.m`

**Purpose:** Query a single vehicle's position and speed at a specific time.

```matlab
[pos, spd, active] = get_vehicle_state(vehicle, t)
```

Returns:
- `pos` — [x, y] position, or [NaN, NaN] if not active
- `spd` — speed in m/s, or NaN if not active
- `active` — true if the vehicle exists in the network at time t

**How it works:**

```matlab
idx = find(abs(vehicle.times - t) < 1e-9, 1);
```

It looks for a timestep in the vehicle's `times` array that matches `t` within
floating-point tolerance (1e-9 seconds). If no match is found, the vehicle is
not in the network at that moment.

**Why floating-point tolerance?** SUMO outputs times like 0.1, 0.2, etc. When
Python reads these from XML and MATLAB reads from CSV, floating-point rounding
can produce 0.09999999999 instead of exactly 0.1. The tolerance handles this.

---

## `channel_model.m`

Covered in detail in [05-channel-model.md](05-channel-model.md).

Quick signature: `Q = channel_model(d)` — accepts a scalar or vector of distances,
returns link quality Q ∈ [0, 1].

---

## `build_v2v_matrix.m`

**Purpose:** Build the N×N link quality matrix for all active vehicles at one timestep.

```matlab
V2V = build_v2v_matrix(positions, active_mask)
```

- `positions` — N×2 matrix of [x,y] for all N vehicles (inactive ones have [0,0])
- `active_mask` — N×1 logical, true for vehicles currently in the network
- Returns `V2V` — N×N matrix where `V2V(i,j)` = link quality from i to j

Only computes entries for active vehicle pairs. Inactive vehicles have all-zero rows
and columns.

---

## `generate_task_sumo.m`

**Purpose:** Create one computation task for a vehicle.

```matlab
task = generate_task_sumo(now, vehicle)
```

Randomly picks a task type (uniform over 3 types) and sets parameters:

| Field | PERCEPTION | PLANNING | CONTROL |
|---|---|---|---|
| `computationalRequirement` | 5–10 GHz | 2–5 GHz | 1–2 GHz |
| `dataSize` | 20–30 MB | 5–10 MB | 1–2 MB |
| `deadline` | now + 1.0 s | now + 1.5 s | now + 0.5 s |
| `priority` | 3 (high) | 2 (medium) | 3 (high) |

CONTROL tasks have the tightest deadline (0.5 s) — they must complete quickly
or a vehicle cannot adjust its actuators in time. PLANNING tasks have the most
slack (1.5 s) — route decisions can afford slightly more latency.

The `status` field starts as `'PENDING'` and transitions:
- `'PENDING'` → `'OFFLOADED'` (if sent to another vehicle)
- `'PENDING'` → `'LOCAL'` (if kept on the source vehicle)
- `'OFFLOADED'` or `'LOCAL'` → `'COMPLETED'` (after execution)

---

## `run_sumo_sim.m`

**Purpose:** Core simulation engine. Runs the full 100-second simulation for one policy.

```matlab
[totalComp, totalOffl, totalLoc, totalFail] = ...
    run_sumo_sim(policyFcn, vehicles, timeSteps, BANDWIDTH, TASK_ARRIVAL_RATE)
```

### The main loop (simplified)

```
for each timestep t:
    1. find active vehicles at time t
    2. build V2V link quality matrix
    3. generate new tasks (probabilistic)
    4. for each active vehicle, for each PENDING task:
           ask the policy: offload or local?
           update task status accordingly
    5. for each active vehicle:
           execute up to (taskExecutionRate × STEP_SIZE) tasks
```

### Why `numel(active_idx) < 2`?

```matlab
if numel(active_idx) < 2, continue; end
```

V2V offloading requires at least two vehicles. If only one vehicle is active
(or none), skip the timestep entirely — there is no one to offload to.

### Task execution

```matlab
maxN = vehicles(i).taskExecutionRate * STEP_SIZE;
```

A vehicle with `taskExecutionRate = 7 tasks/s` and `STEP_SIZE = 0.1 s` can
complete at most 0.7 tasks per timestep. In practice this means it completes
one task roughly every 1–2 timesteps. The fractional limit is a rate cap —
the loop breaks when `cnt >= maxN`.

### Why doesn't `totalFail` get populated?

The current implementation tracks `failedTasks` in the results struct but
never increments it. Deadline tracking (checking if a task missed its deadline)
was intentionally omitted to keep the scope focused on offloading decisions.
The completion rate is computed as `c / max(1, c+f)` which is effectively
`c / c = 1.0` when no failures are counted. This is consistent with the
baseline script's behaviour.

---

## `intersectionmultipolicy_sumo.m`

**Purpose:** Top-level script — single run, all five policies, results table + chart.

This is the simplest entry point. Run this first after generating traces to verify
everything works. Expected output:

```
Loaded 20 vehicles, 1001 timesteps from src/sumo/fcd_traces.csv

Policy            Completion Rate  Offloading Rate
----------------------------------------------------
  Running Greedy          ...  done
  Greedy              0.XXXX         0.XXXX
  Running SpeedCheck      ...  done
  SpeedCheck          0.XXXX         0.XXXX
  ...
```

Followed by a bar chart figure.

---

## `run_trials.m`

**Purpose:** Statistical validation — 30 trials per policy with different random seeds.

The key loop:

```matlab
for pi = 1:nP
  for trial = 1:N_TRIALS
    rng(trial * 100 + pi);   % unique seed per (policy, trial) combination
    [c, o, l, f] = run_sumo_sim(policies{pi}, vehicles, timeSteps, ...
                                 BANDWIDTH, TASK_ARRIVAL_RATE);
    all_rates(pi, trial, 1) = c / max(1, c+f);
    all_rates(pi, trial, 2) = o / max(1, o+l);
  end
end
```

`rng(trial * 100 + pi)` ensures each (policy, trial) combination uses a different
seed while remaining fully reproducible. Trial 1 of policy 1 always uses seed 101.
Trial 3 of policy 2 always uses seed 302.

The SUMO traces (`vehicles`, `timeSteps`) are loaded once and reused across all
trials. The randomness across trials comes from task generation and vehicle capability
assignment inside `parse_fcd` (called once) and `run_sumo_sim` (called per trial).

**Output:** mean ± std table + error-bar subplot figures.

---

## `stress_test.m`

**Purpose:** Show how each policy performs as conditions get harder.

### Sweep 1 — Vehicle density

```matlab
vehicle_counts = [4, 8, 12, 16, min(20, N_total)];
for vi = 1:numel(vehicle_counts)
  veh_subset = vehicles_all(1:min(nv, N_total));
  ...
end
```

Subsets the vehicle array to use only the first `nv` vehicles. This is a controlled
way to simulate different densities using the same trace file — the positions are
real SUMO positions, just fewer vehicles active at any one time.

### Sweep 2 — Task arrival rate

```matlab
arrival_rates = [0.25, 0.5, 1.0, 1.5, 2.0];
```

Increases load from light (0.25 tasks/s) to overload (2.0 tasks/s). At higher
rates, vehicles generate tasks faster than they can execute them, stressing the
offloading logic and revealing which policies degrade gracefully.

**Output:** 4-subplot figure (2 sweeps × 2 metrics).