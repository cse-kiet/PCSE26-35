# 07 — Offloading Policies

## Shared signature

All five policies have the same function signature:

```matlab
function [doOff, target] = policy_name(task, src, cands, vehicles, queues, BANDWIDTH)
```

| Argument | Type | Description |
|---|---|---|
| `task` | struct | The task being considered for offloading |
| `src` | integer | Index of the vehicle that owns the task |
| `cands` | integer vector | Indices of eligible neighbours (V2V quality ≥ 0.5) |
| `vehicles` | struct array | All vehicle structs (for accessing processingPower, etc.) |
| `queues` | cell array | Current task queues of all vehicles |
| `BANDWIDTH` | scalar | Link bandwidth in MB/s |

Returns:
- `doOff` — true to offload, false to execute locally
- `target` — index of target vehicle (0 if not offloading)

**Why pass BANDWIDTH explicitly instead of using a global?**
The original baseline used `global BANDWIDTH`. Globals in MATLAB are fragile —
they must be declared in every function that uses them, they persist between
runs, and they make functions hard to test in isolation. Passing BANDWIDTH
explicitly makes each policy a pure function with no hidden state.

---

## Policy 1 — Greedy (`greedy_policy.m`)

**Core idea:** Always offload to the neighbour that minimises total task completion time.

```matlab
for j = cands
  ttx = task.dataSize / BANDWIDTH;                          % transfer time
  trt = task.computationalRequirement / vehicles(j).processingPower;  % compute time
  tot = ttx + trt;
  if tot < bestT
    bestT = tot; bestK = j;
  end
end
rem   = task.deadline - task.generationTime;
doOff = (bestK > 0) && (bestT < rem);
```

**Transfer time** (`ttx`) — how long it takes to send the task data over the V2V link.
Proportional to data size, inversely proportional to bandwidth.

**Remote compute time** (`trt`) — how long the target vehicle takes to process the task.
Proportional to computational requirement, inversely proportional to target's CPU speed.

**Decision rule:** Offload if the best remote option finishes before the deadline AND
there is at least one candidate.

**Strength:** Maximises the benefit of every offloading decision — always picks the
fastest available option.

**Weakness:** Ignores whether the target vehicle is already overloaded. If all tasks
go to the most powerful vehicle, it may become a bottleneck.

---

## Policy 2 — SpeedCheck (`speedcheck_policy.m`)

**Core idea:** Greedy, but only offload if the remote option is faster than local execution.

```matlab
localT = task.computationalRequirement / vehicles(src).processingPower;
...
doOff = (bestK > 0) && (bestT < rem) && (bestT < localT);
```

Adds one extra condition: `bestT < localT`. If the source vehicle can process the
task faster locally than any neighbour can (including transfer time), keep it local.

**Strength:** Avoids unnecessary offloading. If the source vehicle is powerful, it
does not waste bandwidth sending tasks to slower neighbours.

**Weakness:** Results in a low offloading rate — many tasks stay local. This is
reflected in the results where SpeedCheck has the lowest offloading rate (~0.20).

---

## Policy 3 — Threshold (`threshold_policy.m`)

**Core idea:** First-fit — offload to the first neighbour that can meet the deadline.

```matlab
for j = cands
  ttx = task.dataSize / BANDWIDTH;
  trt = task.computationalRequirement / vehicles(j).processingPower;
  if (ttx + trt) < rem
    doOff = true; target = j; return;   % take the first one that works
  end
end
```

Does not search for the *best* neighbour — accepts the first *acceptable* one and
returns immediately.

**Strength:** Very fast decision (early exit). Low computation overhead per task.

**Weakness:** Does not optimise — the first viable neighbour may not be the best.
The order in which candidates are evaluated (based on their index) can introduce
implicit bias. Does not consider load balance.

---

## Policy 4 — Game-Theoretic (`gametheoretic_policy.m`)

**Core idea:** Model offloading as a utility maximisation game. Offload only if the
utility (time saved minus energy cost) is positive.

```matlab
alpha  = 0.2;
localT = task.computationalRequirement / vehicles(src).processingPower;
...
for j = cands
  tot   = ttx + trt;
  saved = localT - tot;      % time saved by offloading
  ecost = task.dataSize * 0.1;  % energy cost of transmission
  U     = saved - alpha * ecost;
  if U > bestU
    bestU = U; bestK = j;
  end
end
doOff = (bestK > 0) && (bestU > 0) && (bestTot < rem);
```

**Utility function breakdown:**
- `saved` — time saved by offloading vs local execution. Positive means offloading
  is faster; negative means it is slower.
- `ecost` — energy cost of transmitting the data (proportional to data size).
- `alpha = 0.2` — weight on energy cost. Smaller alpha = more willing to offload
  (energy matters less). Larger alpha = more conservative.

**Why game-theoretic?** In a multi-vehicle system, each vehicle is a rational agent
trying to maximise its own outcome. The utility function captures this — a vehicle
only offloads when it genuinely benefits.

**Strength:** Theoretically principled. Only offloads when there is a real benefit.
Achieves high completion rates because it avoids offloading tasks that would arrive
late or waste energy.

**Weakness:** Produces a very low offloading rate (~0.02) because the utility
condition is strict. Most tasks are kept local.

---

## Policy 5 — Load-Balancing (`loadbalance_policy.m`)

**Core idea:** Distribute work evenly. Pick the neighbour with the smallest queue
that can still meet the deadline.

```matlab
for j = cands
  L   = numel(queues{j});       % current queue length of vehicle j
  ttx = task.dataSize / BANDWIDTH;
  trt = task.computationalRequirement / vehicles(j).processingPower;
  tot = ttx + trt;
  if L < bestLen && tot < timeRem
    bestLen = L;
    bestK   = j;
  end
end
```

Selects the vehicle with the fewest pending tasks, subject to the constraint that
the task can still complete before its deadline.

**Strength:** Prevents any single vehicle from becoming overloaded. In dense
scenarios with many vehicles, spreading the load improves throughput for everyone.
This policy achieves the best combination of completion rate and offloading rate.

**Weakness:** Queue length is a proxy for load — it counts tasks but ignores their
size. A vehicle with 2 PERCEPTION tasks (heavy) looks less loaded than one with
3 CONTROL tasks (light), but may actually be busier.

---

## Comparing the policies

| Policy | Offloads often? | Picks best target? | Considers load? | Considers energy? |
|---|---|---|---|---|
| Greedy | Yes | Yes (fastest) | No | No |
| SpeedCheck | No (conservative) | Yes (fastest) | No | No |
| Threshold | Yes | No (first-fit) | No | No |
| Game-Theoretic | No (strict utility) | Yes (max utility) | No | Yes |
| Load-Balancing | Yes | Yes (lightest queue) | Yes | No |

**Why Load-Balancing wins overall:**
It is the only policy that considers the current state of the entire network
(queue lengths) when making a decision. The other policies evaluate each candidate
in isolation. In a dynamic scenario where vehicles constantly enter, leave, and
change loads, network-aware decisions consistently outperform local ones.
