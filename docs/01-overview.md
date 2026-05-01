# 01 — Project Overview

## What problem does this project solve?

Autonomous vehicles run computationally expensive tasks in real time:

- **PERCEPTION** — object detection, lane recognition, obstacle mapping
- **PLANNING** — route calculation, decision making
- **CONTROL** — steering, braking, acceleration commands

These tasks have hard deadlines (0.5–1.5 seconds). A vehicle that misses a CONTROL
deadline could fail to brake in time. On lower-cost hardware, the onboard CPU may not
be fast enough to process everything within the deadline.

The dominant solution is **Vehicle-to-Infrastructure (V2I)** offloading — send the task
to a roadside unit (RSU) or cloud server. The problem: RSUs have limited coverage,
require expensive deployment, and introduce latency from the round trip.

This project investigates **Vehicle-to-Vehicle (V2V) offloading** as an alternative.
When vehicles are physically close (within 300 m), they can communicate directly over
IEEE 802.11p DSRC radio and share spare compute capacity with each other — no RSU needed.

---

## What the project does

It simulates a group of autonomous vehicles at a signalised urban intersection and
evaluates five different strategies for deciding *when* and *where* to offload a task.

For each task that a vehicle generates, the strategy answers:
1. Should I offload this task or process it locally?
2. If offloading, which nearby vehicle should receive it?

The five strategies differ in how they answer these questions.

---

## Two simulation tiers

### Tier 1 — Baseline (original)

| Property | Value |
|---|---|
| Vehicles | 4, fixed starting positions |
| Movement | Straight lines, stops at traffic light |
| Link quality | `Q = max(0, 1 - d/300)` — linear decay |
| Statistical validity | Single run only |
| Files | `intersection3.m`, `intersection-multiploicy.m` |

The baseline was the original submission. It worked but had weaknesses:
- Only 4 vehicles is not a realistic scenario
- Linear link quality is not physically accurate
- Single run means results could be a lucky/unlucky sample
- No stress testing

### Tier 2 — SUMO-upgraded (new)

| Property | Value |
|---|---|
| Vehicles | 20, entering/leaving dynamically |
| Movement | SUMO-generated: real road geometry, traffic signals, speed profiles |
| Link quality | IEEE 802.11p log-distance path loss model |
| Statistical validity | 30 trials with mean ± std |
| Stress testing | Density sweep (4–20 vehicles) + load sweep (0.25–2.0 tasks/s) |
| Files | `intersectionmultipolicy_sumo.m`, `run_trials.m`, `stress_test.m` |

The SUMO upgrade directly addresses every weakness identified by the paper evaluator.

---

## What SUMO is

SUMO (Simulation of Urban MObility) is an open-source microscopic traffic simulator
developed by the German Aerospace Center (DLR). "Microscopic" means it models each
vehicle individually — its position, speed, acceleration, and lane changes — rather
than treating traffic as a fluid.

SUMO outputs **Floating Car Data (FCD)**: a time series of every vehicle's position,
speed, and heading at every timestep. This is the same format used by real GPS probes
in smart city systems.

In this project, SUMO generates the mobility — where vehicles are and how fast they
are going at each moment. MATLAB then reads that data and runs the offloading simulation
on top of it. The two tools are completely decoupled.

---

## What this project is NOT doing

- Not implementing MAC-layer CSMA/CA contention (out of scope for this paper)
- Not implementing Rayleigh fading or multipath (the log-distance model is sufficient)
- Not using TraCI for real-time SUMO-MATLAB coupling (offline FCD parsing is simpler
  and fully reproducible)
- Not deploying on real hardware or real vehicles
