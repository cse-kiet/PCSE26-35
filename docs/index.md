# Documentation Index
## V2V Computation Offloading Simulation — ICCISD-2026, Paper ID 527

This documentation covers every component of the simulation pipeline — what was built,
why each decision was made, and how all the pieces fit together.

---

## Documents

| File | What it covers |
|---|---|
| [01-overview.md](01-overview.md) | What the project is, what problem it solves, and the two simulation tiers |
| [02-architecture.md](02-architecture.md) | How all files relate to each other, data flow, and design decisions |
| [03-sumo-network.md](03-sumo-network.md) | How the SUMO road network was built and why it was designed this way |
| [04-trace-generation.md](04-trace-generation.md) | How `generate_traces.py` works — SUMO pipeline, FCD format, CSV export |
| [05-channel-model.md](05-channel-model.md) | IEEE 802.11p physics — why it replaces the linear model, how it works |
| [06-matlab-scripts.md](06-matlab-scripts.md) | Every MATLAB file explained line by line |
| [07-policies.md](07-policies.md) | Each of the five offloading policies — logic, strengths, weaknesses |
| [08-parameters.md](08-parameters.md) | Every tunable parameter, what it controls, and what values to try |
| [09-visualisation.md](09-visualisation.md) | All output plots — what they show and how to read them |
| [10-pipeline.md](10-pipeline.md) | End-to-end run guide with the Makefile |

---

## Quick orientation

```
Terminal (SUMO)          MATLAB (analysis)
     |                        |
  make sumo              intersectionmultipolicy_sumo
     |                   run_trials
  make visualise         stress_test
     |
  make sumo-gui
```

SUMO generates realistic vehicle positions → MATLAB reads them → policies are evaluated →
results are plotted. The two sides are independent — SUMO runs once, MATLAB runs many times.