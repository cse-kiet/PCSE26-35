# V2V Computation Offloading Simulation

> A comparative study of five heuristic Vehicle-to-Vehicle (V2V) computation offloading strategies for autonomous vehicle networks, simulated at an urban intersection in MATLAB — upgraded with realistic SUMO mobility traces.

---

## Paper

**Title:** A Comparative Study of Multi-Heuristic V2V Computation Offloading Strategies for Autonomous Vehicle Networks
**Conference:** ICCISD-2026, Sharda University
**Paper ID:** 527
**Authors:** Ghanatava Vashu Thakaran, Ayush Agrawal, Sarvagya Pradhan, Kshitiz Agarwal, Gaurav Parashar
**Institution:** Department of Computer Science and Engineering, KIET Group of Institutions, Ghaziabad, India

---

## Description

Autonomous vehicles generate large volumes of latency-sensitive tasks — object detection, path planning, collision avoidance — that cannot always be processed locally. This project investigates **Vehicle-to-Vehicle (V2V) computation offloading** as an infrastructure-free alternative to Vehicle-to-Infrastructure (V2I) models.

The project implements and compares **five heuristic offloading strategies** across two simulation tiers:

- **Baseline** — 4 vehicles, fixed positions, linear link quality model
- **SUMO-upgraded** — 20 vehicles, realistic mobility from SUMO, IEEE 802.11p channel model, statistical evaluation, stress testing

| Strategy | Core Idea |
|---|---|
| **Greedy** | Always pick the neighbour with the lowest total delay |
| **Speed-Aware Greedy** | Greedy + only offload if remote is faster than local |
| **Threshold-Based** | First-fit: accept the first neighbour that meets the deadline |
| **Game-Theoretic** | Maximise a utility function balancing delay savings vs energy cost |
| **Load-Balancing** | Pick the neighbour with the shortest current task queue |

### Key Results (Baseline — 4 vehicles)

| Strategy | Completion Rate | Offloading Rate |
|---|---|---|
| Greedy | 0.9747 | 0.6667 |
| Speed-Aware Greedy | 0.9997 | 0.1983 |
| Threshold-Based | 0.9772 | 0.6620 |
| Game-Theoretic | **1.0000** | 0.0181 |
| Load-Balancing | **1.0000** | **0.6290** |

---

## Repository Structure

```
project/
├── Makefile                              # Automates SUMO + MATLAB pipeline
├── intersection3.m                       # Baseline: single-policy Greedy
├── intersection-multiploicy.m            # Baseline: five-policy comparison
├── multipolicy-comparative-simulation.m  # Baseline: detailed comparison
├── intersectionmultipolicy_sumo.m        # SUMO: single-run five-policy comparison
├── run_trials.m                          # SUMO: 30-trial statistical evaluation
├── stress_test.m                         # SUMO: density + load sweep
├── src/
│   ├── parse_fcd.m                       # Load SUMO CSV into vehicle structs
│   ├── get_vehicle_state.m               # Query vehicle position at time t
│   ├── channel_model.m                   # IEEE 802.11p log-distance path loss
│   ├── build_v2v_matrix.m                # Build NxN link quality matrix
│   ├── generate_task_sumo.m              # Task generator for SUMO vehicles
│   ├── run_sumo_sim.m                    # Shared simulation engine
│   ├── greedy_policy.m                   # Policy implementations (updated signatures)
│   ├── speedcheck_policy.m
│   ├── threshold_policy.m
│   ├── gametheoretic_policy.m
│   ├── loadbalance_policy.m
│   └── sumo/
│       ├── intersection.nod.xml          # Road network nodes
│       ├── intersection.edg.xml          # Road network edges
│       ├── intersection.net.xml          # Compiled network (generated)
│       ├── intersection.rou.xml          # Vehicle routes (generated)
│       ├── intersection.sumocfg          # SUMO run configuration
│       ├── generate_traces.py            # Run SUMO, export FCD to CSV
│       ├── visualise_traces.py           # Plot trajectories, heatmap, speeds
│       └── fcd_traces.csv                # Mobility traces output (generated)
```

---

## Requirements

### Software

| Requirement | Version | Notes |
|---|---|---|
| MATLAB | R2022a or later | Any edition including Student |
| SUMO | 1.8 or later | `sudo apt install sumo sumo-tools` |
| Python | 3.8 or later | Standard library + matplotlib |
| matplotlib | 3.5 or later | `pip install matplotlib` |

### Hardware

| Component | Minimum | Recommended |
|---|---|---|
| RAM | 4 GB | 8 GB |
| CPU | Dual-core 2.0 GHz | Quad-core |
| Disk | 200 MB | 500 MB |
| Display | Any | 1920×1080 for best figure rendering |

---

## Quick Start

### Baseline (MATLAB only, no SUMO needed)

```matlab
cd('path/to/project')
intersectionmultipolicy_sumo   % or the original intersection-multiploicy.m
```

### SUMO-Upgraded (full pipeline)

```bash
# Step 1 — generate mobility traces
make sumo

# Step 2 — visualise vehicle movement (pick one)
make sumo-gui          # live GUI playback (needs desktop)
make visualise         # save PNG plots to src/sumo/

# Step 3 — run MATLAB analysis
make matlab-compare    # single run, policy table + bar chart
make matlab-trials     # 30-trial mean ± std
make matlab-stress     # density + load sweep plots
```

---

## Simulation Parameters

### SUMO Trace Generation

| Parameter | Default | How to change | Effect |
|---|---|---|---|
| Random seed | 42 | `make sumo SEED=123` | Different vehicle patterns |
| Injection period | 5 s | `make sumo-rebuild PERIOD=3` | More vehicles (~33 at period=3) |
| Simulation duration | 100 s | Edit `intersection.sumocfg` `<end>` | Longer/shorter trace |
| Road speed limit | 13.89 m/s (50 km/h) | Edit `intersection.edg.xml` `speed=` | Faster/slower vehicles |

### MATLAB Simulation

| Parameter | Location | Default | Effect |
|---|---|---|---|
| `BANDWIDTH` | top of each script | 50 MB/s | Higher = easier offloading |
| `TASK_ARRIVAL_RATE` | top of each script | 0.5 tasks/s | Higher = heavier load |
| `N_TRIALS` | `run_trials.m` | 30 | More trials = tighter confidence |
| Vehicle subset | `stress_test.m` `vehicle_counts` | [4,8,12,16,20] | Density sweep range |
| Arrival sweep | `stress_test.m` `arrival_rates` | [0.25,0.5,1,1.5,2] | Load sweep range |

### Channel Model (IEEE 802.11p)

Tunable inside `src/channel_model.m`:

| Parameter | Default | Effect |
|---|---|---|
| Path loss exponent `n` | 2.7 (urban) | Lower = better range (suburban: 2.0) |
| Shadowing std `sigma` | 4 dB | Higher = more link variability |
| Transmit power `P_tx` | 20 dBm | Higher = longer range |
| Comm range cutoff | 300 m | Hard limit beyond which Q = 0 |
| Offload threshold | Q ≥ 0.5 | Lower = more offload candidates |

---

## Output Reference

### Baseline scripts

| Script | Output |
|---|---|
| `intersection3.m` | Vehicle paths + per-step task counts, final summary printed |
| `intersection-multiploicy.m` | Grouped bar chart: Completion vs Offloading Rate |

### SUMO-upgraded scripts

| Script / Command | Output |
|---|---|
| `intersectionmultipolicy_sumo` | Results table (5 rows) + bar chart |
| `run_trials` | mean ± std table + error-bar subplots |
| `stress_test` | 4-subplot figure: density sweep + load sweep |
| `make visualise` | `trajectory_plot.png`, `heatmap.png`, `speed_profile.png` |
| `make sumo-gui` | Live SUMO GUI — watch vehicles move in real time |

---

## Useful Make Commands

```bash
make                         # generate traces (default)
make sumo                    # same as above, skip if already done
make sumo-rebuild            # force regenerate network + routes + traces
make sumo-rebuild SEED=123   # different seed, different vehicle pattern
make sumo-rebuild PERIOD=3   # denser traffic (~33 vehicles)
make sumo-gui                # open SUMO GUI for live playback
make visualise               # generate PNG plots from traces
make matlab-compare          # run intersectionmultipolicy_sumo in batch
make matlab-trials           # run run_trials in batch
make matlab-stress           # run stress_test in batch
make clean                   # delete all generated SUMO files
make help                    # print all targets
```

---

## Reproducibility

SUMO traces are deterministic given the same `--seed`. MATLAB randomness (task generation, vehicle capabilities) can be fixed with:

```matlab
rng(42);   % add at top of any script
```

The `run_trials.m` script already sets `rng(trial * 100 + pi)` per trial, so its results are fully reproducible without extra steps.

---

## Contact

| Name | Email |
|---|---|
| Ghanatava Vashu Thakaran | ghanatva@gmail.com |
| Ayush Agrawal | ayush.agr254@gmail.com |
| Sarvagya Pradhan | sarvagyapradhan823@gmail.com |
| Kshitiz Agarwal | kshitizagarwal1710@gmail.com |
| Gaurav Parashar | gauravparashar24@gmail.com |

---

## License

This project was developed for academic research purposes as part of a B.Tech final year project at KIET Group of Institutions, Ghaziabad (Session 2025-26), affiliated to Dr. A.P.J. Abdul Kalam Technical University, Lucknow.