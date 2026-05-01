# V2V Computation Offloading Simulation

> A comparative study of five heuristic Vehicle-to-Vehicle (V2V) computation offloading strategies for autonomous vehicle networks, simulated at an urban intersection in MATLAB.

---

## 📄 Paper

**Title:** A Comparative Study of Multi-Heuristic V2V Computation Offloading Strategies for Autonomous Vehicle Networks  
**Conference:** ICCISD-2026, Sharda University  
**Paper ID:** 527  
**Authors:** Ghanatava Vashu Thakaran, Ayush Agrawal, Sarvagya Pradhan, Kshitiz Agarwal, Gaurav Parashar  
**Institution:** Department of Computer Science and Engineering, KIET Group of Institutions, Ghaziabad, India

---

## 📖 Description

Autonomous vehicles generate large volumes of latency-sensitive tasks — object detection, path planning, collision avoidance — that cannot always be processed locally, especially on lower-cost hardware. This project investigates **Vehicle-to-Vehicle (V2V) computation offloading** as an infrastructure-free alternative to the dominant Vehicle-to-Infrastructure (V2I) model.

Rather than sending tasks to roadside units or cloud servers, vehicles offload computation directly to nearby vehicles with spare processing capacity. This approach reduces dependence on RSU coverage while achieving low latency through physical proximity.

The project implements and compares **five heuristic offloading strategies** under a unified MATLAB simulation of four autonomous vehicles converging at a signalised urban intersection:

| Strategy | Core Idea |
|---|---|
| **Greedy** | Always pick the neighbour with the lowest total delay |
| **Speed-Aware Greedy** | Greedy + only offload if remote is faster than local |
| **Threshold-Based** | First-fit: accept the first neighbour that meets the deadline |
| **Game-Theoretic** | Maximise a utility function balancing delay savings vs energy cost |
| **Load-Balancing** | Pick the neighbour with the shortest current task queue |

### Key Results

| Strategy | Completion Rate | Offloading Rate |
|---|---|---|
| Greedy | 0.9747 | 0.6667 |
| Speed-Aware Greedy | 0.9997 | 0.1983 |
| Threshold-Based | 0.9772 | 0.6620 |
| Game-Theoretic | **1.0000** | 0.0181 |
| Load-Balancing | **1.0000** | **0.6290** |

**Finding:** Game-Theoretic and Load-Balancing both achieve perfect task completion. Load-Balancing additionally maintains a high offloading rate (62.9%), making it the best overall strategy. Greedy achieves the highest throughput but is most vulnerable to mobility-induced deadline misses.

---

## 🗂️ Repository Structure

```
project/
├── intersection3.m              
├── intersection-multipolicy.m            
├── multipolicy-comparitive-simulation.m  
├── Instruction to run.txt      
└── README.md                   
```

---

## ⚙️ Requirements

### Software

| Requirement | Version | Notes |
|---|---|---|
| MATLAB | R2022a or later | Any edition including Student |
| MATLAB Toolboxes | None | Only built-in functions used |

### Hardware

| Component | Minimum | Recommended |
|---|---|---|
| RAM | 4 GB | 8 GB |
| CPU | Dual-core 2.0 GHz | Quad-core |
| Disk | 50 MB | 100 MB |
| Display | Any | 1920×1080 for best figure rendering |

### Operating System

- Windows 10 / 11
- macOS 12 (Monterey) or later
- Ubuntu 20.04 or later
- Any OS that supports MATLAB R2022a+

---

## 🚀 Quick Start

```matlab
% 1. Set MATLAB working directory to the project folder
cd('path/to/project')

% 2. Run the five-policy comparison (main result)
multipolicy-comparitive-simulation.m

% 3. Run the single-policy greedy simulation
intersection3.m

% 4. Run the multipolicy-simulation with assumption of 100% task completion
intersection-multipolicy.m
```

For step-by-step setup instructions, expected outputs, parameter tuning, and troubleshooting, see [`Instruction to run.txt`](Instruction%20to%20run.txt).

---

## 🔬 Simulation Details

### Environment

- **Road:** 500 m × 500 m grid, single intersection at (250, 250)
- **Vehicles:** 4, approaching from East, West, North, South
- **Traffic signal:** 4-phase cycle — 30 s GREEN / 5 s YELLOW per direction
- **Duration:** 100 seconds, 0.1 s time step (1001 steps)

### V2V Link Model

Link quality between vehicles *i* and *j* is computed as:

```
Q(i,j) = max(0, 1 - distance(i,j) / 300)
```

Only links with Q ≥ 0.5 (vehicles within 150 m) are eligible for offloading.

### Task Types

| Type | Comp. Req. | Data Size | Deadline |
|---|---|---|---|
| PERCEPTION | 5–10 GHz | 20–30 MB | +1.0 s |
| PLANNING | 2–5 GHz | 5–10 MB | +1.5 s |
| CONTROL | 1–2 GHz | 1–2 MB | +0.5 s |

### Key Parameters

| Parameter | Value |
|---|---|
| Communication range | 300 m |
| Bandwidth | 50 MB/s |
| Task arrival rate | 0.5 tasks/s per vehicle |
| Vehicle speed | 10–15 m/s |
| Processing power | 10–30 GHz (randomised) |

---

## 📊 Output

### `intersectionmultipolicy.m`
Produces a grouped bar chart comparing **Completion Rate** and **Offloading Rate** across all five strategies.

### `intersection3.m`
Produces two subplots:
- Vehicle trajectory paths during the 100 s simulation
- Per-timestep task counts (Completed, Offloaded, Local, Failed)

And prints to the Command Window:
```
=== Final Simulation Results ===
Completed Tasks : X
Offloaded Tasks : X
Local Tasks     : X
Failed Tasks    : X
```

### `intersection.m`
Produces a 4-subplot figure with vehicle paths, cumulative task metrics, and smoothed efficiency rates over time.

---

## 🔁 Reproducibility

Results vary slightly between runs because `rand()` is used without a fixed seed. To reproduce the exact paper results, add the following line at the top of any script before running:

```matlab
rng(42);
```

---

## 📬 Contact

| Name | Email |
|---|---|
| Ghanatava Vashu Thakaran | ghanatva@gmail.com |
| Ayush Agrawal | ayush.agr254@gmail.com |
| Sarvagya Pradhan | sarvagyapradhan823@gmail.com |
| Kshitiz Agarwal | kshitizagarwal1710@gmail.com |
| Gaurav Parashar | gauravparashar24@gmail.com |

---

## 📜 License

This project was developed for academic research purposes as part of a B.Tech final year project at KIET Group of Institutions, Ghaziabad (Session 2025-26), affiliated to Dr. A.P.J. Abdul Kalam Technical University, Lucknow.