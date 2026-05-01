# 04 — Trace Generation (`generate_traces.py`)

## Purpose

`generate_traces.py` is the bridge between SUMO and MATLAB. It:
1. Builds the road network (calls `netconvert`)
2. Generates vehicle routes (calls `randomTrips.py`)
3. Runs SUMO headless (calls `sumo`)
4. Converts the FCD XML output into a clean CSV that MATLAB can read with `readtable`

---

## How to run it

```bash
# Basic run (uses defaults: seed=42, period=5)
python3 src/sumo/generate_traces.py

# Or via make (same thing)
make sumo

# Force rebuild everything from scratch
python3 src/sumo/generate_traces.py --rebuild
make sumo-rebuild

# Custom seed (different vehicle patterns)
python3 src/sumo/generate_traces.py --rebuild --seed 123
make sumo-rebuild SEED=123

# Denser traffic (one vehicle every 3 s instead of 5 s)
python3 src/sumo/generate_traces.py --rebuild --period 3
make sumo-rebuild PERIOD=3
```

---

## What each step does

### Step 1 — `build_network()`

```python
subprocess.run([
    NETCONVERT,
    "--node-files", "intersection.nod.xml",
    "--edge-files", "intersection.edg.xml",
    "--output-file", "intersection.net.xml",
    "--tls.default-type", "static",
    "--no-warnings",
])
```

`netconvert` reads the hand-written node and edge XML files and compiles them into
`intersection.net.xml`. This step is skipped if `intersection.net.xml` already exists
(unless `--rebuild` is passed).

### Step 2 — `generate_routes()`

```python
subprocess.run([
    sys.executable, RANDOM_TRIPS,
    "-n", "intersection.net.xml",
    "-r", "intersection.rou.xml",
    "--end", "100",
    "--period", str(period),
    "--fringe-factor", "10",
    "--min-distance", "100",
    "--seed", str(seed),
])
```

`randomTrips.py` reads the compiled network and generates `intersection.rou.xml` —
a list of vehicle departure times and routes. Skipped if the file already exists.

### Step 3 — `run_sumo()`

```python
subprocess.run([
    SUMO_BIN,
    "-c", "intersection.sumocfg",
    "--fcd-output", "fcd_traces.xml",
    "--seed", str(seed),
    "--no-step-log",
    "--no-warnings",
    "--duration-log.disable",
])
```

Runs SUMO in headless mode (no GUI). SUMO reads the network and routes, simulates
100 seconds of traffic, and writes `fcd_traces.xml` — the Floating Car Data output.

`--no-step-log` and `--duration-log.disable` suppress console spam.
`--seed` controls SUMO's internal randomness (lane changes, driver behaviour).

This step runs every time — it always produces a fresh `fcd_traces.xml`.

### Step 4 — `fcd_to_csv()`

```python
tree = ET.parse(fcd_xml)
root = tree.getroot()

vehicle_ids = {}   # maps SUMO string IDs ("trip_0") to integers (1, 2, 3...)
next_id = 1

for timestep in root.findall("timestep"):
    t = float(timestep.get("time"))
    for veh in timestep.findall("vehicle"):
        vid_str = veh.get("id")
        if vid_str not in vehicle_ids:
            vehicle_ids[vid_str] = next_id
            next_id += 1
        rows.append({
            "time":       t,
            "vehicle_id": vehicle_ids[vid_str],
            "x":          float(veh.get("x")),
            "y":          float(veh.get("y")),
            "speed":      float(veh.get("speed")),
            "angle":      float(veh.get("angle")),
        })
```

SUMO's FCD XML uses string vehicle IDs like `"trip_0"`, `"trip_1"`. MATLAB's
`readtable` handles string columns awkwardly. This step converts them to sequential
integers (1, 2, 3...) so MATLAB can use them as numeric indices.

---

## The FCD XML format

SUMO's raw output looks like this:

```xml
<fcd-export>
  <timestep time="0.00">
    <vehicle id="trip_3" x="0.00" y="299.80" speed="0.00" angle="180.00" .../>
  </timestep>
  <timestep time="0.10">
    <vehicle id="trip_3" x="0.00" y="298.51" speed="12.60" angle="180.00" .../>
    <vehicle id="trip_7" x="298.30" y="0.00"  speed="13.89" angle="270.00" .../>
  </timestep>
  ...
</fcd-export>
```

Key properties:
- A vehicle only appears in a timestep if it is currently active in the simulation
- Vehicles enter the simulation at their departure time and exit when they reach
  their destination
- `x`, `y` are in metres in SUMO's coordinate space
- `speed` is in m/s
- `angle` is in degrees, 0 = north, 90 = east, 180 = south, 270 = west

---

## The output CSV format

```
time,vehicle_id,x,y,speed,angle
0.0,1,0.0,299.8,0.0,180.0
0.1,1,0.0,298.51,12.6,180.0
0.1,2,298.3,0.0,13.89,270.0
...
```

Each row represents one vehicle at one timestep. A vehicle with ID 3 that was active
for 40 seconds at 10 Hz produces 400 rows.

**Why long format instead of wide?** MATLAB's `readtable` handles long-format CSVs
efficiently. `parse_fcd.m` groups rows by vehicle ID to reconstruct each vehicle's
trajectory.

---

## SUMO_HOME detection

The script needs to know where SUMO's Python tools are installed:

```python
SUMO_HOME = os.environ.get("SUMO_HOME", "")
if not SUMO_HOME:
    for candidate in ["/usr/share/sumo", "/usr/local/share/sumo", ...]:
        if os.path.isdir(candidate):
            SUMO_HOME = candidate
            break
```

On Ubuntu with the `sumo-tools` package, `SUMO_HOME` is `/usr/share/sumo` and
`randomTrips.py` lives at `/usr/share/sumo/tools/randomTrips.py`.

The Makefile hardcodes `SUMO_HOME=/usr/share/sumo` to avoid the detection logic
failing when `$SUMO_HOME` is not set in the environment. If your SUMO is installed
elsewhere, override it:

```bash
make sumo SUMO_HOME=/path/to/your/sumo
```
