# 03 — SUMO Network

## What is a SUMO network?

A SUMO network describes the physical road geometry: where intersections are,
how roads connect them, how many lanes each road has, and what speed limits apply.
It is compiled from two hand-written XML files into a single binary-format `.net.xml`
file by the `netconvert` tool.

---

## The node file — `intersection.nod.xml`

Nodes are points in 2D space — intersections, endpoints, junctions.

```xml
<nodes>
  <node id="centre" x="0"    y="0"    type="traffic_light"/>
  <node id="west"   x="-300" y="0"    type="priority"/>
  <node id="east"   x="300"  y="0"    type="priority"/>
  <node id="south"  x="0"    y="-300" type="priority"/>
  <node id="north"  x="0"    y="300"  type="priority"/>
</nodes>
```

**Why 5 nodes?**
A 4-arm intersection has one centre point and four road endpoints — one at each
compass direction. The centre is the only node that needs a traffic light. The four
arm endpoints are just road terminals where vehicles enter and exit.

**Coordinate system:** SUMO uses metres. The centre is at (0,0). Each arm is 300 m
long, matching the 300 m communication range of IEEE 802.11p DSRC. This means two
vehicles at opposite ends of the same arm are just within communication range.

**`type="traffic_light"`** tells SUMO to auto-generate a traffic signal programme
for the centre node. `type="priority"` means vehicles at the arm endpoints yield
to traffic already in the intersection — standard T-junction behaviour.

---

## The edge file — `intersection.edg.xml`

Edges are directed road segments connecting two nodes.

```xml
<edges>
  <edge id="we" from="west"   to="centre" numLanes="2" speed="13.89"/>
  <edge id="ew" from="centre" to="east"   numLanes="2" speed="13.89"/>
  <edge id="eo" from="east"   to="centre" numLanes="2" speed="13.89"/>
  <edge id="oe" from="centre" to="west"   numLanes="2" speed="13.89"/>
  <edge id="sn" from="south"  to="centre" numLanes="2" speed="13.89"/>
  <edge id="ns" from="centre" to="north"  numLanes="2" speed="13.89"/>
  <edge id="no" from="north"  to="centre" numLanes="2" speed="13.89"/>
  <edge id="on" from="centre" to="south"  numLanes="2" speed="13.89"/>
</edges>
```

**Why 8 edges for 4 arms?**
SUMO edges are one-directional. A two-way road requires two edges — one in each
direction. Each arm has 2 directions = 8 edges total.

**Edge naming convention:** `we` = West to East (centre), `ew` = East (centre) to West,
`eo` = East to centre, etc. The two-letter codes indicate the direction of travel.

**`numLanes="2"`** — two lanes per direction, standard for a urban arterial road.
More lanes allow vehicles to travel side by side and affects SUMO's internal
lane-change model.

**`speed="13.89"` m/s** — converts to exactly 50 km/h, the standard urban speed limit
in India. SUMO enforces this as the maximum speed; vehicles will slow down for traffic
signals and leading vehicles but will not exceed this.

---

## Compiling the network

The two XML files are compiled by `netconvert`:

```bash
netconvert \
  --node-files intersection.nod.xml \
  --edge-files intersection.edg.xml \
  --output-file intersection.net.xml \
  --tls.default-type static
```

`--tls.default-type static` tells `netconvert` to generate a fixed-phase traffic
signal rather than an actuated (sensor-triggered) one. Static signals are predictable
and reproducible — important for scientific experiments.

The resulting `intersection.net.xml` is a compiled binary-format file. You do not
need to read or edit it manually.

---

## Vehicle routes — `intersection.rou.xml`

Routes define where vehicles start, where they go, and when they depart. Rather than
hand-writing routes for 20 vehicles, we use SUMO's `randomTrips.py` tool:

```bash
python3 $SUMO_HOME/tools/randomTrips.py \
  -n intersection.net.xml \
  -r intersection.rou.xml \
  --end 100 \
  --period 5 \
  --fringe-factor 10 \
  --min-distance 100 \
  --seed 42
```

**`--end 100`** — generate trips over a 100 second window, matching the simulation duration.

**`--period 5`** — on average, one new vehicle is injected every 5 seconds.
Over 100 seconds this gives approximately 20 vehicles. Use `--period 3` for ~33 vehicles.

**`--fringe-factor 10`** — weights route origins and destinations toward the network
boundary (the four arm endpoints). This is appropriate for an intersection scenario
where vehicles approach from outside and exit to outside.

**`--min-distance 100`** — only generate trips where the vehicle must travel at least
100 m. This prevents SUMO from generating trivial zero-distance routes.

**`--seed 42`** — fixes the random seed so the same routes are generated every time.
Change this with `make sumo-rebuild SEED=123` to get different vehicle patterns.

---

## The run configuration — `intersection.sumocfg`

```xml
<configuration>
  <input>
    <net-file value="intersection.net.xml"/>
    <route-files value="intersection.rou.xml"/>
  </input>
  <time>
    <begin value="0"/>
    <end value="100"/>
    <step-length value="0.1"/>
  </time>
  <output>
    <fcd-output value="fcd_traces.xml"/>
  </output>
</configuration>
```

**`step-length value="0.1"`** — SUMO advances the simulation in 0.1 second steps.
This matches the MATLAB simulation step size, so vehicle positions in the CSV align
exactly with MATLAB's time vector.

**`fcd-output`** — enables Floating Car Data output. SUMO writes every vehicle's
position, speed, and heading at every timestep into `fcd_traces.xml`.

---

## What the network looks like

```
        north (0, 300)
           |
           | ← edge "no"  (north to centre)
           | → edge "ns"  (centre to north)
           |
(-300,0) --+-- (300,0)
  west     |    east
  ← "oe"   |    → "ew"
  → "we"   |    ← "eo"
           |
           | ← edge "on"  (centre to south)
           | → edge "sn"  (south to centre)
           |
        south (0, -300)
```

All four arms are 300 m long. The centre node is the signalised intersection.
