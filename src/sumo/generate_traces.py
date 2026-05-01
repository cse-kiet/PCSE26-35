#!/usr/bin/env python3
"""
generate_traces.py
Runs SUMO headless and converts FCD output to a CSV that MATLAB can read.

Usage:
    python3 generate_traces.py [--seed 42] [--period 5] [--rebuild]

Output:
    fcd_traces.csv  with columns: time, vehicle_id, x, y, speed, angle
"""

import subprocess
import sys
import os
import xml.etree.ElementTree as ET
import csv
import argparse
import shutil

SUMO_HOME = os.environ.get("SUMO_HOME", "")
if not SUMO_HOME:
    for candidate in ["/usr/share/sumo", "/usr/local/share/sumo",
                       "/opt/sumo", os.path.expanduser("~/sumo")]:
        if os.path.isdir(candidate):
            SUMO_HOME = candidate
            break

SUMO_BIN     = shutil.which("sumo") or os.path.join(SUMO_HOME, "bin", "sumo")
NETCONVERT   = shutil.which("netconvert") or os.path.join(SUMO_HOME, "bin", "netconvert")
RANDOM_TRIPS = os.path.join(SUMO_HOME, "tools", "randomTrips.py")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def build_network():
    """Compile .nod.xml + .edg.xml into .net.xml using netconvert."""
    print("Building SUMO network...")
    result = subprocess.run([
        NETCONVERT,
        "--node-files", os.path.join(SCRIPT_DIR, "intersection.nod.xml"),
        "--edge-files", os.path.join(SCRIPT_DIR, "intersection.edg.xml"),
        "--output-file", os.path.join(SCRIPT_DIR, "intersection.net.xml"),
        "--tls.default-type", "static",
        "--no-warnings",
    ], capture_output=True, text=True)
    if result.returncode != 0:
        print("netconvert error:", result.stderr)
        sys.exit(1)
    print("  Network built: intersection.net.xml")


def generate_routes(seed, period):
    """Generate random vehicle routes using randomTrips.py."""
    print("Generating vehicle routes...")
    result = subprocess.run([
        sys.executable, RANDOM_TRIPS,
        "-n", os.path.join(SCRIPT_DIR, "intersection.net.xml"),
        "-r", os.path.join(SCRIPT_DIR, "intersection.rou.xml"),
        "--end", "100",
        "--period", str(period),
        "--fringe-factor", "10",
        "--min-distance", "100",
        "--seed", str(seed),
    ], capture_output=True, text=True)
    if result.returncode != 0:
        print("randomTrips error:", result.stderr)
        sys.exit(1)
    print("  Routes generated: intersection.rou.xml")


def run_sumo(seed):
    """Run SUMO headless and generate FCD XML output."""
    print("Running SUMO simulation...")
    fcd_xml = os.path.join(SCRIPT_DIR, "fcd_traces.xml")
    result = subprocess.run([
        SUMO_BIN,
        "-c", os.path.join(SCRIPT_DIR, "intersection.sumocfg"),
        "--fcd-output", fcd_xml,
        "--seed", str(seed),
        "--no-step-log",
        "--no-warnings",
        "--duration-log.disable",
    ], capture_output=True, text=True)
    if result.returncode != 0:
        print("SUMO error:", result.stderr)
        sys.exit(1)
    print(f"  Simulation done: {fcd_xml}")
    return fcd_xml


def fcd_to_csv(fcd_xml, out_csv):
    """Parse SUMO FCD XML and write a clean CSV for MATLAB."""
    print("Converting FCD XML to CSV...")
    tree = ET.parse(fcd_xml)
    root = tree.getroot()

    vehicle_ids = {}
    next_id = 1

    rows = []
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

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time","vehicle_id","x","y","speed","angle"])
        writer.writeheader()
        writer.writerows(rows)

    n_vehicles = len(vehicle_ids)
    n_rows     = len(rows)
    print(f"  {n_vehicles} unique vehicles, {n_rows} rows -> {out_csv}")
    return n_vehicles


def main():
    parser = argparse.ArgumentParser(description="Generate SUMO mobility traces for V2V simulation")
    parser.add_argument("--seed",    type=int,   default=42, help="Random seed")
    parser.add_argument("--period",  type=float, default=5,  help="Vehicle injection period (s)")
    parser.add_argument("--rebuild", action="store_true",    help="Force rebuild network and routes")
    args = parser.parse_args()

    net_file = os.path.join(SCRIPT_DIR, "intersection.net.xml")
    rou_file = os.path.join(SCRIPT_DIR, "intersection.rou.xml")
    out_csv  = os.path.join(SCRIPT_DIR, "fcd_traces.csv")

    if args.rebuild or not os.path.exists(net_file):
        build_network()
    else:
        print("Network already built, skipping. Use --rebuild to regenerate.")

    if args.rebuild or not os.path.exists(rou_file):
        generate_routes(args.seed, args.period)
    else:
        print("Routes already generated, skipping. Use --rebuild to regenerate.")

    fcd_xml = run_sumo(args.seed)
    n_veh   = fcd_to_csv(fcd_xml, out_csv)

    print(f"\nDone. {n_veh} vehicles traced over 100 seconds.")
    print(f"Feed this into MATLAB: {out_csv}")


if __name__ == "__main__":
    main()