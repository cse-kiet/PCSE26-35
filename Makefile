
# Makefile — V2V SUMO + MATLAB Simulation
# Run from project root: make <target>

SUMO_HOME    ?= /usr/share/sumo
PYTHON       := python3
MATLAB       := /home/ghanatava/matlab/bin/matlab -batch
MATLAB_VIS   := /home/ghanatava/matlab/bin/matlab -nosplash -nodesktop -r
SUMO_DIR     := src/sumo
FCD_CSV      := $(SUMO_DIR)/fcd_traces.csv
NET_XML      := $(SUMO_DIR)/intersection.net.xml
ROU_XML      := $(SUMO_DIR)/intersection.rou.xml
SEED         ?= 42
PERIOD       ?= 5

.PHONY: all sumo sumo-rebuild sumo-gui visualise matlab-compare matlab-trials matlab-stress clean help

## Default: generate SUMO traces
all: sumo

# ── Trace file rule ──────────────────────────────────────────────────────────

$(FCD_CSV):
	$(MAKE) sumo

# ── SUMO targets ─────────────────────────────────────────────────────────────

## Generate traces (skips steps already done)
sumo:
	SUMO_HOME=$(SUMO_HOME) $(PYTHON) $(SUMO_DIR)/generate_traces.py \
		--seed $(SEED) --period $(PERIOD)

## Force regenerate network, routes, and traces from scratch
sumo-rebuild:
	SUMO_HOME=$(SUMO_HOME) $(PYTHON) $(SUMO_DIR)/generate_traces.py \
		--seed $(SEED) --period $(PERIOD) --rebuild

## Full visual workflow: SUMO GUI + all MATLAB figures side by side
sumo-gui: $(FCD_CSV)
	@echo ""
	@echo "┌─────────────────────────────────────────────────────┐"
	@echo "│  Visual Workflow                                     │"
	@echo "│  • SUMO GUI launching in background                 │"
	@echo "│    → press the green Play button inside SUMO        │"
	@echo "│  • MATLAB figures will appear alongside             │"
	@echo "│    → close all figure windows when done             │"
	@echo "└─────────────────────────────────────────────────────┘"
	@echo ""
	SUMO_HOME=$(SUMO_HOME) sumo-gui -c $(SUMO_DIR)/intersection.sumocfg &
	$(MATLAB_VIS) "cd('$(CURDIR)'); visual_runner"

## Generate trajectory, heatmap, and speed-profile PNG plots from the CSV
visualise:
	$(PYTHON) $(SUMO_DIR)/visualise_traces.py

# ── MATLAB batch targets (no GUI) ────────────────────────────────────────────

## Run single-run five-policy comparison
matlab-compare:
	$(MATLAB) "intersectionmultipolicy_sumo"

## Run 30-trial statistical evaluation
matlab-trials:
	$(MATLAB) "run_trials"

## Run density + load stress test
matlab-stress:
	$(MATLAB) "stress_test"

# ── Housekeeping ──────────────────────────────────────────────────────────────

## Delete all generated SUMO outputs
clean:
	rm -f $(SUMO_DIR)/intersection.net.xml \
	       $(SUMO_DIR)/intersection.rou.xml \
	       $(SUMO_DIR)/fcd_traces.xml \
	       $(SUMO_DIR)/fcd_traces.csv \
	       $(SUMO_DIR)/*.trips.xml

## Print help
help:
	@echo ""
	@echo "Usage: make [target] [SEED=42] [PERIOD=5]"
	@echo ""
	@echo "Visual workflow (recommended entry point):"
	@echo "  make sumo-gui               SUMO GUI + all MATLAB figures together"
	@echo "                              (auto-generates traces if missing)"
	@echo ""
	@echo "SUMO targets:"
	@echo "  make sumo                   Generate traces (skips if already done)"
	@echo "  make sumo-rebuild           Force regenerate everything"
	@echo "  make sumo-rebuild SEED=123  Fresh run with a different seed"
	@echo "  make sumo-rebuild PERIOD=3  Denser traffic (~33 vehicles)"
	@echo "  make visualise              Render trajectory/heatmap/speed PNGs"
	@echo ""
	@echo "MATLAB batch targets (headless, no GUI):"
	@echo "  make matlab-compare         Single-run policy comparison"
	@echo "  make matlab-trials          30-trial statistical evaluation"
	@echo "  make matlab-stress          Density + load stress test"
	@echo ""
	@echo "Other:"
	@echo "  make clean                  Delete all generated SUMO files"
	@echo "  make help                   Show this message"
	@echo ""
	@echo "Note: make sumo-gui handles trace generation automatically."
	@echo "      For batch use, run 'make sumo' first."
	@echo ""