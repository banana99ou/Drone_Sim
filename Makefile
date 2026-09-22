# Convenience wrapper. Everything here is a thin alias -- see README.md.
COMPOSE := docker compose -f docker/compose.yaml
export UID := $(shell id -u)
export GID := $(shell id -g)
# The space-time planner's working copy, mounted at /ws/planner_src. This is
# a SYNC of the MacBook's ~/code/bezier-trajectory-merge worktree (rsync, see
# `make planner-sync`), not a git checkout: the worktree's .git is a pointer
# into the MacBook's main repo and means nothing here.
export PLANNER_REPO ?= $(HOME)/code/bezier-trajectory-merge-sync
PLANNER_HOST ?= you@your-macbook.your-tailnet.ts.net
PLANNER_PATH ?= code/bezier-trajectory-merge

.PHONY: help setup build up down shell sim viz stop status test test-overlay \
        test-viewer test-planner verify assets fly plan planner solve telemetry \
        simctl sensors estimator plot clean planner planner-sync solve

help:
	@echo "make setup    one-time host setup (sudo: docker group + nvidia toolkit)"
	@echo "make build    build the container image"
	@echo "make up       start the container in the background"
	@echo "make shell    open a shell inside it"
	@echo "make sim      build the workspace and launch the simulator"
	@echo "make test     every test suite: C++ invariants, overlay geometry, viewer"
	@echo "make verify   mutation check + generated-asset drift check (host, no docker)"
	@echo "make fly      headless flight check: proves it actually flies"
	@echo "make plan     fly a space-time plan headless and grade it (PLAN=plans/...)"
	@echo "make planner-sync  pull the planner worktree from the MacBook"
	@echo "make planner  build the space-time optimiser's Rust extension"
	@echo "make solve    solve a scenario into a plan (SCENARIO= N= NSEG= VMAX=)"
	@echo "make planner-sync  rsync the planner worktree from the MacBook"
	@echo "make planner  build the planner's Rust solver inside the container"
	@echo "make solve    solve a scenario: SCENARIO=fence3d N=8 NSEG=2 [VMAX=..]"
	@echo "make telemetry  cross-check a RUNNING sim's control telemetry"
	@echo "make simctl     drive pause/speed/reset against a RUNNING sim"
	@echo "make sensors  measure a RUNNING sim's sensor noise vs its config"
	@echo "make estimator  compare a RUNNING sim's /drone/state_est against truth"
	@echo "make viz      start the sim + browser viewer, print the URL"
	@echo "make stop     stop the sim and the viewer"
	@echo "make status   is it up and publishing?"
	@echo "make plot F=logs/circle.csv   render a logged run to SVG"
	@echo "make down     stop the container"

setup:
	bash scripts/host_setup.sh

build:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d

down:
	$(COMPOSE) down

shell:
	$(COMPOSE) exec sim bash

sim:
	$(COMPOSE) exec sim bash -lc "colcon build --symlink-install && \
		source install/setup.bash && ros2 launch dsim_bringup sim.launch.py"

# Three suites, three languages, one command. They cover different things:
#   dsim_control     the control maths and the mixer geometry          (C++)
#   dsim_sensors     the rangefinder and optical-flow error models      (C++)
#   dsim_estimation  the attitude and velocity filters                  (C++)
#   dsim_eval        the referee's signed, moving-obstacle clearance     (C++)
#   dsim_planner     space-time Bezier -> trajectory conversion          (Python)
#   dsim_viz         the overlay geometry the browser is handed        (Python)
#   web/js           the drawing maths and colour mapping in the page  (JS)
test: test-overlay test-viewer test-planner
	$(COMPOSE) exec sim bash -lc \
		"colcon test --packages-select dsim_control dsim_sensors dsim_estimation dsim_eval && \
		colcon test-result --verbose && \
		cd src/dsim_planner && python3 -m pytest -q test/"

# The conversion is pure Python and runs on the host. The message-building
# test (test_bridge.py) needs dsim_msgs, so it is skipped here and run inside
# the container by `make test` above.
test-planner:
	cd src/dsim_planner && python3 -m pytest -q test/

# All overlay LOGIC lives on the ROS side, so this is where an arrow pointing
# the wrong way gets caught -- before it is ever drawn.
test-overlay:
	cd src/dsim_viz && python3 -m pytest -q test/

# Only what the page is allowed to do: projection maths and colour choices.
test-viewer:
	cd web && node --test js/test/viewer.test.js

fly:
	$(COMPOSE) exec sim bash -lc "colcon build --symlink-install >/dev/null && \
		bash scripts/flight_check.sh"

# Flies PLAN (default: the straight seed through the fence, which must HIT)
# headless and grades it with scripts/check_planner.py.
#   make plan PLAN=plans/fence3d_seed.json
plan:
	$(COMPOSE) exec -e PLAN="$(PLAN)" sim bash -lc "colcon build --symlink-install >/dev/null && \
		bash scripts/plan_check.sh"

# The planner. `planner-sync` copies the MacBook worktree (minus its build
# products, caches and papers) to $(PLANNER_REPO); `planner` builds its Rust
# extension against the container's Python and checks the import; `solve`
# runs it on a scenario and writes plans/<scenario>_N<N>_seg<NSEG>.json.
planner-sync:
	rsync -az --delete --stats \
	  --exclude .git --exclude .venv --exclude .claude --exclude .cursor --exclude .specstory \
	  --exclude rust_optimizer/target --exclude __pycache__ --exclude .pytest_cache \
	  --exclude artifacts --exclude figures --exclude doc --exclude paper --exclude cache --exclude tmp \
	  "$(PLANNER_HOST):$(PLANNER_PATH)/" "$(PLANNER_REPO)/" | grep -E "transferred|speedup"

planner:
	$(COMPOSE) exec sim bash -lc "bash scripts/build_planner.sh"

#   make solve SCENARIO=fence3d N=8 NSEG=2 VMAX=3.0
#   make solve SCENARIO=fence3d SOLVE_ARGS=--sweep
# VMAX matters more than it looks. The optimiser has no acceleration cap at
# all, and its only physical bound is this speed cap: an uncapped fence3d
# solve returns a curve demanding 15 m/s, 83 m/s^2 and 109 degrees of tilt,
# against a 40 degree clamp. Measured, and printed by solve_plan.py before the
# plan is written.
SCENARIO ?= fence3d
N ?= 8
NSEG ?= 2
solve:
	$(COMPOSE) exec sim bash -lc "python3 scripts/solve_plan.py --scenario $(SCENARIO) -N $(N) --n-seg $(NSEG) $(if $(VMAX),--v-max $(VMAX),) $(SOLVE_ARGS)"

# WORLD/REFERENCE/RADIUS are passed straight through, e.g.
#   make viz WORLD=fence3d PLAN=plans/fence3d_seed.json
#   make viz WORLD=empty REFERENCE=lemniscate RADIUS=2.0
viz:
	bash scripts/run_sim.sh

stop:
	bash scripts/stop_sim.sh

status:
	bash scripts/sim_status.sh

PLAN ?= plans/fence3d_seed.json
F ?= logs/circle.csv
plot:
	python3 scripts/plot_run.py $(F)

# Runs on the host: the maths has no ROS dependency, so it needs no container.
verify:
	bash scripts/mutation_check.sh
	python3 scripts/gen_assets.py --check

# Regenerate the SDF, configs, worlds and scene from scripts/gen_assets.py.
assets:
	python3 scripts/gen_assets.py

# Needs a sim already running (make viz). Compares numbers produced by
# different paths, so agreement is evidence rather than self-report.
telemetry:
	python3 scripts/check_telemetry.py

# Needs a sim already running WITH controls (make viz, which enables them).
# Drives pause, speed and reset against the live world and checks the vehicle
# still weighs anything afterwards -- the one thing fifty unit tests missed.
simctl:
	python3 scripts/check_simcontrol.py

# Measures the live sensor noise against config/drone.yaml. Needs a HOVERING
# sim: on a still vehicle the spread around zero rate and one g is the noise,
# where on a moving one the real motion would swamp it.
#   make viz REFERENCE=hover WORLD=empty
sensors:
	$(COMPOSE) exec sim bash -lc "cd /ws && python3 scripts/check_sensors.py"

# Compares /drone/state_est and /drone/estimator_debug against /drone/truth
# on a RUNNING sim, over ~20 s. Works in hover and in a turn, and reports
# different things in each: the hover bounds are the sensor-noise floor, the
# turn adds the banked-turn attitude error the filter is known to have. It
# also checks the estimate is NOT a copy of truth: the errors must be
# non-zero and must track the sensor noise.
#   make viz REFERENCE=hover WORLD=empty      (or the default circle)
estimator:
	$(COMPOSE) exec sim bash -lc "cd /ws && python3 scripts/check_estimator.py"

clean:
	$(COMPOSE) down -v
