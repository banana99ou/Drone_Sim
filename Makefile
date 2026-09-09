# Convenience wrapper. Everything here is a thin alias -- see README.md.
COMPOSE := docker compose -f docker/compose.yaml
export UID := $(shell id -u)
export GID := $(shell id -g)

.PHONY: help setup build up down shell sim viz stop status test test-overlay \
        test-viewer verify assets fly telemetry plot clean

help:
	@echo "make setup    one-time host setup (sudo: docker group + nvidia toolkit)"
	@echo "make build    build the container image"
	@echo "make up       start the container in the background"
	@echo "make shell    open a shell inside it"
	@echo "make sim      build the workspace and launch the simulator"
	@echo "make test     every test suite: C++ invariants, overlay geometry, viewer"
	@echo "make verify   mutation check + generated-asset drift check (host, no docker)"
	@echo "make fly      headless flight check: proves it actually flies"
	@echo "make telemetry  cross-check a RUNNING sim's control telemetry"
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
#   dsim_control  the control maths and the mixer geometry           (C++)
#   dsim_viz      the overlay geometry the browser is handed         (Python)
#   web/js        the drawing maths and colour mapping in the page   (JS)
test: test-overlay test-viewer
	$(COMPOSE) exec sim bash -lc "colcon test --packages-select dsim_control && \
		colcon test-result --verbose"

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

# WORLD/REFERENCE/RADIUS are passed straight through, e.g.
#   make viz WORLD=empty REFERENCE=lemniscate RADIUS=2.0
viz:
	bash scripts/run_sim.sh

stop:
	bash scripts/stop_sim.sh

status:
	bash scripts/sim_status.sh

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

clean:
	$(COMPOSE) down -v
