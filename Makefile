# Convenience wrapper. Everything here is a thin alias -- see README.md.
COMPOSE := docker compose -f docker/compose.yaml
export UID := $(shell id -u)
export GID := $(shell id -g)

.PHONY: help setup build up down shell sim viz stop status test verify fly plot clean

help:
	@echo "make setup    one-time host setup (sudo: docker group + nvidia toolkit)"
	@echo "make build    build the container image"
	@echo "make up       start the container in the background"
	@echo "make shell    open a shell inside it"
	@echo "make sim      build the workspace and launch the simulator"
	@echo "make test     run the invariant tests inside the container"
	@echo "make verify   run mutation check + world/referee drift check (host, no docker)"
	@echo "make fly      headless flight check: proves it actually flies"
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

test:
	$(COMPOSE) exec sim bash -lc "colcon test --packages-select dsim_control && \
		colcon test-result --verbose"

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
	python3 scripts/gen_worlds.py --check

clean:
	$(COMPOSE) down -v
