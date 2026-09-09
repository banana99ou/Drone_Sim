# Placeholder for your planner repo

This empty directory is the default mount target for `/ws/planner_src`, so that
the container starts cleanly when no planner is attached.

To develop your planner in here, point the env var at your own repo instead:

```bash
PLANNER_REPO=/mnt/windows/Users/JHY/code/my_planner \
  docker compose -f compose.yaml up -d
```

Your repo then appears at `/ws/planner_src` and `colcon build` picks it up.
The `COLCON_IGNORE` file next to this README is what keeps colcon from trying
to build the placeholder itself.
