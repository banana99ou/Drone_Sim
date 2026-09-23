# Flying a space-time Bezier plan

How a plan from [banana99ou/bezier-trajectory](https://github.com/banana99ou/bezier-trajectory/tree/integrate/rust-into-spacetime)
(`spacetime_bezier`) gets into this simulator, what the referee does with the
scenario it was solved in, and what a run proves.

```
the planner's SCENARIO_MAP
         │  scripts/import_scenarios.py          (--check catches drift)
         ▼
scenarios/<name>.json  ──────────────┬──> worlds/<name>.sdf             (ground, light, the vehicle at `start`)
  obstacles as lifted control         ├──> config/obstacles_<name>.yaml  (the referee's obstacle field)
  points in (x, y, z, t), the         ├──> web/scene.json                (colours and the camera's aim)
  active window intrinsic to          └──> plans/<name>_seed.json        (the straight seed, which mostly HITS)
  the first and last of them                          │
                                                      │
plans/<name>.json  (x, y, z, t) control points ───────┤
                                                      ▼
                          dsim_planner/bridge_node ── samples uniform in TIME,
                          v = p'/t',  a = (p'' t' - p' t'')/t'^3 ──> /drone/trajectory
                                                      │
                                    SE(3) controller (unchanged) ──> the vehicle
                                                      │
                    dsim_eval: signed clearance along each obstacle's Bezier,
                    inside its window, on the scenario clock ──> /drone/eval/clearance
                                                      │
                                    the browser draws THAT ──> what you see
```

Nothing is transcribed anywhere on that path. `import_scenarios.py --check`
fails if the planner's scenarios have moved, and `gen_assets.py --check` fails
if anything downstream has.

## The seven scenarios

```bash
python3 scripts/solve_plan.py --list     # inside the container
```

| | | |
|---|---|---|
| `original` | 2D+t | three moving obstacles |
| `curve` | 2D+t | one obstacle on a **cubic** path; the only non-convex tube |
| `diverse` | 2D+t | seven movers |
| `wall` | 2D+t | a wall that **stands until t=5 and is then gone** |
| `fence3d` | 3D+t | a moving fence; the demonstrated behaviour is the climb |
| `door3d` | 3D+t | a doorway in time; the demonstrated behaviour is **waiting** |
| `loiter` | 3D+t | 200 m scale, T=80 s, four orbiting bodies **and a station**; needs `state:=truth`, see below |

Four are planned in **two** spatial dimensions plus time. They are solved that
way and the resulting curve is flown at 1.5 m; lifting them into 3D would hand
the solver an escape over the top that the 2D problem never had. Their
obstacles are therefore **columns** -- a disc at every altitude -- and not
balls at the flight altitude.

## Run it

```bash
make viz WORLD=fence3d PLAN=plans/fence3d_seed.json     # watch it in the browser
make plan PLAN=plans/fence3d_seed.json                  # headless, graded
```

The seed plan is the straight line from `start` to `end` -- the planner's own
initial guess -- and on six of the seven scenarios it flies straight through
something. Its file says what and when (`expect.first_hit`), and `make plan`
checks the referee agrees to within 0.3 s and 10 cm. A referee that lets the
seed through is broken, and this is how you would find out.

## The obstacles are not in the physics

They never were -- they have no collision geometry, because the referee scores
clearance analytically and a contact solver would only add a second,
differently shaped opinion. They are now not Gazebo bodies at all, because
nothing a rigid body can do expresses what they are:

* two scenarios move theirs along **cubics** (`curve`, `loiter`), not straight
  lines;
* three switch obstacles **off partway through** (`wall` at t=5, `door3d` at
  t=5, `loiter`'s four bodies in sequence). Outside its window an obstacle
  does not exist. It is not far away and it is not parked at an endpoint: the
  planner solves a problem in which the thing is absent, and waiting for it to
  go is the behaviour `wall` and `door3d` demonstrate.

So `/drone/eval/clearance` **is** the obstacle field, and the browser draws
that. There is one account of where anything is rather than two that can drift.

An earlier version did put constant-velocity spheres in the world and checked
Gazebo's poses against the referee's arithmetic. That check read 0.00 mm, but
what it compared was the simulator against its own staging. The replacement is
stronger and lives in `check_planner.py`: the referee's live obstacle field
against the **planner's own** `obstacle_positions_at()` -- two implementations
of the same Bezier motion, in two languages, sampled 41 times across the
window, including whether each obstacle is active at that instant. Measured
agreement is exactly 0.0, against a 1e-6 bound.

## The scenario clock

The planner's t = 0 is when the plan starts executing. `sim.start_s` (5 s) is
when that happens in simulated seconds; the vehicle **spawns at the plan's
first point** and holds there until then, and the bridge stamps the trajectory
with `start_s` so the plan begins on the tick.

There is no transit from a pad. It is not part of any plan, nothing certifies
it, and the version that flew one collected five collisions before the run had
begun -- the fence swept through the pad while the vehicle sat on it. (`loiter`
starts 62.5 m up; a climb to that would be a minute of nothing.)

After the horizon `T` the controller holds the final point. Every obstacle's
window ends at or before `T`, so past it the field is empty.

## The conversion (why the bridge exists)

The optimiser's curve is a Bezier in (x, y, z, t) with parameter tau. Time is
a coordinate of the curve, so dp/dtau is not a velocity. The bridge applies
the chain rule with tau eliminated,

    v = p' / t'                     a = (p'' t' - p' t'') / t'^3

and samples uniform in **time** by inverting t(tau) with bisection. Both
choices are tested against finite differences in time on a deliberately
clustered curve (`src/dsim_planner/test/test_spacetime.py`), which is the one
comparison an algebra slip cannot pass; `scripts/mutation_check.sh` injects
the slips -- dp/dtau as velocity, the t'' term dropped, tau-spaced samples --
and confirms each is caught.

The inversion is safe because the optimiser's hard row `t[i+1] - t[i] >=
min_dt` on the control points makes every derivative control point's time
component positive, and a convex combination of positives is positive: dt/dtau
> 0 on the whole curve, not just at the knots. The bridge checks the row
rather than trusting it.

## What the referee measures

Clearance is **signed** and counts the vehicle radius (0.3 m from
`config/drone.yaml`): `|p - centre(t)| - r - 0.3`, with `centre(t)` from de
Casteljau along the obstacle's control points and `+infinity` outside its
window. Negative is a collision.
Each excursion below zero is one hit, recorded with the vehicle position at
entry, the obstacle, the scenario time and the deepest penetration, and drawn
in the viewer as a red cross at that spot. Every run prints its hits.

This is the same quantity the planner certifies (`compute_min_clearance`),
except that the planner solves for a point and the referee for a vehicle. A
plan certified at +0.16 m for a point is 14 cm INTO the fence for this
vehicle. Either inflate every obstacle by 0.3 m before solving or expect the
referee to say so; the first is the planner's call, the second is a finding
about the plan, not a bug in either.

## Solving, in the container

The optimiser is a Rust extension (`bezier_opt`: pyo3 around clarabel) and the
planner has **no Python fallback** -- `optimize_spacetime()` raises without it.
The toolchain is in the image; the planner's source is a MacBook worktree
rsynced in:

```bash
make planner-sync    # rsync the MacBook worktree -> $PLANNER_REPO (/ws/planner_src)
make up              # (re)create the container so the mount takes
make planner         # build bezier_opt, install into /ws/install/planner_py, import it
make solve SCENARIO=fence3d N=8 NSEG=2 VMAX=3.0
make plan  PLAN=plans/fence3d_N8_seg2.json
```

`make planner` ends by importing the module **from the path it installed to**
and printing where it came from: maturin will build against a different
interpreter and report success, and the planner's own README records being
burned by exactly that. A fence3d solve takes **0.20 s**, a four-config sweep
1.4 s, so replanning in the loop is not off the table.

`scripts/solve_plan.py` hands the optimiser the SIM's `scenarios/<name>.json`,
which `import_scenarios.py` generated from the planner's own `SCENARIO_MAP`.
One source, so the obstacles the referee scores against and the problem the
solver reads are the same numbers. For a 2D scenario it drops the z column
before solving and puts it back afterwards, which is exact because the
altitude is a constant.

It also **inflates every obstacle by the vehicle radius** before solving, and
then states what the referee should measure. Without that the numbers do not
line up: fence3d N8_seg2 solved for a point certifies +0.1623 m, which is
**-0.1377 m** for a 0.3 m vehicle -- a plan the optimiser certifies and the
referee scores as a collision, with neither of them wrong. Inflating is exact
for a sphere. Solved that way it certifies +0.2209 m; the live run measured
+0.2444 m, and `check_planner.py` holds the two together.

### Two things that make a plan unflyable

The optimiser constrains **speed** (`--v-max`, a hard second-order cone on the
control-point differences) and nothing else. In particular:

1. **No acceleration cap.** Uncapped, fence3d N8_seg2 returns a curve
   demanding 15.05 m/s, 82.61 m/s² and **108.6° of tilt** against a 40° clamp:
   the solver loiters and then darts, because nothing costs it. A speed cap
   bounds it only indirectly and not reliably -- N10_seg8 at `--v-max 2.0`
   still demands 40 m/s² and 89°. `solve_plan.py` samples the plan with the
   same converter the bridge uses and prints the verdict before writing the
   file; `check_planner.py` refuses to grade an unflyable plan, because
   tracking error there measures the clamp rather than the plan.
2. **No boundary velocity.** Endpoint *positions* are pinned, velocities are
   not, so a plan opens at the speed cap from a standstill and ends still
   moving -- it is a segment of a flight, not a flight. The vehicle hovers at
   the start point and is handed a step: measured **92 cm** of opening error
   on a 3.00 m/s start, decaying to under 10 cm within 3 s. The catch-up
   distance `v0²/(2·a_max)` is 54 cm at full authority, so most of it is
   unavoidable given the formulation. Fixing it properly is a planner-side
   row (pin the first and last derivative control points); the sim could
   instead fly a lead-in, which is a different claim about what was tested.

### From the viewer

With a plan loaded, the control panel grows a **plan** box: degree N, segments,
v max, and a solve button. It POSTs to `/solve` -- a second write path, kept
separate from `/control` because `/control` is documented as unable to run a
command and solving runs the optimiser. The scenario and the output path are
**not** request fields (the Gazebo world is generated per scenario, so solving
another one would plan against obstacles that are not there), the three numbers
are range-checked, and the subprocess takes an argument list, never a shell.
The bridge notices the new file by mtime and reloads it, so a solve reaches the
flying vehicle in about a second with nothing restarted. A plan that does not
parse is refused and the old one keeps flying.

The **scenario** dropdown above it is a different hammer: it restarts the
simulator with whichever run you pick, in about four seconds, and the page
stays up throughout. Every scenario that some plan declares is offered, plus
the planner-free `hover` and `circle`; the server will only launch a name from
the list the page built its dropdown from. A solved plan is preferred over the
straight seed, because the seed exists to be flown *into* an obstacle. See
[VIEWER.md](VIEWER.md#the-scenario-dropdown).

## The opening transient, measured

Every plan opens with a velocity STEP, because the optimiser pins endpoint
positions and not velocities. The peak tracking error that step causes is
proportional to the step, and the constant is a property of the control loop
rather than of any plan:

| plan | start speed | peak error | ratio |
|---|---|---|---|
| `wall` seed | 0.70 m/s | 0.237 m | 0.339 s |
| `fence3d` seed | 0.90 | 0.290 | 0.322 |
| `loiter` N8_seg16 | 1.60 | 0.522 | 0.326 |
| `diverse` N8_seg4 | 1.68 | 0.526 | 0.313 |
| `fence3d` N8_seg2 | 3.00 | 0.919 | 0.306 |
| `curve` N10_seg8 | 3.00 | 1.011 | 0.337 |

Least squares through the origin: **0.322 s**, residuals under 5 cm over a
4.3x range. So the gate allows `0.5 s x v0` with a 30 cm floor, and a
feedforward that stopped arriving would move that coefficient enough to fail.

The first version used the full-authority catch-up distance `v0^2/(2 a_max)`.
That is quadratic, and the loop is not acceleration-limited here but
bandwidth-limited: it under-predicted by 3x at the low end, passing `fence3d`
and failing `diverse` and `loiter` for reasons to do with neither plan.

## What the reset path taught

Resetting the world mid-plan (the viewer's reset button) exposed a controller
bug that no built-in reference could: with the plan dropped and no post-reset
state yet received, the position hold latched onto the PRE-reset state -- the
goal, nine metres away -- and commanded a 40° tilt from the pad for as long as
it took the bridge to republish. Measured before the fix: a 1.6 m lunge and
two hits before scenario t = 0. The controller now forgets its state on a
reset and idles until a fresh one arrives; three consecutive resets measured
under 2.1° of commanded tilt and 1 cm of travel in the first 2.5 s.

## What a run does not prove

* Nothing senses the obstacles. There is no lidar or depth in the loop and no
  avoidance; the trajectory is fixed before it is flown, and the obstacles are
  not in the physics, so nothing could sense them if it tried. The referee
  knows them because it was told, and a plan that clears them does so because
  the planner was told the same thing.
* The opening transient is the plan's standing start, not the loop: 29 cm for
  the 0.9 m/s seed, 92 cm for a 3.0 m/s optimised plan. The cruise figure --
  the last 40% of the plan, allowed 50 ms of lag at the plan's top speed -- is
  the one that says anything about tracking.
* Nothing checks line of sight. `loiter` carries `stations` and the optimiser
  can add occlusion rows for them, but the referee has no LOS measure at all,
  so an occlusion-constrained plan is certified by the solver and **unchecked**
  here.
* The estimator is a LOW-ALTITUDE one. Its only velocity aiding is optical
  flow, which needs the ground between 0.1 and 3 m, plus a rangefinder good to
  4 m. `loiter` flies at 62.5 m: measured 48 m of tracking error on
  `state:=est` and 2.2 cm on `state:=truth`, same plan. The gate says so
  before it reports the failures. That is a real statement about the vehicle,
  not a simulator artefact -- a machine with these sensors cannot hold station
  at 60 m either.
* The lateral acceleration limit is ~8.3 m/s² at the 40° tilt clamp. That is
  the number the planner is missing a constraint for, and every plan here is
  kept under it by a speed cap chosen by hand rather than by anything in the
  formulation.
