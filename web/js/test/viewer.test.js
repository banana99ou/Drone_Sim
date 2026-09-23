// Tests for the pure parts of the viewer, run with:  node --test web/js/test
//
// These cover only what the browser is allowed to do: maths for DRAWING, and
// colour choices. Anything physical is tested on the ROS side, in
// src/dsim_viz/test/test_overlay.py -- that split is the point.
//
// Each test says what would make it fail.

import test from "node:test";
import assert from "node:assert/strict";

import { quatMat, rotate, unit, bodyX, bodyY, bodyZ, cross, len } from "../vec3.js";
import { shadeHex, droneFaces, sightLines, stationFaces } from "../shapes.js";
import { rotorColour, colourFor, widthFor, PALETTE, LEGEND } from "../palette.js";
import { extendTrail, TRAIL_MAX } from "../trail.js";
import { gridFor, homeFor } from "../framing.js";

const YAW90 = [Math.cos(Math.PI / 4), 0, 0, Math.sin(Math.PI / 4)];

// FAILS IF: quatMat stops producing a rotation. Two properties that cannot
// both survive a wrong matrix: orthonormal rows, and a right-handed frame
// (bodyX x bodyY == bodyZ, not its negation, which would mirror the vehicle).
test("quatMat produces an orthonormal right-handed frame", () => {
  for (const q of [[1, 0, 0, 0], YAW90, [Math.cos(0.4), Math.sin(0.4) * 0.6, Math.sin(0.4) * 0.8, 0]]) {
    const M = quatMat(q);
    for (const axis of [bodyX(M), bodyY(M), bodyZ(M)]) {
      assert.ok(Math.abs(len(axis) - 1) < 1e-12, "axis is not unit length");
    }
    const c = cross(bodyX(M), bodyY(M));
    const z = bodyZ(M);
    for (let i = 0; i < 3; i++) {
      assert.ok(Math.abs(c[i] - z[i]) < 1e-12, "frame is left-handed");
    }
  }
});

// FAILS IF: rotate() and the body-axis accessors disagree. bodyZ is documented
// as the third COLUMN of the matrix, which must equal rotating [0,0,1].
test("bodyZ equals rotating the body z axis", () => {
  const M = quatMat([Math.cos(0.3), Math.sin(0.3), 0, 0]);
  const r = rotate(M, [0, 0, 1]);
  bodyZ(M).forEach((v, i) => assert.ok(Math.abs(v - r[i]) < 1e-12));
});

// FAILS IF: normalising a zero vector yields NaN. Overlay geometry legitimately
// contains zero-length vectors, and one NaN endpoint silently blanks a whole
// projected polygon rather than erroring.
test("unit of a zero vector is zero, not NaN", () => {
  assert.deepEqual(unit([0, 0, 0]), [0, 0, 0]);
});

// FAILS IF: shadeHex overflows past white. k > 1 is used to brighten the top
// faces, and an unclamped channel wraps into a wrong colour.
test("shadeHex clamps instead of wrapping", () => {
  assert.equal(shadeHex("#ffffff", 2), "rgb(255,255,255)");
  assert.equal(shadeHex("#804020", 0), "rgb(0,0,0)");
});

// FAILS IF: the vehicle is drawn without a pose. Rendering the airframe at the
// origin before the first odometry message would put a drone on screen that
// the simulator has never reported.
test("droneFaces needs a pose", () => {
  assert.deepEqual(droneFaces(null, { body: [1, 1, 1], rotors: [] }), []);
});

// FAILS IF: the drawn rotors stop coming from scene.json. The rotor list is
// generated from the same rotor_layout() that writes model.sdf; recomputing it
// from the arm length here would be a second definition of the airframe.
test("droneFaces draws one disc and one arm per rotor in the scene", () => {
  const drone = { body: [0.16, 0.16, 0.08], rotor_radius: 0.09,
                  rotors: [[0.1, 0.1, 0.02], [-0.1, -0.1, 0.02]] };
  const faces = droneFaces({ p: [0, 0, 1], q: [1, 0, 0, 0] }, drone);
  // 6 body quads + 2 rotors * (1 disc + 1 arm)
  assert.equal(faces.length, 6 + 2 * 2);
});

// FAILS IF: a yaw does not carry the drawn rotors with it. A rotor at body
// (+x, 0) must appear at world (0, +x) after 90 degrees of yaw; if it did not,
// the airframe would stay axis-aligned while the drone turned.
test("droneFaces rotates rotor discs with the pose", () => {
  const drone = { body: [0.1, 0.1, 0.05], rotor_radius: 0.01,
                  rotors: [[0.2, 0, 0]] };
  const faces = droneFaces({ p: [0, 0, 0], q: YAW90 }, drone);
  const disc = faces[6];                       // first face after the body box
  const centre = disc.pts.reduce(
    (acc, p) => [acc[0] + p[0] / disc.pts.length,
                 acc[1] + p[1] / disc.pts.length,
                 acc[2] + p[2] / disc.pts.length], [0, 0, 0]);
  assert.ok(Math.abs(centre[0]) < 1e-9, "x should have rotated away");
  assert.ok(Math.abs(centre[1] - 0.2) < 1e-9, "should now be on +y");
});

// FAILS IF: the rotor colour scale loses its direction. Below hover must move
// towards blue and above towards orange; if both went the same way, a
// coordinated turn would look like all four rotors doing the same thing --
// which is precisely the reading the overlay exists to prevent.
test("rotor colour diverges either side of hover", () => {
  const blue = rotorColour(-1);
  const white = rotorColour(0);
  const orange = rotorColour(1);
  const chan = (s) => s.match(/\d+/g).map(Number);
  assert.ok(chan(blue)[2] > chan(white)[2], "below hover should gain blue");
  assert.ok(chan(orange)[0] > chan(white)[0], "above hover should gain red");
  assert.ok(chan(orange)[2] < chan(white)[2], "above hover should lose blue");
  // Monotonic in between, so the eye can read magnitude and not just sign.
  const reds = [-1, -0.5, 0, 0.5, 1].map((t) => chan(rotorColour(t))[0]);
  for (let i = 1; i < reds.length; i++) assert.ok(reds[i] >= reds[i - 1]);
});

// FAILS IF: an arrow kind the ROS side sends has no colour and is drawn in the
// same shade as something else -- two different quantities would become
// indistinguishable.
test("every legend entry names a real palette colour", () => {
  for (const [, kind] of LEGEND) {
    assert.ok(PALETTE[kind], `legend refers to missing palette key ${kind}`);
  }
});

// FAILS IF: rotor arrows stop using the diverging scale, or an unknown kind
// throws instead of falling back to a visible default.
test("colourFor routes rotors to the diverging scale and unknowns to a default", () => {
  assert.equal(colourFor({ kind: "rotor", norm: 1 }), rotorColour(1));
  assert.equal(colourFor({ kind: "thrust" }), PALETTE.thrust);
  assert.ok(colourFor({ kind: "something_new" }).startsWith("#"));
  assert.ok(widthFor({ kind: "something_new" }) > 0);
});

import { sphereFaces, obstacleFaces, hitMarkers } from "../shapes.js";

// FAILS IF: sphereFaces stops describing a sphere of the requested radius at
// the requested centre -- every vertex must sit exactly r from the centre,
// and there must be faces both above and below it (not a hemisphere).
test("sphereFaces vertices all lie on the sphere", () => {
  const faces = sphereFaces(2, -1, 0.5, 0.9, "#e67e22", 0.3);
  assert.ok(faces.length >= 40, "too few faces to read as a sphere");
  let above = 0, below = 0;
  for (const f of faces) {
    assert.ok(f.alpha < 1, "obstacle spheres must be translucent");
    for (const [x, y, z] of f.pts) {
      const d = Math.hypot(x - 2, y + 1, z - 0.5);
      assert.ok(Math.abs(d - 0.9) < 1e-9, `vertex at distance ${d}, not 0.9`);
      if (z > 0.5 + 0.3) above++;
      if (z < 0.5 - 0.3) below++;
    }
  }
  assert.ok(above > 0 && below > 0, "sphere is missing a cap");
});

// FAILS IF: an obstacle is drawn somewhere other than where the referee says
// it is. The report is the only account of that -- these are not Gazebo
// bodies -- so a page that drew a cached pose would show a gap that is not
// there while the referee scored a collision.
test("obstacleFaces draws each sphere at the referee's live position", () => {
  const rep = { obstacles: [{ name: "F5", pos: [5.2, 5, 0], r: 0.9, active: true, type: "sphere" }] };
  const faces = obstacleFaces(rep, { F5: "#e67e22" }, 0.3);
  assert.ok(faces.length > 0);
  for (const f of faces) {
    for (const [x, y, z] of f.pts) {
      const d = Math.hypot(x - 5.2, y - 5, z - 0);
      assert.ok(Math.abs(d - 0.9) < 1e-9, `vertex at ${d} from the reported centre`);
    }
  }
  assert.deepEqual(obstacleFaces(null, {}, 0), []);
});

// FAILS IF: an obstacle outside its active window is still drawn. `wall` and
// `door3d` are built on obstacles that switch off; a ghost would say the
// vehicle is threading a gap that does not exist.
test("obstacleFaces skips inactive obstacles", () => {
  const rep = { obstacles: [
    { name: "W0", pos: [2.5, 0, 1.5], r: 0.5, active: false, type: "column" },
    { name: "W1", pos: [2.5, 1, 1.5], r: 0.5, active: true, type: "column" },
  ] };
  const faces = obstacleFaces(rep, {}, 0);
  assert.ok(faces.length > 0, "the active one must still be drawn");
  for (const f of faces) {
    for (const [, y] of f.pts) {
      assert.ok(Math.abs(y - 1) <= 0.5 + 1e-9, "a face from the inactive obstacle was drawn");
    }
  }
});

// FAILS IF: a column is drawn as a ball. A 2D scenario's obstacle is a disc at
// every altitude, and drawing it as a sphere at the flight altitude shows an
// over-the-top route that the planned problem never had.
test("obstacleFaces draws a column as something tall, not a ball", () => {
  const rep = { obstacles: [{ name: "c", pos: [0, 0, 1.5], r: 1.0, active: true, type: "column" }] };
  const zs = obstacleFaces(rep, {}, 0).flatMap((f) => f.pts.map((p) => p[2]));
  assert.ok(Math.max(...zs) - Math.min(...zs) > 3, "a column must span more than its radius");
  const xs = obstacleFaces(rep, {}, 0).flatMap((f) => f.pts.map((p) => p[0]));
  assert.ok(Math.max(...xs) <= 1 + 1e-9, "and must not be wider than its radius");
});

// FAILS IF: a hit marker is not centred on the hit position, or is drawn at
// the obstacle instead of where the vehicle was. Three axis segments each
// straddle the position by the vehicle radius.
test("hitMarkers put a cross exactly on each hit", () => {
  const rep = { hits: [{ pos: [5.21, 5.0, 0.5], with: "F5", t: 3.71, depth_m: 0.7 }] };
  const segs = hitMarkers(rep, 0.3);
  assert.equal(segs.length, 4);
  for (const s of segs.slice(0, 3)) {
    const mid = s.a.map((v, i) => 0.5 * (v + s.b[i]));
    assert.deepEqual(mid.map((v) => +v.toFixed(9)), [5.21, 5.0, 0.5]);
    const l = Math.hypot(...s.a.map((v, i) => v - s.b[i]));
    assert.ok(Math.abs(l - 0.6) < 1e-9, "cross arm is not 2 x radius");
  }
  assert.deepEqual(hitMarkers(null), []);
});

// ---- trails ---------------------------------------------------------------
//
// A trail is a CLAIM: "it went from here to there". These are about the claim
// being true, which is not the same as the points being recent.

test("a trail records a flown path point by point", () => {
  // FAILS IF: ordinary flight trips the discontinuity test and the trail keeps
  // restarting -- which would leave the page drawing almost nothing.
  const t = [];
  for (let i = 0; i < 50; i++) extendTrail(t, [0.1 * i, 0, 1.5], 1 / 30, 3.0);
  assert.equal(t.length, 50);
});

test("a trail does not draw a line across a reset", () => {
  // FAILS IF: the teleport back to the start is joined to where the run ended.
  // Measured on a real reset: the setpoint stream is NOT synchronised with the
  // world (the planner bridge runs on wall time), so it goes on publishing the
  // finished plan's last point across the page's clear. That point became the
  // first of the fresh trail and the jump to the start drew a 9.01 m line down
  // the middle of the world, which stayed for the rest of the run.
  const t = [];
  extendTrail(t, [9.50, 5.0, 0.50], 1 / 30, 3.0);
  extendTrail(t, [0.50, 5.0, 0.04], 1 / 30, 3.0);
  assert.deepEqual(t, [[0.50, 5.0, 0.04]], "the pre-reset point is still joined on");
});

test("a paused simulator does not accumulate trail points", () => {
  // FAILS IF: an idle sim fills the trail with thousands of identical points
  // and pushes the real path out of the far end.
  const t = [];
  for (let i = 0; i < 100; i++) extendTrail(t, [1, 2, 3], 0, 0);
  assert.equal(t.length, 1);
});

test("the jump bound scales with speed, so fast flight is not cut up", () => {
  // FAILS IF: the bound is a fixed distance. At loiter's speeds a legitimate
  // frame covers more ground than a slow scenario's whole reset, so a constant
  // cannot separate them -- the bound has to come from the motion.
  const fast = [];
  extendTrail(fast, [0, 0, 60], 1 / 30, 25);      // 25 m/s
  extendTrail(fast, [0.8, 0, 60], 1 / 30, 25);    // 0.8 m in one frame: real
  assert.equal(fast.length, 2);

  const slow = [];
  extendTrail(slow, [0, 0, 1.5], 1 / 30, 0.5);
  extendTrail(slow, [0.8, 0, 1.5], 1 / 30, 0.5);  // same step, impossible here
  assert.equal(slow.length, 1);
});

test("a trail is capped and drops from the front, not cleared", () => {
  // FAILS IF: a long run either grows without bound or blanks the trail, both
  // of which lose where the vehicle has just been.
  const t = [];
  for (let i = 0; i < TRAIL_MAX + 25; i++) extendTrail(t, [0.001 * i, 0, 1.5], 1 / 30, 3);
  assert.equal(t.length, TRAIL_MAX);
  assert.ok(t[0][0] > 0, "the oldest points should have been dropped");
});

// ---- framing --------------------------------------------------------------

test("the grid covers the whole scenario, not a fixed patch around the origin", () => {
  // FAILS IF: the grid goes back to a hard-coded extent. Every scenario out of
  // the planner lives in 0..10 m, so a -8..+8 patch put the action in one
  // quarter of it and let the vehicle fly off the edge at 9.5 m.
  const g = gridFor({ bounds: [[0, 0], [9.5, 9.5]] });
  assert.ok(g.lo[0] <= 0 && g.lo[1] <= 0, "grid starts after the scenario does");
  assert.ok(g.hi[0] >= 9.5 && g.hi[1] >= 9.5, "grid stops before the scenario does");
});

test("the grid keeps the origin, so the world frame stays visible", () => {
  // FAILS IF: the grid is recentred on the action. Then nothing on screen says
  // where (0, 0) is, and "the scenario sits in one quadrant" becomes invisible
  // rather than fixed.
  for (const b of [[[0, 0], [9.5, 9.5]], [[-100, -33.7], [100, 37.5]], [[0, -2], [8, 12]]]) {
    const g = gridFor({ bounds: b });
    assert.ok(g.lo[0] <= 0 && g.hi[0] >= 0 && g.lo[1] <= 0 && g.hi[1] >= 0);
  }
});

test("the grid step keeps the line count readable at any scale", () => {
  // FAILS IF: the step is fixed at 1 m. loiter spans 200 m, which would be 200
  // lines of noise; the bare world spans 16 m, where 1 m is right.
  for (const [span, lo, hi] of [[16, 8, 24], [9.5, 8, 24], [200, 8, 24]]) {
    const g = gridFor({ bounds: [[0, 0], [span, span]] });
    const lines = (g.hi[0] - g.lo[0]) / g.step;
    assert.ok(lines >= lo && lines <= hi, `span ${span} gave ${lines} lines`);
  }
});

test("a grid step is always a number you can count in", () => {
  // FAILS IF: the step is span/16 raw, giving 0.59375 m squares.
  for (const span of [1, 3, 9.5, 14, 47, 200, 1234]) {
    const { step } = gridFor({ bounds: [[0, 0], [span, span]] });
    const mantissa = step / Math.pow(10, Math.floor(Math.log10(step)));
    assert.ok([1, 2, 5].some((m) => Math.abs(mantissa - m) < 1e-9),
      `step ${step} for span ${span} is not 1/2/5 x 10^k`);
  }
});

test("the camera frames the obstacles, not just the plan's two ends", () => {
  // FAILS IF: framing goes back to start/end. door3d's obstacles reach from
  // y=-2 to y=12 beside a plan that runs along y=5, so framing on the ends
  // leaves them outside the view chosen for them.
  const home = homeFor({ bounds: [[0, -2], [8, 12]], start: [0.5, 5, 0.5], end: [8, 5, 0.5] });
  assert.ok(Math.abs(home.target[1] - 5) < 1e-9, "target should be the middle of the ground");
  assert.ok(home.dist >= 1.2 * 14, "distance should cover the 14 m spread in y");
});

test("a world with no bounds still frames something", () => {
  // FAILS IF: a scene.json without bounds leaves the camera at NaN and the
  // page renders nothing at all.
  for (const w of [null, undefined, {}, { start: [0, 0, 0], end: [1, 1, 1] }]) {
    const home = homeFor(w);
    assert.ok(home.target.every(Number.isFinite) && Number.isFinite(home.dist));
  }
});

// ---- line of sight --------------------------------------------------------

test("a clear sight line is drawn from every station to the vehicle", () => {
  // FAILS IF: the page draws one line, or none. A scenario may have several
  // stations and the constraint is that they can ALL see the vehicle.
  const cl = { stations: [[0, 0, 0], [5, 5, 0]], los_margin_m: 2.5 };
  const segs = sightLines(cl, { p: [10, 0, 3] });
  assert.equal(segs.length, 2);
  assert.deepEqual(segs[0].a, [0, 0, 0]);
  assert.deepEqual(segs[0].b, [10, 0, 3]);
});

test("a blocked sight line is drawn differently from a clear one", () => {
  // FAILS IF: occlusion looks the same as visibility on screen. It is the one
  // failure a picture of clearances cannot show -- the vehicle can be far from
  // every obstacle and still be behind one.
  const clear = sightLines({ stations: [[0, 0, 0]], los_margin_m: 2.5 }, { p: [10, 0, 0] });
  const blocked = sightLines({ stations: [[0, 0, 0]], los_margin_m: -1.0 }, { p: [10, 0, 0] });
  assert.notEqual(clear[0].colour, blocked[0].colour);
});

test("a scenario with no stations draws no sight lines and no stations", () => {
  // FAILS IF: every scenario grows a line to the origin, which would assert a
  // constraint six of the seven scenarios do not have.
  for (const cl of [null, {}, { stations: [] }]) {
    assert.equal(sightLines(cl, { p: [1, 2, 3] }).length, 0);
    assert.equal(stationFaces(cl).length, 0);
  }
});

test("a station is drawn somewhere, and where the referee says", () => {
  // FAILS IF: the station is drawn at the origin regardless, or not at all.
  const faces = stationFaces({ stations: [[7, -3, 0]] });
  assert.ok(faces.length > 0);
  const xs = faces.flatMap((f) => f.pts.map((p) => p[0]));
  const ys = faces.flatMap((f) => f.pts.map((p) => p[1]));
  assert.ok(Math.min(...xs) > 5 && Math.max(...xs) < 9);
  assert.ok(Math.min(...ys) > -5 && Math.max(...ys) < -1);
});
