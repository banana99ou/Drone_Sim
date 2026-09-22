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
import { shadeHex, droneFaces } from "../shapes.js";
import { rotorColour, colourFor, widthFor, PALETTE, LEGEND } from "../palette.js";

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

import { sphereFaces, obstaclePositions, hitMarkers } from "../shapes.js";

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

// FAILS IF: the live position lookup keys on the wrong field or drops
// entries, so a moving obstacle would be drawn at its t=0 pose forever.
test("obstaclePositions maps referee names to live positions", () => {
  const rep = { obstacles: [{ name: "F5", pos: [5.2, 5, 0], r: 0.9 },
                            { name: "M1", pos: [7, 2.3, 0.81], r: 0.7 }] };
  const live = obstaclePositions(rep);
  assert.deepEqual(live.F5, [5.2, 5, 0]);
  assert.deepEqual(live.M1, [7, 2.3, 0.81]);
  assert.deepEqual(obstaclePositions(null), {});
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
