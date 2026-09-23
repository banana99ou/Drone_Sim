// Where the camera looks and how big the ground grid is.
//
// Pure, and in its own file, so it can be tested without a browser -- see
// js/test/viewer.test.js. Nothing here touches the DOM.
//
// All of it is driven by a world's `bounds`, which gen_assets.py computes from
// the scenario itself (every obstacle control point, the plan's endpoints, and
// the origin). Before that existed the grid was a fixed -8..+8 m patch around
// the origin, and since every scenario out of the planner lives in 0..10 m the
// action sat in one quarter of it and ran off the edge at 9.5 m.

/// The bare world's view: the origin, far enough back for the built-in circle.
export const CAM_HOME = { az: -0.9, el: 0.42, dist: 11, target: [0, 0, 1.0] };

/// Where "recentre" looks: the middle of the ground the scenario occupies,
/// far enough back to see all of it.
///
/// Framed on `bounds` -- every obstacle as well as the plan's endpoints --
/// rather than on the plan's two ends. Framing on the ends alone put door3d's
/// obstacles, which reach from y=-2 to y=12 beside a plan that runs along
/// y=5, outside the view it chose for them.
export function homeFor(world) {
  const home = { ...CAM_HOME, target: CAM_HOME.target.slice() };
  if (world && world.bounds) {
    const [lo, hi] = world.bounds;
    const z = world.start && world.end ? 0.5 * (world.start[2] + world.end[2]) : 1.0;
    home.target = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), z];
    home.dist = Math.max(11, 1.2 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1]));
  }
  return home;
}

/// A grid step that keeps the line count readable at any scale.
///
/// 1 m lines across loiter's 200 m would be 200 lines of noise; 1 m across the
/// bare world is right. Snapped to 1/2/5 x 10^k so the spacing is always a
/// number you can count in, and so the step does not flicker between two
/// arbitrary values as the extent changes.
export function niceStep(span, want = 16) {
  const raw = Math.max(span, 1e-6) / want;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5]) if (raw <= m * mag) return m * mag;
  return 10 * mag;
}

/// The ground grid for a world: its own extent, padded out to whole steps.
export function gridFor(world) {
  const [lo, hi] = (world && world.bounds) || [[-8, -8], [8, 8]];
  const step = niceStep(Math.max(hi[0] - lo[0], hi[1] - lo[1]));
  return {
    step,
    lo: [Math.floor(lo[0] / step - 1) * step, Math.floor(lo[1] / step - 1) * step],
    hi: [Math.ceil(hi[0] / step + 1) * step, Math.ceil(hi[1] / step + 1) * step],
  };
}
