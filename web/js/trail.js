// Trails: the flown path and the commanded one.
//
// Pure, and in its own file, so the rule below can be tested without a browser
// -- see js/test/viewer.test.js. Nothing here touches the DOM or the network.

/// The most points a trail keeps: about two minutes at 30 Hz. Older points are
/// dropped from the front rather than the trail being cleared, so a long run
/// still shows where the vehicle has just been.
export const TRAIL_MAX = 4000;

/// How far a trail may jump between frames before the segment is disowned, on
/// top of what the reported speed allows. Half a metre is far more than 30 Hz
/// flight covers and far less than any reset.
export const TRAIL_JUMP_FLOOR_M = 0.5;

/// Extend a trail, but only along a path the thing could actually have taken.
///
/// A trail is a CLAIM: "it went from here to there". Joining two points it
/// could not have travelled between draws a line through a journey that never
/// happened, and the picture cannot be told apart from a real one.
///
/// The case this exists for is the reset, and the mechanism is not the obvious
/// one. Measured over five resets: the vehicle's pose and the clock come back
/// on the SAME frame, so the flown trail was always fine (longest segment
/// 0.17 m). The SETPOINT does not -- it comes from the planner bridge, which
/// runs on wall time and so is not synchronised with a world reset at all. It
/// went on publishing the finished plan's last point across the clear, which
/// became the first point of the fresh trail, and the jump back to the start
/// then drew a 9.01 m line down the middle of the world that stayed for the
/// rest of the run.
///
/// The bound is derived from the motion, not typed: whatever speed was just
/// reported, times the time between these two frames, times three for
/// transients, plus a floor. Ordinary flight never trips it and a reset always
/// does -- there are four orders of magnitude between them.
export function extendTrail(trail, point, dt, speed) {
  const last = trail[trail.length - 1];
  if (last) {
    if (last[0] === point[0] && last[1] === point[1] && last[2] === point[2]) {
      return;               // a paused sim must not accumulate identical points
    }
    const gap = Math.hypot(point[0] - last[0], point[1] - last[1], point[2] - last[2]);
    const reach = 3 * Math.max(speed || 0, 1) * Math.max(dt, 0) + TRAIL_JUMP_FLOOR_M;
    if (!(gap <= reach)) trail.length = 0;     // also catches a NaN gap
  }
  trail.push(point);
  if (trail.length > TRAIL_MAX) trail.shift();
}

