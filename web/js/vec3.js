// Vector and quaternion helpers. Pure functions on plain [x, y, z] arrays:
// no classes, no allocation tricks, nothing to learn before reading the
// callers. World frame is ROS convention (ENU: +x forward, +y left, +z up).

export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);

export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/// Unit vector. A zero vector returns zero rather than NaN: overlay code
/// normalises quantities that are legitimately zero (no torque demanded while
/// hovering), and one NaN would poison a whole projected polygon.
export function unit(a) {
  const l = len(a);
  return l > 1e-12 ? scale(a, 1 / l) : [0, 0, 0];
}

/// Hamilton quaternion [w, x, y, z] -> rotation matrix as three ROWS.
/// Rotates BODY vectors into WORLD, matching the odometry convention.
export function quatMat(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
    [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
  ];
}

/// M * v, with M as rows.
export const rotate = (M, v) => [dot(M[0], v), dot(M[1], v), dot(M[2], v)];

/// Body axes in world coordinates: the columns of M.
export const bodyX = (M) => [M[0][0], M[1][0], M[2][0]];
export const bodyY = (M) => [M[0][1], M[1][1], M[2][1]];
export const bodyZ = (M) => [M[0][2], M[1][2], M[2][2]];
