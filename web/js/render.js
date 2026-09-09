// Canvas renderer: perspective projection, painter's algorithm, arrows.
//
// Hand-rolled rather than three.js, and deliberately so: the scene is a grid,
// a few boxes and cylinders, one drone and a couple of dozen arrows. That is
// cheap enough that a hundred lines of projection beats a megabyte of engine
// from a CDN -- and it works on an iPad with nothing installed, over a tailnet,
// with no build step.
//
// This is the only file that touches the canvas. Everything it draws is handed
// to it in world coordinates by pure code in vec3.js / shapes.js / overlays.js.

import { add, sub, dot, cross, unit } from "./vec3.js";

const NEAR = 0.02;          // world units; anything nearer is behind the eye
const FOV_RAD = 0.9;

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.px = 1;
    this.view = null;
    this.resize();
  }

  /// Match the backing store to the display. Line widths and fonts are then
  /// multiplied by this.px, otherwise everything is hairline-thin on a retina
  /// screen -- which is most of the devices this gets opened on.
  resize() {
    this.px = window.devicePixelRatio || 1;
    this.canvas.width = Math.floor(window.innerWidth * this.px);
    this.canvas.height = Math.floor(window.innerHeight * this.px);
    this.canvas.style.width = `${window.innerWidth}px`;
    this.canvas.style.height = `${window.innerHeight}px`;
  }

  begin(cam, background) {
    this.cam = cam;
    const ce = Math.cos(cam.el);
    const se = Math.sin(cam.el);
    const eye = add(cam.target, [cam.dist * ce * Math.cos(cam.az),
                                 cam.dist * ce * Math.sin(cam.az),
                                 cam.dist * se]);
    const fwd = unit(sub(cam.target, eye));
    let right = cross(fwd, [0, 0, 1]);
    // Looking straight down, forward is parallel to world up and the cross
    // product collapses; pick an arbitrary but stable right vector.
    right = Math.hypot(right[0], right[1], right[2]) < 1e-6 ? [1, 0, 0] : unit(right);
    const h = this.canvas.height;
    this.view = {
      eye, right, fwd,
      up: cross(right, fwd),
      w: this.canvas.width, h,
      f: (0.5 * h) / Math.tan(0.5 * FOV_RAD),
    };
    this.ctx.fillStyle = background;
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
  }

  /// World point -> {x, y, depth}. depth <= NEAR means behind the camera and
  /// must not be drawn: projecting it produces a point mirrored through the eye.
  project(p) {
    const v = this.view;
    const d = sub(p, v.eye);
    const z = dot(d, v.fwd);
    if (z <= NEAR) return { x: 0, y: 0, depth: z };
    const inv = v.f / z;
    return {
      x: v.w * 0.5 + dot(d, v.right) * inv,
      y: v.h * 0.5 - dot(d, v.up) * inv,
      depth: z,
    };
  }

  faces(list) {
    const ctx = this.ctx;
    const drawable = [];
    for (const f of list) {
      const ps = f.pts.map((p) => this.project(p));
      if (ps.some((q) => q.depth <= NEAR)) continue;
      let z = 0;
      for (const q of ps) z += q.depth;
      drawable.push({ ps, z: z / ps.length, f });
    }
    drawable.sort((a, b) => b.z - a.z);          // painter's algorithm
    for (const d of drawable) {
      ctx.beginPath();
      ctx.moveTo(d.ps[0].x, d.ps[0].y);
      for (let i = 1; i < d.ps.length; i++) ctx.lineTo(d.ps[i].x, d.ps[i].y);
      ctx.closePath();
      ctx.globalAlpha = d.f.alpha;
      ctx.fillStyle = d.f.fill;
      ctx.fill();
      ctx.globalAlpha = 1;
    }
  }

  polyline(pts, colour, width, dashed) {
    if (pts.length < 2) return;
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = colour;
    ctx.lineWidth = width * this.px;
    ctx.lineJoin = "round";
    if (dashed) ctx.setLineDash([7 * this.px, 6 * this.px]);
    ctx.beginPath();
    let pen = false;
    for (const p of pts) {
      const q = this.project(p);
      if (q.depth <= NEAR) { pen = false; continue; }
      if (pen) ctx.lineTo(q.x, q.y);
      else { ctx.moveTo(q.x, q.y); pen = true; }
    }
    ctx.stroke();
    ctx.restore();
  }

  segments(list) {
    const ctx = this.ctx;
    ctx.save();
    ctx.lineWidth = 2 * this.px;
    for (const s of list) {
      const a = this.project(s.a);
      const b = this.project(s.b);
      if (a.depth <= NEAR || b.depth <= NEAR) continue;
      ctx.strokeStyle = s.colour;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  /// Arrows are drawn AFTER the solid faces rather than depth-sorted with
  /// them, so an arrow is never hidden inside the airframe it belongs to.
  /// That is a legibility choice, and the reason overlay arrows always read
  /// even when the camera is looking through the vehicle.
  arrows(list) {
    const ctx = this.ctx;
    const px = this.px;
    ctx.save();
    ctx.lineCap = "round";
    ctx.font = `${11 * px}px ui-sans-serif, -apple-system, system-ui, sans-serif`;
    ctx.textBaseline = "middle";
    for (const a of list) {
      const f = this.project(a.from);
      const t = this.project(a.to);
      if (f.depth <= NEAR || t.depth <= NEAR) continue;
      const dx = t.x - f.x;
      const dy = t.y - f.y;
      const l = Math.hypot(dx, dy);
      if (l < 2 * px) continue;        // shorter than its own head; skip

      ctx.strokeStyle = a.colour;
      ctx.fillStyle = a.colour;
      ctx.lineWidth = (a.width || 2) * px;
      ctx.setLineDash(a.dashed ? [6 * px, 5 * px] : []);
      ctx.beginPath();
      ctx.moveTo(f.x, f.y);
      ctx.lineTo(t.x, t.y);
      ctx.stroke();
      ctx.setLineDash([]);

      // A clamped arrow is not the length of the thing it represents, so it
      // must not look like one. A hollow head says "longer than this" instead
      // of quietly asserting a magnitude that was cut off.
      const head = Math.min(11 * px, l * 0.45);
      const ux = dx / l;
      const uy = dy / l;
      const nx = -uy;
      const ny = ux;
      ctx.beginPath();
      ctx.moveTo(t.x, t.y);
      ctx.lineTo(t.x - ux * head + nx * head * 0.42,
                 t.y - uy * head + ny * head * 0.42);
      ctx.lineTo(t.x - ux * head - nx * head * 0.42,
                 t.y - uy * head - ny * head * 0.42);
      ctx.closePath();
      if (a.clamped) { ctx.lineWidth = 1.5 * px; ctx.stroke(); } else { ctx.fill(); }

      // Only label an arrow long enough that the text will not sit on top of
      // its neighbours. Below that the colour still carries the information.
      // 26 px was too high: the rotor arrows sit at 15-17 px at the default
      // camera, so their newton values never appeared at all.
      if (a.label && l > 15 * px) {
        ctx.fillText(a.label, t.x + ux * 7 * px + 4 * px, t.y + uy * 7 * px);
      }
    }
    ctx.restore();
  }

  grid(n = 8, step = 1) {
    const ctx = this.ctx;
    ctx.save();
    ctx.lineWidth = this.px;
    for (let i = -n; i <= n; i++) {
      ctx.strokeStyle = i === 0 ? "#3d4763" : "#20242f";
      const segs = [
        [[i * step, -n * step, 0], [i * step, n * step, 0]],
        [[-n * step, i * step, 0], [n * step, i * step, 0]],
      ];
      for (const [s0, s1] of segs) {
        const a = this.project(s0);
        const b = this.project(s1);
        if (a.depth <= NEAR || b.depth <= NEAR) continue;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  /// The collision envelope, as a ring on the ground under the vehicle plus a
  /// drop line. This is the radius a planner has to keep clear, so it is worth
  /// seeing where it actually is rather than trusting a number in the HUD.
  envelope(pose, radius) {
    const p = pose.p;
    const pts = [];
    for (let i = 0; i <= 36; i++) {
      const a = (i / 36) * Math.PI * 2;
      pts.push([p[0] + radius * Math.cos(a), p[1] + radius * Math.sin(a), 0.005]);
    }
    this.polyline(pts, "rgba(96,165,250,.5)", 1.5, false);
    this.segments([{ a: [p[0], p[1], 0.005], b: p, colour: "rgba(96,165,250,.28)" }]);
  }
}
