// Drone_Sim remote viewer: wiring.
//
// Streams state over a same-origin event stream and renders it locally, rather
// than streaming video of a Gazebo GUI. Video would need a virtual display, a
// software GL rasteriser and an H.264 encoder on the sim host for a blurry
// laggy result; this is a few KB/s and renders crisply at whatever resolution
// the viewing device has.
//
// THE RULE: all logic lives in the ROS simulator and its scripts. This page
// only draws. Overlay geometry arrives from dsim_viz/overlay.py as world-space
// line segments already in metres, tagged with a `kind` and a `group`; the
// page projects them, picks colours, filters by checkbox and strokes them.
// There is no physics here -- no mass times acceleration, no frame rotation of
// a measured quantity, no opinion about what a force is.
//
// Layout of the code:
//   vec3.js       vector / quaternion maths for drawing   pure, tested in node
//   shapes.js     scene geometry as face lists            pure, tested in node
//   palette.js    overlay colours and labels              pure, tested in node
//   render.js     the only file that touches the canvas
//   telemetry.js  the event stream and its fallback
//   app.js        state, HUD, input, the frame loop       (this file)

import { add, scale } from "./vec3.js";
import { boxFaces, cylinderFaces, droneFaces } from "./shapes.js";
import { GROUP_LABEL, LEGEND, PALETTE, TICK_COLOUR, colourFor, widthFor }
  from "./palette.js";
import { Renderer } from "./render.js";
import { Telemetry } from "./telemetry.js";

const TRAIL_MAX = 4000;                 // ~2 min of trail at 30 Hz
const BACKGROUND = "#0d0f14";
const CAM_HOME = { az: -0.9, el: 0.42, dist: 11, target: [0, 0, 1.0] };
const PREFS_KEY = "dsim.prefs";

const el = (id) => document.getElementById(id);

const S = {
  scene: null,
  world: "pillars",
  pose: null,
  status: null,
  control: null,
  overlay: null,
  sim: null,
  flown: [],
  planned: [],
};

const prefs = {
  gain: 1,
  show: Object.fromEntries(Object.keys(GROUP_LABEL).map((g) => [g, true])),
};

let cam = { ...CAM_HOME, target: CAM_HOME.target.slice() };
let hudError = "";

const renderer = new Renderer(el("view"));
const telemetry = new Telemetry({ onSnapshot: ingest });

// ---- preferences ---------------------------------------------------------
// Wrapped because localStorage throws outright in some privacy modes rather
// than returning null, and a viewer that refuses to start because it could not
// remember a checkbox would be a poor trade.
function loadPrefs() {
  try {
    const saved = JSON.parse(localStorage.getItem(PREFS_KEY) || "null");
    if (!saved) return;
    if (typeof saved.gain === "number") prefs.gain = saved.gain;
    if (saved.show) Object.assign(prefs.show, saved.show);
    if (saved.world) S.world = saved.world;
  } catch (_) { /* private browsing, or a stale format */ }
}

function savePrefs() {
  try {
    localStorage.setItem(PREFS_KEY, JSON.stringify(
      { gain: prefs.gain, show: prefs.show, world: S.world }));
  } catch (_) { /* not important enough to surface */ }
}

// ---- incoming state ------------------------------------------------------
function ingest(snap) {
  if (!snap) return;
  if (snap.pose) {
    const p = snap.pose.p;
    const last = S.flown[S.flown.length - 1];
    S.pose = snap.pose;
    // Extend the trail only when the vehicle actually moved, so a paused sim
    // does not accumulate thousands of identical points.
    if (!last || last[0] !== p[0] || last[1] !== p[1] || last[2] !== p[2]) {
      S.flown.push(p);
      if (S.flown.length > TRAIL_MAX) S.flown.shift();
    }
  }
  if (snap.setpoint) {
    const sp = snap.setpoint;
    const last = S.planned[S.planned.length - 1];
    if (!last || last[0] !== sp[0] || last[1] !== sp[1] || last[2] !== sp[2]) {
      S.planned.push(sp);
      if (S.planned.length > TRAIL_MAX) S.planned.shift();
    }
  }
  if (snap.status) S.status = snap.status;
  if (snap.control) S.control = snap.control;
  if (snap.overlay !== undefined) S.overlay = snap.overlay;
  if (snap.sim) S.sim = snap.sim;
}

// ---- frame rate ----------------------------------------------------------
// Measured here rather than guessed. The viewer's own cost is the one thing a
// remote user can see directly, and "it feels slower" is not something you can
// act on -- a number is.
const fps = { frames: 0, last: 0, value: 0 };
function tickFps(now) {
  fps.frames++;
  if (!fps.last) { fps.last = now; return; }
  const dt = now - fps.last;
  if (dt >= 500) {
    fps.value = (fps.frames * 1000) / dt;
    fps.frames = 0;
    fps.last = now;
  }
}

// ---- overlay: filter, zoom, colour --------------------------------------
/// Scale an arrow about its own anchor. This is a display zoom -- the same
/// class of operation as the camera's -- so it cannot change the ratio between
/// two arrows, which is what makes their lengths comparable.
function zoomAbout(anchor, point, gain) {
  return add(anchor, scale([point[0] - anchor[0], point[1] - anchor[1],
                            point[2] - anchor[2]], gain));
}

function overlayToDraw() {
  if (!S.overlay) return { arrows: [], ticks: [] };
  const g = prefs.gain;
  // An arrow whose group has no checkbox is drawn rather than hidden: if the
  // ROS side starts publishing something new, it should show up, not vanish.
  const arrows = S.overlay.arrows
    .filter((a) => prefs.show[a.group] !== false)
    .map((a) => ({
      from: a.from,
      to: zoomAbout(a.from, a.to, g),
      colour: colourFor(a),
      width: widthFor(a),
      label: a.label,
      dashed: a.dashed === true,
      clamped: a.clamped === true,
    }));
  const ticks = (prefs.show.motors === false ? [] : S.overlay.ticks)
    .map((t) => ({
      a: zoomAbout(t.anchor, t.a, g),
      b: zoomAbout(t.anchor, t.b, g),
      colour: TICK_COLOUR,
    }));
  return { arrows, ticks };
}

// ---- frame ---------------------------------------------------------------
function render(now) {
  // The reschedule sits OUTSIDE the try on purpose: this loop re-arms itself,
  // so one exception inside would stop the picture permanently.
  try {
    tickFps(now || performance.now());
    renderOnce();
  } catch (e) {
    hudError = `render: ${e && e.message ? e.message : e}`;
  }
  requestAnimationFrame(render);
}

function renderOnce() {
  renderer.begin(cam, BACKGROUND);
  if (!S.scene) return;
  renderer.grid();

  const faces = [];
  const world = S.scene.worlds[S.world] || { obstacles: [] };
  for (const o of world.obstacles) {
    const [x, y, z] = o.pose;
    const [sx, sy, sz] = o.size;
    faces.push(...(o.type === "cylinder"
      ? cylinderFaces(x, y, z, sx, sz, "#8d5a3c", cam.az)
      : boxFaces(x, y, z, sx, sy, sz, "#8d5a3c")));
  }
  faces.push(...droneFaces(S.pose, S.scene.drone));
  renderer.faces(faces);

  renderer.polyline(S.planned, "rgba(248,113,113,.85)", 2, true);
  renderer.polyline(S.flown, "rgba(96,165,250,.95)", 2, false);
  if (S.pose) renderer.envelope(S.pose, S.scene.drone.radius);

  const draw = overlayToDraw();
  renderer.segments(draw.ticks);
  renderer.arrows(draw.arrows);
}

// ---- HUD -----------------------------------------------------------------
function updateHud() {
  try {
    updateHudOnce();
  } catch (e) {
    hudError = e && e.message ? e.message : String(e);
  }
  setTimeout(updateHud, 100);
}

const fmt = (v, unit, digits = 2) =>
  (v === null || v === undefined || Number.isNaN(v) ? "—" : v.toFixed(digits) + unit);

function updateHudOnce() {
  const t = telemetry.status;
  el("link").className = `dot ${t.connected ? "up" : "down"}`;
  el("linktext").textContent = hudError
    ? `hud error: ${hudError}`
    : t.connected
      ? `live · ${t.via}`
      : `disconnected — ${t.via}${t.note ? ` (${t.note})` : ""}`;

  if (S.pose) el("alt").textContent = fmt(S.pose.p[2], " m");

  // Every control number is read straight from the overlay's readout: it was
  // computed once, on the ROS side, from the message that produced the arrows.
  // A second calculation here could disagree with the picture.
  const r = S.overlay && S.overlay.readout;
  const c = S.control;
  if (r) {
    el("spd").textContent = fmt(r.speed_mps, " m/s");
    el("tilt").textContent = fmt(r.tilt_deg, "°", 1);
    el("tau").textContent = fmt(r.torque_nm, " N·m", 3);
    el("aero").textContent = r.aero_n === null ? "—" : fmt(r.aero_n, " N");
  }
  if (c) {
    el("thr").textContent = fmt(c.realised_thrust_n, " N");
    el("rot").textContent = c.rotor_thrust_n.map((f) => f.toFixed(2)).join("  ");
    el("tilt").className = c.tilt_clamped ? "bad" : "";
    el("sat").textContent = c.saturated ? "SATURATED" : (c.armed ? "ok" : "disarmed");
    el("sat").className = c.saturated || !c.armed ? "bad" : "good";
  }

  el("fps").textContent = fps.value ? `${fps.value.toFixed(0)} /s` : "—";

  // Simulator pause and speed, as OBSERVED by the server, not as requested.
  const sim = S.sim;
  if (sim) {
    if (!sim.enabled) {
      el("simstate").textContent = "controls off";
      el("simstate").className = "";
    } else if (sim.error) {
      el("simstate").textContent = sim.error;
      el("simstate").className = "bad";
    } else {
      const m = sim.measured_rtf;
      el("simstate").textContent = sim.paused
        ? "PAUSED"
        : `${m === null || m === undefined ? "?" : m.toFixed(2)}x real time`;
      el("simstate").className = sim.paused ? "bad" : "good";
    }
    el("pause").textContent = sim.paused ? "resume" : "pause";
    el("pause").disabled = !sim.enabled;
    el("speed").disabled = !sim.enabled;
  }

  const st = S.status;
  if (st) {
    // Already in centimetres when it arrives: the page does not convert
    // physical quantities. See _status_of() in viz_server.py.
    el("err").textContent = fmt(st.tracking_error_cm, " cm", 1);
    el("rmse").textContent = fmt(st.tracking_rmse_cm, " cm", 1);
    const clr = st.min_obstacle_clearance_m;
    el("clr").textContent = clr === null ? "n/a" : fmt(clr, " m");
    el("clr").className = clr !== null && clr < 0 ? "bad" : "";
    el("col").textContent = st.collided ? `HIT (${st.collision_count})` : "clear";
    el("col").className = st.collided ? "bad" : "good";
    el("ela").textContent = fmt(st.elapsed_s, " s", 1);
  }
}

// ---- controls ------------------------------------------------------------
function buildControls() {
  const box = el("groups");
  for (const [group, label] of Object.entries(GROUP_LABEL)) {
    const row = document.createElement("label");
    row.className = "check";
    row.innerHTML =
      `<input type="checkbox"${prefs.show[group] ? " checked" : ""}>` +
      `<span>${label}</span>`;
    box.appendChild(row);
    row.querySelector("input").onchange = (e) => {
      prefs.show[group] = e.target.checked;
      savePrefs();
    };
  }

  const slider = el("gain");
  slider.value = String(prefs.gain);
  const showGain = () => { el("gainval").textContent = `${prefs.gain.toFixed(1)}x`; };
  showGain();
  slider.oninput = (e) => {
    prefs.gain = Number(e.target.value);
    showGain();
    savePrefs();
  };

  // The legend is built from the same palette the arrows use, so it cannot
  // end up telling a different story from the picture.
  const legend = el("legend");
  for (const [name, kind] of LEGEND) {
    const row = document.createElement("div");
    row.innerHTML = `<i style="background:${PALETTE[kind]}"></i>${name}`;
    legend.appendChild(row);
  }
  const note = document.createElement("div");
  note.className = "note";
  note.textContent = "rotor arrows: blue below hover, orange above";
  legend.appendChild(note);
}

function bindInput() {
  const canvas = el("view");
  window.addEventListener("resize", () => renderer.resize());

  let drag = null;
  canvas.addEventListener("pointerdown", (e) => {
    drag = { x: e.clientX, y: e.clientY, pan: e.shiftKey };
    canvas.classList.add("dragging");
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x;
    const dy = e.clientY - drag.y;
    drag.x = e.clientX;
    drag.y = e.clientY;
    if (drag.pan) {
      const k = cam.dist * 0.0016;
      const v = renderer.view || { right: [1, 0, 0], up: [0, 0, 1] };
      cam.target = add(cam.target, add(scale(v.right, -dx * k), scale(v.up, dy * k)));
    } else {
      cam.az -= dx * 0.006;
      cam.el = Math.max(-1.45, Math.min(1.45, cam.el + dy * 0.006));
    }
  });
  const endDrag = () => { drag = null; canvas.classList.remove("dragging"); };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);

  const zoom = (factor) => {
    cam.dist = Math.max(0.8, Math.min(60, cam.dist * factor));
  };
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    zoom(Math.exp(e.deltaY * 0.0013));
  }, { passive: false });

  let pinch = null;
  const spread = (e) => Math.hypot(
    e.touches[0].clientX - e.touches[1].clientX,
    e.touches[0].clientY - e.touches[1].clientY);
  canvas.addEventListener("touchstart", (e) => {
    if (e.touches.length === 2) pinch = spread(e);
  });
  canvas.addEventListener("touchmove", (e) => {
    if (e.touches.length === 2 && pinch) {
      const d = spread(e);
      zoom(pinch / d);
      pinch = d;
      e.preventDefault();
    }
  }, { passive: false });
  canvas.addEventListener("touchend", () => { pinch = null; });

  // ---- simulator controls ------------------------------------------------
  // The page sends intent only. Validation, the range check and the actual
  // service call all live in dsim_viz/simcontrol.py, and the reply reports
  // what the simulator was observed to do afterwards.
  const post = (body) => fetch("/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then((r) => r.json()).then((r) => {
    if (r.error) { hudError = r.error; } else { S.sim = r; hudError = ""; }
  }).catch((e) => { hudError = `control: ${e.message}`; });

  el("pause").onclick = () => post({ paused: !(S.sim && S.sim.paused) });

  const speed = el("speed");
  const speedLabel = () => {
    el("speedval").textContent = `${Number(speed.value).toFixed(2)}x`;
  };
  speedLabel();
  speed.oninput = speedLabel;
  // On release, not on every drag pixel: each change is a service call into
  // the simulator, and 40 of them while dragging would be 40 subprocesses.
  speed.onchange = () => post({ rtf: Number(speed.value) });

  el("reset").onclick = () => {
    cam = { ...CAM_HOME, target: CAM_HOME.target.slice() };
  };
  el("clear").onclick = () => { S.flown = []; S.planned = []; };
  el("world").onchange = (e) => {
    S.world = e.target.value;
    savePrefs();
  };
}

// ---- boot ----------------------------------------------------------------
function loadScene() {
  return fetch("scene.json")
    .then((r) => r.json())
    .then((scene) => {
      S.scene = scene;
      const sel = el("world");
      for (const name of Object.keys(scene.worlds)) {
        const o = document.createElement("option");
        o.value = name;
        o.textContent = name;
        sel.appendChild(o);
      }
      if (!scene.worlds[S.world]) S.world = Object.keys(scene.worlds)[0];
      sel.value = S.world;
      // What the sim is ACTUALLY running overrides the remembered choice.
      // Drawing obstacles that are not in the running world is worse than
      // drawing none: you would trust a clearance that does not exist.
      return fetch("current.json", { cache: "no-store" })
        .then((r) => r.json())
        .then((cur) => {
          if (cur && cur.world && scene.worlds[cur.world]) {
            S.world = cur.world;
            sel.value = S.world;
          }
        })
        .catch(() => { /* viz launched without it; the dropdown stays manual */ });
    })
    .catch(() => {
      el("linktext").textContent = "scene.json missing";
    });
}

loadPrefs();
buildControls();
bindInput();
loadScene();
telemetry.start();
updateHud();
render();
