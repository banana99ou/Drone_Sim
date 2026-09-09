// Live state transport: Server-Sent Events, with polling as a fallback.
//
// SSE is plain HTTP on the same origin as the page, which means it can be
// verified end to end with `curl /state` and no browser in the loop. That
// matters more than it sounds: the WebSocket transport this replaced could not
// be diagnosed from outside the browser at all.
//
// The fallback is driven by a WATCHDOG, not only by error events. An
// EventSource that hangs in CONNECTING never fires `error`, so a
// failure-counting fallback waits forever -- which is exactly the symptom that
// showed up as a page rendering the scene with a dead HUD. The server sends a
// frame every 1/30 s even when nothing has changed, so silence longer than a
// couple of seconds is unambiguous evidence that the stream is not working,
// whatever the EventSource thinks its readyState is.

export class Telemetry {
  constructor({ onSnapshot, pollHz = 10, watchdogMs = 2500 }) {
    this.onSnapshot = onSnapshot;
    this.pollMs = 1000 / pollHz;
    this.watchdogMs = watchdogMs;
    this.source = null;
    this.pollTimer = null;
    this.failures = 0;
    this.lastData = 0;
    // What the HUD reports. `note` explains a fallback rather than leaving the
    // user with a bare "disconnected" and nothing to act on.
    this.status = { connected: false, via: "starting", note: "" };
  }

  start() {
    // ?poll=1 forces the fallback. A deliberate diagnostic: if the page works
    // this way and not otherwise, the problem is the event stream and nothing
    // else, which takes one reload to establish instead of an afternoon.
    const forced = new URLSearchParams(location.search).get("poll") === "1";
    if (forced || typeof EventSource === "undefined") {
      this.status.note = forced ? "forced by ?poll=1" : "no EventSource in this browser";
      this._startPolling();
      return;
    }
    this._connectStream();
    setInterval(() => this._watchdog(), 500);
  }

  _connectStream() {
    this.status.via = "/state";
    this.lastData = Date.now();
    const src = new EventSource("/state");
    this.source = src;
    src.onopen = () => { this.failures = 0; };
    src.onmessage = (ev) => {
      this.lastData = Date.now();
      this.status.connected = true;
      this.failures = 0;
      try {
        this.onSnapshot(JSON.parse(ev.data));
      } catch (_) {
        // A partial frame is normal at the moment a connection drops; the next
        // one arrives in 33 ms. Not worth reporting.
      }
    };
    src.onerror = () => {
      this.status.connected = false;
      // EventSource reconnects on its own, which is what we want while the sim
      // is restarting. Only persistent failure justifies giving up on it.
      if (++this.failures >= 4) {
        this.status.note = "event stream failed 4 times";
        this._fallback();
      }
    };
  }

  _watchdog() {
    if (this.pollTimer) return;
    if (Date.now() - this.lastData < this.watchdogMs) return;
    this.status.connected = false;
    this.status.note = `no frame for ${(this.watchdogMs / 1000).toFixed(1)} s`;
    this._fallback();
  }

  _fallback() {
    if (this.source) {
      this.source.close();
      this.source = null;
    }
    this._startPolling();
  }

  _startPolling() {
    if (this.pollTimer) return;
    this.status.via = "/snapshot (polling)";
    this.pollTimer = setInterval(() => {
      fetch("/snapshot", { cache: "no-store" })
        .then((r) => r.json())
        .then((snap) => {
          this.status.connected = true;
          this.onSnapshot(snap);
        })
        .catch(() => { this.status.connected = false; });
    }, this.pollMs);
  }
}
