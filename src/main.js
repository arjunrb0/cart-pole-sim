/**
 * main.js — the only file allowed to know about all the other modules.
 *
 * It owns the clock, the simulation's lifecycle (running / paused /
 * failed), and which controller is in charge. Each physics step:
 *
 *     active controller decides force -> motor saturates -> integrate (with
 *     any external forces: your hand, a tap, wind) -> check for failure
 */

import { derivative, failureReason, forceAtTip, initialState, linearize, params, saturate, X, THETA } from "./physics/cartPole.js";
import { rk4Step } from "./physics/integrator.js";
import { createPidController, defaultPidGains } from "./control/pid.js";
import { createLqrController, defaultLqrTolerances } from "./control/lqr.js";
import { eigenvalues, matMul, sub } from "./control/linalg.js";
import { createSettlingMeter } from "./analysis/settling.js";
import { createRenderer } from "./render/renderer.js";
import { createPlot } from "./render/plot.js";
import { createInteraction } from "./ui/interaction.js";
import { createPanel } from "./ui/controls.js";
import { createComparison } from "./ui/comparison.js";

// Physics runs at a FIXED timestep, independent of the screen refresh rate.
// The accumulator banks real elapsed time and spends it in exact chunks.
const PHYSICS_DT = 1 / 500; // s
const MAX_FRAME_TIME = 0.1; // never simulate more than 100 ms in one frame
const PLOT_EVERY = 5; // record every 5th step -> 100 Hz on the plot
const RESET_TILT = 0.15; // rad: start each run slightly off vertical

// Things in the world that aren't the plant or the controller.
const environment = {
  wind: 0, // constant force on the cart, N (not the motor, so not saturated)
  tapImpulse: 0.05, // N s delivered by one click
};

// --- controllers ------------------------------------------------------------
// One setpoint object shared by both, so switching keeps the same target.
const target = { x: 0 };

// Kleinman's iteration must start from a gain that already stabilises the
// plant. The DEFAULT PID gains do; the live ones might not (you can drag
// them anywhere), so always seed from the defaults.
const seedGain = () => [createPidController().linearGain()];

const controllers = {
  PID: createPidController(defaultPidGains(), target),
  LQR: createLqrController({ target, initialGain: seedGain }),
};
let active = controllers.PID;

// --- modules ---------------------------------------------------------------
const sceneCanvas = document.getElementById("scene");
const renderer = createRenderer(sceneCanvas);
const plot = createPlot(document.getElementById("plot"));
const meter = createSettlingMeter();

// --- simulation state ------------------------------------------------------
let state = initialState(RESET_TILT);
let simTime = 0;
let stepCount = 0;
let lastForce = 0;
let paused = false;
let failure = null; // null, or a string saying why the run ended

const interaction = createInteraction(sceneCanvas, {
  renderer,
  environment,
  isActive: () => !paused && !failure,
  getState: () => state,
  onDisturb: () => meter.disturb(simTime),
});

function reset() {
  state = initialState(RESET_TILT);
  active.reset();
  interaction.release();
  plot.clear();
  meter.disturb(0);
  simTime = 0;
  stepCount = 0;
  lastForce = 0;
  failure = null;
  paused = false;
}

function step(dt) {
  // Time-dependent force: constant over this step.
  const tapFx = interaction.takeTapForce(dt);

  // The derivative RK4 integrates: the plant's equations plus every
  // external force. The spring is evaluated HERE, per RK4 probe, because it
  // depends on where the tip is at that probe. rk4Step itself is unchanged
  // from Stage 2: it still just sees a function f(state, u).
  const f = (s, u) => {
    const hand = interaction.springForce(s);
    const q = forceAtTip(s, hand.fx + tapFx, hand.fy);
    return derivative(s, u, { qx: q.qx + environment.wind, qTheta: q.qTheta });
  };

  lastForce = saturate(active.update(state, dt));
  state = rk4Step(f, state, lastForce, dt);
  simTime += dt;
  meter.sample(simTime, state, target.x);

  if (++stepCount % PLOT_EVERY === 0) {
    plot.push(simTime, { theta: (state[THETA] * 180) / Math.PI, x: state[X], u: lastForce });
  }

  failure = failureReason(state);
  if (failure) interaction.release();
}

// --- DOM: readouts and buttons --------------------------------------------
const $ = (id) => document.getElementById(id);
const statusEl = $("status");
const settleEl = $("ro-settle");
const readouts = { t: $("ro-t"), x: $("ro-x"), theta: $("ro-theta"), u: $("ro-u") };
const pauseButton = $("btn-pause");
const switchButtons = [...document.querySelectorAll("[data-controller]")];

function togglePause() {
  if (failure) return;
  paused = !paused;
}

function useController(name) {
  if (controllers[name] !== active) {
    active = controllers[name];
    active.reset(); // no stale integral from the last time it was in charge
    meter.disturb(simTime);
  }
  for (const b of switchButtons) b.setAttribute("aria-pressed", String(b.dataset.controller === name));
  for (const g of pidPanel.groups) g.hidden = name !== "PID";
  for (const g of lqrPanel.groups) g.hidden = name !== "LQR";
}

pauseButton.addEventListener("click", togglePause);
$("btn-reset").addEventListener("click", reset);
$("btn-defaults").addEventListener("click", () => {
  // Object.assign copies values INTO the existing objects. Replacing the
  // objects would leave the controllers holding references to the old ones.
  const d = defaultPidGains();
  Object.assign(controllers.PID.gains.angle, d.angle);
  Object.assign(controllers.PID.gains.position, d.position);
  Object.assign(controllers.LQR.tolerances, defaultLqrTolerances());
});
for (const b of switchButtons) b.addEventListener("click", () => useController(b.dataset.controller));

window.addEventListener("keydown", (event) => {
  // Don't steal keys from a focused button, slider, tab or table row.
  if (event.target.closest?.("button, input, [tabindex]")) return;
  if (event.code === "Space") {
    event.preventDefault(); // otherwise Space scrolls the page
    togglePause();
  } else if (event.code === "KeyR" && !event.metaKey && !event.ctrlKey) {
    reset();
  }
});

function updateReadouts() {
  readouts.t.textContent = simTime.toFixed(2);
  readouts.x.textContent = state[X].toFixed(3);
  readouts.theta.textContent = ((state[THETA] * 180) / Math.PI).toFixed(2);
  readouts.u.textContent = lastForce.toFixed(1);

  const status = failure ? "failed" : paused ? "paused" : "running";
  statusEl.dataset.status = status;
  statusEl.textContent = failure ? "Failed" : paused ? "Paused" : "Running";
  pauseButton.textContent = paused ? "Resume" : "Pause";
  pauseButton.disabled = !!failure;

  const s = meter.status(simTime);
  settleEl.textContent = failure ? "—" : s.settled ? `settled in ${s.time.toFixed(2)} s` : `settling… ${s.elapsed.toFixed(1)} s`;
  settleEl.dataset.settled = String(!failure && s.settled);
}

// --- slider panels ------------------------------------------------------------
const panelRoot = $("panel");
const g = controllers.PID.gains;
const pidPanel = createPanel(panelRoot, [
  {
    title: "PID · angle loop (inner, fast)",
    note: "Keeps the pole upright. Error = θ (rad).",
    sliders: [
      { obj: g.angle, key: "kp", label: "Kp", unit: "N/rad", min: 0, max: 150, step: 0.5, hint: "Must exceed 10.8 to beat gravity" },
      { obj: g.angle, key: "ki", label: "Ki", unit: "N/(rad·s)", min: 0, max: 20, step: 0.1, hint: "Adds lag; rarely helps here" },
      { obj: g.angle, key: "kd", label: "Kd", unit: "N·s/rad", min: 0, max: 20, step: 0.1, hint: "Damping. At 0, unstable (the cart loop eats it)" },
    ],
  },
  {
    title: "PID · position loop (outer, slow)",
    note: "Keeps the cart on the track. Error = x − target (m).",
    sliders: [
      { obj: g.position, key: "kp", label: "Kp", unit: "N/m", min: -5, max: 15, step: 0.1, hint: "Must be positive. Try negative" },
      { obj: g.position, key: "ki", label: "Ki", unit: "N/(m·s)", min: 0, max: 5, step: 0.05, hint: "Removes the wind offset" },
      { obj: g.position, key: "kd", label: "Kd", unit: "N·s/m", min: -5, max: 15, step: 0.1, hint: "Must be positive too" },
    ],
  },
]);

const tol = controllers.LQR.tolerances;
const lqrPanel = createPanel(panelRoot, [
  {
    title: "LQR · tolerances (Bryson's rule)",
    note: "How much of each you'd accept. Weight = 1 / tolerance². Smaller = cares more. Prices, not hard limits.",
    sliders: [
      { obj: tol, key: "x", label: "Cart position", unit: "m", min: 0.05, max: 1, step: 0.01 },
      { obj: tol, key: "xDot", label: "Cart speed", unit: "m/s", min: 0.1, max: 3, step: 0.05 },
      { obj: tol, key: "thetaDeg", label: "Pole angle", unit: "°", min: 1, max: 45, step: 0.5 },
      { obj: tol, key: "thetaDotDeg", label: "Pole rate", unit: "°/s", min: 10, max: 360, step: 5 },
      { obj: tol, key: "force", label: "Motor force", unit: "N", min: 0.5, max: 40, step: 0.5, hint: "Smaller = gentler, slower controller" },
    ],
  },
]);

const commonPanel = createPanel(panelRoot, [
  {
    title: "Setpoint & environment",
    sliders: [
      { obj: target, key: "x", label: "Target x", unit: "m", min: -2, max: 2, step: 0.05, hint: "Watch for the wrong-way dip" },
      { obj: environment, key: "wind", label: "Wind on cart", unit: "N", min: -5, max: 5, step: 0.1, hint: "Constant push, bypasses the motor. LQR has no integral term" },
      { obj: environment, key: "tapImpulse", label: "Tap strength", unit: "N·s", min: 0.01, max: 0.3, step: 0.01 },
    ],
  },
]);

// "Copy into PID": LQR's K is the same four-gain structure as PD + PD, so
// loading it into the PID sliders reproduces LQR exactly (integrals off).
const copyButton = document.createElement("button");
copyButton.type = "button";
copyButton.className = "copy-gains";
copyButton.textContent = "Copy these gains into PID";
copyButton.addEventListener("click", () => {
  const K = controllers.LQR.linearGain();
  Object.assign(g.position, { kp: -K[0], kd: -K[1], ki: 0 });
  Object.assign(g.angle, { kp: -K[2], kd: -K[3], ki: 0 });
  useController("PID");
});
const copyHint = document.createElement("span");
copyHint.className = "slider-hint";
copyHint.textContent = "PD on angle + PD on position is the same structure as LQR, so PID with these gains behaves identically.";
lqrPanel.groups[0].append(copyButton, copyHint);

// --- linear analysis of whichever controller is active -------------------
// Built once; its text is refreshed only when the gains actually change.
const analysis = document.createElement("section");
analysis.className = "group analysis";
analysis.innerHTML = `
  <h2 class="group-title">Linear analysis · <span data-a="name"></span></h2>
  <p class="group-note">From the linearised model, u = −Kx. <span data-a="caveat"></span></p>
  <dl>
    <dt>Gains as PID</dt><dd data-a="gains"></dd>
    <dt>Closed-loop poles</dt><dd data-a="poles"></dd>
    <dt>Slowest decay σ</dt><dd data-a="sigma"></dd>
    <dt>Settling ≈ 4/σ</dt><dd data-a="estimate"></dd>
    <dt data-a="solverLabel">Solver</dt><dd data-a="solver"></dd>
  </dl>`;
panelRoot.insertBefore(analysis, commonPanel.groups[0]);
const field = (name) => analysis.querySelector(`[data-a="${name}"]`);

const fmtPole = (p) => {
  const re = p.re.toFixed(2).replace("-", "−");
  return p.im === 0 ? re : `${re} ± ${Math.abs(p.im).toFixed(2)}j`;
};
let analysisKey = "";
function updateAnalysis() {
  const name = active.name;
  const K = active.linearGain();
  const key = name + K.join(",");
  if (key === analysisKey) return;
  analysisKey = key;

  const { A, B } = linearize();
  const poles = eigenvalues(sub(A, matMul(B, [K])));
  // Show each complex-conjugate pair once.
  const shown = poles.filter((p) => p.im >= 0);
  const sigma = -poles[0].re; // poles are sorted rightmost first
  const hasIntegral = name === "PID" && (g.angle.ki > 0 || g.position.ki > 0);

  field("name").textContent = name;
  field("caveat").textContent = hasIntegral ? "Integral terms are left out of this analysis." : "";
  field("gains").textContent =
    `angle kp ${(-K[2]).toFixed(1)}, kd ${(-K[3]).toFixed(1)} · position kp ${(-K[0]).toFixed(1)}, kd ${(-K[1]).toFixed(1)}`;
  field("poles").textContent = shown.map(fmtPole).join(",  ");
  field("sigma").textContent = sigma > 0 ? `${sigma.toFixed(3)} s⁻¹` : "unstable (a pole has Re ≥ 0)";
  field("estimate").textContent = sigma > 0 ? `${(4 / sigma).toFixed(1)} s (rule of thumb, 2% band)` : "—";

  const info = name === "LQR" ? controllers.LQR.info : null;
  field("solverLabel").hidden = field("solver").hidden = !info;
  if (info) {
    field("solver").textContent = info.error
      ? `error: ${info.error}`
      : `Kleinman, ${info.iterations} iterations, Riccati residual ${info.residual.toExponential(1)}`;
  }
}

// --- PID vs LQR comparison -------------------------------------------------
// Snapshot the CURRENT settings at the moment "Run" is pressed.
const comparison = createComparison($("comparison"), {
  snapshot: () => {
    const pidGains = structuredClone(controllers.PID.gains);
    const lqrTolerances = { ...controllers.LQR.tolerances };
    return {
      PID: (t) => createPidController(structuredClone(pidGains), t),
      LQR: (t) => createLqrController({ tolerances: lqrTolerances, target: t, initialGain: seedGain }),
    };
  },
});

// --- the loop ---------------------------------------------------------------
let lastTime = performance.now() / 1000;
let accumulator = 0;
let lastTargetX = target.x;
let lastWind = environment.wind;

function frame(nowMs) {
  const now = nowMs / 1000;
  // Clamp: after the tab was backgrounded, don't try to catch up on
  // seconds of physics in one frame.
  const frameTime = Math.min(now - lastTime, MAX_FRAME_TIME);
  lastTime = now;

  // A new target or a change of wind is a disturbance as far as settling
  // is concerned: time the recovery from here.
  if (target.x !== lastTargetX || environment.wind !== lastWind) {
    lastTargetX = target.x;
    lastWind = environment.wind;
    meter.disturb(simTime);
  }

  if (paused || failure) {
    accumulator = 0;
  } else {
    accumulator += frameTime;
    while (accumulator >= PHYSICS_DT && !failure) {
      step(PHYSICS_DT);
      accumulator -= PHYSICS_DT;
    }
  }

  const iv = interaction.view;
  renderer.draw(state, {
    force: lastForce,
    targetX: target.x,
    wind: environment.wind,
    anchor: iv.anchor,
    hoverTip: iv.hoverTip,
    tap: iv.tap,
    overlay: failure
      ? { title: failure, subtitle: "Press R or click Reset to try again" }
      : paused
        ? { title: "Paused", subtitle: "Space to resume" }
        : null,
  });
  plot.draw({ x: target.x });
  updateReadouts();
  updateAnalysis();
  // Keep sliders truthful if anything else changed a value.
  pidPanel.refresh();
  lqrPanel.refresh();
  commonPanel.refresh();

  requestAnimationFrame(frame);
}

useController("PID");
comparison.run();
requestAnimationFrame(frame);

// In development builds, expose the live objects for console poking.
// `import.meta.env.DEV` is a Vite feature: false in production builds.
if (import.meta.env.DEV) {
  window.sim = {
    controllers,
    get controller() {
      return active;
    },
    useController,
    target,
    params,
    environment,
    reset,
    get state() {
      return state;
    },
  };
}
