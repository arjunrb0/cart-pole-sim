/**
 * benchmark.js — run controllers through identical, repeatable scenarios.
 *
 * Live disturbances (your taps and drags) are never quite the same twice,
 * so they can't settle an argument about which controller is better. Here
 * every controller gets exactly the same starting state, the same
 * disturbance, the same RK4 physics at the same 500 Hz, and the same motor
 * limit. The only thing that differs is the control law.
 *
 * Pure computation, no DOM: the in-app comparison and the Node test scripts
 * run exactly this code.
 */

import { derivative, failureReason, forceAtTip, saturate, NO_EXTERNAL, X, THETA } from "../physics/cartPole.js";
import { rk4Step } from "../physics/integrator.js";
import { inBand, settlingTime } from "./settling.js";

// Same 50 ms horizontal pulse at the tip as a click in the app
// (ui/interaction.js), so "tap 0.05 N s" means the same thing in both.
const TAP_DURATION = 0.05;

export const SCENARIOS = [
  { id: "tilt", label: "Released at 8.6°", initial: [0, 0, 0.15, 0] },
  { id: "tap", label: "Tap, 0.05 N·s", tapImpulse: 0.05 },
  { id: "step", label: "Move target 0 → 1 m", targetX: 1 },
  { id: "hardTap", label: "Hard tap, 0.12 N·s", tapImpulse: 0.12 },
];

/**
 * @param makeController  (target) => controller. A FRESH controller per run,
 *                        so integral terms from one run can't leak into the next.
 * @returns { times, theta (deg), x, u, settling, maxTiltDeg, maxTravel,
 *            peakForce, failure }
 */
export function runScenario(makeController, scenario, { duration = 20, dt = 1 / 500 } = {}) {
  const target = { x: scenario.targetX ?? 0 };
  const controller = makeController(target);
  let state = (scenario.initial ?? [0, 0, 0, 0]).slice();

  const tapSteps = scenario.tapImpulse ? Math.round(TAP_DURATION / dt) : 0;
  const tapFx = scenario.tapImpulse ? scenario.tapImpulse / TAP_DURATION : 0;

  const steps = Math.round(duration / dt);
  const times = new Float64Array(steps + 1);
  const theta = new Float64Array(steps + 1);
  const x = new Float64Array(steps + 1);
  const u = new Float64Array(steps + 1);
  const settled = new Array(steps + 1);

  let failure = null;
  let n = 0;
  const record = (t, force) => {
    times[n] = t;
    theta[n] = (state[THETA] * 180) / Math.PI;
    x[n] = state[X];
    u[n] = force;
    settled[n] = inBand(state, target.x);
    n++;
  };

  let force = 0;
  for (let i = 0; i < steps; i++) {
    force = saturate(controller.update(state, dt));
    record(i * dt, force);
    const tapping = i < tapSteps;
    const f = (s, uu) => derivative(s, uu, tapping ? forceAtTip(s, tapFx, 0) : NO_EXTERNAL);
    state = rk4Step(f, state, force, dt);
    failure = failureReason(state);
    if (failure) break;
  }
  // Final state. Its force is the one still being held (zero-order hold);
  // recording 0 here drew a fake drop at the end of every trace.
  record(n * dt, force);

  const used = (arr) => arr.subarray(0, n);
  const maxAbs = (arr) => used(arr).reduce((m, v) => Math.max(m, Math.abs(v)), 0);

  return {
    times: used(times),
    theta: used(theta),
    x: used(x),
    u: used(u),
    settling: failure ? Infinity : settlingTime(settled.slice(0, n), times),
    maxTiltDeg: maxAbs(theta),
    maxTravel: maxAbs(x),
    peakForce: maxAbs(u),
    failure,
  };
}

/** Every scenario, for every named controller factory. */
export function runComparison(factories, options) {
  return SCENARIOS.map((scenario) => ({
    scenario,
    results: Object.fromEntries(Object.entries(factories).map(([name, make]) => [name, runScenario(make, scenario, options)])),
  }));
}
