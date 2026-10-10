/**
 * Shared test helpers. Not a test file itself (no `.test.js` suffix), so the
 * runner doesn't execute it directly.
 */

import { derivative, params, saturate } from "../src/physics/cartPole.js";
import { rk4Step } from "../src/physics/integrator.js";
import { createPidController } from "../src/control/pid.js";

export const DT = 1 / 500;

/** Integrate the plant with a constant force (default 0) for T seconds. */
export function simulate(stepper, state, T, { dt = DT, u = 0, f = derivative } = {}) {
  let s = state.slice();
  const steps = Math.round(T / dt);
  for (let i = 0; i < steps; i++) s = stepper(f, s, u, dt);
  return s;
}

/**
 * Closed-loop run: controller -> motor saturation -> RK4, like the app.
 * `external(s)` may return generalised forces { qx, qTheta } (e.g. wind).
 * `onStep(i, s, controller)` lets a test change gains mid-run.
 */
export function runClosedLoop(controller, state, T, { dt = DT, external, onStep } = {}) {
  let s = state.slice();
  const f = external ? (x, u) => derivative(x, u, external(x)) : derivative;
  for (let i = 0; i < Math.round(T / dt); i++) {
    onStep?.(i, s, controller);
    s = rk4Step(f, s, saturate(controller.update(s, dt)), dt);
  }
  return s;
}

/**
 * Temporarily override physical parameters, always restoring them, so one
 * test can't leak a modified plant into the next.
 */
export function withParams(overrides, fn) {
  const saved = { ...params };
  Object.assign(params, overrides);
  try {
    return fn();
  } finally {
    Object.assign(params, saved);
  }
}

/** Small seeded PRNG (mulberry32) so "random" tests are repeatable. */
export function rng(seed = 12345) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** The stabilising initial gain LQR's Kleinman iteration is seeded with. */
export const seedGain = () => [createPidController().linearGain()];

/** x' M x for a plain array x and matrix M. */
export function quadForm(x, M) {
  return x.reduce((acc, xi, i) => acc + xi * x.reduce((b, xj, j) => b + M[i][j] * xj, 0), 0);
}
