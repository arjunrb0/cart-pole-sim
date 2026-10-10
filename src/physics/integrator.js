/**
 * integrator.js — how we march the state forward in time.
 *
 * The integrator is deliberately GENERIC: it takes any function
 *
 *     f(state, u) -> dState/dt
 *
 * and advances `state` by `dt`. It has no idea it is simulating a pendulum.
 * That separation is what lets Stage 2 run the exact same cart-pole physics
 * through Euler and through RK4 and compare the error side by side.
 *
 * If you know scipy: `f` here plays the role of the `fun` argument to
 * solve_ivp, and this file is a hand-rolled fixed-step version of it.
 */

/** Small array helpers — JS has no numpy, so vector maths is explicit. */
export function addScaled(a, b, k) {
  // returns a + k*b, elementwise
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + k * b[i];
  return out;
}

/**
 * Forward Euler — the simplest possible integrator.
 * Kept around so Stage 2 can demonstrate exactly how it fails.
 */
export function eulerStep(f, state, u, dt) {
  const k1 = f(state, u);
  return addScaled(state, k1, dt);
}

/**
 * Classical 4th-order Runge-Kutta.
 *
 * Euler asks "how fast am I moving right now?" once, and walks in a straight
 * line. On a curved trajectory the tangent always lands OUTSIDE the true
 * path, every step, in the same direction -- so for a pendulum Euler
 * steadily manufactures energy until the thing spins itself apart.
 *
 * RK4 probes the slope four times and takes a weighted average:
 *
 *   k1 - slope at the start
 *   k2 - slope at the midpoint, having stepped there using k1
 *   k3 - slope at the midpoint again, this time using the better k2
 *   k4 - slope at the far end, using k3
 *
 * The midpoints carry double weight because they best represent the average
 * behaviour across the interval; they are also what lets RK4 sense the
 * curvature Euler is blind to.
 *
 * Global error: O(dt^4) versus Euler's O(dt). Halving dt makes Euler 2x
 * better and RK4 16x better. Cost is 4 derivative evaluations per step.
 *
 * `u` is held constant across the four probes -- a zero-order hold, which is
 * exactly what a real digital controller does between sample instants.
 */
export function rk4Step(f, state, u, dt) {
  const k1 = f(state, u);
  const k2 = f(addScaled(state, k1, dt / 2), u);
  const k3 = f(addScaled(state, k2, dt / 2), u);
  const k4 = f(addScaled(state, k3, dt), u);

  const out = new Array(state.length);
  for (let i = 0; i < state.length; i++) {
    out[i] = state[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
  return out;
}
