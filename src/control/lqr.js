/**
 * lqr.js — the Linear Quadratic Regulator.
 *
 * Minimises the cost
 *
 *     J = integral of ( x' Q x + R u^2 ) dt
 *
 * for the linearised plant  dx/dt = A x + B u. The answer is linear state
 * feedback  u = -K x  with  K = R^-1 B' P, where P solves the algebraic
 * Riccati equation
 *
 *     A' P + P A - P B R^-1 B' P + Q = 0.
 *
 * Solved here by Kleinman's iteration (Newton's method on the Riccati
 * equation, and also exactly "policy iteration"):
 *
 *     evaluate:  solve  (A-BK)' P + P (A-BK) + Q + K' R K = 0  for P
 *     improve:   K <- R^-1 B' P
 *
 * Each "evaluate" is a Lyapunov equation, which is LINEAR in P.
 */

import { X, X_DOT, THETA, THETA_DOT, linearize } from "../physics/cartPole.js";
import { add, diag, eigenvalues, matMul, maxAbs, rank, scale, solve, sub, transpose, zeros } from "./linalg.js";

/**
 * Solve the Lyapunov equation  Acl' P + P Acl + C = 0  for symmetric P.
 *
 * Entry (i, j) of Acl' P + P Acl is  sum_k ( Acl[k][i] P[k][j] + P[i][k] Acl[k][j] ),
 * which is linear in the n^2 unknowns P[.][.]. Stack them into a vector
 * ("vectorising" P), build the n^2 x n^2 coefficient matrix, and solve.
 * For n = 4 that's a 16 x 16 system.
 *
 * The solution exists and is unique whenever Acl is stable.
 */
export function solveLyapunov(Acl, C) {
  const n = Acl.length;
  const N = n * n;
  const idx = (i, j) => i * n + j;
  const M = zeros(N, N);
  const rhs = new Array(N);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      const row = idx(i, j);
      for (let k = 0; k < n; k++) {
        M[row][idx(k, j)] += Acl[k][i];
        M[row][idx(i, k)] += Acl[k][j];
      }
      rhs[row] = -C[i][j];
    }
  const p = solve(M, rhs);
  const P = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) P[i][j] = p[idx(i, j)];
  // Round-off makes P very slightly asymmetric; symmetrise.
  return scale(add(P, transpose(P)), 0.5);
}

/** [B, AB, A^2 B, ..., A^(n-1) B] — full rank means every state is reachable. */
export function controllabilityMatrix(A, B) {
  const n = A.length;
  const cols = [];
  let AkB = B;
  for (let k = 0; k < n; k++) {
    cols.push(AkB.map((row) => row[0]));
    AkB = matMul(A, AkB);
  }
  return transpose(cols);
}

const isStable = (M) => eigenvalues(M).every((e) => e.re < 0);

/**
 * Continuous-time LQR for a single input.
 *
 * @param A   n x n
 * @param B   n x 1
 * @param Q   n x n, symmetric positive (semi)definite
 * @param R   scalar > 0
 * @param K0  1 x n STABILISING initial gain (A - B K0 has all poles in the
 *            left half-plane). Kleinman's method needs one to start from.
 * @returns   { K (1 x n), P (n x n), iterations, residual }
 */
export function solveLqr(A, B, Q, R, K0, { tol = 1e-12, maxIter = 50 } = {}) {
  const n = A.length;
  if (rank(controllabilityMatrix(A, B)) < n) {
    throw new Error("LQR: system is not controllable");
  }
  if (!isStable(sub(A, matMul(B, K0)))) {
    throw new Error("LQR: initial gain K0 does not stabilise the system");
  }

  const Bt = transpose(B);
  let K = K0;
  let P = null;
  let iterations = 0;

  for (; iterations < maxIter; iterations++) {
    // Evaluate: what does the current controller cost?
    const Acl = sub(A, matMul(B, K));
    const C = add(Q, scale(matMul(transpose(K), K), R));
    P = solveLyapunov(Acl, C);

    // Improve: the gain that is greedy with respect to that cost.
    const Knext = scale(matMul(Bt, P), 1 / R);
    const change = maxAbs(sub(Knext, K)) / Math.max(1, maxAbs(Knext));
    K = Knext;
    if (change < tol) {
      iterations++;
      break;
    }
  }

  // How well does P satisfy the Riccati equation? Should be ~1e-12.
  const PB = matMul(P, B);
  const riccati = add(
    sub(add(matMul(transpose(A), P), matMul(P, A)), scale(matMul(PB, transpose(PB)), 1 / R)),
    Q,
  );
  return { K, P, iterations, residual: maxAbs(riccati) };
}

/**
 * Bryson's rule: weight each quantity by 1 / (largest acceptable value)^2,
 * so every term in the cost is ~1 when that quantity reaches its limit.
 * Tolerances are in friendly units (degrees); converted to SI here.
 */
export function brysonWeights(t) {
  const rad = Math.PI / 180;
  const Q = diag([
    1 / t.x ** 2,
    1 / t.xDot ** 2,
    1 / (t.thetaDeg * rad) ** 2,
    1 / (t.thetaDotDeg * rad) ** 2,
  ]);
  const R = 1 / t.force ** 2;
  return { Q, R };
}

/**
 * Default tolerances, picked from a sweep of the benchmark scenarios rather
 * than derived. Criteria: stay under the 40 N motor limit for the tilt, tap
 * and target-step scenarios; survive the hard tap; and give gains that fit
 * the PID sliders, so "copy LQR gains into PID" works.
 *
 * These are PRICES, not limits. Bryson's rule makes each cost term ~1 when
 * its quantity reaches its tolerance, but the controller will still exceed
 * a tolerance whenever that is the cheapest way to cut the TOTAL cost. A
 * 3 N "force tolerance" can still produce a 20 N push after a tap.
 *
 * The first guess (3° of tilt, 15 N) produced gains near 400 N/rad that
 * sat at the motor limit in every scenario: the linear theory knows nothing
 * about saturation, so tolerances have to be checked against the real plant.
 */
export function defaultLqrTolerances() {
  return {
    x: 0.25, // m of cart error
    xDot: 1.0, // m/s of cart speed
    thetaDeg: 20, // degrees of tilt
    thetaDotDeg: 90, // degrees/s of tilt rate
    force: 3, // N of motor effort
  };
}

/**
 * The LQR controller, with the same interface as the PID one:
 *
 *     update(state, dt) -> force
 *
 * `tolerances` is read live: change one and K is recomputed before the next
 * physics step. `initialGain` must stabilise the plant (the Stage 3 PID
 * gains do).
 */
export function createLqrController({ tolerances = defaultLqrTolerances(), target = { x: 0 }, initialGain }) {
  let solved = null; // { K, P, iterations, residual, poles }
  let solvedFor = ""; // the tolerances the current K was computed for
  let error = null;

  function recompute() {
    const key = JSON.stringify(tolerances);
    if (key === solvedFor) return;
    solvedFor = key;
    try {
      const { A, B } = linearize();
      const { Q, R } = brysonWeights(tolerances);
      const result = solveLqr(A, B, Q, R, initialGain());
      const poles = eigenvalues(sub(A, matMul(B, result.K)));
      solved = { ...result, poles };
      error = null;
    } catch (e) {
      // Keep the last good K rather than leaving the pole uncontrolled.
      error = e.message;
    }
  }
  recompute();

  return {
    name: "LQR",
    tolerances,
    target,

    update(state) {
      recompute();
      const K = solved.K[0];
      // Regulate the error from the target, not the raw state. Because
      // column 0 of A is zero, every cart position is an equilibrium, so
      // shifting x by the target is all a setpoint needs.
      const e = [state[X] - target.x, state[X_DOT], state[THETA], state[THETA_DOT]];
      return -(K[0] * e[0] + K[1] * e[1] + K[2] * e[2] + K[3] * e[3]);
    },

    reset() {},

    /** u = -K x form, for analysis and display. */
    linearGain() {
      recompute();
      return solved.K[0].slice();
    },

    get info() {
      recompute();
      return { ...solved, error };
    },
  };
}
