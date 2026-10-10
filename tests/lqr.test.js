/**
 * Stage 5: LQR. The Riccati solution is checked against an independent
 * method, its cost prediction against simulation, and its optimality by
 * trying to beat it.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import { derivative, linearize, saturate } from "../src/physics/cartPole.js";
import { rk4Step } from "../src/physics/integrator.js";
import { createPidController, defaultPidGains } from "../src/control/pid.js";
import {
  brysonWeights,
  controllabilityMatrix,
  createLqrController,
  defaultLqrTolerances,
  solveLqr,
  solveLyapunov,
} from "../src/control/lqr.js";
import { add, eigenvalues, matMul, maxAbs, rank, scale, sub, transpose } from "../src/control/linalg.js";
import { DT, quadForm, rng, seedGain } from "./helpers.js";

const { A, B } = linearize();
const { Q, R } = brysonWeights(defaultLqrTolerances());
const sol = solveLqr(A, B, Q, R, seedGain());
const K = sol.K[0];

test("the cart-pole is controllable: rank [B AB A²B A³B] = 4", () => {
  assert.equal(rank(controllabilityMatrix(A, B)), 4);
});

test("Kleinman's iteration solves the Riccati equation", () => {
  assert.ok(sol.residual < 1e-9, `residual ${sol.residual}`);
  assert.ok(sol.iterations < 20);
  assert.ok(maxAbs(sub(sol.P, transpose(sol.P))) < 1e-12, "P should be symmetric");
  assert.ok(eigenvalues(sol.P).every((e) => e.re > 0), "P should be positive definite");
  assert.ok(eigenvalues(sub(A, matMul(B, sol.K))).every((e) => e.re < 0), "closed loop should be stable");
});

test("independent check: the backward Riccati ODE converges to the same P", () => {
  const ric = (P) => {
    const PB = matMul(P, B);
    return add(sub(add(matMul(transpose(A), P), matMul(P, A)), scale(matMul(PB, transpose(PB)), 1 / R)), Q);
  };
  let P = A.map((row) => row.map(() => 0));
  const h = 1e-3;
  for (let t = 0; t < 60; t += h) {
    const k1 = ric(P);
    const k2 = ric(add(P, scale(k1, h / 2)));
    const k3 = ric(add(P, scale(k2, h / 2)));
    const k4 = ric(add(P, scale(k3, h)));
    P = add(P, scale(add(add(k1, scale(k2, 2)), add(scale(k3, 2), k4)), h / 6));
  }
  assert.ok(maxAbs(sub(P, sol.P)) / maxAbs(sol.P) < 1e-9);
});

test("x0'Px0 predicts the cost the closed loop actually accumulates", () => {
  const Acl = sub(A, matMul(B, sol.K));
  for (const x0 of [[0, 0, 0.15, 0], [0.5, 0, 0, 0], [0.2, -0.3, 0.05, 0.4]]) {
    // Augmented state [x, cost]: integrate dynamics and running cost together.
    const g = (z) => {
      const x = z.slice(0, 4);
      const u = -K.reduce((a, k, j) => a + k * x[j], 0);
      const dx = Acl.map((row) => row.reduce((a, v, j) => a + v * x[j], 0));
      return [...dx, quadForm(x, Q) + R * u * u];
    };
    let z = [...x0, 0];
    const h = 1e-3;
    for (let t = 0; t < 40; t += h) {
      const k1 = g(z);
      const k2 = g(z.map((v, i) => v + (h / 2) * k1[i]));
      const k3 = g(z.map((v, i) => v + (h / 2) * k2[i]));
      const k4 = g(z.map((v, i) => v + h * k3[i]));
      z = z.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    }
    const predicted = quadForm(x0, sol.P);
    assert.ok(Math.abs(z[4] - predicted) / predicted < 1e-6, `${z[4]} vs ${predicted}`);
  }
});

test("optimality: no nearby gain is cheaper", () => {
  const x0 = [0, 0, 0.15, 0];
  const costOf = (k) => {
    const Acl = sub(A, matMul(B, [k]));
    if (!eigenvalues(Acl).every((e) => e.re < 0)) return Infinity;
    return quadForm(x0, solveLyapunov(Acl, add(Q, scale(matMul(transpose([k]), [k]), R))));
  };
  const best = costOf(K);
  const rand = rng(42);
  for (let i = 0; i < 300; i++) {
    const k = K.map((v) => v * (1 + (rand() - 0.5) * 0.2));
    assert.ok(costOf(k) >= best - 1e-12);
  }
  // The hand-tuned PID, by LQR's own cost, is clearly worse.
  assert.ok(costOf(createPidController().linearGain()) > 2 * best);
});

test("guaranteed gain margin: stable for every gain scaled by 1/2 up to 1000", () => {
  for (const k of [0.5, 1, 10, 1000]) {
    assert.ok(eigenvalues(sub(A, matMul(B, [K.map((v) => v * k)]))).every((e) => e.re < 0), `unstable at x${k}`);
  }
});

test("position gain identity: |Kx| = sqrt(qx / R) = force tolerance / position tolerance", () => {
  const t = defaultLqrTolerances();
  assert.ok(Math.abs(Math.abs(K[0]) - t.force / t.x) < 1e-9);
});

test("PID loaded with LQR's gains IS the LQR controller (same trajectory)", () => {
  const g = defaultPidGains();
  Object.assign(g.position, { kp: -K[0], kd: -K[1], ki: 0 });
  Object.assign(g.angle, { kp: -K[2], kd: -K[3], ki: 0 });
  const pid = createPidController(g);
  const lqr = createLqrController({ initialGain: seedGain });
  let a = [0, 0, 0.15, 0];
  let b = a.slice();
  for (let i = 0; i < 10 / DT; i++) {
    a = rk4Step(derivative, a, saturate(pid.update(a, DT)), DT);
    b = rk4Step(derivative, b, saturate(lqr.update(b, DT)), DT);
  }
  assert.ok(Math.max(...a.map((v, j) => Math.abs(v - b[j]))) < 1e-12);
});

test("solveLqr refuses a non-stabilising starting gain", () => {
  assert.throws(() => solveLqr(A, B, Q, R, [[0, 0, 0, 0]]), /does not stabilise/);
});

test("the LQR controller re-solves when a tolerance changes", () => {
  const c = createLqrController({ initialGain: seedGain });
  const before = c.linearGain();
  c.tolerances.force = 20;
  const after = c.linearGain();
  assert.ok(Math.abs(after[0]) > Math.abs(before[0]) * 5, "bigger force budget should mean stiffer gains");
});
