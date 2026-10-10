/**
 * Stage 2 and 4: the equations of motion and the integrator.
 *
 * Each test checks the physics against a fact known INDEPENDENTLY of the
 * derivation (a conservation law, a textbook period, a torque balance), so
 * a passing test means the model is right, not just that it runs.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import {
  derivative,
  forceAtTip,
  linearize,
  params,
  tipPosition,
  tipVelocity,
  totalEnergy,
  X,
  X_DOT,
  THETA,
  THETA_DOT,
} from "../src/physics/cartPole.js";
import { eulerStep, rk4Step } from "../src/physics/integrator.js";
import { DT, rng, simulate, withParams } from "./helpers.js";

const SWINGING = [0, 0, 0.6, 1.5]; // well off vertical with a shove: properly nonlinear

test("RK4 conserves energy of the unforced, frictionless system to ~1e-9", () => {
  withParams({ cartFriction: 0 }, () => {
    const E0 = totalEnergy(SWINGING);
    for (const T of [1, 20, 60]) {
      const drift = Math.abs(totalEnergy(simulate(rk4Step, SWINGING, T)) - E0) / E0;
      assert.ok(drift < 1e-9, `relative energy drift ${drift} after ${T} s`);
    }
  });
});

test("Euler steadily manufactures energy (why the sim uses RK4)", () => {
  withParams({ cartFriction: 0 }, () => {
    const E0 = totalEnergy(SWINGING);
    const drifts = [1, 5, 20, 60].map((T) => (totalEnergy(simulate(eulerStep, SWINGING, T)) - E0) / E0);
    // Always gaining, never losing: the error is biased, not random.
    for (let i = 1; i < drifts.length; i++) assert.ok(drifts[i] > drifts[i - 1]);
    assert.ok(drifts.at(-1) > 0.5, `Euler gained only ${drifts.at(-1)} after 60 s`);
  });
});

test("horizontal momentum is conserved with no applied force (checks the coupling term)", () => {
  withParams({ cartFriction: 0 }, () => {
    const p = (s) => (params.cartMass + params.poleMass) * s[X_DOT] + params.poleMass * params.poleLength * Math.cos(s[THETA]) * s[THETA_DOT];
    const s0 = [0, 0.3, 0.6, 1.5];
    const drift = Math.abs(p(simulate(rk4Step, s0, 60)) - p(s0));
    assert.ok(drift < 1e-9, `momentum drift ${drift}`);
  });
});

test("cart locked: small-swing period matches the compound pendulum 2π√(4l/3g)", () => {
  withParams({ cartMass: 1e9, cartFriction: 0 }, () => {
    let s = [0, 0, Math.PI + 0.01, 0];
    let prev = 0;
    const crossings = [];
    for (let i = 1; crossings.length < 3; i++) {
      s = rk4Step(derivative, s, 0, DT);
      if (prev < 0 !== s[THETA_DOT] < 0) crossings.push(i * DT);
      prev = s[THETA_DOT];
    }
    const analytic = 2 * Math.PI * Math.sqrt((4 * params.poleLength) / (3 * params.gravity));
    const measured = crossings[2] - crossings[0];
    // Tolerance = one timestep of zero-crossing resolution.
    assert.ok(Math.abs(measured - analytic) <= DT * 1.5, `period ${measured} vs ${analytic}`);
  });
});

test("uncontrolled pole falls away from upright, and the cart recoils the other way", () => {
  const s = simulate(rk4Step, [0, 0, 0.15, 0], 0.5);
  assert.ok(s[THETA] > 0.4, "pole should have tipped further right");
  assert.ok(s[X] < 0, "cart should have been kicked left");
});

test("linearize() matches a numerical Jacobian of the nonlinear derivative at upright", () => {
  const { A, B } = linearize();
  const h = 1e-6;
  const zero = [0, 0, 0, 0];
  for (let j = 0; j < 4; j++) {
    const plus = zero.slice();
    const minus = zero.slice();
    plus[j] += h;
    minus[j] -= h;
    const dp = derivative(plus, 0);
    const dm = derivative(minus, 0);
    for (let i = 0; i < 4; i++) assert.ok(Math.abs((dp[i] - dm[i]) / (2 * h) - A[i][j]) < 1e-8, `A[${i}][${j}]`);
  }
  const dp = derivative(zero, h);
  const dm = derivative(zero, -h);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs((dp[i] - dm[i]) / (2 * h) - B[i][0]) < 1e-8, `B[${i}]`);
});

test("open loop has exactly one unstable mode, doubling in ~0.17 s", () => {
  const { A } = linearize();
  // For this A, the characteristic polynomial is s^2 (s^2 - A[3][2]).
  const lambda = Math.sqrt(A[3][2]);
  assert.ok(Math.abs(Math.LN2 / lambda - 0.174) < 0.001);
});

test("forceAtTip obeys virtual work: F·v_tip = Qx·xDot + Qθ·θDot for any state", () => {
  const rand = rng(7);
  for (let k = 0; k < 500; k++) {
    const s = [rand() * 4 - 2, rand() * 4 - 2, rand() * 7 - 3.5, rand() * 10 - 5];
    const fx = rand() * 20 - 10;
    const fy = rand() * 20 - 10;
    const v = tipVelocity(s);
    const q = forceAtTip(s, fx, fy);
    assert.ok(Math.abs(fx * v.x + fy * v.y - (q.qx * s[X_DOT] + q.qTheta * s[THETA_DOT])) < 1e-10);
  }
});

test("tipVelocity is the time derivative of tipPosition", () => {
  const s = [0.3, 0.7, 0.4, -1.2];
  const h = 1e-6;
  const later = rk4Step(derivative, s, 0, h);
  const p1 = tipPosition(s);
  const p2 = tipPosition(later);
  const v = tipVelocity(s);
  assert.ok(Math.abs((p2.x - p1.x) / h - v.x) < 1e-4);
  assert.ok(Math.abs((p2.y - p1.y) / h - v.y) < 1e-4);
});

test("torque balance: a hanging pole pushed sideways with mg/2 at the tip rests at 45°", () => {
  withParams({ cartMass: 1e12 }, () => {
    const Fx = 0.5 * params.poleMass * params.gravity;
    const theta = Math.PI - Math.PI / 4; // tan(phi) = 2Fx/(mg) = 1
    const s = [0, 0, theta, 0];
    const d = derivative(s, 0, forceAtTip(s, Fx, 0));
    assert.ok(Math.abs(d[3]) < 1e-9, `angular acceleration ${d[3]} at the predicted rest angle`);
  });
});
