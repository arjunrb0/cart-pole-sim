/**
 * Stage 3: PID control. Each claim made in the explanation, as a check.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import { derivative, failureReason, linearize, params, saturate, X, THETA } from "../src/physics/cartPole.js";
import { rk4Step } from "../src/physics/integrator.js";
import { createPidController, createPidLoop, defaultPidGains } from "../src/control/pid.js";
import { eigenvalues, matMul, sub } from "../src/control/linalg.js";
import { DT, runClosedLoop } from "./helpers.js";

const gains = (edit) => {
  const g = defaultPidGains();
  edit(g);
  return g;
};
const closedLoopPoles = (controller) => {
  const { A, B } = linearize();
  return eigenvalues(sub(A, matMul(B, [controller.linearGain()])));
};
const isStable = (controller) => closedLoopPoles(controller).every((p) => p.re < 0);
const wind = (d) => () => ({ qx: d, qTheta: 0 });

test("P must beat gravity: the threshold is Kp = 10.79 N/rad", () => {
  const { A, B } = linearize();
  const threshold = A[3][2] / -B[3][0];
  assert.ok(Math.abs(threshold - 10.79) < 0.01, `threshold ${threshold}`);

  const s = runClosedLoop(createPidController(gains((g) => (g.angle.kp = 10))), [0, 0, 0.15, 0], 3);
  assert.ok(Math.abs(s[THETA]) > 0.5, "Kp = 10 should let the pole fall");
});

test("default gains balance the pole and bring the cart home", () => {
  const s = runClosedLoop(createPidController(), [0, 0, 0.15, 0], 15);
  assert.ok(Math.abs(s[THETA]) < 0.001);
  assert.ok(Math.abs(s[X]) < 0.02);
});

test("angle loop alone balances the pole but the cart runs off the track", () => {
  const c = createPidController(gains((g) => Object.assign(g.position, { kp: 0, kd: 0 })));
  const s = runClosedLoop(c, [0, 0, 0.15, 0], 10);
  assert.ok(Math.abs(s[THETA]) < 0.001, "pole should be balanced");
  assert.ok(Math.abs(s[X]) > params.trackLimit, `cart only reached ${s[X]} m`);
});

test("the position gain must have the 'wrong-way' sign", () => {
  assert.ok(isStable(createPidController()));
  assert.ok(!isStable(createPidController(gains((g) => (g.position.kp = -1)))), "negative position Kp should be unstable");
  assert.ok(!isStable(createPidController(gains((g) => (g.position.kd = -1)))), "negative position Kd should be unstable");
});

test("angle Kd = 0 is unstable with the cart loop on (negative s^3 coefficient)", () => {
  assert.ok(!isStable(createPidController(gains((g) => (g.angle.kd = 0)))));
});

test("P-only on angle grows slowly, from the sampling delay (x1.33 per 10 s)", () => {
  const c = createPidController(gains((g) => {
    g.angle.kd = 0;
    Object.assign(g.position, { kp: 0, kd: 0 });
  }));
  // Track the peak |theta| in the first and the sixth 10 s windows.
  let s = [0, 0, 0.02, 0];
  const peaks = [];
  for (let w = 0; w < 6; w++) {
    let peak = 0;
    for (let i = 0; i < 10 / DT; i++) {
      s = rk4Step(derivative, s, saturate(c.update(s, DT)), DT);
      peak = Math.max(peak, Math.abs(s[THETA]));
    }
    peaks.push(peak);
  }
  const growthPer10s = (peaks[5] / peaks[0]) ** (1 / 5);
  // Predicted from a half-sample delay: exp(beta*Kp*dt/4 * 10 s) ~ 1.34.
  assert.ok(Math.abs(growthPer10s - 1.33) < 0.02, `growth ${growthPer10s}`);
});

test("wind parks a PD cart at -d/Kx; only the POSITION integral removes the offset", () => {
  const pd = runClosedLoop(createPidController(), [0, 0, 0, 0], 30, { external: wind(1) });
  assert.ok(Math.abs(pd[X] - -1 / defaultPidGains().position.kp) < 0.005, `PD parked at ${pd[X]}`);

  const posI = runClosedLoop(createPidController(gains((g) => (g.position.ki = 0.2))), [0, 0, 0, 0], 30, { external: wind(1) });
  assert.ok(Math.abs(posI[X]) < 0.01, `position Ki left the cart at ${posI[X]}`);

  const angI = runClosedLoop(createPidController(gains((g) => (g.angle.ki = 5))), [0, 0, 0, 0], 30, { external: wind(1) });
  assert.ok(Math.abs(angI[X] + 1) < 0.01, `angle Ki should not fix the offset, got ${angI[X]}`);
});

test("regression: switching Ki on mid-run causes no windup kick (live-tuning bug)", () => {
  // The old loop integrated raw error while Ki = 0, then released it all as
  // a force step when Ki was raised, which threw the cart off the track.
  const c = createPidController();
  let worstAfter = 0;
  const s = runClosedLoop(c, [0, 0, 0.15, 0], 55, {
    external: wind(1),
    onStep: (i, state, ctrl) => {
      if (i === 25 / DT) ctrl.gains.position.ki = 0.2;
      if (i > 25 / DT) worstAfter = Math.max(worstAfter, Math.abs(state[X]));
    },
  });
  assert.equal(failureReason(s), null);
  assert.ok(worstAfter < 1.01, `cart strayed to ${worstAfter} m after the switch`);
  assert.ok(Math.abs(s[X]) < 0.01);
});

test("a PID loop accumulates nothing while Ki = 0", () => {
  const g = { kp: 0, ki: 0, kd: 0, integralLimit: 5 };
  const loop = createPidLoop(g);
  for (let i = 0; i < 1000; i++) loop.update(1, 0, DT);
  g.ki = 1;
  // One step of error 1 at Ki = 1 should contribute exactly Ki*e*dt.
  assert.ok(Math.abs(loop.update(1, 0, DT) - DT) < 1e-12);
});

test("anti-windup caps the integral term at integralLimit newtons", () => {
  const loop = createPidLoop({ kp: 0, ki: 10, kd: 0, integralLimit: 5 });
  let out = 0;
  for (let i = 0; i < 10000; i++) out = loop.update(1, 0, DT);
  assert.equal(out, 5);
});
