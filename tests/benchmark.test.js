/**
 * Stage 5: settling-time measurement and the PID vs LQR comparison.
 *
 * The "README numbers" test pins the results table in README.md. If you
 * deliberately retune a controller this test fails, which is the point:
 * update the README table along with the expected values here.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import { derivative, saturate } from "../src/physics/cartPole.js";
import { rk4Step } from "../src/physics/integrator.js";
import { createPidController } from "../src/control/pid.js";
import { createLqrController } from "../src/control/lqr.js";
import { createSettlingMeter, settlingTime } from "../src/analysis/settling.js";
import { runComparison, runScenario, SCENARIOS } from "../src/analysis/benchmark.js";
import { DT, seedGain } from "./helpers.js";

const factories = {
  PID: (t) => createPidController(undefined, t),
  LQR: (t) => createLqrController({ target: t, initialGain: seedGain }),
};
const results = Object.fromEntries(runComparison(factories).map((e) => [e.scenario.id, e.results]));

test("settling time is the LAST entry into the band, not the first", () => {
  const times = [0, 1, 2, 3, 4, 5];
  assert.equal(settlingTime([false, true, false, true, true, true], times), 3);
  assert.equal(settlingTime([true, true, true], times.slice(0, 3)), 0);
  assert.equal(settlingTime([true, false], times.slice(0, 2)), Infinity);
});

test("README numbers: the default PID vs LQR results table", () => {
  const expected = {
    tilt: { PID: 9.57, LQR: 2.2 },
    tap: { PID: 11.53, LQR: 2.56 },
    step: { PID: 9.58, LQR: 2.53 },
    hardTap: { PID: null, LQR: 2.46 }, // null = PID fails
  };
  for (const [id, exp] of Object.entries(expected)) {
    for (const name of ["PID", "LQR"]) {
      const r = results[id][name];
      if (exp[name] === null) {
        assert.match(r.failure ?? "", /track/, `${name} on ${id} should run off the track`);
      } else {
        assert.equal(r.failure, null, `${name} on ${id} failed`);
        assert.ok(Math.abs(r.settling - exp[name]) < 0.02, `${name} on ${id}: ${r.settling} s, README says ${exp[name]} s`);
      }
    }
  }
});

test("LQR settles faster than PID in every scenario, and survives what PID doesn't", () => {
  for (const { id } of SCENARIOS) {
    const { PID, LQR } = results[id];
    assert.equal(LQR.failure, null);
    if (!PID.failure) assert.ok(LQR.settling < PID.settling / 3, `${id}: only ${PID.settling / LQR.settling}x faster`);
  }
  assert.ok(results.hardTap.PID.failure);
});

test("matched effort: a gentle LQR still settles >3x faster than PID", () => {
  // Tolerances chosen so LQR's peak force on the tilt test is close to PID's.
  const gentle = { x: 0.5, xDot: 1, thetaDeg: 20, thetaDotDeg: 90, force: 2 };
  const lqr = runScenario((t) => createLqrController({ tolerances: gentle, target: t, initialGain: seedGain }), SCENARIOS[0]);
  const pid = results.tilt.PID;
  assert.ok(lqr.peakForce < pid.peakForce * 1.2, `gentle LQR used ${lqr.peakForce} N vs PID ${pid.peakForce} N`);
  assert.ok(lqr.settling < pid.settling / 3);
});

test("the benchmark is deterministic", () => {
  const again = runScenario(factories.PID, SCENARIOS[1]);
  assert.equal(again.settling, results.tap.PID.settling);
  assert.equal(again.peakForce, results.tap.PID.peakForce);
});

test("regression: the live settling meter agrees with the offline measurement", () => {
  // The first version restarted its clock whenever a slow response drifted
  // back out of the band, and reported PID ~3x faster than it really was.
  for (const [name, make] of Object.entries(factories)) {
    const meter = createSettlingMeter();
    const c = make({ x: 0 });
    let s = [0, 0, 0.15, 0];
    meter.disturb(0);
    for (let i = 0; i < 20 / DT; i++) {
      s = rk4Step(derivative, s, saturate(c.update(s, DT)), DT);
      meter.sample((i + 1) * DT, s, 0);
    }
    const live = meter.status(20);
    assert.ok(live.settled, `${name} meter never settled`);
    assert.ok(Math.abs(live.time - results.tilt[name].settling) < 0.005, `${name}: live ${live.time} vs offline ${results.tilt[name].settling}`);
  }
});
