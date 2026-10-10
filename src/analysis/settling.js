/**
 * settling.js — what "settled" means, measured one consistent way.
 *
 * A response has SETTLED once it enters a tolerance band and never leaves
 * it again. The settling time is the moment of the LAST entry into the band,
 * not the first: a pole that swings through zero on its way to overshooting
 * has not settled.
 *
 * Both conditions must hold at once: the pole near upright AND the cart near
 * its target. Balancing the pole while the cart wanders is not "settled".
 */

import { X, THETA } from "../physics/cartPole.js";

export const SETTLE_BAND = {
  thetaDeg: 0.5, // |theta| below this...
  x: 0.02, // ...and |x - target| below this (m)
};

export function inBand(state, targetX, band = SETTLE_BAND) {
  return Math.abs(state[THETA]) < (band.thetaDeg * Math.PI) / 180 && Math.abs(state[X] - targetX) < band.x;
}

/**
 * Settling time of a RECORDED run, where the whole future is known.
 *
 * @param inBandAt  array of booleans, one per sample
 * @param times     matching sample times (s)
 * @returns time of the last entry into the band, or Infinity if the run
 *          ends outside it
 */
export function settlingTime(inBandAt, times) {
  const n = inBandAt.length;
  if (n === 0 || !inBandAt[n - 1]) return Infinity;
  let i = n - 1;
  while (i > 0 && inBandAt[i - 1]) i--;
  return times[i];
}

/**
 * Settling time LIVE, where the future is unknown. The meter can only
 * declare "settled" after the response has stayed in the band for a while
 * (`confirm` seconds), then reports when it entered.
 *
 * That answer is PROVISIONAL. A slow oscillation can sit inside the band for
 * longer than any confirmation window and then drift out again (PID's slow
 * cart mode has a ~12 s period). When that happens the meter goes back to
 * "settling", and still times from the original disturbance, so its final
 * answer matches the offline measurement.
 *
 * Time is measured only from real disturbances: call disturb(t) for a tap,
 * a drag release, a reset, a new target, a wind change, a controller switch.
 * Leaving the band is NOT treated as a new disturbance: that was a bug that
 * made a slowly-settling run look three times faster than it was.
 */
export function createSettlingMeter({ band = SETTLE_BAND, confirm = 1.0 } = {}) {
  let disturbedAt = 0;
  let enteredAt = null;
  let settledAfter = null;

  return {
    disturb(t) {
      disturbedAt = t;
      enteredAt = null;
      settledAfter = null;
    },

    sample(t, state, targetX) {
      if (inBand(state, targetX, band)) {
        if (enteredAt === null) enteredAt = t;
        if (settledAfter === null && t - enteredAt >= confirm) settledAfter = enteredAt - disturbedAt;
      } else {
        enteredAt = null;
        settledAfter = null;
      }
    },

    /** { settled: true, time } or { settled: false, elapsed } */
    status(t) {
      return settledAfter !== null ? { settled: true, time: settledAfter } : { settled: false, elapsed: t - disturbedAt };
    },
  };
}
