/**
 * pid.js — PID control for the cart-pole.
 *
 * A controller's entire job: look at the state, return a force.
 *
 *     update(state, dt) -> force in newtons
 *
 * Every controller in this project exposes that same shape, which is what
 * makes the Stage 5 LQR a drop-in replacement rather than a rewrite.
 *
 * Structure: TWO loops whose outputs are summed.
 *
 *   angle loop    (inner, fast) keeps the pole upright
 *   position loop (outer, slow) keeps the cart on the track
 *
 * One actuator, two things to control (the system is "underactuated"), so
 * both loops have to share the same force.
 */

import { X, X_DOT, THETA, THETA_DOT } from "../physics/cartPole.js";

/**
 * Default gains, chosen from the linearised model rather than by trial:
 *
 *   angle loop  — the pole obeys  thetaDDot = 15.79*theta - 1.463*u.
 *                 Picking natural frequency wn = 6 rad/s and damping
 *                 ratio zeta = 0.7 gives kp ~ 35, kd ~ 5.7, rounded up a
 *                 little to leave margin for the position loop, which
 *                 eats into the pole's damping (see the s^3 coefficient).
 *
 *   position loop — several times slower than the angle loop, so the two
 *                 don't fight. Both gains are POSITIVE, which means
 *                 "cart too far right -> push further right": the only way
 *                 to move a balancing cart left is to tip the pole left
 *                 first. The other sign is unstable at any magnitude.
 *
 * Integral gains start at zero; see `integralLimit` and the notes below.
 */
export function defaultPidGains() {
  return {
    angle: { kp: 40, ki: 0, kd: 6, integralLimit: 5 },
    position: { kp: 1, ki: 0, kd: 2, integralLimit: 5 },
  };
}

/**
 * One generic PID loop. Knows nothing about pendulums.
 *
 *   error      how far the measurement is past its target
 *   errorRate  how fast that error is changing
 *
 * The derivative is passed in rather than computed by differencing the
 * error, because the state already contains the true velocity (on hardware:
 * a gyro or encoder). Numerical differentiation amplifies sensor noise, and
 * differentiating the error makes the D term spike whenever the setpoint
 * jumps ("derivative kick").
 */
export function createPidLoop(gains) {
  // Private state, captured by the closure below. Equivalent to a Python
  // class attribute self._integralTerm that nothing outside can touch.
  //
  // This stores the integral TERM itself, in newtons — the running sum of
  // ki * error * dt — not the raw integral of the error. The difference only
  // shows up when gains change while running, which is exactly what sliders
  // do. Storing the raw integral and multiplying by ki afterwards meant that
  // error kept piling up silently while ki was 0, and the moment ki was
  // raised all of that history fired at once as a large force step. In
  // testing, that sent the cart off the track. With the term stored
  // directly, nothing accumulates while ki is 0, and a change of ki applies
  // only to error from then on ("bumpless" tuning).
  let integralTerm = 0;

  return {
    gains,

    update(error, errorRate, dt) {
      // Read gains fresh every call: a slider that edits `gains` takes
      // effect on the very next physics step.
      const { kp, ki, kd, integralLimit } = gains;

      if (ki === 0) {
        // ki = 0 means "no integral action", not "keep whatever was left".
        integralTerm = 0;
      } else {
        integralTerm += ki * error * dt;
        // Anti-windup: cap how many NEWTONS the integral may contribute.
        // Without this, a stretch where the error can't be corrected (motor
        // saturated, pole fallen over) piles up a huge stored correction
        // that then fires all at once on recovery.
        integralTerm = Math.max(-integralLimit, Math.min(integralLimit, integralTerm));
      }

      return kp * error + integralTerm + kd * errorRate;
    },

    reset() {
      integralTerm = 0;
    },
  };
}

/**
 * The cart-pole PID controller: angle loop + position loop.
 *
 * Error sign convention: error = measurement - target ("how far PAST the
 * target"), the reverse of the textbook (target - measurement). Pushing the
 * cart right tips the pole LEFT, so the plant has negative gain from force to
 * angle; flipping the error sign is what lets every gain be a positive number.
 */
export function createPidController(gains = defaultPidGains(), target = { x: 0 }) {
  const angleLoop = createPidLoop(gains.angle);
  const positionLoop = createPidLoop(gains.position);

  // `target` is where we want the cart to be. Change target.x and watch the
  // cart move AWAY from it first: that's the "wrong-way" position loop at
  // work. Passed in so several controllers can share one setpoint.

  return {
    name: "PID",
    gains,
    target,

    update(state, dt) {
      const uAngle = angleLoop.update(state[THETA], state[THETA_DOT], dt);
      const uPosition = positionLoop.update(state[X] - target.x, state[X_DOT], dt);
      return uAngle + uPosition;
    },

    reset() {
      angleLoop.reset();
      positionLoop.reset();
    },

    /**
     * The proportional and derivative parts, rewritten in LQR's u = -K x
     * form over [x, xDot, theta, thetaDot]. Without integral terms, PD on
     * angle plus PD on position IS linear state feedback; this is the same
     * controller, just written the way LQR writes it. Integral terms are
     * left out, so any ki > 0 makes this an approximation.
     */
    linearGain() {
      const { angle, position } = gains;
      return [-position.kp, -position.kd, -angle.kp, -angle.kd];
    },
  };
}
