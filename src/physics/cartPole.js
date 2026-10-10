/**
 * cartPole.js — the "plant": the thing being controlled.
 *
 * This module knows ONLY about physics. It has never heard of a canvas, a
 * slider, or a PID gain. Given a state and an applied force it reports how
 * fast each state variable is changing. Nothing more.
 *
 * STATE VECTOR CONVENTION (the contract every other module relies on):
 *
 *   state[0] = x        cart position           (m)     + = right
 *   state[1] = xDot     cart velocity           (m/s)
 *   state[2] = theta    pole angle              (rad)   0 = straight UP,
 *                                                       + = leaning RIGHT (+x)
 *   state[3] = thetaDot pole angular velocity   (rad/s)
 *
 * The applied force `u` is a scalar in newtons, positive = push cart right.
 *
 * Angle sign: measured from vertical, positive toward +x. Stated this way
 * rather than "clockwise" / "counter-clockwise" because those flip meaning
 * between world coordinates (y up) and canvas coordinates (y down), which is
 * a reliable source of sign bugs. This convention pairs naturally with the
 * force: pole falls right (theta > 0) -> push cart right (u > 0) to recover.
 */

// Index names, so we never write a bare state[2] and wonder what it meant.
export const X = 0;
export const X_DOT = 1;
export const THETA = 2;
export const THETA_DOT = 3;

/**
 * Physical parameters of the system.
 *
 * These are deliberately in one object rather than module-level constants:
 * Stage 4 will let you change the pole length live, and the derivative
 * function should read whatever the current values are.
 */
export const params = {
  cartMass: 1.0, // M, kg
  poleMass: 0.1, // m, kg
  poleLength: 0.5, // l, m — distance from pivot to pole's CENTRE OF MASS
  gravity: 9.81, // g, m/s^2
  cartFriction: 0.0, // damping on the cart's motion (N per m/s)
  trackLimit: 2.4, // |x| beyond this counts as a failure (m)
  maxForce: 40, // actuator saturation: the motor cannot push harder (N)
};

/**
 * Has this run ended? Returns a reason string, or null if all is well.
 * Shared by the live simulation and the benchmark so both judge the same way.
 */
export function failureReason(state) {
  if (!state.every(Number.isFinite)) return "Simulation diverged";
  if (Math.abs(state[THETA]) > Math.PI / 2) return "The pole fell";
  if (Math.abs(state[X]) > params.trackLimit) return "The cart ran off the track";
  return null;
}

/**
 * The actuator. Whatever force a controller asks for, the motor delivers at
 * most maxForce. Lives with the plant, not the controller, because it is a
 * property of the hardware — and so every controller hits the same limit.
 */
export function saturate(force) {
  return Math.max(-params.maxForce, Math.min(params.maxForce, force));
}

/**
 * Convenience: a fresh state, slightly off-vertical so there is something
 * for the controller to correct.
 */
export function initialState(thetaOffset = 0.05) {
  return [0, 0, thetaOffset, 0];
}

/**
 * Rotational inertia of the pole about its OWN centre of mass.
 *
 * Modelled as a uniform rod of full length 2l (since l is pivot-to-centre),
 * so I = (1/12) * m * (2l)^2 = (1/3) * m * l^2.
 *
 * Computed rather than stored so that changing poleLength at runtime stays
 * physically consistent.
 */
function poleInertia() {
  const { poleMass: m, poleLength: l } = params;
  return (1 / 3) * m * l * l;
}

/**
 * The equations of motion.
 *
 * Returns d(state)/dt as [xDot, xDotDot, thetaDot, thetaDotDot].
 *
 * Derived two ways (Newtonian free-body and Euler-Lagrange), both landing on
 * the same coupled pair, written here as a 2x2 linear system:
 *
 *   | M+m        m*l*cos(th) | | xDotDot  |   | u - b*xDot + m*l*sin(th)*thDot^2 |
 *   | m*l*cos(th)  I + m*l^2 | | thDotDot | = | m*g*l*sin(th)                    |
 *     \___________________/
 *      the "mass matrix"; its off-diagonal IS the cart<->pole coupling
 *
 * Solving a 2x2 explicitly (rather than using the single mashed-together
 * expression common in RL codebases) keeps each physical term visible.
 *
 * `external` carries GENERALISED forces from anything other than the motor
 * (a hand on the pole, wind on the cart): qx adds to the x equation, qTheta
 * (a torque about the pivot) to the theta equation. See forceAtTip().
 */
export const NO_EXTERNAL = Object.freeze({ qx: 0, qTheta: 0 });

export function derivative(state, force, external = NO_EXTERNAL) {
  const { cartMass: M, poleMass: m, poleLength: l, gravity: g, cartFriction: b } = params;

  const xDot = state[X_DOT];
  const theta = state[THETA];
  const thetaDot = state[THETA_DOT];

  const sin = Math.sin(theta);
  const cos = Math.cos(theta);
  const I = poleInertia();

  // Right-hand side: everything that is NOT an acceleration.
  //   f1 - applied force, minus cart friction, plus the centripetal reaction
  //        of the swinging pole tugging on the cart.
  //   f2 - gravity's torque about the pivot. Positive theta gives positive
  //        torque: gravity drives the pole AWAY from upright. This one sign
  //        is what makes the system unstable and the whole project necessary.
  const f1 = force + external.qx - b * xDot + m * l * sin * thetaDot * thetaDot;
  const f2 = m * g * l * sin + external.qTheta;

  // Mass matrix entries (symmetric, as any Lagrangian system's must be).
  const m11 = M + m;
  const m12 = m * l * cos; // <- the coupling term
  const m22 = I + m * l * l;

  // det = (M+m)(I+m*l^2) - (m*l*cos)^2.
  // Always strictly positive: with I = (1/3)m*l^2 this reduces to
  // m*l^2 * [ (4/3)(M+m) - m*cos^2 ], and (4/3)(M+m) > m >= m*cos^2.
  // So there is no configuration where this blows up. No guard needed.
  const det = m11 * m22 - m12 * m12;

  // Cramer's rule on the 2x2.
  const xDotDot = (m22 * f1 - m12 * f2) / det;
  const thetaDotDot = (m11 * f2 - m12 * f1) / det;

  return [xDot, xDotDot, thetaDot, thetaDotDot];
}

/**
 * The linearised model around upright:  d(state)/dt ~= A state + B u.
 *
 * Small-angle approximations sin(th) ~ th, cos(th) ~ 1, thDot^2 ~ 0 turn
 * the mass-matrix system above into constant matrices. With
 * J = I + m l^2 and D = (M+m) J - (m l)^2 (the mass-matrix determinant at
 * theta = 0):
 *
 *        x   xDot        theta              thetaDot
 *   A = [ 0   1           0                  0 ]      B = [ 0       ]
 *       [ 0  -b J/D      -(m l)^2 g / D      0 ]          [ J/D     ]
 *       [ 0   0           0                  1 ]          [ 0       ]
 *       [ 0   m l b/D     (M+m) m g l / D    0 ]          [ -m l/D  ]
 *
 * Column 0 is all zeros: nothing in the physics depends on WHERE the cart
 * is. That's why the cart can balance at any target position.
 *
 * Exact only at the upright equilibrium; the LQR controller built on it is
 * therefore optimal only near upright.
 */
export function linearize() {
  const { cartMass: M, poleMass: m, poleLength: l, gravity: g, cartFriction: b } = params;
  const J = poleInertia() + m * l * l;
  const D = (M + m) * J - m * l * m * l;
  const A = [
    [0, 1, 0, 0],
    [0, (-b * J) / D, (-(m * l) * (m * l) * g) / D, 0],
    [0, 0, 0, 1],
    [0, (m * l * b) / D, ((M + m) * m * g * l) / D, 0],
  ];
  const B = [[0], [J / D], [0], [(-m * l) / D]];
  return { A, B };
}

/**
 * The tip of the pole, in world metres. Frame: x along the track, y UP,
 * origin level with the pivot. The pole's full length is 2l, because
 * poleLength is pivot-to-centre-of-mass.
 */
export function tipPosition(state) {
  const L = 2 * params.poleLength;
  return {
    x: state[X] + L * Math.sin(state[THETA]),
    y: L * Math.cos(state[THETA]),
  };
}

/** Time derivative of tipPosition(). */
export function tipVelocity(state) {
  const L = 2 * params.poleLength;
  const theta = state[THETA];
  const thetaDot = state[THETA_DOT];
  return {
    x: state[X_DOT] + L * Math.cos(theta) * thetaDot,
    y: -L * Math.sin(theta) * thetaDot,
  };
}

/**
 * Convert a force (fx, fy) in newtons, acting AT THE TIP, into the
 * generalised forces that derivative() understands.
 *
 * Virtual work: a force F at point r(x, theta) contributes
 *
 *     Q_q = F . dr/dq      to the equation for coordinate q.
 *
 * With r = (x + L sin th, L cos th):
 *     dr/dx  = (1, 0)                   ->  qx     = fx
 *     dr/dth = (L cos th, -L sin th)    ->  qTheta = L (fx cos th - fy sin th)
 *
 * qTheta is just the torque about the pivot: lever arm times the
 * perpendicular part of the force. A vertical push on an upright pole goes
 * straight through the pivot and produces none.
 */
export function forceAtTip(state, fx, fy) {
  const L = 2 * params.poleLength;
  const theta = state[THETA];
  return {
    qx: fx,
    qTheta: L * (fx * Math.cos(theta) - fy * Math.sin(theta)),
  };
}

/**
 * Total mechanical energy (kinetic + potential), in joules.
 *
 * Not needed to run the simulation — it exists to CHECK it. With no applied
 * force and no friction this quantity must stay constant forever; watching
 * how fast an integrator lets it drift is the sharpest test of that
 * integrator's quality. See the RK4-vs-Euler comparison.
 *
 * Note V is at a MAXIMUM when theta = 0. Balancing upright means parking the
 * system on the summit of a potential-energy hill.
 */
export function totalEnergy(state) {
  const { cartMass: M, poleMass: m, poleLength: l, gravity: g } = params;
  const xDot = state[X_DOT];
  const theta = state[THETA];
  const thetaDot = state[THETA_DOT];

  const kinetic =
    0.5 * (M + m) * xDot * xDot +
    m * l * Math.cos(theta) * xDot * thetaDot +
    0.5 * (m * l * l + poleInertia()) * thetaDot * thetaDot;

  const potential = m * g * l * Math.cos(theta);

  return kinetic + potential;
}
