# Inverted Pendulum Control Simulator

**[▶ Try it live](https://arjunrb0.github.io/cart-pole-sim/)**: runs in the browser, nothing to install.

An interactive cart-pole simulator built from first principles: derived equations of motion, an RK4 integrator, a cascaded PID controller, and an LQR controller with a hand-written Riccati solver. You can tune both live, shove the pendulum with the mouse, and benchmark them against each other on identical scenarios.

Vanilla JavaScript and HTML Canvas, built with Vite. No physics, control or maths libraries: every equation is in the source, and every claim below is checked by the test suite.

**Headline result:** with default settings, LQR settles about **4× faster** than the hand-tuned PID, and recovers from a disturbance that sends PID's cart off the track.

| Scenario | PID settling | LQR settling | Peak force (PID / LQR) | Cart travel (PID / LQR) |
|---|---|---|---|---|
| Released at 8.6° | 9.57 s | **2.20 s** | 6.0 / 10.3 N | 0.31 / 0.23 m |
| Tap, 0.05 N·s | 11.53 s | **2.56 s** | 9.1 / 21.1 N | 1.30 / 0.60 m |
| Move target 0 → 1 m | 9.58 s | **2.53 s** | 1.0 / 12.0 N | 1.12 / 1.01 m |
| Hard tap, 0.12 N·s | fails: cart leaves the track | **2.46 s** | 21.7 / 40.0 N | 2.40 / 1.39 m |

*Settled* means |θ| < 0.5° and |x − target| < 2 cm, measured from the **last** entry into that band. Both controllers face the same physics, the same 500 Hz sampling and the same 40 N motor limit.

## Running it

```bash
npm install
npm run dev     # simulator at http://localhost:5173
npm test        # 42 checks, about 3 s
npm run build   # static site in dist/
```

Requires Node 22 or newer.

**In the app:** drag the ball at the pole's tip to push it (a spring-damper attaches your pointer to the tip), or click anywhere else to tap it. <kbd>Space</kbd> pauses and <kbd>R</kbd> resets. Switch between PID and LQR in the toolbar. The **PID vs LQR** section at the bottom runs the benchmark using your current slider settings.

## Architecture

The physics, controllers and rendering never see each other's internals. The only things crossing module boundaries are the **state vector** $\mathbf{x} = [x, \dot x, \theta, \dot\theta]$ and a **force** $u$. That's what makes LQR a drop-in replacement for PID, and lets the same physics run under Euler or RK4.

```
src/
├── main.js                  the only module that knows about all the others; owns the clock
├── physics/
│   ├── cartPole.js          equations of motion, linearised model, tip forces, failure rules
│   └── integrator.js        generic Euler and RK4 steppers: f(state, u) -> next state
├── control/
│   ├── pid.js               two summed PID loops (angle + cart position)
│   ├── lqr.js               Riccati solver (Kleinman iteration) and LQR controller
│   └── linalg.js            the small amount of linear algebra LQR needs
├── analysis/
│   ├── settling.js          what "settled" means: offline and live measurement
│   └── benchmark.js         identical, repeatable scenarios for any controller
├── render/                  canvas scene, live strip chart, comparison chart
└── ui/                      sliders, pointer interaction, comparison panel
tests/                       node:test suite, one file per module
```

Physics runs at a **fixed 500 Hz** using a time accumulator, independent of the display's refresh rate, just as a microcontroller runs its control loop on a timer interrupt. Rendering is a read-only function of the state.

## Physics

The state is cart position and velocity plus pole angle and angular velocity. θ is measured from vertical, positive when the pole leans toward +x, so "pole falls right → push right" has matching signs.

The equations of motion are derived twice, by a Newtonian free-body analysis and by Euler–Lagrange, and both give the same coupled pair. Written as a mass-matrix system:

$$\begin{bmatrix} M+m & ml\cos\theta \\ ml\cos\theta & I+ml^2 \end{bmatrix}\begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} u - b\dot{x} + ml\sin\theta\,\dot{\theta}^2 \\ mgl\sin\theta \end{bmatrix}$$

The off-diagonal $ml\cos\theta$ is the cart–pole coupling. Its determinant is provably positive, so the solve never becomes singular.

External forces, such as your pointer on the tip or wind on the cart, enter as **generalised forces from virtual work**, $Q_q = \mathbf{F}\cdot\partial\mathbf{r}/\partial q$. Disturbances are therefore real forces, never edits to the state.

**Why RK4 instead of Euler:** forward Euler always steps along the tangent of a curved trajectory, so on a pendulum it adds energy at every step. With no force and no friction, this system's energy should stay constant:

| after | Euler | RK4 |
|---|---|---|
| 1 s | +2.4% | −6×10⁻¹⁰ % |
| 60 s | **+90.6%** | −1×10⁻⁹ % |

## Control

### PID: two loops sharing one motor

The system is **underactuated**: one motor has to control two things. The design is a fast inner loop on pole angle plus a slow outer loop on cart position, summed.

- **P** must overpower gravity's "negative spring". The linearised model gives a hard minimum, $K_p > 10.79$ N/rad.
- **D** supplies damping. Without it, the controller's sampling delay acts as negative damping and the oscillation grows by ×1.33 every 10 s.
- **The position loop needs the "wrong-way" sign.** To move a balancing cart left, you first push it right so the pole tips left. The closed-loop characteristic polynomial proves the opposite sign is unstable at any gain.
- **I** earns its place on the *position* loop, where it removes the offset caused by a constant disturbance. On the angle loop it only adds lag.

### LQR: choosing all four gains at once

Ignoring the integral terms, PID's two PD loops together are just linear state feedback, $u = -K\mathbf{x}$. LQR picks $K$ to minimise

$$\mathcal{J} = \int_0^\infty \left( \mathbf{x}^T Q\,\mathbf{x} + R\,u^2 \right) dt$$

with weights from **Bryson's rule** (weight = 1 / largest acceptable value²). In the app you tune those tolerances in physical units.

Completing the square shows that $\mathcal{J} \ge \mathbf{x}_0^T P \mathbf{x}_0$ for *any* input, with equality exactly at $K = R^{-1}B^TP$, where $P$ solves the algebraic Riccati equation

$$A^TP + PA - PBR^{-1}B^TP + Q = 0$$

The solver is **Kleinman's iteration**, which is Newton's method on the Riccati equation and is also policy iteration. It repeatedly solves a linear Lyapunov equation (a 16×16 system) and improves the gain, seeded with the PID gains as the initial stabilising controller.

**What the comparison does and doesn't show:**
- LQR uses more force. At *matched* effort (a gentle LQR peaking at 6.8 N against PID's 6.0 N on the tilt test), it still settles in 2.85 s against 9.57 s.
- PID loaded with LQR's four gains reproduces the LQR trajectory **exactly**. The app's "Copy these gains into PID" button demonstrates this. LQR doesn't use a different structure; it finds better numbers.
- LQR needs an accurate model and all four states measured. It has no integral action, so it can't reject constant wind, and it is only optimal near upright. The 40 N saturation isn't part of its model.

## Verification

`npm test` runs 42 checks in about 3 seconds. The physics is tested against facts that don't depend on the derivation:

- energy and momentum conservation, to about 10⁻⁹
- the compound-pendulum period $2\pi\sqrt{4l/3g}$, within one timestep
- a static torque balance and the virtual-work power identity
- the analytic linearisation against a numerical Jacobian of the nonlinear model

The LQR solution is checked against an **independent method** (integrating the Riccati differential equation backwards) and against simulation: the predicted cost $\mathbf{x}_0^TP\mathbf{x}_0$ matches the accumulated cost to six digits. Optimality is tested by trying to beat it with 300 perturbed gains; none succeed.

The suite was itself checked by deliberately reintroducing bugs. Flipping the sign of the coupling term fails 13 tests; restoring the integral-windup bug fails its regression test.

## Bugs worth knowing about

Problems found while building, kept as regression tests:

1. **Integral windup from live tuning.** The PID loop integrated raw error even while Ki = 0. Raising Ki after the cart had sat displaced released all of it as one force step and threw the cart off the track. The fix is to accumulate the integral *term* (`Ki·e·dt`), so gain changes are bumpless.
2. **The live settling meter overstated PID's performance 3×.** It treated "left the tolerance band after looking settled" as a new disturbance. PID's slow cart mode (period ~12 s) drifts back out of the band, so the meter kept restarting its clock and reported 3.18 s instead of 9.57 s. It now times only from real disturbances, and matches the offline measurement exactly.
3. **Textbook LQR weights ignored the motor limit.** The first tolerances produced gains near 400 N/rad that held the motor at saturation in every scenario. The defaults were then chosen by sweeping the benchmark against the real nonlinear, saturating plant.

## Possible extensions

- **LQI**: add an integrator state so LQR rejects constant disturbances
- **Kalman filter (LQG)**: noisy sensors measuring only x and θ, with estimated velocities
- **Swing-up**: energy-based control from hanging, handing over to LQR near the top
