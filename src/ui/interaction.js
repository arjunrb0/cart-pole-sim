/**
 * interaction.js — turns pointer input into REAL forces on the pole.
 *
 * Nothing here ever writes to the state. Disturbances enter the physics the
 * same way any force does, so momentum and energy stay honest and the
 * controller faces a situation that could actually happen.
 *
 *   drag the ball at the tip  -> a spring-damper between pointer and tip
 *   click anywhere else       -> a short horizontal tap at the tip, pushing
 *                                away from where you clicked
 *
 * The two are queried differently by main.js, and that difference matters:
 *
 *   springForce(state)  depends on WHERE the tip is, so it is evaluated
 *                       inside every RK4 probe and integrated properly.
 *   takeTapForce(dt)    depends only on TIME, so holding it constant across
 *                       one step is exact.
 */

import { tipPosition, tipVelocity } from "../physics/cartPole.js";
import { BOB_RADIUS } from "../render/renderer.js";

// Rubber band between your pointer and the tip. Tuned so the pole follows
// the pointer briskly without ringing (damping ratio ~0.8 for a pole this
// light), and capped so a wild fling can't produce an absurd force.
const SPRING_K = 25; // N/m
const SPRING_C = 1.5; // N per m/s
const SPRING_MAX = 20; // N

const TAP_DURATION = 0.05; // s: the impulse is spread over this long
const TAP_SHOW_TIME = 0.25; // s: how long the tap arrow stays on screen
const GRAB_SLOP_PX = 14; // extra hit radius around the ball, for fingers

/**
 * @param canvas       the scene canvas
 * @param renderer     for screen<->world conversion
 * @param environment  { tapImpulse } read live, so its slider works
 * @param isActive     () => boolean; input is ignored while paused/failed
 * @param getState     () => current state, for hit-testing the ball
 * @param onDisturb    () => void; called on a tap and when a drag ends, so
 *                     the settling meter can start timing the recovery
 */
export function createInteraction(canvas, { renderer, environment, isActive, getState, onDisturb = () => {} }) {
  let anchor = null; // world point being held, or null
  let hoverTip = false;
  let tapRemaining = 0; // s of force still to deliver
  let tapForce = 0; // N, while tapRemaining > 0
  let tapShown = 0; // s of arrow display left
  let tapDirection = 0;

  // Pointer position relative to the canvas, in CSS pixels. Because the
  // renderer draws in CSS pixels too, these line up with no extra scaling.
  function localPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function isOverTip(p) {
    const tw = tipPosition(getState());
    const tip = renderer.worldToScreen(tw.x, tw.y);
    const r = Math.max(7, BOB_RADIUS * renderer.view.ppm) + GRAB_SLOP_PX;
    return Math.hypot(p.x - tip.x, p.y - tip.y) <= r;
  }

  canvas.addEventListener("pointerdown", (event) => {
    if (!isActive()) return;
    const p = localPoint(event);

    if (isOverTip(p)) {
      anchor = renderer.screenToWorld(p.x, p.y);
      // Keep receiving move/up events even if the pointer leaves the canvas.
      canvas.setPointerCapture(event.pointerId);
    } else {
      // Tap: push the tip AWAY from the click, horizontally.
      const tw = tipPosition(getState());
      const tipScreenX = renderer.worldToScreen(tw.x, tw.y).x;
      tapDirection = p.x <= tipScreenX ? 1 : -1;
      tapForce = (tapDirection * environment.tapImpulse) / TAP_DURATION;
      tapRemaining = TAP_DURATION;
      tapShown = TAP_SHOW_TIME;
      onDisturb();
    }
    event.preventDefault();
  });

  canvas.addEventListener("pointermove", (event) => {
    const p = localPoint(event);
    if (anchor) {
      anchor = renderer.screenToWorld(p.x, p.y);
    } else {
      hoverTip = isActive() && isOverTip(p);
    }
    canvas.style.cursor = anchor ? "grabbing" : hoverTip ? "grab" : "pointer";
  });

  function release() {
    anchor = null;
  }
  function endDrag() {
    if (anchor) onDisturb(); // recovery is timed from the moment you let go
    release();
  }
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("pointerleave", () => {
    hoverTip = false;
  });

  return {
    /** Spring-damper force on the tip, for THIS (possibly mid-RK4) state. */
    springForce(state) {
      if (!anchor) return { fx: 0, fy: 0 };
      const tip = tipPosition(state);
      const vel = tipVelocity(state);
      let fx = SPRING_K * (anchor.x - tip.x) - SPRING_C * vel.x;
      let fy = SPRING_K * (anchor.y - tip.y) - SPRING_C * vel.y;
      const mag = Math.hypot(fx, fy);
      if (mag > SPRING_MAX) {
        fx *= SPRING_MAX / mag;
        fy *= SPRING_MAX / mag;
      }
      return { fx, fy };
    },

    /** Horizontal tap force for the next `dt`, and advance the tap timer. */
    takeTapForce(dt) {
      tapShown = Math.max(0, tapShown - dt);
      if (tapRemaining <= 0) return 0;
      // If less than a full step remains, deliver a proportional share so
      // the total impulse comes out exactly right.
      const share = Math.min(dt, tapRemaining) / dt;
      tapRemaining -= dt;
      return tapForce * share;
    },

    release,

    /** What the renderer needs to show the interaction. */
    get view() {
      return {
        anchor,
        hoverTip,
        tap: tapShown > 0 ? tapDirection : 0,
      };
    },
  };
}
