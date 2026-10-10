/**
 * renderer.js — draws a state onto a canvas. Read-only.
 *
 * The renderer never modifies the state and never computes physics. Give it
 * a state and a description of the scene and it paints that instant. Drawing
 * is a pure function of its inputs, which is why physics can run at 500 Hz
 * while the screen updates at 60 Hz.
 *
 * It also owns the ONE mapping between world coordinates (metres, y up,
 * origin level with the pivot) and screen coordinates (CSS pixels, y down).
 * Input handling uses screenToWorld() so a click lands in metres.
 */

import { X, THETA, params, tipPosition } from "../physics/cartPole.js";

const COLORS = {
  track: "#3a4454",
  tick: "#58637a",
  label: "#8f9bb0",
  endStop: "#c4564a",
  cart: "#5b7fd4",
  cartEdge: "#7c9be6",
  wheel: "#26303f",
  pole: "#e0a24a",
  pivot: "#26303f",
  target: "#5fbf8a",
  force: "#e6ebf2",
  wind: "#56b6c2",
  spring: "#e6ebf2",
  overlay: "rgba(10, 13, 19, 0.72)",
  overlayText: "#e6ebf2",
};

// Cart geometry, in metres, so it scales with the view.
const CART_W = 0.5;
const CART_H = 0.2;
const WHEEL_R = 0.045;
const POLE_WIDTH = 0.035;
export const BOB_RADIUS = 0.06;

// A force of maxForce newtons draws as an arrow this many metres long.
const FORCE_ARROW_FULL_SCALE = 1.0;

export function createRenderer(canvas) {
  const ctx = canvas.getContext("2d");

  // Everything that depends on the canvas size. Recomputed on resize.
  const view = { width: 0, height: 0, ppm: 1, originX: 0, groundY: 0, pivotY: 0 };

  /**
   * Match the canvas's internal pixel buffer to its on-screen size.
   *
   * On a retina display one CSS pixel is 2-3 physical pixels; a canvas whose
   * buffer is only CSS-sized gets stretched and looks blurry. So the buffer is
   * CSS size x devicePixelRatio, and setTransform() scales the context so
   * that all drawing code below still thinks in CSS pixels.
   */
  function resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    view.width = rect.width;
    view.height = rect.height;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // Fit the whole track horizontally and the full pole vertically.
    const worldWidth = 2 * params.trackLimit + CART_W + 0.4;
    const worldHeight = WHEEL_R + CART_H + 2 * params.poleLength + BOB_RADIUS + 0.55;
    view.ppm = Math.min(view.width / worldWidth, view.height / worldHeight);
    view.originX = view.width / 2;
    view.groundY = view.height - 0.3 * view.ppm; // room for tick labels below
    view.pivotY = view.groundY - (WHEEL_R + CART_H) * view.ppm;
  }

  new ResizeObserver(resize).observe(canvas);
  resize();

  // --- coordinate transforms -------------------------------------------------
  function worldToScreen(wx, wy) {
    return { x: view.originX + wx * view.ppm, y: view.pivotY - wy * view.ppm };
  }
  function screenToWorld(sx, sy) {
    return { x: (sx - view.originX) / view.ppm, y: (view.pivotY - sy) / view.ppm };
  }

  // --- drawing helpers -------------------------------------------------------
  function arrow(x0, y0, x1, y1, color, width = 2.5) {
    const len = Math.hypot(x1 - x0, y1 - y0);
    if (len < 2) return;
    const head = Math.min(10, len * 0.5);
    const ang = Math.atan2(y1 - y0, x1 - x0);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1 - head * 0.6 * Math.cos(ang), y1 - head * 0.6 * Math.sin(ang));
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - head * Math.cos(ang - 0.45), y1 - head * Math.sin(ang - 0.45));
    ctx.lineTo(x1 - head * Math.cos(ang + 0.45), y1 - head * Math.sin(ang + 0.45));
    ctx.closePath();
    ctx.fill();
  }

  function text(str, x, y, { color = COLORS.label, size = 11, align = "center", weight = 400 } = {}) {
    ctx.fillStyle = color;
    ctx.font = `${weight} ${size}px ui-sans-serif, system-ui, sans-serif`;
    ctx.textAlign = align;
    ctx.textBaseline = "middle";
    ctx.fillText(str, x, y);
  }

  function drawTrack(targetX) {
    const { ppm, originX, groundY } = view;
    const lim = params.trackLimit;

    ctx.strokeStyle = COLORS.track;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(originX - lim * ppm, groundY);
    ctx.lineTo(originX + lim * ppm, groundY);
    ctx.stroke();

    // Ticks every 0.5 m, labelled every metre.
    for (let m = -Math.floor(lim * 2) / 2; m <= lim + 1e-9; m += 0.5) {
      const sx = originX + m * ppm;
      const major = Math.abs(m - Math.round(m)) < 1e-9;
      ctx.strokeStyle = COLORS.tick;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx, groundY + 2);
      ctx.lineTo(sx, groundY + (major ? 8 : 5));
      ctx.stroke();
      if (major) text(m === 0 ? "0" : `${m} m`, sx, groundY + 18);
    }

    // End stops: leaving the track counts as failure.
    for (const side of [-1, 1]) {
      const sx = originX + side * lim * ppm;
      ctx.strokeStyle = COLORS.endStop;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(sx, groundY + 4);
      ctx.lineTo(sx, groundY - 0.18 * ppm);
      ctx.stroke();
    }

    // Target marker: a small triangle pointing up at the track.
    const tx = originX + targetX * ppm;
    ctx.fillStyle = COLORS.target;
    ctx.beginPath();
    ctx.moveTo(tx, groundY + 3);
    ctx.lineTo(tx - 6, groundY + 12);
    ctx.lineTo(tx + 6, groundY + 12);
    ctx.closePath();
    ctx.fill();
  }

  function drawWind(wind) {
    if (Math.abs(wind) < 1e-6) return;
    // Streaks in the sky, length proportional to the wind force.
    const len = (Math.abs(wind) / params.maxForce) * FORCE_ARROW_FULL_SCALE * view.ppm * 3;
    const dir = Math.sign(wind);
    const rows = [0.18, 0.3, 0.42];
    rows.forEach((fy, i) => {
      const y = view.height * fy;
      const cx = view.width * (0.2 + 0.3 * i);
      arrow(cx - (dir * len) / 2, y, cx + (dir * len) / 2, y, COLORS.wind, 1.5);
    });
    text(`wind ${wind.toFixed(1)} N`, 12, 16, { color: COLORS.wind, align: "left" });
  }

  function drawCart(state, force) {
    const { ppm } = view;
    const c = worldToScreen(state[X], 0);
    const w = CART_W * ppm;
    const h = CART_H * ppm;
    const top = view.pivotY;

    // Body
    ctx.fillStyle = COLORS.cart;
    ctx.strokeStyle = COLORS.cartEdge;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.roundRect(c.x - w / 2, top, w, h, 5);
    ctx.fill();
    ctx.stroke();

    // Wheels
    ctx.fillStyle = COLORS.wheel;
    for (const side of [-0.3, 0.3]) {
      ctx.beginPath();
      ctx.arc(c.x + side * w, view.groundY - WHEEL_R * ppm, WHEEL_R * ppm, 0, Math.PI * 2);
      ctx.fill();
    }

    // Motor force: arrow from the cart's edge, in the direction of the push.
    const len = (force / params.maxForce) * FORCE_ARROW_FULL_SCALE * ppm;
    const y = top + h / 2;
    const x0 = c.x + Math.sign(force) * (w / 2 + 4);
    arrow(x0, y, x0 + len, y, COLORS.force);
    if (Math.abs(len) > 18) {
      text(`${force.toFixed(1)} N`, x0 + len / 2, y - 11, { color: COLORS.force });
    }
  }

  function drawPole(state, { highlight }) {
    const { ppm } = view;
    const pivot = worldToScreen(state[X], 0);
    const tipW = tipPosition(state);
    const tip = worldToScreen(tipW.x, tipW.y);

    ctx.strokeStyle = COLORS.pole;
    ctx.lineWidth = Math.max(4, POLE_WIDTH * ppm);
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(pivot.x, pivot.y);
    ctx.lineTo(tip.x, tip.y);
    ctx.stroke();

    // The ball at the tip is the grab handle.
    const r = Math.max(7, BOB_RADIUS * ppm);
    ctx.fillStyle = COLORS.pole;
    ctx.beginPath();
    ctx.arc(tip.x, tip.y, r, 0, Math.PI * 2);
    ctx.fill();
    if (highlight) {
      ctx.strokeStyle = COLORS.force;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(tip.x, tip.y, r + 5, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.fillStyle = COLORS.pivot;
    ctx.beginPath();
    ctx.arc(pivot.x, pivot.y, 5, 0, Math.PI * 2);
    ctx.fill();

    return tip;
  }

  function drawSpring(tip, anchorWorld) {
    const a = worldToScreen(anchorWorld.x, anchorWorld.y);
    ctx.strokeStyle = COLORS.spring;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(a.x, a.y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = COLORS.spring;
    ctx.beginPath();
    ctx.arc(a.x, a.y, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawTap(tip, direction) {
    // An arrow arriving at the ball from the side the tap came from.
    const len = 0.35 * view.ppm;
    const r = Math.max(7, BOB_RADIUS * view.ppm) + 4;
    const x1 = tip.x - direction * r;
    arrow(x1 - direction * len, tip.y, x1, tip.y, COLORS.force, 3);
  }

  function drawOverlay(title, subtitle) {
    ctx.fillStyle = COLORS.overlay;
    ctx.fillRect(0, 0, view.width, view.height);
    text(title, view.width / 2, view.height / 2 - 12, { color: COLORS.overlayText, size: 20, weight: 600 });
    text(subtitle, view.width / 2, view.height / 2 + 16, { color: COLORS.label, size: 13 });
  }

  /**
   * scene = {
   *   force      motor force actually applied (N)
   *   targetX    controller's position setpoint (m)
   *   wind       constant external force on the cart (N)
   *   anchor     {x, y} world point the pointer is holding, or null
   *   hoverTip   true if the pointer is over the grab handle
   *   tap        +1 / -1 while a tap is being shown, else 0
   *   overlay    { title, subtitle } or null
   * }
   */
  function draw(state, scene) {
    ctx.clearRect(0, 0, view.width, view.height);
    drawWind(scene.wind);
    drawTrack(scene.targetX);
    drawCart(state, scene.force);
    const tip = drawPole(state, { highlight: scene.hoverTip || !!scene.anchor });
    if (scene.anchor) drawSpring(tip, scene.anchor);
    if (scene.tap) drawTap(tip, scene.tap);
    if (scene.overlay) drawOverlay(scene.overlay.title, scene.overlay.subtitle);
  }

  return { draw, worldToScreen, screenToWorld, view };
}
