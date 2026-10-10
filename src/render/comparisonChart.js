/**
 * comparisonChart.js — PID and LQR on the same axes, for one scenario.
 *
 * Three stacked lanes sharing one time axis (pole angle, cart position,
 * motor force). Each lane has its OWN y-scale: they are separate charts
 * stacked, never two scales on one plot. Colour carries controller
 * identity only, and every value is also reachable as text (the legend,
 * the settling labels, the tooltip, and the results table beside it).
 *
 * Hover (or focus + arrow keys) shows a crosshair that snaps to the nearest
 * sample, with one tooltip listing every series at that instant.
 */

// Categorical slots 1 and 2, validated against the panel surface (#1a2130)
// for colour-vision deficiency, normal-vision separation and contrast.
export const SERIES_COLORS = { PID: "#3987e5", LQR: "#d95926" };

const LANES = [
  { key: "theta", label: "Pole angle", unit: "°", digits: 2 },
  { key: "x", label: "Cart position", unit: " m", digits: 3 },
  { key: "u", label: "Motor force", unit: " N", digits: 1 },
];

const INK = { primary: "#e6ebf2", secondary: "#a9b3c4", muted: "#6b768a", grid: "#2a3242", surface: "#1a2130" };
const FONT = "ui-sans-serif, system-ui, sans-serif";
const MONO = "ui-monospace, SFMono-Regular, Menlo, monospace";

/**
 * Tick spacing of 1, 2 or 5 x 10^k so axis labels are clean numbers.
 * Rounds the raw spacing to the NEAREST nice value (Heckbert's "nice
 * numbers"). Always rounding up was a bug: a raw spacing of 5.06 became 10,
 * which left a -5.5..9.6 axis with a single tick.
 */
function niceTicks(min, max, target = 4) {
  const raw = (max - min) / target || 1;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const f = raw / mag;
  const step = (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * mag;
  const ticks = [];
  for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-9; v += step) ticks.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  const digits = Math.max(0, -Math.floor(Math.log10(step)));
  return { ticks, digits };
}

export function createComparisonChart(canvas, tooltip) {
  const ctx = canvas.getContext("2d");
  let width = 0;
  let height = 0;
  let entry = null; // { scenario, results: { PID: run, LQR: run } }
  let hoverIndex = null;
  let layout = null; // computed in draw(), reused by hover

  function resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    width = rect.width;
    height = rect.height;
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw();
  }
  new ResizeObserver(resize).observe(canvas);

  const names = () => Object.keys(entry.results);

  /** Show enough time to see both controllers finish (settle or fail). */
  function timeWindow() {
    let end = 4;
    for (const r of Object.values(entry.results)) {
      const last = r.times[r.times.length - 1];
      end = Math.max(end, Number.isFinite(r.settling) ? r.settling : last);
    }
    const duration = Math.max(...Object.values(entry.results).map((r) => r.times[r.times.length - 1]));
    return Math.min(duration, end * 1.2 + 0.5);
  }

  function draw() {
    ctx.clearRect(0, 0, width, height);
    if (!entry || width === 0) return;

    const padL = 58;
    const padR = 14;
    const headerH = 40; // two rows of settling labels
    const axisH = 24;
    const gap = 10;
    const titleH = 18; // each lane's title sits in its own row, above its plot
    const laneH = (height - headerH - axisH - (gap + titleH) * LANES.length + gap) / LANES.length;
    const plotW = width - padL - padR;
    const tMax = timeWindow();
    const toX = (t) => padL + (t / tMax) * plotW;
    layout = { padL, plotW, tMax, toX, laneTops: [], scales: [] };

    // --- time gridlines (drawn first, behind everything) ---
    const tTicks = niceTicks(0, tMax, Math.max(3, Math.floor(plotW / 90)));
    ctx.strokeStyle = INK.grid;
    ctx.lineWidth = 1;
    for (const t of tTicks.ticks) {
      const x = Math.round(toX(t)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(x, headerH);
      ctx.lineTo(x, height - axisH);
      ctx.stroke();
    }

    LANES.forEach((lane, li) => {
      const titleTop = headerH + li * (titleH + laneH + gap);
      const top = titleTop + titleH;
      layout.laneTops.push(top);

      // y-range over both series within the visible window, always including 0
      let lo = 0;
      let hi = 0;
      for (const r of Object.values(entry.results)) {
        const arr = r[lane.key];
        for (let i = 0; i < arr.length && r.times[i] <= tMax; i++) {
          lo = Math.min(lo, arr[i]);
          hi = Math.max(hi, arr[i]);
        }
      }
      if (hi - lo < 1e-9) hi = lo + 1;
      const pad = (hi - lo) * 0.08;
      lo -= pad;
      hi += pad;
      const toY = (v) => top + laneH - ((v - lo) / (hi - lo)) * laneH;
      layout.scales.push(toY);

      // horizontal gridlines + tick labels
      const yTicks = niceTicks(lo, hi, 3);
      ctx.font = `10px ${MONO}`;
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      for (const v of yTicks.ticks) {
        const y = Math.round(toY(v)) + 0.5;
        ctx.strokeStyle = v === 0 ? "#3a4454" : INK.grid;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(padL + plotW, y);
        ctx.stroke();
        ctx.fillStyle = INK.muted;
        ctx.fillText(v.toFixed(yTicks.digits), padL - 6, y);
      }

      // lane title, in text ink
      ctx.fillStyle = INK.secondary;
      ctx.font = `600 11px ${FONT}`;
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(`${lane.label} (${lane.unit.trim()})`, padL, titleTop + 3);

      // series: 2px lines, round joins
      for (const name of names()) {
        const r = entry.results[name];
        const arr = r[lane.key];
        const stride = Math.max(1, Math.floor(arr.length / (plotW * 2)));
        ctx.strokeStyle = SERIES_COLORS[name];
        ctx.lineWidth = 2;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < arr.length && r.times[i] <= tMax; i += stride) {
          const x = toX(r.times[i]);
          const y = toY(arr[i]);
          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // A failed run ends in a marker with a surface ring.
        if (r.failure && li === 1) {
          const k = arr.length - 1;
          const x = toX(r.times[k]);
          const y = toY(arr[k]);
          ctx.fillStyle = INK.surface;
          ctx.beginPath();
          ctx.arc(x, y, 6, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillStyle = SERIES_COLORS[name];
          ctx.beginPath();
          ctx.arc(x, y, 4, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    });

    // --- settling markers: a vertical line per controller + a header label ---
    names().forEach((name, row) => {
      const r = entry.results[name];
      const y = 8 + row * 16;
      // line key in the series colour, text in ink
      ctx.strokeStyle = SERIES_COLORS[name];
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(padL, y + 6);
      ctx.lineTo(padL + 14, y + 6);
      ctx.stroke();
      ctx.fillStyle = INK.primary;
      ctx.font = `11px ${FONT}`;
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      const text = r.failure
        ? `${name}: ${r.failure.toLowerCase()} at ${r.times[r.times.length - 1].toFixed(2)} s`
        : Number.isFinite(r.settling)
          ? `${name}: settled in ${r.settling.toFixed(2)} s`
          : `${name}: did not settle within ${r.times[r.times.length - 1].toFixed(0)} s`;
      ctx.fillText(text, padL + 20, y + 6);

      if (Number.isFinite(r.settling) && r.settling <= tMax) {
        const x = Math.round(toX(r.settling)) + 0.5;
        ctx.strokeStyle = SERIES_COLORS[name];
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(x, headerH);
        ctx.lineTo(x, height - axisH);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    });

    // --- time axis labels ---
    ctx.fillStyle = INK.muted;
    ctx.font = `10px ${MONO}`;
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (const t of tTicks.ticks) ctx.fillText(`${t.toFixed(tTicks.digits)} s`, toX(t), height - axisH + 6);

    if (hoverIndex !== null) drawCrosshair();
  }

  function drawCrosshair() {
    const first = Object.values(entry.results)[0];
    const t = first.times[Math.min(hoverIndex, first.times.length - 1)];
    if (t > layout.tMax) return;
    const x = Math.round(layout.toX(t)) + 0.5;
    ctx.strokeStyle = INK.secondary;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, layout.laneTops[0]);
    ctx.lineTo(x, height - 24);
    ctx.stroke();

    LANES.forEach((lane, li) => {
      for (const name of names()) {
        const r = entry.results[name];
        if (hoverIndex >= r.times.length) continue; // that run already ended
        const y = layout.scales[li](r[lane.key][hoverIndex]);
        ctx.fillStyle = INK.surface;
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = SERIES_COLORS[name];
        ctx.beginPath();
        ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    });
  }

  // --- tooltip (DOM, built with textContent) --------------------------------
  function renderTooltip(px) {
    const first = Object.values(entry.results)[0];
    const dt = first.times[1] - first.times[0];
    tooltip.replaceChildren();
    const head = document.createElement("div");
    head.className = "cmp-tip-time";
    head.textContent = `t = ${(hoverIndex * dt).toFixed(2)} s`;
    tooltip.append(head);

    for (const name of names()) {
      const r = entry.results[name];
      const row = document.createElement("div");
      row.className = "cmp-tip-row";
      const key = document.createElement("span");
      key.className = "line-key";
      key.style.background = SERIES_COLORS[name];
      const label = document.createElement("span");
      label.className = "cmp-tip-name";
      label.textContent = name;
      row.append(key, label);
      if (hoverIndex >= r.times.length) {
        const ended = document.createElement("span");
        ended.className = "cmp-tip-ended";
        ended.textContent = "ended";
        row.append(ended);
      } else {
        for (const lane of LANES) {
          const v = document.createElement("b");
          v.textContent = `${r[lane.key][hoverIndex].toFixed(lane.digits)}${lane.unit}`;
          row.append(v);
        }
      }
      tooltip.append(row);
    }
    tooltip.hidden = false;
    // Keep the tooltip inside the chart: flip to the left of the crosshair
    // when there isn't room on the right.
    const w = tooltip.offsetWidth;
    const left = px + 14 + w > width ? px - 14 - w : px + 14;
    tooltip.style.left = `${Math.max(0, left)}px`;
    tooltip.style.top = `${layout.laneTops[0]}px`;
  }

  function setHoverFromX(px) {
    if (!entry || !layout) return;
    const first = Object.values(entry.results)[0];
    const dt = first.times[1] - first.times[0];
    const t = Math.min(Math.max(0, ((px - layout.padL) / layout.plotW) * layout.tMax), layout.tMax);
    const longest = Math.max(...Object.values(entry.results).map((r) => r.times.length));
    hoverIndex = Math.min(Math.round(t / dt), longest - 1);
    draw();
    renderTooltip(layout.toX(hoverIndex * dt));
  }

  function clearHover() {
    hoverIndex = null;
    tooltip.hidden = true;
    draw();
  }

  canvas.addEventListener("pointermove", (e) => setHoverFromX(e.clientX - canvas.getBoundingClientRect().left));
  canvas.addEventListener("pointerleave", clearHover);
  canvas.addEventListener("blur", clearHover);
  // Keyboard: focus the chart, then step through time with the arrow keys.
  canvas.addEventListener("keydown", (e) => {
    if (!entry || !layout) return;
    const first = Object.values(entry.results)[0];
    const dt = first.times[1] - first.times[0];
    const stepSamples = Math.round(0.1 / dt);
    if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
    e.preventDefault();
    const current = hoverIndex ?? 0;
    const next = Math.max(0, current + (e.key === "ArrowRight" ? stepSamples : -stepSamples));
    setHoverFromX(layout.toX(next * dt));
  });

  return {
    show(next) {
      entry = next;
      hoverIndex = null;
      tooltip.hidden = true;
      draw();
    },
  };
}
