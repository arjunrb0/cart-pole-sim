/**
 * plot.js — a scrolling strip chart of the last few seconds.
 *
 * Three stacked lanes sharing one time axis: pole angle, cart position and
 * motor force. Each lane autoscales to what is on screen (with a minimum
 * span), so a 0.5 degree wobble is still visible while you fine-tune.
 *
 * Samples are stored in fixed-size typed arrays used as ring buffers: when
 * the buffer is full, the newest sample overwrites the oldest. Float64Array
 * is the closest JS gets to a numpy array: a fixed length of raw numbers.
 */

const LANES = [
  { key: "theta", label: "θ", unit: "°", color: "#e0a24a", minSpan: 2 },
  { key: "x", label: "x", unit: "m", color: "#5b7fd4", minSpan: 0.1 },
  { key: "u", label: "u", unit: "N", color: "#e6ebf2", minSpan: 2 },
];

export function createPlot(canvas, { seconds = 10, sampleHz = 100 } = {}) {
  const ctx = canvas.getContext("2d");
  const capacity = seconds * sampleHz;
  const buffers = Object.fromEntries(LANES.map((l) => [l.key, new Float64Array(capacity)]));
  const times = new Float64Array(capacity);
  let head = 0; // next write position
  let count = 0; // how many slots hold real data

  let width = 0;
  let height = 0;
  function resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    width = rect.width;
    height = rect.height;
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  new ResizeObserver(resize).observe(canvas);
  resize();

  function push(t, sample) {
    times[head] = t;
    for (const l of LANES) buffers[l.key][head] = sample[l.key];
    head = (head + 1) % capacity;
    count = Math.min(count + 1, capacity);
  }

  function clear() {
    head = 0;
    count = 0;
  }

  // i-th oldest sample -> its slot in the ring buffer
  const slot = (i) => (head - count + i + capacity) % capacity;

  /** refs: optional dashed reference line per lane, e.g. { x: targetX } */
  function draw(refs = {}) {
    ctx.clearRect(0, 0, width, height);
    const labelW = 64;
    const plotW = width - labelW - 8;
    const laneH = height / LANES.length;
    const tEnd = count ? times[slot(count - 1)] : 0;
    const toX = (t) => labelW + ((t - (tEnd - seconds)) / seconds) * plotW;

    LANES.forEach((lane, li) => {
      const top = li * laneH;
      const mid = top + laneH / 2;
      const buf = buffers[lane.key];

      // Autoscale to the largest magnitude currently visible.
      let peak = lane.minSpan / 2;
      for (let i = 0; i < count; i++) peak = Math.max(peak, Math.abs(buf[slot(i)]));
      const ref = refs[lane.key];
      if (ref !== undefined) peak = Math.max(peak, Math.abs(ref));
      const scale = (laneH / 2 - 6) / (peak * 1.1);

      // Lane separator and zero line
      ctx.strokeStyle = "#2a3242";
      ctx.lineWidth = 1;
      if (li > 0) {
        ctx.beginPath();
        ctx.moveTo(0, top);
        ctx.lineTo(width, top);
        ctx.stroke();
      }
      ctx.strokeStyle = "#323b4d";
      ctx.beginPath();
      ctx.moveTo(labelW, mid);
      ctx.lineTo(labelW + plotW, mid);
      ctx.stroke();

      if (ref !== undefined && ref !== 0) {
        ctx.strokeStyle = "#5fbf8a";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(labelW, mid - ref * scale);
        ctx.lineTo(labelW + plotW, mid - ref * scale);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // Trace
      ctx.strokeStyle = lane.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < count; i++) {
        const k = slot(i);
        const px = toX(times[k]);
        const py = mid - buf[k] * scale;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // Labels: name, current value, scale
      const latest = count ? buf[slot(count - 1)] : 0;
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillStyle = lane.color;
      ctx.font = "600 12px ui-sans-serif, system-ui, sans-serif";
      ctx.fillText(lane.label, 8, mid - 8);
      ctx.font = "11px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = "#c9d1dd";
      ctx.fillText(`${latest >= 0 ? " " : ""}${latest.toFixed(2)}${lane.unit}`, 8, mid + 8);
      // Scale lives in the label column, where the scrolling trace can't
      // draw over it.
      ctx.fillStyle = "#6b768a";
      ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillText(`±${(peak * 1.1).toPrecision(2)}`, 8, top + 10);
    });
  }

  return { push, clear, draw };
}
