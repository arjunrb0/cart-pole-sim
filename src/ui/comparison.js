/**
 * comparison.js — the "PID vs LQR" section: run the benchmark, show a
 * results table for every scenario and a chart for the selected one.
 *
 * Each run uses a SNAPSHOT of the current slider settings, taken when you
 * press the button, so tuning afterwards doesn't change results already
 * on screen.
 */

import { runComparison } from "../analysis/benchmark.js";
import { SETTLE_BAND } from "../analysis/settling.js";
import { params } from "../physics/cartPole.js";
import { createComparisonChart, SERIES_COLORS } from "../render/comparisonChart.js";

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

/**
 * @param root       container element
 * @param snapshot   () => { PID: (target) => controller, LQR: ... }, each
 *                   built from a copy of the CURRENT settings
 */
export function createComparison(root, { snapshot }) {
  const head = el("div", "cmp-head");
  const intro = el(
    "p",
    "cmp-intro",
    `Both controllers face identical scenarios: same physics, same 500 Hz sampling, same ${params.maxForce} N motor limit, ` +
      `fresh controller state. Settled means |θ| < ${SETTLE_BAND.thetaDeg}° and |x − target| < ${SETTLE_BAND.x * 100} cm, ` +
      `and staying there. Uses your current slider settings.`,
  );
  const runButton = el("button", "cmp-run", "Run comparison");
  runButton.type = "button";
  head.append(intro, runButton);

  // Filter row: scenario tabs on the left, legend on the right.
  const controls = el("div", "cmp-controls");
  const tabs = el("div", "cmp-tabs");
  tabs.setAttribute("role", "tablist");
  tabs.setAttribute("aria-label", "Scenario");
  const legend = el("div", "cmp-legend");
  for (const [name, color] of Object.entries(SERIES_COLORS)) {
    const item = el("span", "cmp-legend-item");
    const key = el("span", "line-key");
    key.style.background = color;
    item.append(key, document.createTextNode(name));
    legend.append(item);
  }
  controls.append(tabs, legend);

  const chartWrap = el("div", "cmp-chart-wrap");
  const canvas = el("canvas", "cmp-chart");
  canvas.tabIndex = 0;
  canvas.setAttribute("aria-label", "Comparison chart. Focus and use the left and right arrow keys to read values.");
  const tooltip = el("div", "cmp-tooltip");
  tooltip.hidden = true;
  chartWrap.append(canvas, tooltip);

  const tableWrap = el("div", "cmp-table-wrap");
  root.append(head, controls, chartWrap, tableWrap);

  const chart = createComparisonChart(canvas, tooltip);
  let results = [];
  let selected = 0;

  function select(i) {
    selected = i;
    [...tabs.children].forEach((b, j) => b.setAttribute("aria-selected", String(j === i)));
    [...tableWrap.querySelectorAll("tbody tr")].forEach((tr, j) => tr.classList.toggle("is-selected", j === i));
    chart.show(results[i]);
  }

  const fmtSettle = (r) => (r.failure ? "failed" : Number.isFinite(r.settling) ? `${r.settling.toFixed(2)} s` : "not settled");
  const fmtFail = (r) => (r.failure ? `${r.failure.replace(/^The /, "")} at ${r.times[r.times.length - 1].toFixed(2)} s` : "");

  function renderTable() {
    const table = el("table", "cmp-table");
    const thead = el("thead");
    const h1 = el("tr");
    h1.append(el("th", "", "Scenario"));
    for (const title of ["Settling time", "Peak force", "Max cart travel"]) {
      const th = el("th", "group", title);
      th.colSpan = 2;
      h1.append(th);
    }
    const h2 = el("tr");
    h2.append(el("th"));
    for (let i = 0; i < 3; i++) for (const name of Object.keys(SERIES_COLORS)) h2.append(el("th", "sub", name));
    thead.append(h1, h2);

    const tbody = el("tbody");
    results.forEach((entry, i) => {
      const tr = el("tr");
      tr.tabIndex = 0;
      tr.addEventListener("click", () => select(i));
      tr.addEventListener("keydown", (e) => {
        if (e.key === "Enter" || e.key === " ") {
          e.preventDefault();
          select(i);
        }
      });
      tr.append(el("td", "scenario", entry.scenario.label));

      const runs = Object.values(entry.results);
      // Settling: the faster finisher is bold (a failure never wins).
      const times = runs.map((r) => (r.failure ? Infinity : r.settling));
      const best = Math.min(...times);
      runs.forEach((r, j) => {
        const td = el("td", "num", fmtSettle(r));
        if (times[j] === best && Number.isFinite(best) && times.filter((t) => t === best).length === 1) td.classList.add("best");
        if (r.failure) td.title = fmtFail(r);
        tr.append(td);
      });
      runs.forEach((r) => tr.append(el("td", "num", `${r.peakForce.toFixed(1)} N`)));
      runs.forEach((r) => tr.append(el("td", "num", `${r.maxTravel.toFixed(2)} m`)));
      tbody.append(tr);
    });
    table.append(thead, tbody);

    const notes = el("p", "cmp-note");
    const failures = results.flatMap((e) =>
      Object.entries(e.results)
        .filter(([, r]) => r.failure)
        .map(([name, r]) => `${name} on “${e.scenario.label}”: ${fmtFail(r).toLowerCase()}.`),
    );
    notes.textContent = failures.length ? `Failures — ${failures.join(" ")}` : "";
    tableWrap.replaceChildren(table, notes);
  }

  function run() {
    results = runComparison(snapshot());
    tabs.replaceChildren(
      ...results.map((entry, i) => {
        const b = el("button", "cmp-tab", entry.scenario.label);
        b.type = "button";
        b.setAttribute("role", "tab");
        b.addEventListener("click", () => select(i));
        return b;
      }),
    );
    renderTable();
    select(Math.min(selected, results.length - 1));
  }

  runButton.addEventListener("click", run);
  return { run };
}
