/**
 * controls.js — builds slider panels from a plain description.
 *
 * Each slider is described as DATA:
 *
 *   { obj: gains.angle, key: "kp", label: "Kp", min: 0, max: 150, step: 0.5 }
 *
 * and is bound BY REFERENCE: moving it writes obj[key] directly. Since the
 * controller reads its gains object fresh every physics step, the change
 * takes effect within 2 ms. This module knows nothing about PID, physics,
 * or what any of the numbers mean.
 */

/** document.createElement with a class name, to keep the builders short. */
function el(tag, className, textContent) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (textContent !== undefined) node.textContent = textContent;
  return node;
}

function decimalsFor(step) {
  const s = String(step);
  return s.includes(".") ? s.split(".")[1].length : 0;
}

function createSlider(parent, spec) {
  const { obj, key, label, unit = "", min, max, step, hint } = spec;
  const digits = spec.digits ?? decimalsFor(step);

  const row = el("label", "slider");
  const head = el("div", "slider-head");
  const name = el("span", "slider-label", label);
  const value = el("output", "slider-value");
  head.append(name, value);

  const input = el("input");
  input.type = "range";
  input.min = min;
  input.max = max;
  input.step = step;

  row.append(head, input);
  if (hint) row.append(el("span", "slider-hint", hint));
  parent.append(row);

  // The value this slider last displayed, so refresh() can skip the DOM
  // entirely when nothing changed — it is called every frame.
  let shown;

  function show() {
    shown = obj[key];
    value.textContent = `${shown.toFixed(digits)}${unit ? " " + unit : ""}`;
  }

  // The browser calls this every time the thumb moves.
  input.addEventListener("input", () => {
    obj[key] = Number(input.value);
    show();
  });

  // Pull the current value back from the object if something other than
  // this slider changed it (the "default gains" button, the console), so
  // the panel never displays a value that isn't the real one.
  function refresh() {
    if (obj[key] === shown) return;
    input.value = obj[key];
    show();
  }
  refresh();

  return { refresh };
}

/**
 * sections = [{ title, note?, sliders: [spec, ...] }, ...]
 * Returns {
 *   refresh()   resyncs any slider whose object changed; cheap enough to
 *               call every frame
 *   groups      the section elements, so callers can show/hide them or
 *               append extra content
 * }
 */
export function createPanel(container, sections) {
  const sliders = [];
  const groups = [];
  for (const section of sections) {
    const group = el("section", "group");
    group.append(el("h2", "group-title", section.title));
    if (section.note) group.append(el("p", "group-note", section.note));
    for (const spec of section.sliders) sliders.push(createSlider(group, spec));
    container.append(group);
    groups.push(group);
  }
  return {
    groups,
    refresh() {
      for (const s of sliders) s.refresh();
    },
  };
}
