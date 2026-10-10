/**
 * The hand-written linear algebra, against answers known in closed form.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import { charPoly, eigenvalues, matMul, rank, solve, transpose } from "../src/control/linalg.js";

const close = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) < tol, `${a} vs ${b}`);

test("solve() inverts a system that needs pivoting", () => {
  // Zero in the top-left corner: fails without partial pivoting.
  const M = [[0, 2, 1], [1, 1, 1], [2, 1, 3]];
  const x = solve(M, [7, 6, 13]); // = M · [1, 2, 3]
  [1, 2, 3].forEach((v, i) => close(x[i], v));
});

test("solve() rejects a singular matrix", () => {
  assert.throws(() => solve([[1, 2], [2, 4]], [1, 2]), /singular/);
});

test("eigenvalues of a symmetric matrix are real", () => {
  const ev = eigenvalues([[2, 1], [1, 2]]).map((e) => e.re);
  close(ev[0], 3);
  close(ev[1], 1);
});

test("eigenvalues of a rotation are a complex-conjugate pair", () => {
  const ev = eigenvalues([[0, -1], [1, 0]]);
  ev.forEach((e) => close(e.re, 0));
  close(Math.abs(ev[0].im), 1);
  close(ev[0].im, -ev[1].im);
});

test("charPoly of a companion matrix recovers its polynomial", () => {
  // s^3 + 6s^2 + 11s + 6 = (s+1)(s+2)(s+3)
  const c = charPoly([[0, 1, 0], [0, 0, 1], [-6, -11, -6]]);
  [1, 6, 11, 6].forEach((v, i) => close(c[i], v));
  const roots = eigenvalues([[0, 1, 0], [0, 0, 1], [-6, -11, -6]]).map((e) => e.re);
  [-1, -2, -3].forEach((v, i) => close(roots[i], v, 1e-8));
});

test("rank, transpose and matMul", () => {
  assert.equal(rank([[1, 2], [2, 4]]), 1);
  assert.equal(rank([[1, 0], [0, 1]]), 2);
  assert.deepEqual(transpose([[1, 2, 3]]), [[1], [2], [3]]);
  assert.deepEqual(matMul([[1, 2]], [[3], [4]]), [[11]]);
});
