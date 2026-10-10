/**
 * linalg.js — the small amount of linear algebra LQR needs.
 *
 * JS has no numpy, so this is the minimum written out by hand. A matrix is
 * an array of rows: [[a, b], [c, d]]. A column vector is an n x 1 matrix.
 * Everything here is O(n^3) or less on n = 4, so clarity wins over speed.
 */

export function zeros(rows, cols) {
  return Array.from({ length: rows }, () => new Array(cols).fill(0));
}

export function identity(n) {
  const I = zeros(n, n);
  for (let i = 0; i < n; i++) I[i][i] = 1;
  return I;
}

export function transpose(M) {
  return M[0].map((_, j) => M.map((row) => row[j]));
}

export function matMul(A, B) {
  const out = zeros(A.length, B[0].length);
  for (let i = 0; i < A.length; i++)
    for (let k = 0; k < B.length; k++) {
      const a = A[i][k];
      if (a === 0) continue;
      for (let j = 0; j < B[0].length; j++) out[i][j] += a * B[k][j];
    }
  return out;
}

export function add(A, B) {
  return A.map((row, i) => row.map((v, j) => v + B[i][j]));
}

export function sub(A, B) {
  return A.map((row, i) => row.map((v, j) => v - B[i][j]));
}

export function scale(A, k) {
  return A.map((row) => row.map((v) => v * k));
}

export function diag(values) {
  const D = zeros(values.length, values.length);
  values.forEach((v, i) => (D[i][i] = v));
  return D;
}

/** Largest absolute entry: a simple matrix "size" for convergence checks. */
export function maxAbs(A) {
  return Math.max(...A.flat().map(Math.abs));
}

/**
 * Solve M x = b by Gaussian elimination with partial pivoting.
 * M is n x n, b is a plain array of length n. Throws if M is singular.
 *
 * Partial pivoting (swapping in the row with the largest pivot) keeps
 * round-off from blowing up when a pivot would otherwise be tiny.
 */
export function solve(M, b) {
  const n = b.length;
  // Augmented matrix [M | b], copied so the inputs are untouched.
  const a = M.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
    if (Math.abs(a[pivot][col]) < 1e-14) throw new Error("solve(): matrix is singular");
    [a[col], a[pivot]] = [a[pivot], a[col]];

    for (let r = col + 1; r < n; r++) {
      const f = a[r][col] / a[col][col];
      if (f === 0) continue;
      for (let c = col; c <= n; c++) a[r][c] -= f * a[col][c];
    }
  }

  // Back-substitution.
  const x = new Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = a[r][n];
    for (let c = r + 1; c < n; c++) s -= a[r][c] * x[c];
    x[r] = s / a[r][r];
  }
  return x;
}

/** Rank by Gaussian elimination; `tol` is relative to the largest entry. */
export function rank(M, tol = 1e-9) {
  const a = M.map((row) => [...row]);
  const rows = a.length;
  const cols = a[0].length;
  const eps = tol * Math.max(1, maxAbs(a));
  let r = 0;
  for (let c = 0; c < cols && r < rows; c++) {
    let p = r;
    for (let i = r + 1; i < rows; i++) if (Math.abs(a[i][c]) > Math.abs(a[p][c])) p = i;
    if (Math.abs(a[p][c]) <= eps) continue;
    [a[r], a[p]] = [a[p], a[r]];
    for (let i = r + 1; i < rows; i++) {
      const f = a[i][c] / a[r][c];
      for (let j = c; j < cols; j++) a[i][j] -= f * a[r][j];
    }
    r++;
  }
  return r;
}

/**
 * Characteristic polynomial det(sI - A), by the Faddeev-LeVerrier
 * recurrence. Returns coefficients highest power first: [1, c1, ..., cn].
 * Fine for small, well-scaled matrices like ours.
 */
export function charPoly(A) {
  const n = A.length;
  const I = identity(n);
  const c = [1];
  let Mk = zeros(n, n);
  for (let k = 1; k <= n; k++) {
    Mk = add(matMul(A, Mk), scale(I, c[k - 1]));
    const AM = matMul(A, Mk);
    let trace = 0;
    for (let i = 0; i < n; i++) trace += AM[i][i];
    c.push(-trace / k);
  }
  return c;
}

/**
 * All complex roots of a real polynomial (highest power first), by the
 * Durand-Kerner method: refine n guesses simultaneously, each one pushed
 * away from the others. Returns [{ re, im }, ...] sorted by real part,
 * least stable (rightmost) first.
 */
export function polyRoots(coeffs) {
  const n = coeffs.length - 1;
  const a = coeffs.map((v) => v / coeffs[0]);
  const mul = (p, q) => ({ re: p.re * q.re - p.im * q.im, im: p.re * q.im + p.im * q.re });
  const div = (p, q) => {
    const d = q.re * q.re + q.im * q.im;
    return { re: (p.re * q.re + p.im * q.im) / d, im: (p.im * q.re - p.re * q.im) / d };
  };
  // Starting guesses on a circle sized to the roots, at a non-symmetric angle.
  const radius = 1 + Math.max(...a.slice(1).map(Math.abs));
  let z = Array.from({ length: n }, (_, k) => ({
    re: radius * Math.cos((2 * Math.PI * k) / n + 0.4),
    im: radius * Math.sin((2 * Math.PI * k) / n + 0.4),
  }));

  for (let iter = 0; iter < 500; iter++) {
    let moved = 0;
    z = z.map((zi, i) => {
      let p = { re: 1, im: 0 }; // Horner: evaluate the polynomial at zi
      for (let k = 1; k <= n; k++) {
        p = mul(p, zi);
        p.re += a[k];
      }
      let den = { re: 1, im: 0 };
      z.forEach((zj, j) => {
        if (j !== i) den = mul(den, { re: zi.re - zj.re, im: zi.im - zj.im });
      });
      const step = div(p, den);
      moved = Math.max(moved, Math.hypot(step.re, step.im));
      return { re: zi.re - step.re, im: zi.im - step.im };
    });
    if (moved < 1e-13) break;
  }

  // Snap numerically-tiny imaginary parts to zero: those roots are real.
  for (const r of z) if (Math.abs(r.im) < 1e-9 * (1 + Math.abs(r.re))) r.im = 0;
  return z.sort((p, q) => q.re - p.re);
}

export function eigenvalues(A) {
  return polyRoots(charPoly(A));
}
