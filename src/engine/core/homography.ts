/**
 * Homography estimation from point correspondences via the normalized DLT,
 * solved with plain Gaussian elimination (no external linear algebra).
 *
 * Homographies are stored row-major as 9-element arrays with h[8] === 1
 * whenever possible.
 */

export type Mat3 = number[]; // row-major, length 9

export interface Point2 {
  x: number;
  y: number;
}

/** Solve A x = b for a dense n x n system with partial pivoting. Returns null if singular. */
export function solveLinearSystem(A: Float64Array, b: Float64Array, n: number): Float64Array | null {
  // Augmented in-place elimination on copies.
  const M = new Float64Array(A);
  const rhs = new Float64Array(b);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    let maxAbs = Math.abs(M[col * n + col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(M[r * n + col]);
      if (v > maxAbs) {
        maxAbs = v;
        pivot = r;
      }
    }
    if (maxAbs < 1e-12) return null;
    if (pivot !== col) {
      for (let c = col; c < n; c++) {
        const tmp = M[col * n + c];
        M[col * n + c] = M[pivot * n + c];
        M[pivot * n + c] = tmp;
      }
      const tmp = rhs[col];
      rhs[col] = rhs[pivot];
      rhs[pivot] = tmp;
    }
    const inv = 1 / M[col * n + col];
    for (let r = col + 1; r < n; r++) {
      const f = M[r * n + col] * inv;
      if (f === 0) continue;
      M[r * n + col] = 0;
      for (let c = col + 1; c < n; c++) M[r * n + c] -= f * M[col * n + c];
      rhs[r] -= f * rhs[col];
    }
  }
  const x = new Float64Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = rhs[r];
    for (let c = r + 1; c < n; c++) s -= M[r * n + c] * x[c];
    x[r] = s / M[r * n + r];
  }
  return x;
}

interface Normalization {
  T: Mat3;
  pts: Point2[];
}

/** Hartley normalization: translate centroid to origin, scale mean distance to sqrt(2). */
function normalizePoints(pts: Point2[]): Normalization {
  let cx = 0;
  let cy = 0;
  for (const p of pts) {
    cx += p.x;
    cy += p.y;
  }
  cx /= pts.length;
  cy /= pts.length;
  let meanDist = 0;
  for (const p of pts) meanDist += Math.hypot(p.x - cx, p.y - cy);
  meanDist /= pts.length;
  const s = meanDist > 1e-9 ? Math.SQRT2 / meanDist : 1;
  return {
    T: [s, 0, -s * cx, 0, s, -s * cy, 0, 0, 1],
    pts: pts.map((p) => ({ x: s * (p.x - cx), y: s * (p.y - cy) })),
  };
}

export function matMul3(a: Mat3, b: Mat3): Mat3 {
  const r = new Array<number>(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    }
  }
  return r;
}

export function invert3(m: Mat3): Mat3 | null {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h;
  const B = c * h - b * i;
  const C = b * f - c * e;
  const det = a * A + d * B + g * C;
  if (Math.abs(det) < 1e-12) return null;
  const inv = 1 / det;
  return [
    A * inv, B * inv, C * inv,
    (f * g - d * i) * inv, (a * i - c * g) * inv, (c * d - a * f) * inv,
    (d * h - e * g) * inv, (b * g - a * h) * inv, (a * e - b * d) * inv,
  ];
}

export function applyHomography(H: Mat3, x: number, y: number, out?: Point2): Point2 {
  const w = H[6] * x + H[7] * y + H[8];
  const invW = w !== 0 ? 1 / w : 0;
  const p = out ?? { x: 0, y: 0 };
  p.x = (H[0] * x + H[1] * y + H[2]) * invW;
  p.y = (H[3] * x + H[4] * y + H[5]) * invW;
  return p;
}

/**
 * Estimate H such that dst ~ H * src (least squares for n > 4), using the
 * normalized DLT with h33 fixed to 1. Returns null for degenerate input.
 */
export function computeHomography(src: Point2[], dst: Point2[]): Mat3 | null {
  const n = src.length;
  if (n < 4 || dst.length !== n) return null;

  const ns = normalizePoints(src);
  const nd = normalizePoints(dst);

  // Rows of the 2n x 8 system A h = rhs (h33 = 1).
  const rows = 2 * n;
  const A = new Float64Array(rows * 8);
  const rhs = new Float64Array(rows);
  for (let i = 0; i < n; i++) {
    const { x, y } = ns.pts[i];
    const { x: X, y: Y } = nd.pts[i];
    let r = 2 * i * 8;
    A[r] = x; A[r + 1] = y; A[r + 2] = 1;
    A[r + 6] = -x * X; A[r + 7] = -y * X;
    rhs[2 * i] = X;
    r += 8;
    A[r + 3] = x; A[r + 4] = y; A[r + 5] = 1;
    A[r + 6] = -x * Y; A[r + 7] = -y * Y;
    rhs[2 * i + 1] = Y;
  }

  // Normal equations: (A^T A) h = A^T rhs. 8x8 solve.
  const AtA = new Float64Array(64);
  const Atb = new Float64Array(8);
  for (let r = 0; r < rows; r++) {
    const off = r * 8;
    for (let i = 0; i < 8; i++) {
      const ai = A[off + i];
      if (ai === 0) continue;
      Atb[i] += ai * rhs[r];
      for (let j = i; j < 8; j++) AtA[i * 8 + j] += ai * A[off + j];
    }
  }
  for (let i = 0; i < 8; i++) for (let j = 0; j < i; j++) AtA[i * 8 + j] = AtA[j * 8 + i];

  const h = solveLinearSystem(AtA, Atb, 8);
  if (!h) return null;

  const Hn: Mat3 = [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
  const TdInv = invert3(nd.T);
  if (!TdInv) return null;
  const H = matMul3(matMul3(TdInv, Hn), ns.T);
  // Normalize so H[8] = 1 when possible (keeps comparisons simple).
  if (Math.abs(H[8]) > 1e-12) {
    const inv = 1 / H[8];
    for (let i = 0; i < 9; i++) H[i] *= inv;
  }
  return H;
}

/** Squared symmetric transfer error of a correspondence under H (and Hinv). */
export function symmetricTransferError2(
  H: Mat3,
  Hinv: Mat3,
  sx: number,
  sy: number,
  dx: number,
  dy: number
): number {
  const wf = H[6] * sx + H[7] * sy + H[8];
  if (Math.abs(wf) < 1e-12) return Infinity;
  const fx = (H[0] * sx + H[1] * sy + H[2]) / wf - dx;
  const fy = (H[3] * sx + H[4] * sy + H[5]) / wf - dy;
  const wb = Hinv[6] * dx + Hinv[7] * dy + Hinv[8];
  if (Math.abs(wb) < 1e-12) return Infinity;
  const bx = (Hinv[0] * dx + Hinv[1] * dy + Hinv[2]) / wb - sx;
  const by = (Hinv[3] * dx + Hinv[4] * dy + Hinv[5]) / wb - sy;
  return Math.max(fx * fx + fy * fy, bx * bx + by * by);
}
