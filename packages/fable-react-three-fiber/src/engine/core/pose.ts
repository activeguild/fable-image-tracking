/**
 * 6-DoF pose recovery from a plane-to-image homography and camera intrinsics.
 *
 * Conventions here are the usual computer-vision ones: camera at the origin,
 * x right, y down, z forward. `H` maps target-plane coordinates in metres
 * (x right, y up, z = 0) to image pixels: s * [u v 1]^T = H [X Y 1]^T with
 * H = K [r1 r2 t].
 */

import { invert3, matMul3, type Mat3 } from './homography';

export interface CameraIntrinsics {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
}

export interface Pose {
  /** Rotation matrix, row-major 3x3 (CV camera convention). */
  R: Mat3;
  /** Translation in metres (CV camera convention). */
  t: [number, number, number];
}

export function intrinsicsMatrix(K: CameraIntrinsics): Mat3 {
  return [K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1];
}

/**
 * Decompose H (plane -> image, in pixels) into R, t. Returns null when H is
 * degenerate. The plane is assumed z = 0 with points in metres.
 */
export function poseFromHomography(H: Mat3, K: CameraIntrinsics): Pose | null {
  const Kinv = invert3(intrinsicsMatrix(K));
  if (!Kinv) return null;
  const G = matMul3(Kinv, H);

  let g1 = [G[0], G[3], G[6]];
  let g2 = [G[1], G[4], G[7]];
  let g3 = [G[2], G[5], G[8]];

  const n1 = Math.hypot(g1[0], g1[1], g1[2]);
  const n2 = Math.hypot(g2[0], g2[1], g2[2]);
  if (n1 < 1e-9 || n2 < 1e-9) return null;
  const lambda = 2 / (n1 + n2);

  let r1 = g1.map((v) => v * lambda);
  let r2 = g2.map((v) => v * lambda);
  let t = g3.map((v) => v * lambda);

  // The target must be in front of the camera (positive z in CV convention).
  if (t[2] < 0) {
    r1 = r1.map((v) => -v);
    r2 = r2.map((v) => -v);
    t = t.map((v) => -v);
  }

  // Orthonormalise: r1 exact, r3 = r1 x r2, r2 = r3 x r1.
  r1 = normalize(r1);
  let r3 = cross(r1, r2);
  const n3 = Math.hypot(r3[0], r3[1], r3[2]);
  if (n3 < 1e-9) return null;
  r3 = r3.map((v) => v / n3);
  r2 = cross(r3, r1);

  const R: Mat3 = [
    r1[0], r2[0], r3[0],
    r1[1], r2[1], r3[1],
    r1[2], r2[2], r3[2],
  ];
  return { R, t: [t[0], t[1], t[2]] };
}

/**
 * How badly K^-1 H violates the rotation constraints (columns r1, r2 must be
 * orthogonal and equal-length). Zero for the true intrinsics under a tilted
 * view; insensitive (flat) for fronto-parallel views. Used for online focal
 * self-calibration.
 */
export function orthogonalityDefect(H: Mat3, K: CameraIntrinsics): number {
  const Kinv = invert3(intrinsicsMatrix(K));
  if (!Kinv) return Infinity;
  const G = matMul3(Kinv, H);
  const g1 = [G[0], G[3], G[6]];
  const g2 = [G[1], G[4], G[7]];
  const n1 = Math.hypot(g1[0], g1[1], g1[2]);
  const n2 = Math.hypot(g2[0], g2[1], g2[2]);
  if (n1 < 1e-12 || n2 < 1e-12) return Infinity;
  const dot = Math.abs(g1[0] * g2[0] + g1[1] * g2[1] + g1[2] * g2[2]) / (n1 * n2);
  const aniso = Math.abs(n1 - n2) / ((n1 + n2) / 2);
  return dot + aniso;
}

/**
 * Refine a decomposed pose by minimizing the reprojection error against the
 * measured homography (Gauss-Newton on SE(3), left perturbation).
 *
 * The direct decomposition forces K^-1 H's first two columns into an
 * orthonormal pair, which under measurement noise (or an imperfect K) moves
 * the reprojection of the plane by several pixels - visible as content
 * offset for pose-anchored AR. Fitting the pose to a grid of plane points
 * mapped through H makes the rigid pose reproject the plane as closely as
 * geometrically possible, the same trick commercial engines use.
 *
 * `H` maps plane meters -> image pixels (same convention as
 * poseFromHomography). Returns the input pose if refinement cannot improve.
 */
export function refinePlanarPose(
  pose: Pose,
  K: CameraIntrinsics,
  H: Mat3,
  widthMeters: number,
  heightMeters: number
): Pose {
  // 3x3 grid over the target in plane meters.
  const pts: [number, number][] = [];
  for (let gy = -1; gy <= 1; gy++) {
    for (let gx = -1; gx <= 1; gx++) {
      pts.push([(gx * widthMeters) / 2, (gy * heightMeters) / 2]);
    }
  }
  // Observations: the plane points as the measured homography maps them.
  const obs: [number, number][] = [];
  for (const [X, Y] of pts) {
    const w = H[6] * X + H[7] * Y + H[8];
    if (Math.abs(w) < 1e-12) return pose;
    obs.push([(H[0] * X + H[1] * Y + H[2]) / w, (H[3] * X + H[4] * Y + H[5]) / w]);
  }

  let R = pose.R.slice() as Mat3;
  let t: [number, number, number] = [pose.t[0], pose.t[1], pose.t[2]];
  let best = { R, t, err: reprojError(R, t, K, pts, obs) };

  for (let iter = 0; iter < 5; iter++) {
    // Normal equations JtJ (6x6) and Jtr for residual r = proj - obs,
    // parameters d = (omega, nu): P' = exp([omega]x) P + nu.
    const JtJ = new Float64Array(36);
    const Jtr = new Float64Array(6);
    let valid = true;
    for (let i = 0; i < pts.length; i++) {
      const [X, Y] = pts[i];
      const Px = R[0] * X + R[1] * Y + t[0];
      const Py = R[3] * X + R[4] * Y + t[1];
      const Pz = R[6] * X + R[7] * Y + t[2];
      if (Pz < 1e-6) {
        valid = false;
        break;
      }
      const iz = 1 / Pz;
      const u = K.fx * Px * iz + K.cx;
      const v = K.fy * Py * iz + K.cy;
      const ru = u - obs[i][0];
      const rv = v - obs[i][1];
      // du/dP = (a, 0, b), dv/dP = (0, c, d);
      // dP/d(omega) = -[P]x = [[0, Pz, -Py], [-Pz, 0, Px], [Py, -Px, 0]],
      // dP/d(nu) = I. Chain rule gives the two 1x6 Jacobian rows.
      const a = K.fx * iz, b = -K.fx * Px * iz * iz;
      const c = K.fy * iz, d = -K.fy * Py * iz * iz;
      const rowU = [b * Py, a * Pz - b * Px, -a * Py, a, 0, b];
      const rowV = [-c * Pz + d * Py, -d * Px, c * Px, 0, c, d];
      for (let r0 = 0; r0 < 6; r0++) {
        Jtr[r0] += rowU[r0] * ru + rowV[r0] * rv;
        for (let c0 = 0; c0 < 6; c0++) {
          JtJ[r0 * 6 + c0] += rowU[r0] * rowU[c0] + rowV[r0] * rowV[c0];
        }
      }
    }
    if (!valid) break;
    const delta = solve6(JtJ, Jtr);
    if (!delta) break;
    // Newton step: parameters move against the gradient.
    const dR = rodrigues(-delta[0], -delta[1], -delta[2]);
    const Rn = matMul3(dR, R);
    const tn: [number, number, number] = [
      dR[0] * t[0] + dR[1] * t[1] + dR[2] * t[2] - delta[3],
      dR[3] * t[0] + dR[4] * t[1] + dR[5] * t[2] - delta[4],
      dR[6] * t[0] + dR[7] * t[1] + dR[8] * t[2] - delta[5],
    ];
    const err = reprojError(Rn, tn, K, pts, obs);
    if (!(err < best.err)) break;
    R = Rn;
    t = tn;
    best = { R, t, err };
    if (err < 1e-8) break;
  }
  return { R: best.R, t: best.t };
}

function reprojError(
  R: Mat3,
  t: [number, number, number],
  K: CameraIntrinsics,
  pts: [number, number][],
  obs: [number, number][]
): number {
  let sum = 0;
  for (let i = 0; i < pts.length; i++) {
    const [X, Y] = pts[i];
    const Pz = R[6] * X + R[7] * Y + t[2];
    if (Pz < 1e-6) return Infinity;
    const u = (K.fx * (R[0] * X + R[1] * Y + t[0])) / Pz + K.cx;
    const v = (K.fy * (R[3] * X + R[4] * Y + t[1])) / Pz + K.cy;
    sum += (u - obs[i][0]) ** 2 + (v - obs[i][1]) ** 2;
  }
  return sum;
}

function rodrigues(wx: number, wy: number, wz: number): Mat3 {
  const theta = Math.hypot(wx, wy, wz);
  if (theta < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const kx = wx / theta, ky = wy / theta, kz = wz / theta;
  const c = Math.cos(theta), s = Math.sin(theta), T = 1 - c;
  return [
    c + kx * kx * T, kx * ky * T - kz * s, kx * kz * T + ky * s,
    ky * kx * T + kz * s, c + ky * ky * T, ky * kz * T - kx * s,
    kz * kx * T - ky * s, kz * ky * T + kx * s, c + kz * kz * T,
  ];
}

/** Solve a symmetric positive-definite 6x6 system by Gaussian elimination. */
function solve6(A: Float64Array, b: Float64Array): number[] | null {
  const M = new Float64Array(42);
  for (let r = 0; r < 6; r++) {
    for (let c = 0; c < 6; c++) M[r * 7 + c] = A[r * 6 + c];
    M[r * 7 + 6] = b[r];
  }
  for (let col = 0; col < 6; col++) {
    let piv = col;
    for (let r = col + 1; r < 6; r++) {
      if (Math.abs(M[r * 7 + col]) > Math.abs(M[piv * 7 + col])) piv = r;
    }
    if (Math.abs(M[piv * 7 + col]) < 1e-12) return null;
    if (piv !== col) {
      for (let c = col; c < 7; c++) {
        const tmp = M[col * 7 + c];
        M[col * 7 + c] = M[piv * 7 + c];
        M[piv * 7 + c] = tmp;
      }
    }
    const inv = 1 / M[col * 7 + col];
    for (let r = 0; r < 6; r++) {
      if (r === col) continue;
      const f = M[r * 7 + col] * inv;
      for (let c = col; c < 7; c++) M[r * 7 + c] -= f * M[col * 7 + c];
    }
  }
  const x = new Array<number>(6);
  for (let r = 0; r < 6; r++) x[r] = M[r * 7 + 6] / M[r * 7 + r];
  return x;
}

function cross(a: number[], b: number[]): number[] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function normalize(v: number[]): number[] {
  const n = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / n, v[1] / n, v[2] / n];
}
