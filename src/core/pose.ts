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

function cross(a: number[], b: number[]): number[] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function normalize(v: number[]): number[] {
  const n = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / n, v[1] / n, v[2] / n];
}
