/**
 * Gyroscope helpers for camera-rotation priors.
 *
 * For a pure camera rotation R (camera-frame), static scene points transform
 * between consecutive images as x1 = K R^T K^-1 x0. Feeding that homography
 * into the tracker as a motion prior makes fast pans predictable even when
 * the image itself is motion-blurred - the classic IMU+vision fusion win.
 */

import { invert3, matMul3, type Mat3 } from './homography';
import { intrinsicsMatrix, type CameraIntrinsics } from './pose';

export interface GyroDelta {
  /** Integrated rotation vector in camera coordinates (radians). */
  wx: number;
  wy: number;
  wz: number;
}

/** Rodrigues: rotation vector (radians) -> rotation matrix (row-major). */
export function rotationFromRotVec(wx: number, wy: number, wz: number): Mat3 {
  const theta = Math.hypot(wx, wy, wz);
  if (theta < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const kx = wx / theta;
  const ky = wy / theta;
  const kz = wz / theta;
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const t = 1 - c;
  return [
    c + kx * kx * t, kx * ky * t - kz * s, kx * kz * t + ky * s,
    ky * kx * t + kz * s, c + ky * ky * t, ky * kz * t - kx * s,
    kz * kx * t - ky * s, kz * ky * t + kx * s, c + kz * kz * t,
  ];
}

/**
 * Image-space homography induced by a camera rotation: x_new = K R^T K^-1 x.
 * Returns null for a degenerate K.
 */
export function gyroHomography(delta: GyroDelta, K: CameraIntrinsics): Mat3 | null {
  const R = rotationFromRotVec(delta.wx, delta.wy, delta.wz);
  const Rt: Mat3 = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const Km = intrinsicsMatrix(K);
  const Kinv = invert3(Km);
  if (!Kinv) return null;
  const H = matMul3(matMul3(Km, Rt), Kinv);
  if (Math.abs(H[8]) < 1e-12) return null;
  const inv = 1 / H[8];
  for (let i = 0; i < 9; i++) H[i] *= inv;
  return H;
}

/**
 * Map a DeviceMotion rotation rate (deg/s, device axes: x right, y toward
 * screen top, z out of the screen) to camera-frame axes (x right, y down,
 * z forward through the rear camera), honouring the current screen rotation.
 */
export function deviceRateToCamera(
  alphaDeg: number, // around device z
  betaDeg: number, // around device x
  gammaDeg: number, // around device y
  screenAngleDeg = 0
): { x: number; y: number; z: number } {
  const D = Math.PI / 180;
  // Portrait mapping: cam x = device x, cam y = -device y, cam z = -device z.
  let x = betaDeg * D;
  let y = -gammaDeg * D;
  const z = -alphaDeg * D;
  // The image axes rotate with the screen orientation.
  if (screenAngleDeg !== 0) {
    const a = (screenAngleDeg * Math.PI) / 180;
    const c = Math.cos(a);
    const s = Math.sin(a);
    const xr = c * x + s * y;
    const yr = -s * x + c * y;
    x = xr;
    y = yr;
  }
  return { x, y, z };
}
