/**
 * Constant-velocity pose prediction. The tracker produces poses for camera
 * frames captured slightly in the past (worker latency + processing time);
 * extrapolating the last two samples to the render timestamp lets the main
 * thread draw at display rate with effectively zero perceived lag.
 */

import type { Mat3 } from './homography';
import type { Pose } from './pose';

export type Quat = [number, number, number, number]; // x, y, z, w

export function quatFromMat3(m: Mat3): Quat {
  // Shepperd's method.
  const trace = m[0] + m[4] + m[8];
  let x: number, y: number, z: number, w: number;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = s / 4;
    x = (m[7] - m[5]) / s;
    y = (m[2] - m[6]) / s;
    z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
    w = (m[7] - m[5]) / s;
    x = s / 4;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = s / 4;
    z = (m[5] + m[7]) / s;
  } else {
    const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = s / 4;
  }
  return quatNormalize([x, y, z, w]);
}

export function mat3FromQuat(q: Quat): Mat3 {
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return [
    1 - (yy + zz), xy - wz, xz + wy,
    xy + wz, 1 - (xx + zz), yz - wx,
    xz - wy, yz + wx, 1 - (xx + yy),
  ];
}

export function quatMul(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

export function quatConjugate(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]];
}

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [0, 0, 0, 1];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

interface Sample {
  t: number;
  p: [number, number, number];
  q: Quat;
}

export interface PredictorOptions {
  /** Samples older than this are considered stale (seconds). */
  maxAge?: number;
  /** Never extrapolate further than this (seconds). */
  maxHorizon?: number;
  /** Cap on the extrapolated rotation (radians). */
  maxAngle?: number;
}

export class PosePredictor {
  private s0: Sample | null = null;
  private s1: Sample | null = null;
  private readonly maxAge: number;
  private readonly maxHorizon: number;
  private readonly maxAngle: number;

  constructor(options: PredictorOptions = {}) {
    this.maxAge = options.maxAge ?? 0.3;
    this.maxHorizon = options.maxHorizon ?? 0.08;
    this.maxAngle = options.maxAngle ?? 0.2;
  }

  addSample(pose: Pose, timeSec: number): void {
    const sample: Sample = {
      t: timeSec,
      p: [pose.t[0], pose.t[1], pose.t[2]],
      q: quatFromMat3(pose.R),
    };
    // Keep quaternion continuity for velocity estimation.
    if (this.s1 && dot(sample.q, this.s1.q) < 0) {
      sample.q = [-sample.q[0], -sample.q[1], -sample.q[2], -sample.q[3]];
    }
    this.s0 = this.s1;
    this.s1 = sample;
  }

  clear(): void {
    this.s0 = null;
    this.s1 = null;
  }

  /** Predicted pose at `timeSec`, or null when there is no fresh sample. */
  predict(timeSec: number): Pose | null {
    const s1 = this.s1;
    if (!s1) return null;
    const age = timeSec - s1.t;
    if (age > this.maxAge) return null;

    const s0 = this.s0;
    const dt = s0 ? s1.t - s0.t : 0;
    if (!s0 || dt <= 1e-4 || dt > 0.15 || age <= 0) {
      return { R: mat3FromQuat(s1.q), t: [s1.p[0], s1.p[1], s1.p[2]] };
    }

    const h = Math.min(age, this.maxHorizon);
    const k = h / dt;
    const p: [number, number, number] = [
      s1.p[0] + (s1.p[0] - s0.p[0]) * k,
      s1.p[1] + (s1.p[1] - s0.p[1]) * k,
      s1.p[2] + (s1.p[2] - s0.p[2]) * k,
    ];

    // Relative rotation over dt, scaled to the horizon via axis-angle.
    let dq = quatNormalize(quatMul(s1.q, quatConjugate(s0.q)));
    if (dq[3] < 0) dq = [-dq[0], -dq[1], -dq[2], -dq[3]];
    const halfAngle = Math.acos(Math.min(1, dq[3]));
    const angle = 2 * halfAngle;
    let q = s1.q;
    if (angle > 1e-6) {
      const scaled = Math.min(angle * k, this.maxAngle);
      const sinHalf = Math.sin(halfAngle);
      const axis: [number, number, number] = [dq[0] / sinHalf, dq[1] / sinHalf, dq[2] / sinHalf];
      const sh = Math.sin(scaled / 2);
      const step: Quat = [axis[0] * sh, axis[1] * sh, axis[2] * sh, Math.cos(scaled / 2)];
      q = quatNormalize(quatMul(step, s1.q));
    }
    return { R: mat3FromQuat(q), t: p };
  }
}

function dot(a: Quat, b: Quat): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}
