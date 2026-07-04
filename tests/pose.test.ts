import { describe, expect, it } from 'vitest';
import { computeHomography, matMul3, type Mat3 } from '../src/core/homography';
import {
  intrinsicsMatrix,
  orthogonalityDefect,
  defaultPosePrior,
  poseFromHomography,
  refinePlanarPose,
  type CameraIntrinsics,
} from '../src/core/pose';

const K: CameraIntrinsics = { fx: 300, fy: 300, cx: 180, cy: 135 };

function rotationZYX(rz: number, ry: number, rx: number): Mat3 {
  const cz = Math.cos(rz), sz = Math.sin(rz);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const Rz: Mat3 = [cz, -sz, 0, sz, cz, 0, 0, 0, 1];
  const Ry: Mat3 = [cy, 0, sy, 0, 1, 0, -sy, 0, cy];
  const Rx: Mat3 = [1, 0, 0, 0, cx, -sx, 0, sx, cx];
  return matMul3(matMul3(Rz, Ry), Rx);
}

function homographyFromPose(R: Mat3, t: number[]): Mat3 {
  // H = K [r1 r2 t]
  const Rt: Mat3 = [R[0], R[1], t[0], R[3], R[4], t[1], R[6], R[7], t[2]];
  return matMul3(intrinsicsMatrix(K), Rt);
}

describe('poseFromHomography', () => {
  it('recovers a frontal pose', () => {
    const R = rotationZYX(0, 0, 0);
    const t = [0.02, -0.01, 0.5];
    const pose = poseFromHomography(homographyFromPose(R, t), K)!;
    expect(pose).not.toBeNull();
    pose.R.forEach((v, i) => expect(v).toBeCloseTo(R[i], 5));
    pose.t.forEach((v, i) => expect(v).toBeCloseTo(t[i], 5));
  });

  it('recovers a tilted pose and stays right-handed', () => {
    const R = rotationZYX(0.4, -0.3, 0.25);
    const t = [-0.05, 0.08, 0.7];
    const pose = poseFromHomography(homographyFromPose(R, t), K)!;
    expect(pose).not.toBeNull();
    pose.R.forEach((v, i) => expect(v).toBeCloseTo(R[i], 4));
    pose.t.forEach((v, i) => expect(v).toBeCloseTo(t[i], 4));

    // det(R) must be +1.
    const m = pose.R;
    const det =
      m[0] * (m[4] * m[8] - m[5] * m[7]) -
      m[1] * (m[3] * m[8] - m[5] * m[6]) +
      m[2] * (m[3] * m[7] - m[4] * m[6]);
    expect(det).toBeCloseTo(1, 6);
  });

  it('orthogonality defect is minimal at the true focal length under tilt', () => {
    const R = rotationZYX(0.15, -0.45, 0.3); // clearly tilted view
    const t = [0.02, -0.03, 0.6];
    const H = homographyFromPose(R, t); // built with the true K (fx = 300)

    const defectAt = (f: number) =>
      orthogonalityDefect(H, { fx: f, fy: f, cx: K.cx, cy: K.cy });

    expect(defectAt(300)).toBeLessThan(1e-10);
    expect(defectAt(300)).toBeLessThan(defectAt(230));
    expect(defectAt(300)).toBeLessThan(defectAt(390));
    // The defect should decrease monotonically toward the truth.
    expect(defectAt(270)).toBeLessThan(defectAt(230));
    expect(defectAt(330)).toBeLessThan(defectAt(390));
  });

  it('normalizes scale: lambda-scaled homographies give the same pose', () => {
    const R = rotationZYX(0.1, 0.2, -0.15);
    const t = [0.03, 0.02, 0.6];
    const H = homographyFromPose(R, t);
    const scaled = H.map((v) => v * -3.7) as Mat3; // arbitrary (negative) scale
    const pose = poseFromHomography(scaled, K)!;
    expect(pose).not.toBeNull();
    pose.t.forEach((v, i) => expect(v).toBeCloseTo(t[i], 4));
    // Target must be in front of the camera.
    expect(pose.t[2]).toBeGreaterThan(0);
  });
});

describe('refinePlanarPose', () => {
  const W = 0.2;
  const HT = 0.15;

  function reprojectPlane(pose: { R: Mat3; t: [number, number, number] }, X: number, Y: number) {
    const { R, t } = pose;
    const Px = R[0] * X + R[1] * Y + t[0];
    const Py = R[3] * X + R[4] * Y + t[1];
    const Pz = R[6] * X + R[7] * Y + t[2];
    return { u: (K.fx * Px) / Pz + K.cx, v: (K.fy * Py) / Pz + K.cy };
  }

  function mapH(H: Mat3, X: number, Y: number) {
    const w = H[6] * X + H[7] * Y + H[8];
    return { u: (H[0] * X + H[1] * Y + H[2]) / w, v: (H[3] * X + H[4] * Y + H[5]) / w };
  }

  function maxCornerError(pose: { R: Mat3; t: [number, number, number] }, H: Mat3): number {
    let worst = 0;
    for (const [X, Y] of [[-W / 2, -HT / 2], [W / 2, -HT / 2], [W / 2, HT / 2], [-W / 2, HT / 2]] as const) {
      const p = reprojectPlane(pose, X, Y);
      const q = mapH(H, X, Y);
      worst = Math.max(worst, Math.hypot(p.u - q.u, p.v - q.v));
    }
    return worst;
  }

  it('matches the homography reprojection far better than raw decomposition', () => {
    // Simulate the real measurement chain: a true rigid pose, corners
    // projected through it, +/-1 px pixel noise, then H estimated from the
    // noisy correspondences. The raw decomposition amplifies the noise into
    // a visible plane offset; the refined pose must fit the measured
    // mapping down to the noise floor.
    const R = rotationZYX(0.35, -0.25, 0.2);
    const t = [-0.04, 0.05, 0.6];
    const truth = { R, t: t as [number, number, number] };
    const corners: [number, number][] = [
      [-W / 2, -HT / 2], [W / 2, -HT / 2], [W / 2, HT / 2], [-W / 2, HT / 2],
    ];
    const noise = [[0.9, -0.7], [-0.8, 0.6], [0.7, 0.9], [-0.6, -0.8]];
    const src = corners.map(([X, Y]) => ({ x: X, y: Y }));
    const dst = corners.map(([X, Y], i) => {
      const p = reprojectPlane(truth, X, Y);
      return { x: p.u + noise[i][0], y: p.v + noise[i][1] };
    });
    const noisy = computeHomography(src, dst)!;
    expect(noisy).not.toBeNull();

    const raw = poseFromHomography(noisy, K)!;
    const refined = refinePlanarPose(raw, K, noisy, W, HT);

    const rawErr = maxCornerError(raw, noisy);
    const refErr = maxCornerError(refined, noisy);
    expect(rawErr).toBeGreaterThan(1.2); // decomposition alone visibly off
    expect(refErr).toBeLessThan(1.0); // refined pose fits within noise
    expect(refErr).toBeLessThan(rawErr * 0.65);

    // R stays a rotation: columns orthonormal, det +1.
    const Rr = refined.R;
    const dot01 = Rr[0] * Rr[1] + Rr[3] * Rr[4] + Rr[6] * Rr[7];
    const n0 = Math.hypot(Rr[0], Rr[3], Rr[6]);
    const det =
      Rr[0] * (Rr[4] * Rr[8] - Rr[5] * Rr[7]) -
      Rr[1] * (Rr[3] * Rr[8] - Rr[5] * Rr[6]) +
      Rr[2] * (Rr[3] * Rr[7] - Rr[4] * Rr[6]);
    expect(Math.abs(dot01)).toBeLessThan(1e-6);
    expect(n0).toBeCloseTo(1, 6);
    expect(det).toBeCloseTo(1, 6);
  });

  it('is a no-op on an exact homography', () => {
    const R = rotationZYX(0.2, 0.1, -0.15);
    const t = [0.03, -0.02, 0.5];
    const H = homographyFromPose(R, t);
    const raw = poseFromHomography(H, K)!;
    const refined = refinePlanarPose(raw, K, H, W, HT);
    expect(maxCornerError(refined, H)).toBeLessThan(1e-3);
    refined.R.forEach((v, i) => expect(v).toBeCloseTo(R[i], 4));
  });
});

describe('refinePlanarPose temporal prior', () => {
  const W = 0.2;
  const HT = 0.15;

  function lcg(seed: number): () => number {
    let a = seed >>> 0;
    return () => {
      a = (a * 1664525 + 1013904223) >>> 0;
      return a / 4294967296 - 0.5;
    };
  }

  function project(pose: { R: Mat3; t: [number, number, number] }, X: number, Y: number, Z: number) {
    const { R, t } = pose;
    const Px = R[0] * X + R[1] * Y + R[2] * Z + t[0];
    const Py = R[3] * X + R[4] * Y + R[5] * Z + t[1];
    const Pz = R[6] * X + R[7] * Y + R[8] * Z + t[2];
    return { u: (K.fx * Px) / Pz + K.cx, v: (K.fy * Py) / Pz + K.cy };
  }

  function measuredH(truth: { R: Mat3; t: [number, number, number] }, rnd: () => number): Mat3 {
    const corners: [number, number][] = [
      [-W / 2, -HT / 2], [W / 2, -HT / 2], [W / 2, HT / 2], [-W / 2, HT / 2],
    ];
    const src = corners.map(([X, Y]) => ({ x: X, y: Y }));
    const dst = corners.map(([X, Y]) => {
      const p = project(truth, X, Y, 0);
      return { x: p.u + rnd() * 1.6, y: p.v + rnd() * 1.6 }; // ~+/-0.8 px noise
    });
    return computeHomography(src, dst)!;
  }

  function runChain(
    poses: { R: Mat3; t: [number, number, number] }[],
    rnd: () => number,
    usePrior: boolean
  ) {
    let prev: ReturnType<typeof poseFromHomography> = null;
    const out: { u: number; v: number }[] = [];
    for (const truth of poses) {
      const H = measuredH(truth, rnd);
      let pose = poseFromHomography(H, K)!;
      pose = refinePlanarPose(pose, K, H, W, HT, usePrior && prev ? defaultPosePrior(prev) : undefined);
      prev = pose;
      // A point raised half a target-width above the plane: the lever arm
      // that turns tilt noise into visible float.
      out.push(project(pose, 0, 0, 0.1));
    }
    return out;
  }

  it('damps out-of-plane wobble for elevated content while static', () => {
    const truth = { R: rotationZYX(0.05, 0.03, 0.09), t: [0.01, -0.01, 0.5] as [number, number, number] };
    const frames = Array.from({ length: 50 }, () => truth);
    const plain = runChain(frames, lcg(7), false).slice(10);
    const damped = runChain(frames, lcg(7), true).slice(10);
    const std = (pts: { u: number; v: number }[]) => {
      const mu = pts.reduce((a, p) => a + p.u, 0) / pts.length;
      const mv = pts.reduce((a, p) => a + p.v, 0) / pts.length;
      return Math.sqrt(pts.reduce((a, p) => a + (p.u - mu) ** 2 + (p.v - mv) ** 2, 0) / pts.length);
    };
    expect(std(plain)).toBeGreaterThan(1); // wobble is real without the prior
    expect(std(damped)).toBeLessThan(std(plain) * 0.5); // at least halved
  });

  it('still follows a genuine rotation within a few frames', () => {
    const before = { R: rotationZYX(0.05, 0.03, 0.09), t: [0.01, -0.01, 0.5] as [number, number, number] };
    const after = { R: rotationZYX(0.05, 0.03, 0.25), t: [0.01, -0.01, 0.5] as [number, number, number] };
    const frames = [...Array.from({ length: 15 }, () => before), ...Array.from({ length: 15 }, () => after)];
    const damped = runChain(frames, lcg(11), true);
    const target = project(after, 0, 0, 0.1);
    const stepStart = project(before, 0, 0, 0.1);
    const stepSize = Math.hypot(target.u - stepStart.u, target.v - stepStart.v);
    // 6 frames after the step, the elevated point must have covered most of
    // the way (the robust weight releases large deviations).
    const at = damped[15 + 5];
    const remaining = Math.hypot(at.u - target.u, at.v - target.v);
    expect(remaining).toBeLessThan(stepSize * 0.3 + 2);
  });
});
