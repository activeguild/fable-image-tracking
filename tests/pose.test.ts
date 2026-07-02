import { describe, expect, it } from 'vitest';
import { matMul3, type Mat3 } from '../src/core/homography';
import { intrinsicsMatrix, poseFromHomography, type CameraIntrinsics } from '../src/core/pose';

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
