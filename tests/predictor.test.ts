import { describe, expect, it } from 'vitest';
import { mat3FromQuat, quatFromMat3, PosePredictor, type Quat } from '../src/core/predictor';
import { matMul3, type Mat3 } from '../src/core/homography';

function rotZ(a: number): Mat3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

function rotX(a: number): Mat3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}

describe('quaternion utilities', () => {
  it('round-trips rotation matrices', () => {
    const cases = [rotZ(0.3), rotX(-1.1), matMul3(rotZ(2.5), rotX(0.7)), rotZ(0)];
    for (const R of cases) {
      const q = quatFromMat3(R);
      const back = mat3FromQuat(q);
      R.forEach((v, i) => expect(back[i]).toBeCloseTo(v, 10));
      const norm = Math.hypot(...(q as Quat));
      expect(norm).toBeCloseTo(1, 12);
    }
  });
});

describe('PosePredictor', () => {
  it('returns the latest pose with a single sample', () => {
    const pred = new PosePredictor();
    pred.addSample({ R: rotZ(0.2), t: [1, 2, 3] }, 10.0);
    const pose = pred.predict(10.02)!;
    expect(pose).not.toBeNull();
    pose.t.forEach((v, i) => expect(v).toBeCloseTo([1, 2, 3][i], 10));
  });

  it('extrapolates position and rotation linearly', () => {
    const pred = new PosePredictor();
    // Constant velocity: +0.1 m/s in x, +0.5 rad/s around z, samples 40 ms apart.
    pred.addSample({ R: rotZ(0.0), t: [0, 0, 0.5] }, 10.0);
    pred.addSample({ R: rotZ(0.02), t: [0.004, 0, 0.5] }, 10.04);

    const pose = pred.predict(10.08)!; // 40 ms past the last sample
    expect(pose).not.toBeNull();
    expect(pose.t[0]).toBeCloseTo(0.008, 6);
    const expected = rotZ(0.04);
    pose.R.forEach((v, i) => expect(v).toBeCloseTo(expected[i], 5));
  });

  it('clamps the extrapolation horizon', () => {
    const pred = new PosePredictor({ maxHorizon: 0.05 });
    pred.addSample({ R: rotZ(0), t: [0, 0, 0.5] }, 10.0);
    pred.addSample({ R: rotZ(0), t: [0.01, 0, 0.5] }, 10.04);
    // 200 ms ahead, but the horizon caps at 50 ms => x = 0.01 + 0.01*(0.05/0.04)
    const pose = pred.predict(10.24)!;
    expect(pose.t[0]).toBeCloseTo(0.01 + 0.01 * (0.05 / 0.04), 6);
  });

  it('goes stale after maxAge and clears on demand', () => {
    const pred = new PosePredictor({ maxAge: 0.3 });
    pred.addSample({ R: rotZ(0), t: [0, 0, 0.5] }, 10.0);
    expect(pred.predict(10.2)).not.toBeNull();
    expect(pred.predict(10.4)).toBeNull();
    pred.addSample({ R: rotZ(0), t: [0, 0, 0.5] }, 11.0);
    pred.clear();
    expect(pred.predict(11.01)).toBeNull();
  });
});
