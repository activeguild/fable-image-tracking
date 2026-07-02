import { describe, expect, it } from 'vitest';
import { deviceRateToCamera, gyroHomography, rotationFromRotVec } from '../src/core/imu';
import { applyHomography, invert3, matMul3, type Mat3 } from '../src/core/homography';
import { sampleBilinear } from '../src/core/imageops';
import { compileTarget } from '../src/tracker/target';
import { ImageTracker } from '../src/tracker/tracker';
import { randomTexture } from './helpers';

const K = { fx: 300, fy: 300, cx: 180, cy: 135 };

describe('rotationFromRotVec', () => {
  it('returns identity for a zero vector', () => {
    const R = rotationFromRotVec(0, 0, 0);
    [1, 0, 0, 0, 1, 0, 0, 0, 1].forEach((v, i) => expect(R[i]).toBeCloseTo(v, 12));
  });

  it('matches the known 90-degree z rotation', () => {
    const R = rotationFromRotVec(0, 0, Math.PI / 2);
    const expected = [0, -1, 0, 1, 0, 0, 0, 0, 1];
    expected.forEach((v, i) => expect(R[i]).toBeCloseTo(v, 10));
  });

  it('is inverted by the negated vector', () => {
    const R = rotationFromRotVec(0.3, -0.2, 0.5);
    const Rinv = rotationFromRotVec(-0.3, 0.2, -0.5);
    const P = matMul3(R, Rinv);
    [1, 0, 0, 0, 1, 0, 0, 0, 1].forEach((v, i) => expect(P[i]).toBeCloseTo(v, 10));
  });
});

describe('gyroHomography', () => {
  it('a rightward pan (positive yaw) moves image content left', () => {
    const H = gyroHomography({ wx: 0, wy: 0.1, wz: 0 }, K)!;
    const p = applyHomography(H, K.cx, K.cy);
    expect(p.x).toBeLessThan(K.cx - 10); // ~ f * tan(0.1) ≈ 30 px
    expect(Math.abs(p.y - K.cy)).toBeLessThan(1);
  });

  it('a roll rotates the image around the principal point', () => {
    const H = gyroHomography({ wx: 0, wy: 0, wz: 0.1 }, K)!;
    const centre = applyHomography(H, K.cx, K.cy);
    expect(centre.x).toBeCloseTo(K.cx, 4);
    expect(centre.y).toBeCloseTo(K.cy, 4);
    const right = applyHomography(H, K.cx + 100, K.cy);
    expect(Math.abs(right.y - K.cy)).toBeGreaterThan(5); // moved off the axis
  });
});

describe('deviceRateToCamera', () => {
  it('maps portrait device axes to camera axes', () => {
    const r = deviceRateToCamera(10, 20, 30, 0); // alpha(z), beta(x), gamma(y) deg/s
    const D = Math.PI / 180;
    expect(r.x).toBeCloseTo(20 * D, 10);
    expect(r.y).toBeCloseTo(-30 * D, 10);
    expect(r.z).toBeCloseTo(-10 * D, 10);
  });
});

describe('gyro-aided tracking', () => {
  const FRAME_W = 360;
  const FRAME_H = 270;
  const TARGET = 256;
  const targetImg = randomTexture(TARGET, TARGET, 77, 9);
  const Kf = { fx: 0.8 * FRAME_W, fy: 0.8 * FRAME_W, cx: FRAME_W / 2, cy: FRAME_H / 2 };

  function similarity(scale: number, tx: number, ty: number): Mat3 {
    const c = scale;
    return [c, 0, tx - c * (TARGET / 2), 0, c, ty - c * (TARGET / 2), 0, 0, 1];
  }

  function renderFrame(H: Mat3): Uint8Array {
    const Hinv = invert3(H)!;
    const frame = new Uint8Array(FRAME_W * FRAME_H);
    for (let y = 0; y < FRAME_H; y++) {
      for (let x = 0; x < FRAME_W; x++) {
        const p = applyHomography(Hinv, x, y);
        frame[y * FRAME_W + x] =
          p.x >= 0 && p.y >= 0 && p.x < TARGET - 1 && p.y < TARGET - 1
            ? sampleBilinear(targetImg, TARGET, TARGET, p.x, p.y) | 0
            : 48 + ((x * 31 + y * 17) % 23);
      }
    }
    return frame;
  }

  it('survives a sudden large rotation thanks to the gyro prior', () => {
    const compiled = compileTarget(targetImg, TARGET, TARGET, { widthMeters: 0.2 });
    const tracker = new ImageTracker(compiled, FRAME_W, FRAME_H, {
      detectEveryN: 1,
      intrinsics: { ...Kf },
    });

    let H = similarity(0.55, 180, 135);
    let r = tracker.processFrame(renderFrame(H));
    expect(r.state).toBe('tracking');

    // Three consecutive frames with abrupt camera yaw (~28 px/frame image
    // shift, far beyond the LK pyramid search range with no warm start).
    for (let f = 0; f < 3; f++) {
      const delta = { wx: 0, wy: 0.098, wz: 0 };
      const gyroH = gyroHomography(delta, Kf)!;
      H = matMul3(gyroH, H);
      r = tracker.processFrame(renderFrame(H), gyroH);
      expect(r.state, `frame ${f}`).toBe('tracking');

      let maxErr = 0;
      for (const [x, y] of [[0, 0], [TARGET, 0], [TARGET, TARGET], [0, TARGET]]) {
        const a = applyHomography(r.H!, x, y);
        const b = applyHomography(H, x, y);
        maxErr = Math.max(maxErr, Math.hypot(a.x - b.x, a.y - b.y));
      }
      expect(maxErr, `frame ${f}`).toBeLessThan(3);
    }
  });
});
