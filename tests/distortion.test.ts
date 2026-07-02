/**
 * Lens distortion model and photometric self-calibration tests. Frames are
 * rendered through a known Brown-k1 radial model; the tracker (which starts
 * at k1 = 0) must move its estimate toward the truth and keep edge accuracy.
 */

import { describe, expect, it } from 'vitest';
import { distortPoint, undistortPoint, type RadialDistortion } from '../src/core/densealign';
import { applyHomography, invert3, type Mat3 } from '../src/core/homography';
import { sampleBilinear } from '../src/core/imageops';
import { compileTarget } from '../src/tracker/target';
import { ImageTracker } from '../src/tracker/tracker';
import { randomTexture } from './helpers';

const FRAME_W = 360;
const FRAME_H = 270;
const TARGET = 256;
const targetImg = randomTexture(TARGET, TARGET, 77, 9);

const K1_TRUE = -0.12; // strong barrel distortion
const DIST: RadialDistortion = { k1: K1_TRUE, cx: FRAME_W / 2, cy: FRAME_H / 2, f: 0.8 * FRAME_W };

function similarity(scale: number, angle: number, tx: number, ty: number): Mat3 {
  const c = Math.cos(angle) * scale;
  const s = Math.sin(angle) * scale;
  const cx = TARGET / 2;
  const cy = TARGET / 2;
  return [c, -s, tx - c * cx + s * cy, s, c, ty - s * cx - c * cy, 0, 0, 1];
}

/** Render a frame observed through the true lens: H is target -> ideal px. */
function renderDistortedFrame(H: Mat3): Uint8Array {
  const Hinv = invert3(H)!;
  const frame = new Uint8Array(FRAME_W * FRAME_H);
  const u = { x: 0, y: 0 };
  for (let y = 0; y < FRAME_H; y++) {
    for (let x = 0; x < FRAME_W; x++) {
      undistortPoint(DIST, x, y, u); // sensor px -> ideal px
      const p = applyHomography(Hinv, u.x, u.y);
      frame[y * FRAME_W + x] =
        p.x >= 0 && p.y >= 0 && p.x < TARGET - 1 && p.y < TARGET - 1
          ? sampleBilinear(targetImg, TARGET, TARGET, p.x, p.y) | 0
          : 48 + ((x * 31 + y * 17) % 23);
    }
  }
  return frame;
}

describe('radial distortion model', () => {
  it('distort/undistort round-trips', () => {
    const pts = [
      [10, 10],
      [350, 20],
      [180, 135],
      [40, 260],
      [355, 265],
    ];
    const out = { x: 0, y: 0 };
    const back = { x: 0, y: 0 };
    for (const [x, y] of pts) {
      distortPoint(DIST, x, y, out);
      undistortPoint(DIST, out.x, out.y, back);
      expect(back.x).toBeCloseTo(x, 2);
      expect(back.y).toBeCloseTo(y, 2);
    }
  });

  it('barrel distortion pulls periphery inward', () => {
    const out = { x: 0, y: 0 };
    distortPoint(DIST, 350, 135, out); // far right of centre
    expect(out.x).toBeLessThan(350);
    distortPoint(DIST, 180, 135, out); // at the principal point: unchanged
    expect(out.x).toBeCloseTo(180, 6);
  });
});

describe('distortion self-calibration', () => {
  it('moves k1 toward the truth and keeps edge accuracy', () => {
    const compiled = compileTarget(targetImg, TARGET, TARGET, { widthMeters: 0.2 });
    const tracker = new ImageTracker(compiled, FRAME_W, FRAME_H, { detectEveryN: 1 });

    // Target large in frame so its corners reach the periphery (k1 observable).
    const H = similarity(1.0, 0.05, 180, 135);
    const frame = renderDistortedFrame(H);

    let r = tracker.processFrame(frame);
    expect(r.state).toBe('tracking');

    // The hysteresis (consecutive agreeing evaluations, every 8th frame)
    // slows convergence by design; give it a few seconds of frames.
    for (let f = 0; f < 200; f++) r = tracker.processFrame(frame);
    expect(tracker.distortion.k1).toBeLessThan(-0.05);
    expect(tracker.distortion.k1).toBeGreaterThan(-0.16);

    // Corners returned by the tracker are in sensor space; compare against
    // the ground truth (ideal corners pushed through the true lens).
    const out = { x: 0, y: 0 };
    let maxErr = 0;
    const truth = [
      [0, 0],
      [TARGET, 0],
      [TARGET, TARGET],
      [0, TARGET],
    ].map(([x, y]) => {
      const ideal = applyHomography(H, x, y);
      distortPoint(DIST, ideal.x, ideal.y, out);
      return { x: out.x, y: out.y };
    });
    for (let i = 0; i < 4; i++) {
      maxErr = Math.max(maxErr, Math.hypot(r.corners![i].x - truth[i].x, r.corners![i].y - truth[i].y));
    }
    expect(maxErr).toBeLessThan(2.0);
  });
});
