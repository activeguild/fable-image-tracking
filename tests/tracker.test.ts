/**
 * End-to-end tracker test on synthetic frames: render a known target into a
 * frame with a known homography and check that detection + tracking recover it.
 */

import { describe, expect, it } from 'vitest';
import { applyHomography, invert3, type Mat3 } from '../src/core/homography';
import { sampleBilinear } from '../src/core/imageops';
import { compileTarget } from '../src/tracker/target';
import { ImageTracker } from '../src/tracker/tracker';
import { randomTexture } from './helpers';

const FRAME_W = 360;
const FRAME_H = 270;
const TARGET_SIZE = 256;

/** Render the target into a frame through H (target px -> frame px). */
function renderFrame(target: Uint8Array, H: Mat3): Uint8Array {
  const Hinv = invert3(H)!;
  const frame = new Uint8Array(FRAME_W * FRAME_H).fill(64);
  // Light background gradient so the frame is not perfectly flat.
  for (let y = 0; y < FRAME_H; y++) {
    for (let x = 0; x < FRAME_W; x++) {
      frame[y * FRAME_W + x] = 48 + ((x * 31 + y * 17) % 23);
    }
  }
  for (let y = 0; y < FRAME_H; y++) {
    for (let x = 0; x < FRAME_W; x++) {
      const p = applyHomography(Hinv, x, y);
      if (p.x >= 0 && p.y >= 0 && p.x < TARGET_SIZE - 1 && p.y < TARGET_SIZE - 1) {
        frame[y * FRAME_W + x] = sampleBilinear(target, TARGET_SIZE, TARGET_SIZE, p.x, p.y) | 0;
      }
    }
  }
  return frame;
}

/** Similarity transform placing the target centre at (tx, ty) in the frame. */
function similarity(scale: number, angle: number, tx: number, ty: number): Mat3 {
  const c = Math.cos(angle) * scale;
  const s = Math.sin(angle) * scale;
  const cx = TARGET_SIZE / 2;
  const cy = TARGET_SIZE / 2;
  return [c, -s, tx - c * cx + s * cy, s, c, ty - s * cx - c * cy, 0, 0, 1];
}

function maxCornerError(H: Mat3, trueH: Mat3): number {
  let maxErr = 0;
  for (const [x, y] of [
    [0, 0],
    [TARGET_SIZE, 0],
    [TARGET_SIZE, TARGET_SIZE],
    [0, TARGET_SIZE],
  ]) {
    const a = applyHomography(H, x, y);
    const b = applyHomography(trueH, x, y);
    maxErr = Math.max(maxErr, Math.hypot(a.x - b.x, a.y - b.y));
  }
  return maxErr;
}

describe('ImageTracker end-to-end', () => {
  const targetImg = randomTexture(TARGET_SIZE, TARGET_SIZE, 77, 9);
  const compiled = compileTarget(targetImg, TARGET_SIZE, TARGET_SIZE, { widthMeters: 0.2 });

  it('compiles a feature-rich target', () => {
    expect(compiled.points.length / 2).toBeGreaterThan(150);
    expect(compiled.descriptors.length).toBe((compiled.points.length / 2) * 8);
  });

  it('detects the target and then tracks it across frames', () => {
    const tracker = new ImageTracker(compiled, FRAME_W, FRAME_H, { detectEveryN: 1 });

    // Frame 1: target at scale 0.55, slight rotation, centred-ish.
    const H1 = similarity(0.55, 0.12, 150, 110);
    const r1 = tracker.processFrame(renderFrame(targetImg, H1));
    expect(r1.state).toBe('tracking');
    expect(r1.H).not.toBeNull();
    expect(maxCornerError(r1.H!, H1)).toBeLessThan(4);
    expect(r1.pose).not.toBeNull();
    expect(r1.pose!.t[2]).toBeGreaterThan(0);

    // Following frames: the target slides and rotates slowly; LK must follow.
    for (let f = 0; f < 5; f++) {
      const H2 = similarity(0.55 + f * 0.01, 0.12 + (f + 1) * 0.02, 150 + (f + 1) * 4, 110 + (f + 1) * 2.5);
      const r = tracker.processFrame(renderFrame(targetImg, H2));
      expect(r.state).toBe('tracking');
      expect(maxCornerError(r.H!, H2)).toBeLessThan(5);
    }
  });

  it('reports searching when the target is absent', () => {
    const tracker = new ImageTracker(compiled, FRAME_W, FRAME_H, { detectEveryN: 1 });
    const empty = randomTexture(FRAME_W, FRAME_H, 500, 7);
    for (let i = 0; i < 3; i++) {
      const r = tracker.processFrame(empty);
      expect(r.state).toBe('searching');
      expect(r.pose).toBeNull();
    }
  });
});
