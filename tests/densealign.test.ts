import { describe, expect, it } from 'vitest';
import { DenseAligner } from '../src/core/densealign';
import { applyHomography, invert3, matMul3, type Mat3 } from '../src/core/homography';
import { sampleBilinear } from '../src/core/imageops';
import { randomTexture } from './helpers';

const TARGET = 256;
const FRAME_W = 360;
const FRAME_H = 270;
const targetImg = randomTexture(TARGET, TARGET, 77, 9);

function similarity(scale: number, angle: number, tx: number, ty: number): Mat3 {
  const c = Math.cos(angle) * scale;
  const s = Math.sin(angle) * scale;
  const cx = TARGET / 2;
  const cy = TARGET / 2;
  return [c, -s, tx - c * cx + s * cy, s, c, ty - s * cx - c * cy, 0, 0, 1];
}

function renderFrame(H: Mat3, gain = 1, bias = 0): Uint8Array {
  const Hinv = invert3(H)!;
  const frame = new Uint8Array(FRAME_W * FRAME_H);
  for (let y = 0; y < FRAME_H; y++) {
    for (let x = 0; x < FRAME_W; x++) {
      const p = applyHomography(Hinv, x, y);
      let v: number;
      if (p.x >= 0 && p.y >= 0 && p.x < TARGET - 1 && p.y < TARGET - 1) {
        v = sampleBilinear(targetImg, TARGET, TARGET, p.x, p.y) * gain + bias;
      } else {
        v = 60 + ((x * 13 + y * 7) % 17);
      }
      frame[y * FRAME_W + x] = Math.max(0, Math.min(255, v)) | 0;
    }
  }
  return frame;
}

function maxCornerError(a: Mat3, b: Mat3): number {
  let err = 0;
  for (const [x, y] of [[0, 0], [TARGET, 0], [TARGET, TARGET], [0, TARGET]]) {
    const pa = applyHomography(a, x, y);
    const pb = applyHomography(b, x, y);
    err = Math.max(err, Math.hypot(pa.x - pb.x, pa.y - pb.y));
  }
  return err;
}

/** Perturb H by composing a small similarity on the target side. */
function perturb(H: Mat3, dx: number, dy: number, dAngle: number, dScale: number): Mat3 {
  return matMul3(H, similarity(dScale, dAngle, TARGET / 2 + dx, TARGET / 2 + dy));
}

describe('DenseAligner', () => {
  const trueH = similarity(0.6, 0.2, 170, 130);
  const frame = renderFrame(trueH);

  it('builds a valid template', () => {
    const aligner = new DenseAligner(targetImg, TARGET, TARGET);
    expect(aligner.valid).toBe(true);
  });

  it('refines a perturbed homography to subpixel accuracy', () => {
    const aligner = new DenseAligner(targetImg, TARGET, TARGET);
    const rough = perturb(trueH, 1.8, -1.2, 0.012, 1.015);
    expect(maxCornerError(rough, trueH)).toBeGreaterThan(2);

    const refined = aligner.align(rough, frame, FRAME_W, FRAME_H)!;
    expect(refined).not.toBeNull();
    expect(maxCornerError(refined.H, trueH)).toBeLessThan(0.5);
  });

  it('is invariant to gain and bias lighting changes', () => {
    const aligner = new DenseAligner(targetImg, TARGET, TARGET);
    const darker = renderFrame(trueH, 0.7, 25);
    const rough = perturb(trueH, -1.5, 1.0, -0.01, 0.99);
    const refined = aligner.align(rough, darker, FRAME_W, FRAME_H)!;
    expect(refined).not.toBeNull();
    expect(maxCornerError(refined.H, trueH)).toBeLessThan(0.6);
  });

  it('stays accurate under partial occlusion (Huber weights)', () => {
    const aligner = new DenseAligner(targetImg, TARGET, TARGET);
    // Cover ~25% of the target (one quadrant) with unrelated noise.
    const occluded = Uint8Array.from(frame);
    const Hquad = trueH;
    let seed = 12345;
    const rand = () => {
      seed = (seed * 1664525 + 1013904223) >>> 0;
      return seed / 4294967296;
    };
    for (let y = 0; y < FRAME_H; y++) {
      for (let x = 0; x < FRAME_W; x++) {
        const p = applyHomography(invert3(Hquad)!, x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < TARGET / 2 && p.y < TARGET / 2) {
          occluded[y * FRAME_W + x] = (rand() * 255) | 0;
        }
      }
    }
    const rough = perturb(trueH, 1.5, -1.0, 0.01, 1.01);
    const refined = aligner.align(rough, occluded, FRAME_W, FRAME_H)!;
    expect(refined).not.toBeNull();
    expect(maxCornerError(refined.H, trueH)).toBeLessThan(1.0);
  });

  it('returns null when the target is mostly outside the frame', () => {
    const aligner = new DenseAligner(targetImg, TARGET, TARGET);
    const offscreen = similarity(0.6, 0, -200, -200);
    expect(aligner.align(offscreen, frame, FRAME_W, FRAME_H)).toBeNull();
  });
});
