import { describe, expect, it } from 'vitest';
import { computeOrientation, detectFast, selectSpread } from '../src/core/fast';

function blank(w: number, h: number, value = 0): Uint8Array {
  return new Uint8Array(w * h).fill(value);
}

describe('detectFast', () => {
  it('finds nothing in a flat image', () => {
    expect(detectFast(blank(64, 64, 128), 64, 64, 20, 8)).toHaveLength(0);
  });

  it('detects the corners of a bright square', () => {
    const w = 96;
    const h = 96;
    const img = blank(w, h, 20);
    for (let y = 30; y < 66; y++) {
      for (let x = 30; x < 66; x++) img[y * w + x] = 220;
    }
    const kps = detectFast(img, w, h, 30, 8);
    expect(kps.length).toBeGreaterThan(0);

    // Every expected square corner should have a detection within 3px.
    const expected = [
      [30, 30],
      [65, 30],
      [30, 65],
      [65, 65],
    ];
    for (const [ex, ey] of expected) {
      const near = kps.some((kp) => Math.hypot(kp.x - ex, kp.y - ey) <= 3);
      expect(near, `corner near (${ex},${ey})`).toBe(true);
    }

    // Edges (non-corner) should not fire: no detections along the middle of the top edge.
    const midEdge = kps.some((kp) => Math.abs(kp.y - 30) <= 1 && kp.x > 40 && kp.x < 56);
    expect(midEdge).toBe(false);
  });

  it('respects the border margin', () => {
    const w = 64;
    const h = 64;
    const img = blank(w, h, 20);
    // Bright blob touching the image corner.
    for (let y = 0; y < 12; y++) for (let x = 0; x < 12; x++) img[y * w + x] = 250;
    const kps = detectFast(img, w, h, 30, 16);
    for (const kp of kps) {
      expect(kp.x).toBeGreaterThanOrEqual(16);
      expect(kp.y).toBeGreaterThanOrEqual(16);
    }
  });
});

describe('selectSpread', () => {
  it('caps the total and keeps the strongest points', () => {
    const kps = [];
    for (let i = 0; i < 500; i++) {
      kps.push({ x: (i * 13) % 320, y: (i * 29) % 240, score: i, angle: 0 });
    }
    const selected = selectSpread(kps, 320, 240, 100);
    expect(selected.length).toBeLessThanOrEqual(100);
    // The single strongest keypoint must survive.
    expect(selected.some((kp) => kp.score === 499)).toBe(true);
  });
});

describe('computeOrientation', () => {
  it('points along the intensity gradient', () => {
    const w = 64;
    const h = 64;
    const img = new Uint8Array(w * h);
    // Brighter to the right -> centroid to the right -> angle ~ 0.
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) img[y * w + x] = x * 3;
    const angle = computeOrientation(img, w, h, 32, 32);
    expect(Math.abs(angle)).toBeLessThan(0.05);

    // Brighter downward -> angle ~ +PI/2 (image y down).
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) img[y * w + x] = y * 3;
    const angle2 = computeOrientation(img, w, h, 32, 32);
    expect(Math.abs(angle2 - Math.PI / 2)).toBeLessThan(0.05);
  });
});
