import { describe, expect, it } from 'vitest';
import { buildPyramid } from '../src/core/imageops';
import { trackPyrLK } from '../src/core/opticalflow';
import { randomTexture, shiftImage } from './helpers';

describe('trackPyrLK', () => {
  it('recovers a subpixel translation', () => {
    const w = 200;
    const h = 160;
    const dx = 3.7;
    const dy = -2.3;
    const img = randomTexture(w, h, 31, 5);
    const moved = shiftImage(img, w, h, dx, dy);

    const prevPyr = buildPyramid(img, w, h, 3);
    const nextPyr = buildPyramid(moved, w, h, 3);

    const points = [];
    for (let y = 30; y <= 130; y += 25) {
      for (let x = 30; x <= 170; x += 30) points.push({ x, y });
    }
    const flows = trackPyrLK(prevPyr, nextPyr, points);

    let tracked = 0;
    for (let i = 0; i < points.length; i++) {
      if (!flows[i].ok) continue;
      tracked++;
      expect(flows[i].x - points[i].x).toBeCloseTo(dx, 0);
      expect(flows[i].y - points[i].y).toBeCloseTo(dy, 0);
      expect(Math.hypot(flows[i].x - points[i].x - dx, flows[i].y - points[i].y - dy)).toBeLessThan(0.5);
    }
    expect(tracked).toBeGreaterThan(points.length * 0.8);
  });

  it('recovers a larger motion thanks to the pyramid', () => {
    const w = 240;
    const h = 200;
    const dx = 9;
    const dy = 6;
    const img = randomTexture(w, h, 32, 12);
    const moved = shiftImage(img, w, h, dx, dy);
    const prevPyr = buildPyramid(img, w, h, 4);
    const nextPyr = buildPyramid(moved, w, h, 4);

    const points = [
      { x: 60, y: 60 },
      { x: 120, y: 100 },
      { x: 170, y: 80 },
    ];
    const flows = trackPyrLK(prevPyr, nextPyr, points);
    const okFlows = flows.filter((f) => f.ok);
    expect(okFlows.length).toBeGreaterThanOrEqual(2);
    for (let i = 0; i < points.length; i++) {
      if (!flows[i].ok) continue;
      expect(Math.hypot(flows[i].x - points[i].x - dx, flows[i].y - points[i].y - dy)).toBeLessThan(1.0);
    }
  });

  it('follows motion beyond the search range when warm-started', () => {
    const w = 240;
    const h = 200;
    const dx = 25;
    const dy = -14;
    const img = randomTexture(w, h, 33, 6);
    const moved = shiftImage(img, w, h, dx, dy);
    const prevPyr = buildPyramid(img, w, h, 3);
    const nextPyr = buildPyramid(moved, w, h, 3);
    const points = [
      { x: 80, y: 80 },
      { x: 140, y: 110 },
      { x: 100, y: 140 },
    ];

    // Cold start: the shift is far outside the pyramid search range.
    const cold = trackPyrLK(prevPyr, nextPyr, points);
    const coldGood = cold.filter(
      (f, i) => f.ok && Math.hypot(f.x - points[i].x - dx, f.y - points[i].y - dy) < 1
    );

    // Warm start with an (imperfect) motion prediction.
    const guess = points.map((p) => ({ x: p.x + dx - 2, y: p.y + dy + 1.5 }));
    const warm = trackPyrLK(prevPyr, nextPyr, points, { initialGuess: guess });
    const warmGood = warm.filter(
      (f, i) => f.ok && Math.hypot(f.x - points[i].x - dx, f.y - points[i].y - dy) < 1
    );

    expect(warmGood.length).toBe(points.length);
    expect(warmGood.length).toBeGreaterThan(coldGood.length);
  });

  it('flags untrackable (flat) regions', () => {
    const w = 128;
    const h = 128;
    const img = new Uint8Array(w * h).fill(100);
    const pyr = buildPyramid(img, w, h, 3);
    const flows = trackPyrLK(pyr, pyr, [{ x: 64, y: 64 }]);
    expect(flows[0].ok).toBe(false);
  });
});
