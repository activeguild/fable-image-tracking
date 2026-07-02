import { describe, expect, it } from 'vitest';
import { QuadFilter } from '../src/core/quadfilter';
import type { Point2 } from '../src/core/homography';

function quad(offset: number): Point2[] {
  return [
    { x: 10 + offset, y: 20 },
    { x: 110 + offset, y: 20 },
    { x: 110 + offset, y: 120 },
    { x: 10 + offset, y: 120 },
  ];
}

describe('QuadFilter', () => {
  it('returns the last quad with a single sample', () => {
    const f = new QuadFilter();
    f.addSample(quad(0), 10.0);
    const q = f.predict(10.01)!;
    expect(q).not.toBeNull();
    expect(q[0].x).toBeCloseTo(10, 5);
    expect(q[2].y).toBeCloseTo(120, 5);
  });

  it('extrapolates constant-velocity motion to the query time', () => {
    const f = new QuadFilter({ minCutoff: 1000, beta: 1000 }); // effectively unfiltered
    // 100 px/s to the right, samples 40 ms apart.
    f.addSample(quad(0), 10.0);
    f.addSample(quad(4), 10.04);
    const q = f.predict(10.08)!;
    expect(q[0].x).toBeCloseTo(18, 1);
    expect(q[1].x).toBeCloseTo(118, 1);
    expect(q[0].y).toBeCloseTo(20, 1);
  });

  it('clamps the extrapolation horizon and goes stale', () => {
    const f = new QuadFilter({ maxHorizon: 0.05, maxAge: 0.3, minCutoff: 1000, beta: 1000 });
    f.addSample(quad(0), 10.0);
    f.addSample(quad(4), 10.04);
    // 200 ms ahead: capped at 50 ms => x = 4 + 4 * (0.05/0.04) = 9
    const q = f.predict(10.24)!;
    expect(q[0].x).toBeCloseTo(10 + 9, 1);
    expect(f.predict(10.4)).toBeNull(); // past maxAge
  });

  it('clears cleanly', () => {
    const f = new QuadFilter();
    f.addSample(quad(0), 10.0);
    f.clear();
    expect(f.predict(10.01)).toBeNull();
  });
});
