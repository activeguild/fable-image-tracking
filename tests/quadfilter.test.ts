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

  it('an external motion model removes reversal overshoot', () => {
    // Hand motion: +100 px/s until t=10.04, then it reverses to -100 px/s.
    // Constant velocity overshoots; the measured model does not.
    const velocity = (t: number) => (t < 10.04 ? 100 : -100);
    const displacement = (from: number, to: number) => {
      // integrate the piecewise velocity
      let d = 0;
      const mid = 10.04;
      if (to <= mid || from >= mid) return velocity(from) * (to - from);
      d += velocity(from) * (mid - from);
      d += velocity(mid) * (to - mid);
      return d;
    };
    const advance = (corners: { x: number; y: number }[], fromSec: number, toSec: number) =>
      corners.map((c) => ({ x: c.x + displacement(fromSec, toSec), y: c.y }));

    const unfiltered = { minCutoff: 1000, beta: 1000 };
    const plain = new QuadFilter(unfiltered);
    const fused = new QuadFilter(unfiltered);
    plain.addSample(quad(0), 10.0);
    plain.addSample(quad(4), 10.04);
    fused.addSample(quad(0), 10.0);
    fused.addSample(quad(4), 10.04);

    // Truth at 10.08: reversed motion brings x back to 0.
    const overshoot = plain.predict(10.08)!;
    expect(overshoot[0].x).toBeCloseTo(10 + 8, 1); // constant velocity keeps going

    const corrected = fused.predict(10.08, advance)!;
    expect(corrected[0].x).toBeCloseTo(10 + 0, 1); // model followed the reversal
  });

  it('never lets a bad motion model deform the quad (rigidity guard)', () => {
    const f = new QuadFilter({ minCutoff: 1000, beta: 1000 });
    f.addSample(quad(0), 10.0);
    f.addSample(quad(2), 10.03);
    // Pathological model: flings a single corner far away (what a bad
    // residual/glitch used to do under violent motion).
    const evil = (corners: { x: number; y: number }[]) =>
      corners.map((c, i) => (i === 2 ? { x: c.x + 300, y: c.y - 200 } : { ...c }));
    const q = f.predict(10.06, evil)!;
    expect(q).not.toBeNull();
    // Guarded output stays close to the last measured quad, still rigid.
    const drift = Math.hypot(q[2].x - (110 + 2), q[2].y - 120);
    expect(drift).toBeLessThan(30);
    // Shape sanity: opposite edges stay near-equal.
    const top = Math.hypot(q[1].x - q[0].x, q[1].y - q[0].y);
    const bottom = Math.hypot(q[2].x - q[3].x, q[2].y - q[3].y);
    expect(Math.abs(top - bottom) / top).toBeLessThan(0.2);
  });

  it('clears cleanly', () => {
    const f = new QuadFilter();
    f.addSample(quad(0), 10.0);
    f.clear();
    expect(f.predict(10.01)).toBeNull();
  });
});
