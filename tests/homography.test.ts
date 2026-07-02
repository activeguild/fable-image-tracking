import { describe, expect, it } from 'vitest';
import {
  applyHomography,
  computeHomography,
  invert3,
  matMul3,
  type Mat3,
  type Point2,
} from '../src/core/homography';
import { seededRandom } from './helpers';

const TRUE_H: Mat3 = [1.2, 0.1, 30, -0.15, 0.95, 12, 0.0004, -0.0002, 1];

function makeCorrespondences(n: number, noise = 0, seed = 7): { src: Point2[]; dst: Point2[] } {
  const rand = seededRandom(seed);
  const src: Point2[] = [];
  const dst: Point2[] = [];
  for (let i = 0; i < n; i++) {
    const p = { x: rand() * 300, y: rand() * 300 };
    const q = applyHomography(TRUE_H, p.x, p.y);
    q.x += (rand() - 0.5) * 2 * noise;
    q.y += (rand() - 0.5) * 2 * noise;
    src.push(p);
    dst.push(q);
  }
  return { src, dst };
}

describe('computeHomography', () => {
  it('recovers an exact homography from 4 points', () => {
    const { src, dst } = makeCorrespondences(4);
    const H = computeHomography(src, dst)!;
    expect(H).not.toBeNull();
    for (let i = 0; i < 50; i++) {
      const x = (i * 7) % 300;
      const y = (i * 13) % 300;
      const expected = applyHomography(TRUE_H, x, y);
      const actual = applyHomography(H, x, y);
      expect(actual.x).toBeCloseTo(expected.x, 4);
      expect(actual.y).toBeCloseTo(expected.y, 4);
    }
  });

  it('fits least-squares over many noisy points', () => {
    const { src, dst } = makeCorrespondences(60, 0.5);
    const H = computeHomography(src, dst)!;
    expect(H).not.toBeNull();
    let maxErr = 0;
    for (let i = 0; i < 50; i++) {
      const x = 20 + ((i * 11) % 260);
      const y = 20 + ((i * 17) % 260);
      const expected = applyHomography(TRUE_H, x, y);
      const actual = applyHomography(H, x, y);
      maxErr = Math.max(maxErr, Math.hypot(actual.x - expected.x, actual.y - expected.y));
    }
    expect(maxErr).toBeLessThan(1.0);
  });

  it('returns null for degenerate (collinear) input', () => {
    const src = [
      { x: 0, y: 0 },
      { x: 10, y: 10 },
      { x: 20, y: 20 },
      { x: 30, y: 30 },
    ];
    const dst = src.map((p) => ({ x: p.x + 1, y: p.y + 1 }));
    expect(computeHomography(src, dst)).toBeNull();
  });
});

describe('mat3 utilities', () => {
  it('invert3 gives the identity when multiplied back', () => {
    const inv = invert3(TRUE_H)!;
    const prod = matMul3(TRUE_H, inv);
    const id = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    prod.forEach((v, i) => expect(v).toBeCloseTo(id[i], 8));
  });
});
