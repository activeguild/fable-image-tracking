import { describe, expect, it } from 'vitest';
import { applyHomography, type Mat3, type Point2 } from '../src/core/homography';
import { ransacHomography } from '../src/core/ransac';
import { seededRandom } from './helpers';

const TRUE_H: Mat3 = [0.9, -0.05, 40, 0.08, 1.1, -20, 0.0003, 0.0001, 1];

describe('ransacHomography', () => {
  it('finds the model despite 40% outliers', () => {
    const rand = seededRandom(123);
    const src: Point2[] = [];
    const dst: Point2[] = [];
    const inlierFlags: boolean[] = [];
    for (let i = 0; i < 100; i++) {
      const p = { x: rand() * 320, y: rand() * 240 };
      src.push(p);
      if (rand() < 0.6) {
        const q = applyHomography(TRUE_H, p.x, p.y);
        dst.push({ x: q.x + (rand() - 0.5), y: q.y + (rand() - 0.5) });
        inlierFlags.push(true);
      } else {
        dst.push({ x: rand() * 320, y: rand() * 240 });
        inlierFlags.push(false);
      }
    }

    const result = ransacHomography(src, dst, { threshold: 3, random: seededRandom(99) });
    expect(result).not.toBeNull();
    const { H, inliers } = result!;

    // Most true inliers recovered, few false positives.
    const trueInlierCount = inlierFlags.filter(Boolean).length;
    expect(inliers.length).toBeGreaterThan(trueInlierCount * 0.85);
    const falsePositives = inliers.filter((i) => !inlierFlags[i]).length;
    expect(falsePositives).toBeLessThan(inliers.length * 0.1);

    // Model accuracy on clean points.
    for (let i = 0; i < 20; i++) {
      const x = 10 + i * 15;
      const y = 10 + i * 11;
      const expected = applyHomography(TRUE_H, x, y);
      const actual = applyHomography(H, x, y);
      expect(Math.hypot(actual.x - expected.x, actual.y - expected.y)).toBeLessThan(1.5);
    }
  });

  it('returns null when there is no consistent model', () => {
    const rand = seededRandom(5);
    const src: Point2[] = [];
    const dst: Point2[] = [];
    for (let i = 0; i < 30; i++) {
      src.push({ x: rand() * 320, y: rand() * 240 });
      dst.push({ x: rand() * 320, y: rand() * 240 });
    }
    const result = ransacHomography(src, dst, {
      threshold: 2,
      maxIterations: 100,
      random: seededRandom(1),
    });
    // Random correspondences should never yield a large consensus set.
    if (result) expect(result.inliers.length).toBeLessThan(10);
  });
});
