import { describe, expect, it } from 'vitest';
import { computeDescriptors, DESCRIPTOR_WORDS, hammingDistance, PATCH_BORDER } from '../src/core/orb';
import { computeOrientation, detectFast } from '../src/core/fast';
import { matchDescriptors } from '../src/core/matcher';
import { randomTexture, shiftImage } from './helpers';

describe('ORB descriptors', () => {
  it('is deterministic for the same keypoint', () => {
    const img = randomTexture(128, 128, 11);
    const kps = [{ x: 64, y: 64, score: 1, angle: 0.3 }];
    const d1 = computeDescriptors(img, 128, 128, kps);
    const d2 = computeDescriptors(img, 128, 128, kps);
    expect(hammingDistance(d1, 0, d2, 0)).toBe(0);
  });

  it('produces distant descriptors for different points', () => {
    const img = randomTexture(160, 160, 12);
    const kps = [
      { x: 50, y: 50, score: 1, angle: 0 },
      { x: 110, y: 100, score: 1, angle: 0 },
    ];
    const d = computeDescriptors(img, 160, 160, kps);
    // Random binary strings differ in ~128 bits; unrelated patches should be far apart.
    expect(hammingDistance(d, 0, d, DESCRIPTOR_WORDS)).toBeGreaterThan(60);
  });

  it('matches keypoints across a translated image', () => {
    const w = 240;
    const h = 200;
    const img = randomTexture(w, h, 21, 5);
    const shifted = shiftImage(img, w, h, 8, -5);

    const detect = (im: Uint8Array) => {
      const kps = detectFast(im, w, h, 12, PATCH_BORDER);
      for (const kp of kps) kp.angle = computeOrientation(im, w, h, kp.x, kp.y);
      return kps;
    };
    const kA = detect(img);
    const kB = detect(shifted);
    expect(kA.length).toBeGreaterThan(20);
    expect(kB.length).toBeGreaterThan(20);

    const dA = computeDescriptors(img, w, h, kA);
    const dB = computeDescriptors(shifted, w, h, kB);
    const matches = matchDescriptors(dA, dB, { maxDistance: 64, ratio: 0.85 });
    expect(matches.length).toBeGreaterThan(10);

    // The dominant displacement among matches must be the true shift.
    let good = 0;
    for (const m of matches) {
      const dx = kB[m.b].x - kA[m.a].x;
      const dy = kB[m.b].y - kA[m.a].y;
      if (Math.hypot(dx - 8, dy + 5) < 2) good++;
    }
    expect(good / matches.length).toBeGreaterThan(0.7);
  });
});
