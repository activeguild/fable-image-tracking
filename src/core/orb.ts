/**
 * ORB-style rotated BRIEF descriptor, from scratch.
 *
 * 256 binary tests compare box-filtered intensities at point pairs inside a
 * 31x31 patch. The pair pattern is generated once from a fixed-seed PRNG so
 * the compiled target and the live frames always use the identical pattern.
 * Each test pair is rotated by the keypoint orientation (steered BRIEF).
 */

import { boxSum, integralImage } from './imageops';
import type { Keypoint } from './fast';

export const DESCRIPTOR_WORDS = 8; // 256 bits = 8 x Uint32

/** Border margin required around a keypoint: rotated pattern reach + box radius. */
export const PATCH_BORDER = 22;

const PATTERN_SIZE = 256;
const PATTERN_SIGMA = 6.5;
const PATTERN_CLAMP = 13; // 13 * sqrt(2) + box radius 2 < PATCH_BORDER

/** Deterministic PRNG (mulberry32). */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function buildPattern(): Int8Array {
  const rand = mulberry32(0x51f7a3);
  const gauss = () => {
    // Box-Muller
    let u = 0;
    let v = 0;
    while (u === 0) u = rand();
    while (v === 0) v = rand();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };
  const pat = new Int8Array(PATTERN_SIZE * 4); // x1,y1,x2,y2 per test
  for (let i = 0; i < PATTERN_SIZE * 4; i++) {
    let c = Math.round(gauss() * PATTERN_SIGMA);
    if (c > PATTERN_CLAMP) c = PATTERN_CLAMP;
    else if (c < -PATTERN_CLAMP) c = -PATTERN_CLAMP;
    pat[i] = c;
  }
  return pat;
}

const PATTERN = buildPattern();

/** The shared test-pair pattern, exposed so the WASM engine can upload it. */
export function getPattern(): Int8Array {
  return PATTERN;
}

/**
 * Compute descriptors for keypoints (which must lie at least PATCH_BORDER away
 * from image edges). Returns a flat Uint32Array of 8 words per keypoint.
 */
export function computeDescriptors(
  img: Uint8Array,
  width: number,
  height: number,
  keypoints: Keypoint[]
): Uint32Array {
  const ii = integralImage(img, width, height);
  const out = new Uint32Array(keypoints.length * DESCRIPTOR_WORDS);
  const maxX = width - 3;
  const maxY = height - 3;

  for (let k = 0; k < keypoints.length; k++) {
    const kp = keypoints[k];
    const cos = Math.cos(kp.angle);
    const sin = Math.sin(kp.angle);
    const cx = Math.round(kp.x);
    const cy = Math.round(kp.y);
    const base = k * DESCRIPTOR_WORDS;

    let word = 0;
    let bit = 0;
    let wordIdx = base;
    for (let i = 0; i < PATTERN_SIZE; i++) {
      const p = i * 4;
      const v1 = smoothed(ii, width, cx, cy, PATTERN[p], PATTERN[p + 1], cos, sin, maxX, maxY);
      const v2 = smoothed(ii, width, cx, cy, PATTERN[p + 2], PATTERN[p + 3], cos, sin, maxX, maxY);
      if (v1 < v2) word |= 1 << bit;
      bit++;
      if (bit === 32) {
        out[wordIdx++] = word >>> 0;
        word = 0;
        bit = 0;
      }
    }
  }
  return out;
}

function smoothed(
  ii: Uint32Array,
  width: number,
  cx: number,
  cy: number,
  px: number,
  py: number,
  cos: number,
  sin: number,
  maxX: number,
  maxY: number
): number {
  // Rotate the pattern offset by the keypoint orientation.
  let x = cx + Math.round(cos * px - sin * py);
  let y = cy + Math.round(sin * px + cos * py);
  if (x < 2) x = 2;
  else if (x > maxX) x = maxX;
  if (y < 2) y = 2;
  else if (y > maxY) y = maxY;
  // 5x5 box filter via the integral image.
  return boxSum(ii, width, x - 2, y - 2, x + 2, y + 2);
}

/** Hamming distance between two 8-word descriptors stored in flat arrays. */
export function hammingDistance(a: Uint32Array, ai: number, b: Uint32Array, bi: number): number {
  let dist = 0;
  for (let w = 0; w < DESCRIPTOR_WORDS; w++) {
    let v = (a[ai + w] ^ b[bi + w]) >>> 0;
    v -= (v >>> 1) & 0x55555555;
    v = (v & 0x33333333) + ((v >>> 2) & 0x33333333);
    dist += (Math.imul((v + (v >>> 4)) & 0x0f0f0f0f, 0x01010101) >>> 24);
  }
  return dist;
}
