/**
 * Brute-force Hamming matcher with cross-check and Lowe ratio test.
 */

import { DESCRIPTOR_WORDS, hammingDistance } from './orb';

export interface Match {
  a: number; // index into descriptor set A
  b: number; // index into descriptor set B
  dist: number;
}

export interface MatchOptions {
  maxDistance?: number; // reject matches with Hamming distance above this
  ratio?: number; // Lowe ratio: best < ratio * secondBest
  crossCheck?: boolean;
}

export function matchDescriptors(
  descA: Uint32Array,
  descB: Uint32Array,
  options: MatchOptions = {}
): Match[] {
  const { maxDistance = 64, ratio = 0.85, crossCheck = true } = options;
  const nA = descA.length / DESCRIPTOR_WORDS;
  const nB = descB.length / DESCRIPTOR_WORDS;
  if (nA === 0 || nB === 0) return [];

  const bestForB = crossCheck ? new Int32Array(nB).fill(-1) : null;
  const bestDistForB = crossCheck ? new Int32Array(nB).fill(0x7fffffff) : null;

  const matches: Match[] = [];
  for (let a = 0; a < nA; a++) {
    const ai = a * DESCRIPTOR_WORDS;
    let best = 0x7fffffff;
    let second = 0x7fffffff;
    let bestB = -1;
    for (let b = 0; b < nB; b++) {
      const d = hammingDistance(descA, ai, descB, b * DESCRIPTOR_WORDS);
      if (d < best) {
        second = best;
        best = d;
        bestB = b;
      } else if (d < second) {
        second = d;
      }
      if (bestForB && bestDistForB && d < bestDistForB[b]) {
        bestDistForB[b] = d;
        bestForB[b] = a;
      }
    }
    if (bestB >= 0 && best <= maxDistance && best < ratio * second) {
      matches.push({ a, b: bestB, dist: best });
    }
  }

  if (!bestForB) return matches;
  // Cross-check: keep a->b only if a is also b's best partner.
  return matches.filter((m) => bestForB[m.b] === m.a);
}
