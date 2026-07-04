/**
 * RANSAC homography estimation with adaptive iteration count, degeneracy
 * checks and a final least-squares refit on the inlier set.
 */

import {
  applyHomography,
  computeHomography,
  invert3,
  symmetricTransferError2,
  type Mat3,
  type Point2,
} from './homography';

export interface RansacResult {
  H: Mat3;
  inliers: number[]; // indices into the input correspondence arrays
}

export interface RansacOptions {
  threshold?: number; // inlier reprojection threshold in pixels
  maxIterations?: number;
  confidence?: number;
  /** Optional deterministic RNG for tests. */
  random?: () => number;
}

export function ransacHomography(
  src: Point2[],
  dst: Point2[],
  options: RansacOptions = {}
): RansacResult | null {
  const n = src.length;
  if (n < 4 || dst.length !== n) return null;
  const { threshold = 3, maxIterations = 500, confidence = 0.995, random = Math.random } = options;
  const thr2 = threshold * threshold;

  let bestInliers: number[] = [];
  let bestH: Mat3 | null = null;
  let iterations = maxIterations;
  const sample = new Int32Array(4);
  const sSrc: Point2[] = new Array(4);
  const sDst: Point2[] = new Array(4);

  for (let iter = 0; iter < iterations; iter++) {
    // Sample 4 distinct indices.
    let ok = true;
    for (let i = 0; i < 4 && ok; i++) {
      let v = -1;
      for (let tries = 0; tries < 50; tries++) {
        const cand = (random() * n) | 0;
        let dup = false;
        for (let j = 0; j < i; j++) {
          if (sample[j] === cand) {
            dup = true;
            break;
          }
        }
        if (!dup) {
          v = cand;
          break;
        }
      }
      if (v < 0) ok = false;
      else sample[i] = v;
    }
    if (!ok) continue;

    for (let i = 0; i < 4; i++) {
      sSrc[i] = src[sample[i]];
      sDst[i] = dst[sample[i]];
    }
    if (isDegenerate(sSrc) || isDegenerate(sDst)) continue;

    const H = computeHomography(sSrc, sDst);
    if (!H) continue;
    const Hinv = invert3(H);
    if (!Hinv) continue;

    const inliers: number[] = [];
    for (let i = 0; i < n; i++) {
      const e = symmetricTransferError2(H, Hinv, src[i].x, src[i].y, dst[i].x, dst[i].y);
      if (e < thr2) inliers.push(i);
    }

    if (inliers.length > bestInliers.length) {
      bestInliers = inliers;
      bestH = H;
      // Adaptive iteration count.
      const w = inliers.length / n;
      const pNoOutlier = 1 - Math.pow(w, 4);
      if (pNoOutlier < 1e-9) break;
      const needed = Math.ceil(Math.log(1 - confidence) / Math.log(pNoOutlier));
      if (needed < iterations) iterations = Math.min(iterations, Math.max(iter + 1, needed));
    }
  }

  if (!bestH || bestInliers.length < 4) return null;

  // Refit on all inliers, then recompute the inlier set once with the refined H.
  const refined = refit(src, dst, bestInliers, thr2);
  if (refined) return refined;
  return { H: bestH, inliers: bestInliers };
}

function refit(src: Point2[], dst: Point2[], inliers: number[], thr2: number): RansacResult | null {
  const is = inliers.map((i) => src[i]);
  const id = inliers.map((i) => dst[i]);
  const H = computeHomography(is, id);
  if (!H) return null;
  const Hinv = invert3(H);
  if (!Hinv) return null;
  const finalInliers: number[] = [];
  for (let i = 0; i < src.length; i++) {
    const e = symmetricTransferError2(H, Hinv, src[i].x, src[i].y, dst[i].x, dst[i].y);
    if (e < thr2) finalInliers.push(i);
  }
  if (finalInliers.length < 4) return null;
  return { H, inliers: finalInliers };
}

/** Reject samples where any 3 of the 4 points are (nearly) collinear. */
function isDegenerate(pts: Point2[]): boolean {
  for (let a = 0; a < 2; a++) {
    for (let b = a + 1; b < 3; b++) {
      for (let c = b + 1; c < 4; c++) {
        const area =
          (pts[b].x - pts[a].x) * (pts[c].y - pts[a].y) -
          (pts[b].y - pts[a].y) * (pts[c].x - pts[a].x);
        if (Math.abs(area) < 1e-3) return true;
      }
    }
  }
  return false;
}

/** Convenience: project the 4 corners of a w x h rectangle through H. */
export function projectCorners(H: Mat3, w: number, h: number): Point2[] {
  return [
    applyHomography(H, 0, 0),
    applyHomography(H, w, 0),
    applyHomography(H, w, h),
    applyHomography(H, 0, h),
  ];
}
