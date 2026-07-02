/**
 * Pyramidal Lucas-Kanade optical flow (Bouguet's formulation), from scratch.
 * Tracks sparse points from a previous image pyramid into the next one.
 */

import { sampleBilinear, type PyramidLevel } from './imageops';
import type { Point2 } from './homography';

export interface FlowResult {
  x: number;
  y: number;
  ok: boolean;
  /** Mean absolute residual at convergence (intensity units). */
  err: number;
}

export interface FlowOptions {
  windowRadius?: number; // half window size (window = 2r+1)
  maxIterations?: number;
  epsilon?: number; // stop when the update is smaller than this (pixels)
  maxError?: number; // reject tracks with residual above this
}

export function trackPyrLK(
  prevPyr: PyramidLevel[],
  nextPyr: PyramidLevel[],
  points: Point2[],
  options: FlowOptions = {}
): FlowResult[] {
  const { windowRadius = 4, maxIterations = 12, epsilon = 0.01, maxError = 24 } = options;
  const numLevels = Math.min(prevPyr.length, nextPyr.length);
  const win = 2 * windowRadius + 1;
  const winArea = win * win;

  const gradX = new Float32Array(winArea);
  const gradY = new Float32Array(winArea);
  const template = new Float32Array(winArea);

  return points.map((p) => trackPoint(p));

  function trackPoint(p: Point2): FlowResult {
    let gx = 0; // flow guess, in the coordinates of the current level
    let gy = 0;
    let outX = p.x;
    let outY = p.y;
    let finalErr = Infinity;

    for (let L = numLevels - 1; L >= 0; L--) {
      const prev = prevPyr[L];
      const next = nextPyr[L];
      const px = p.x / prev.scale;
      const py = p.y / prev.scale;
      const margin = windowRadius + 1;

      let converged = false;
      let vx = 0;
      let vy = 0;
      let err = Infinity;

      const inPrev =
        px >= margin && py >= margin && px < prev.width - margin && py < prev.height - margin;

      if (inPrev) {
        // Template values and spatial gradients from the previous image.
        let sxx = 0;
        let sxy = 0;
        let syy = 0;
        let idx = 0;
        for (let dy = -windowRadius; dy <= windowRadius; dy++) {
          for (let dx = -windowRadius; dx <= windowRadius; dx++, idx++) {
            const x = px + dx;
            const y = py + dy;
            template[idx] = sampleBilinear(prev.data, prev.width, prev.height, x, y);
            const gxv =
              (sampleBilinear(prev.data, prev.width, prev.height, x + 1, y) -
                sampleBilinear(prev.data, prev.width, prev.height, x - 1, y)) * 0.5;
            const gyv =
              (sampleBilinear(prev.data, prev.width, prev.height, x, y + 1) -
                sampleBilinear(prev.data, prev.width, prev.height, x, y - 1)) * 0.5;
            gradX[idx] = gxv;
            gradY[idx] = gyv;
            sxx += gxv * gxv;
            sxy += gxv * gyv;
            syy += gyv * gyv;
          }
        }

        const det = sxx * syy - sxy * sxy;
        if (det >= 1e-4) {
          const invDet = 1 / det;
          for (let it = 0; it < maxIterations; it++) {
            const qx = px + gx + vx;
            const qy = py + gy + vy;
            if (
              qx < margin || qy < margin ||
              qx >= next.width - margin || qy >= next.height - margin
            ) {
              break;
            }
            let bx = 0;
            let by = 0;
            let absSum = 0;
            idx = 0;
            for (let dy = -windowRadius; dy <= windowRadius; dy++) {
              for (let dx = -windowRadius; dx <= windowRadius; dx++, idx++) {
                const dI =
                  sampleBilinear(next.data, next.width, next.height, qx + dx, qy + dy) -
                  template[idx];
                bx += dI * gradX[idx];
                by += dI * gradY[idx];
                absSum += Math.abs(dI);
              }
            }
            err = absSum / winArea;
            // Solve G d = -b for the incremental flow d.
            const dxStep = (-bx * syy + by * sxy) * invDet;
            const dyStep = (-by * sxx + bx * sxy) * invDet;
            vx += dxStep;
            vy += dyStep;
            converged = true;
            if (dxStep * dxStep + dyStep * dyStep < epsilon * epsilon) break;
          }
        }
      }

      if (L > 0) {
        // Propagate the guess to the next (finer) level's coordinates.
        const ratio = prev.scale / prevPyr[L - 1].scale;
        gx = (gx + vx) * ratio;
        gy = (gy + vy) * ratio;
      } else {
        if (!converged) return { x: p.x, y: p.y, ok: false, err: Infinity };
        outX = (px + gx + vx) * prev.scale;
        outY = (py + gy + vy) * prev.scale;
        finalErr = err;
      }
    }

    const ok =
      finalErr <= maxError &&
      outX >= 0 && outY >= 0 &&
      outX < prevPyr[0].width && outY < prevPyr[0].height;
    return { x: outX, y: outY, ok, err: finalErr };
  }
}
