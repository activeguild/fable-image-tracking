/**
 * Dense homography refinement by inverse-compositional Gauss-Newton image
 * alignment (Baker & Matthews / ESM style), the "last mile" used by
 * commercial trackers: after the point-based homography estimate, align the
 * warped reference target directly against the camera frame to squeeze the
 * estimate down to subpixel accuracy. Point-based estimates carry the noise
 * of individual corners; dense alignment averages over the whole texture.
 *
 * Robustness: residuals are reweighted with a Huber M-estimator (IRLS) each
 * iteration, so partial occlusion (a hand over a corner of the target) or
 * specular highlights cannot drag the estimate - outliers saturate instead
 * of contributing quadratically. Gain/bias lighting changes are handled by
 * normalising both sides to the template's statistics.
 *
 * Inverse compositional: Jacobians live on the fixed template, so the
 * expensive parts (gradients, per-point Jacobians) are precomputed once at
 * target compile time. Each frame iteration samples the camera image,
 * reweights, and solves one 8x8 system.
 *
 * An optional radial distortion model (k1, division-free Brown model) can be
 * applied on the sampling side, so the homography stays defined in ideal
 * (undistorted) pixel coordinates while samples are read from the real,
 * distorted camera image.
 */

import { invert3, matMul3, solveLinearSystem, type Mat3 } from './homography';
import { resizeBilinear, sampleBilinear } from './imageops';

export interface DenseAlignOptions {
  /** Template width in pixels (the reference is downscaled to this). */
  templateWidth?: number;
  /** Maximum number of high-gradient sample points. */
  maxPoints?: number;
  maxIterations?: number;
  /** Stop when the robust residual improves less than this factor. */
  minImprovement?: number;
}

/** Radial lens distortion (Brown, k1 only), in normalized camera units. */
export interface RadialDistortion {
  k1: number;
  cx: number;
  cy: number;
  /** Focal length in the same pixel units as cx/cy. */
  f: number;
}

export interface AlignResult {
  H: Mat3;
  /** Robust mean absolute residual (intensity units) at the solution. */
  err: number;
}

/** Ideal (undistorted) pixel -> observed (distorted) pixel. */
export function distortPoint(
  d: RadialDistortion,
  x: number,
  y: number,
  out: { x: number; y: number }
): void {
  const nx = (x - d.cx) / d.f;
  const ny = (y - d.cy) / d.f;
  const s = 1 + d.k1 * (nx * nx + ny * ny);
  out.x = d.cx + nx * s * d.f;
  out.y = d.cy + ny * s * d.f;
}

/** Observed (distorted) pixel -> ideal (undistorted) pixel (Newton on radius). */
export function undistortPoint(
  d: RadialDistortion,
  x: number,
  y: number,
  out: { x: number; y: number }
): void {
  const dx = (x - d.cx) / d.f;
  const dy = (y - d.cy) / d.f;
  const rd = Math.hypot(dx, dy);
  if (rd < 1e-12) {
    out.x = x;
    out.y = y;
    return;
  }
  // Solve r * (1 + k1 r^2) = rd for the ideal radius r.
  let r = rd;
  for (let i = 0; i < 6; i++) {
    const f = r * (1 + d.k1 * r * r) - rd;
    const df = 1 + 3 * d.k1 * r * r;
    if (Math.abs(df) < 1e-9) break;
    r -= f / df;
  }
  const scale = r / rd;
  out.x = d.cx + dx * scale * d.f;
  out.y = d.cy + dy * scale * d.f;
}

export class DenseAligner {
  private readonly tmplW: number;
  private readonly tmplH: number;
  /** template px -> reference-target px */
  private readonly tmplToTarget: number;
  private readonly px: Float32Array; // sample coords (template px)
  private readonly py: Float32Array;
  private readonly tVal: Float32Array; // template intensities at samples
  private readonly J: Float32Array; // 8 Jacobian entries per sample
  private readonly meanT: number;
  private readonly stdT: number;
  private readonly maxIterations: number;
  private readonly minImprovement: number;
  readonly valid: boolean;

  constructor(targetGray: Uint8Array, targetW: number, targetH: number, options: DenseAlignOptions = {}) {
    const { templateWidth = 132, maxPoints = 1400, maxIterations = 8, minImprovement = 0.01 } = options;
    this.maxIterations = maxIterations;
    this.minImprovement = minImprovement;

    const scale = Math.min(1, templateWidth / targetW);
    this.tmplW = Math.round(targetW * scale);
    this.tmplH = Math.round(targetH * scale);
    this.tmplToTarget = targetW / this.tmplW;
    const tmpl =
      scale < 1 ? resizeBilinear(targetGray, targetW, targetH, this.tmplW, this.tmplH) : targetGray;

    // Pick the strongest-gradient pixels (away from the border), spread out
    // by taking at most one per small cell.
    interface Cand {
      x: number;
      y: number;
      gx: number;
      gy: number;
      mag: number;
    }
    const w = this.tmplW;
    const h = this.tmplH;
    const cell = 2;
    const bestPerCell = new Map<number, Cand>();
    for (let y = 2; y < h - 2; y++) {
      for (let x = 2; x < w - 2; x++) {
        const gx = (tmpl[y * w + x + 1] - tmpl[y * w + x - 1]) * 0.5;
        const gy = (tmpl[(y + 1) * w + x] - tmpl[(y - 1) * w + x]) * 0.5;
        const mag = gx * gx + gy * gy;
        if (mag < 25) continue;
        const key = ((y / cell) | 0) * 4096 + ((x / cell) | 0);
        const cur = bestPerCell.get(key);
        if (!cur || mag > cur.mag) bestPerCell.set(key, { x, y, gx, gy, mag });
      }
    }
    const cands = [...bestPerCell.values()].sort((a, b) => b.mag - a.mag).slice(0, maxPoints);
    const n = cands.length;
    this.valid = n >= 64;

    this.px = new Float32Array(n);
    this.py = new Float32Array(n);
    this.tVal = new Float32Array(n);
    this.J = new Float32Array(n * 8);

    let meanT = 0;
    for (let i = 0; i < n; i++) {
      const c = cands[i];
      this.px[i] = c.x;
      this.py[i] = c.y;
      const t = tmpl[c.y * w + c.x];
      this.tVal[i] = t;
      meanT += t;
      // J = gx * dWx/dp + gy * dWy/dp at the identity warp.
      const { x, y, gx, gy } = c;
      const base = i * 8;
      this.J[base] = gx * x;
      this.J[base + 1] = gy * x;
      this.J[base + 2] = gx * y;
      this.J[base + 3] = gy * y;
      this.J[base + 4] = gx;
      this.J[base + 5] = gy;
      this.J[base + 6] = -gx * x * x - gy * x * y;
      this.J[base + 7] = -gx * x * y - gy * y * y;
    }
    meanT /= Math.max(1, n);
    let varT = 0;
    for (let i = 0; i < n; i++) varT += (this.tVal[i] - meanT) ** 2;
    this.meanT = meanT;
    this.stdT = Math.sqrt(varT / Math.max(1, n)) || 1;
  }

  /**
   * Refine `H` (reference-target px -> ideal frame px) against the frame.
   * When `distortion` is given, samples are read through the radial model
   * (the image is distorted; H stays ideal). Returns null when alignment is
   * unreliable.
   */
  align(
    H: Mat3,
    frame: Uint8Array,
    frameW: number,
    frameH: number,
    distortion?: RadialDistortion
  ): AlignResult | null {
    if (!this.valid) return null;
    const s = this.tmplToTarget;
    const S: Mat3 = [s, 0, 0, 0, s, 0, 0, 0, 1];
    const Sinv: Mat3 = [1 / s, 0, 0, 0, 1 / s, 0, 0, 0, 1];
    let Ht = matMul3(H, S); // template px -> frame px

    const n = this.px.length;
    const fVal = new Float32Array(n);
    const resid = new Float32Array(n);
    const validMask = new Uint8Array(n);
    const absResid = new Float32Array(n);
    const dpt = { x: 0, y: 0 };
    const useDist = distortion !== undefined && distortion.k1 !== 0;
    let bestErr = Infinity;
    let bestHt = Ht;

    for (let iter = 0; iter < this.maxIterations; iter++) {
      // Sample the frame at the warped template points.
      let meanF = 0;
      let count = 0;
      for (let i = 0; i < n; i++) {
        const u = this.px[i];
        const v = this.py[i];
        const dw = Ht[6] * u + Ht[7] * v + Ht[8];
        if (Math.abs(dw) < 1e-9) {
          validMask[i] = 0;
          continue;
        }
        let fx = (Ht[0] * u + Ht[1] * v + Ht[2]) / dw;
        let fy = (Ht[3] * u + Ht[4] * v + Ht[5]) / dw;
        if (useDist) {
          distortPoint(distortion!, fx, fy, dpt);
          fx = dpt.x;
          fy = dpt.y;
        }
        if (fx < 1 || fy < 1 || fx > frameW - 2 || fy > frameH - 2) {
          validMask[i] = 0;
          continue;
        }
        validMask[i] = 1;
        const val = sampleBilinear(frame, frameW, frameH, fx, fy);
        fVal[i] = val;
        meanF += val;
        count++;
      }
      if (count < n * 0.5) return null;
      meanF /= count;
      let varF = 0;
      for (let i = 0; i < n; i++) if (validMask[i]) varF += (fVal[i] - meanF) ** 2;
      const stdF = Math.sqrt(varF / count) || 1;
      const gain = this.stdT / stdF;

      // Normalized residuals + robust scale (MAD) for the Huber weights.
      let m = 0;
      for (let i = 0; i < n; i++) {
        if (!validMask[i]) continue;
        const r = (fVal[i] - meanF) * gain - (this.tVal[i] - this.meanT);
        resid[i] = r;
        absResid[m++] = Math.abs(r);
      }
      const scale = medianOf(absResid, m) * 1.4826;
      const delta = Math.max(4, 1.345 * scale); // intensity units; floor for clean scenes

      // Weighted Gauss-Newton: accumulate Hessian and rhs with Huber weights.
      const Hgn = new Float64Array(64);
      const rhs = new Float64Array(8);
      let errSum = 0;
      let wSum = 0;
      for (let i = 0; i < n; i++) {
        if (!validMask[i]) continue;
        const r = resid[i];
        const a = Math.abs(r);
        const wgt = a <= delta ? 1 : delta / a;
        errSum += wgt * a;
        wSum += wgt;
        const base = i * 8;
        for (let row = 0; row < 8; row++) {
          const jr = this.J[base + row] * wgt;
          if (jr === 0) continue;
          rhs[row] += jr * r;
          for (let col = row; col < 8; col++) Hgn[row * 8 + col] += jr * this.J[base + col];
        }
      }
      for (let row = 0; row < 8; row++) {
        for (let col = 0; col < row; col++) Hgn[row * 8 + col] = Hgn[col * 8 + row];
      }
      const err = errSum / Math.max(1e-9, wSum);
      if (err < bestErr) {
        const improvement = iter > 0 ? (bestErr - err) / bestErr : 1;
        bestErr = err;
        bestHt = Ht;
        if (improvement < this.minImprovement) break; // converged
      } else if (iter > 0) {
        break; // diverging: keep the best seen
      }

      // Inverse compositional: delta = Hgn^-1 * (J^T W r) with r = I - T,
      // then compose the *inverse* of the incremental warp: Ht <- Ht * dH^-1.
      const d = solveLinearSystem(Hgn, rhs, 8);
      if (!d) break;
      const dH: Mat3 = [1 + d[0], d[2], d[4], d[1], 1 + d[3], d[5], d[6], d[7], 1];
      const dHinv = invert3(dH);
      if (!dHinv) break;
      Ht = matMul3(Ht, dHinv);
      if (Math.abs(Ht[8]) > 1e-12) {
        const invW = 1 / Ht[8];
        for (let k = 0; k < 9; k++) Ht[k] *= invW;
      }

      // Convergence: a tiny parameter step means we're done.
      let stepMag = 0;
      for (let k = 0; k < 8; k++) stepMag += d[k] * d[k];
      if (stepMag < 1e-8) {
        bestHt = Ht;
        break;
      }
    }

    const refined = matMul3(bestHt, Sinv);
    if (Math.abs(refined[8]) < 1e-12) return null;
    const invW = 1 / refined[8];
    for (let k = 0; k < 9; k++) refined[k] *= invW;
    return { H: refined, err: bestErr };
  }
}

/** Median of the first `m` entries (partial sort on a copy). */
function medianOf(values: Float32Array, m: number): number {
  if (m === 0) return 0;
  const copy = Array.from(values.subarray(0, m));
  copy.sort((a, b) => a - b);
  return copy[m >> 1];
}
