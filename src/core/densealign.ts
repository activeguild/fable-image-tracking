/**
 * Dense homography refinement by inverse-compositional Gauss-Newton image
 * alignment (Baker & Matthews / ESM style), the "last mile" used by
 * commercial trackers: after the point-based homography estimate, align the
 * warped reference target directly against the camera frame to squeeze the
 * estimate down to subpixel accuracy. Point-based estimates carry the noise
 * of individual corners; dense alignment averages over the whole texture.
 *
 * Inverse compositional: Jacobians and the Gauss-Newton Hessian live on the
 * fixed template, so everything expensive is precomputed once at target
 * compile time. Each frame iteration only samples the camera image and
 * accumulates an 8-vector. Gain/bias lighting changes are handled by
 * normalising both sides to the template's statistics.
 */

import { invert3, matMul3, solveLinearSystem, type Mat3 } from './homography';
import { resizeBilinear, sampleBilinear } from './imageops';

export interface DenseAlignOptions {
  /** Template width in pixels (the reference is downscaled to this). */
  templateWidth?: number;
  /** Maximum number of high-gradient sample points. */
  maxPoints?: number;
  maxIterations?: number;
  /** Stop when the mean absolute residual improves less than this factor. */
  minImprovement?: number;
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
  private readonly hessInv: Float64Array; // 8x8
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

    // Gauss-Newton Hessian (J^T J) and its inverse, both fixed for the template.
    const H = new Float64Array(64);
    for (let i = 0; i < n; i++) {
      const base = i * 8;
      for (let r = 0; r < 8; r++) {
        const jr = this.J[base + r];
        if (jr === 0) continue;
        for (let c = r; c < 8; c++) H[r * 8 + c] += jr * this.J[base + c];
      }
    }
    for (let r = 0; r < 8; r++) for (let c = 0; c < r; c++) H[r * 8 + c] = H[c * 8 + r];
    const inv = new Float64Array(64);
    let ok = this.valid;
    for (let col = 0; col < 8 && ok; col++) {
      const e = new Float64Array(8);
      e[col] = 1;
      const x = solveLinearSystem(H, e, 8);
      if (!x) {
        ok = false;
        break;
      }
      for (let r = 0; r < 8; r++) inv[r * 8 + col] = x[r];
    }
    this.hessInv = inv;
    this.valid = ok;
  }

  /**
   * Refine `H` (reference-target px -> frame px) against the frame. Returns
   * the refined homography, or null when alignment is unreliable.
   */
  align(H: Mat3, frame: Uint8Array, frameW: number, frameH: number): Mat3 | null {
    if (!this.valid) return null;
    const s = this.tmplToTarget;
    const S: Mat3 = [s, 0, 0, 0, s, 0, 0, 0, 1];
    const Sinv: Mat3 = [1 / s, 0, 0, 0, 1 / s, 0, 0, 0, 1];
    let Ht = matMul3(H, S); // template px -> frame px

    const n = this.px.length;
    const fVal = new Float32Array(n);
    const validMask = new Uint8Array(n);
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
        const fx = (Ht[0] * u + Ht[1] * v + Ht[2]) / dw;
        const fy = (Ht[3] * u + Ht[4] * v + Ht[5]) / dw;
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

      // Normalized residuals and the Gauss-Newton right-hand side.
      const rhs = new Float64Array(8);
      let errSum = 0;
      for (let i = 0; i < n; i++) {
        if (!validMask[i]) continue;
        const r = (fVal[i] - meanF) * gain - (this.tVal[i] - this.meanT);
        errSum += Math.abs(r);
        const base = i * 8;
        for (let k = 0; k < 8; k++) rhs[k] += this.J[base + k] * r;
      }
      const err = errSum / count;
      if (err < bestErr) {
        const improvement = iter > 0 ? (bestErr - err) / bestErr : 1;
        bestErr = err;
        bestHt = Ht;
        if (improvement < this.minImprovement) break; // converged
      } else if (iter > 0) {
        break; // diverging: keep the best seen
      }

      // Inverse compositional: delta = Hinv * (J^T r) with r = I - T, then
      // compose the *inverse* of the incremental warp: Ht <- Ht * dH^-1.
      const d = new Float64Array(8);
      for (let r = 0; r < 8; r++) {
        let acc = 0;
        for (let c = 0; c < 8; c++) acc += this.hessInv[r * 8 + c] * rhs[c];
        d[r] = acc;
      }
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
    return refined;
  }
}
