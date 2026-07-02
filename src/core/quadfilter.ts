/**
 * Corner-quad prediction and smoothing for planar content.
 *
 * Flat content that lies exactly on the target plane does not need the 3-D
 * pose (whose accuracy depends on the assumed camera intrinsics): the tracked
 * homography maps it onto the image exactly. This filter takes the projected
 * target corners from the tracker, extrapolates them to the render timestamp
 * (constant velocity) and smooths each coordinate with a One-Euro filter, so
 * the main thread can pin content per display frame.
 */

import { OneEuroFilter } from './filter';
import type { Point2 } from './homography';

interface Sample {
  t: number;
  c: number[]; // 8 values: x0,y0,...,x3,y3
}

/**
 * External motion model: advance a corner quad from one timestamp to another
 * (e.g. by the gyroscope-measured camera rotation). Used instead of the
 * constant-velocity assumption for the motion it can explain, which removes
 * extrapolation overshoot at abrupt direction changes.
 */
export type QuadAdvance = (corners: Point2[], fromSec: number, toSec: number) => Point2[];

export interface QuadFilterOptions {
  maxAge?: number; // seconds; samples older than this are stale
  maxHorizon?: number; // never extrapolate further than this
  minCutoff?: number;
  beta?: number;
}

export class QuadFilter {
  private s0: Sample | null = null;
  private s1: Sample | null = null;
  private readonly filters: OneEuroFilter[];
  private readonly maxAge: number;
  private readonly maxHorizon: number;

  constructor(options: QuadFilterOptions = {}) {
    this.maxAge = options.maxAge ?? 0.3;
    this.maxHorizon = options.maxHorizon ?? 0.1;
    const minCutoff = options.minCutoff ?? 1.2;
    const beta = options.beta ?? 0.08;
    this.filters = Array.from({ length: 8 }, () => new OneEuroFilter(minCutoff, beta, 1.0));
  }

  private rejections = 0;

  addSample(corners: Point2[], timeSec: number): void {
    const c = new Array<number>(8);
    for (let i = 0; i < 4; i++) {
      c[i * 2] = corners[i].x;
      c[i * 2 + 1] = corners[i].y;
    }
    // Admission control: a quad whose shape differs wildly from a sample
    // taken a few frames ago is a measurement glitch (no real motion changes
    // shape that fast). Skip it (coast) - unless it persists, then accept it
    // as a genuine change so we can never lock out real measurements.
    const s1 = this.s1;
    if (s1 && timeSec - s1.t < 0.15 && !shapeConsistent(c, s1.c, 0.25)) {
      this.rejections++;
      if (this.rejections <= 2) return;
    }
    this.rejections = 0;
    this.s0 = this.s1;
    this.s1 = { t: timeSec, c };
  }

  clear(): void {
    this.s0 = null;
    this.s1 = null;
    for (const f of this.filters) f.reset();
  }

  /**
   * Predicted, smoothed quad at `timeSec`, or null when stale/empty.
   * With `advance` (e.g. gyro rotation), the measured motion model handles
   * rotation and only the residual (translation-ish) part is extrapolated
   * at constant velocity.
   *
   * Rigidity guard: extrapolation must never deform the quad. Per-corner
   * prediction (velocities, filters) can desynchronise the corners under
   * violent motion with sparse samples, which shows up as grotesque shearing.
   * Any candidate whose edge/diagonal lengths deviate too much from the last
   * measurement falls back to progressively safer predictions - slight lag is
   * acceptable, a non-rigid quad never is.
   */
  predict(timeSec: number, advance?: QuadAdvance): Point2[] | null {
    const s1 = this.s1;
    if (!s1) return null;
    const age = timeSec - s1.t;
    if (age > this.maxAge) return null;

    const s0 = this.s0;
    const dt = s0 ? s1.t - s0.t : 0;
    const diag = Math.hypot(s1.c[4] - s1.c[0], s1.c[5] - s1.c[1]) || 1;
    const advHorizon = Math.min(age, 0.15); // never advance the model further

    let c = s1.c;
    if (advance && age > 0) {
      const base = flatten(advance(unflatten(s1.c), s1.t, s1.t + advHorizon));
      if (s0 && dt > 1e-4 && dt <= 0.08) {
        // Residual velocity: what the last sample moved beyond the model.
        // Capped per corner - a bad residual must not fling corners around.
        const explained = flatten(advance(unflatten(s0.c), s0.t, s1.t));
        const k = Math.min(age, this.maxHorizon) / dt;
        const cap = 0.2 * diag;
        c = base.map((v, i) => {
          const step = (s1.c[i] - explained[i]) * k;
          return v + Math.max(-cap, Math.min(cap, step));
        });
      } else {
        c = base;
      }
      if (!shapeConsistent(c, s1.c, 0.12)) {
        c = base; // drop the residual term
        if (!shapeConsistent(c, s1.c, 0.12)) {
          c = flatten(advance(unflatten(s1.c), s1.t, s1.t + Math.min(age, 0.05)));
          if (!shapeConsistent(c, s1.c, 0.12)) c = s1.c;
        }
      }
    } else if (s0 && dt > 1e-4 && dt <= 0.08 && age > 0) {
      const k = Math.min(age, this.maxHorizon) / dt;
      const cap = 0.2 * diag;
      c = s1.c.map((v, i) => {
        const step = (v - s0.c[i]) * k;
        return v + Math.max(-cap, Math.min(cap, step));
      });
      if (!shapeConsistent(c, s1.c, 0.12)) c = s1.c;
    }

    const out: Point2[] = new Array(4);
    for (let i = 0; i < 4; i++) {
      out[i] = {
        x: this.filters[i * 2].filter(c[i * 2], timeSec),
        y: this.filters[i * 2 + 1].filter(c[i * 2 + 1], timeSec),
      };
    }
    // The One-Euro filters are also per-coordinate; make sure their combined
    // output still forms an (almost) rigid quad, else bypass to the raw
    // (guarded) prediction for this frame.
    if (!shapeConsistent(flatten(out), s1.c, 0.18)) {
      for (let i = 0; i < 4; i++) {
        out[i] = { x: c[i * 2], y: c[i * 2 + 1] };
      }
      for (const f of this.filters) f.reset();
    }
    return out;
  }
}

/** Edge and diagonal lengths must stay within `tol` of the reference quad. */
function shapeConsistent(cand: number[], ref: number[], tol: number): boolean {
  // 4 edges + 2 diagonals, as index pairs into the corner list.
  const pairs = [
    [0, 1], [1, 2], [2, 3], [3, 0], [0, 2], [1, 3],
  ];
  for (const [a, b] of pairs) {
    const lr = Math.hypot(ref[b * 2] - ref[a * 2], ref[b * 2 + 1] - ref[a * 2 + 1]);
    if (lr < 1e-6) return false;
    const lc = Math.hypot(cand[b * 2] - cand[a * 2], cand[b * 2 + 1] - cand[a * 2 + 1]);
    const ratio = lc / lr;
    if (ratio < 1 - tol || ratio > 1 + tol) return false;
  }
  return true;
}

function unflatten(c: number[]): Point2[] {
  return [
    { x: c[0], y: c[1] },
    { x: c[2], y: c[3] },
    { x: c[4], y: c[5] },
    { x: c[6], y: c[7] },
  ];
}

function flatten(pts: Point2[]): number[] {
  const c = new Array<number>(8);
  for (let i = 0; i < 4; i++) {
    c[i * 2] = pts[i].x;
    c[i * 2 + 1] = pts[i].y;
  }
  return c;
}
