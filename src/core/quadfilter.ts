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

  addSample(corners: Point2[], timeSec: number): void {
    const c = new Array<number>(8);
    for (let i = 0; i < 4; i++) {
      c[i * 2] = corners[i].x;
      c[i * 2 + 1] = corners[i].y;
    }
    this.s0 = this.s1;
    this.s1 = { t: timeSec, c };
  }

  clear(): void {
    this.s0 = null;
    this.s1 = null;
    for (const f of this.filters) f.reset();
  }

  /** Predicted, smoothed quad at `timeSec`, or null when stale/empty. */
  predict(timeSec: number): Point2[] | null {
    const s1 = this.s1;
    if (!s1) return null;
    const age = timeSec - s1.t;
    if (age > this.maxAge) return null;

    const s0 = this.s0;
    const dt = s0 ? s1.t - s0.t : 0;
    let c = s1.c;
    if (s0 && dt > 1e-4 && dt <= 0.15 && age > 0) {
      const k = Math.min(age, this.maxHorizon) / dt;
      c = s1.c.map((v, i) => v + (v - s0.c[i]) * k);
    }

    const out: Point2[] = new Array(4);
    for (let i = 0; i < 4; i++) {
      out[i] = {
        x: this.filters[i * 2].filter(c[i * 2], timeSec),
        y: this.filters[i * 2 + 1].filter(c[i * 2 + 1], timeSec),
      };
    }
    return out;
  }
}
