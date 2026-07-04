/**
 * One-Euro filter (Casiez et al. 2012) for jitter-free, low-latency smoothing
 * of noisy tracking signals.
 */

class LowPassFilter {
  private y = 0;
  private initialized = false;

  filter(value: number, alpha: number): number {
    if (!this.initialized) {
      this.initialized = true;
      this.y = value;
      return value;
    }
    this.y = alpha * value + (1 - alpha) * this.y;
    return this.y;
  }

  last(): number {
    return this.y;
  }

  reset(): void {
    this.initialized = false;
  }
}

export class OneEuroFilter {
  private x = new LowPassFilter();
  private dx = new LowPassFilter();
  private lastTime: number | null = null;

  constructor(
    private minCutoff = 1.0,
    private beta = 0.01,
    private dCutoff = 1.0
  ) {}

  private static alpha(cutoff: number, dt: number): number {
    const tau = 1 / (2 * Math.PI * cutoff);
    return 1 / (1 + tau / dt);
  }

  /** `timeSec` must be monotonically increasing. */
  filter(value: number, timeSec: number): number {
    if (this.lastTime === null) {
      this.lastTime = timeSec;
      this.dx.filter(0, 1);
      return this.x.filter(value, 1);
    }
    const dt = Math.max(1e-4, timeSec - this.lastTime);
    this.lastTime = timeSec;
    const dValue = (value - this.x.last()) / dt;
    const edValue = this.dx.filter(dValue, OneEuroFilter.alpha(this.dCutoff, dt));
    const cutoff = this.minCutoff + this.beta * Math.abs(edValue);
    return this.x.filter(value, OneEuroFilter.alpha(cutoff, dt));
  }

  reset(): void {
    this.x.reset();
    this.dx.reset();
    this.lastTime = null;
  }
}

/** Independent one-euro filtering of a 3-vector (e.g. a position). */
export class Vector3Filter {
  private fx: OneEuroFilter;
  private fy: OneEuroFilter;
  private fz: OneEuroFilter;

  constructor(minCutoff = 1.0, beta = 0.3, dCutoff = 1.0) {
    this.fx = new OneEuroFilter(minCutoff, beta, dCutoff);
    this.fy = new OneEuroFilter(minCutoff, beta, dCutoff);
    this.fz = new OneEuroFilter(minCutoff, beta, dCutoff);
  }

  filter(v: [number, number, number], timeSec: number): [number, number, number] {
    return [this.fx.filter(v[0], timeSec), this.fy.filter(v[1], timeSec), this.fz.filter(v[2], timeSec)];
  }

  reset(): void {
    this.fx.reset();
    this.fy.reset();
    this.fz.reset();
  }
}
