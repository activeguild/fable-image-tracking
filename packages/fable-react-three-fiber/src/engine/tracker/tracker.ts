/**
 * The runtime tracker: a detect -> track state machine, like the classic
 * WebAR pipelines (8th Wall, Vuforia, MindAR).
 *
 * - SEARCHING: detect FAST/ORB features on the camera frame, match against
 *   the compiled target bank, estimate a homography with RANSAC.
 * - TRACKING: follow the inlier points frame-to-frame with pyramidal
 *   Lucas-Kanade (with a forward-backward consistency check) and re-estimate
 *   the homography every frame; replenish points from the target model when
 *   they decay. Much cheaper and much more stable than re-detecting.
 */

import { sampleBilinear, type PyramidLevel } from '../core/imageops';
import { selectSpread } from '../core/fast';
import { PATCH_BORDER } from '../core/orb';
import { jsKernels, type CVKernels } from '../core/kernels';
import { ransacHomography, projectCorners, type RansacResult } from '../core/ransac';
import {
  applyHomography,
  computeHomography,
  invert3,
  matMul3,
  symmetricTransferError2,
  type Mat3,
  type Point2,
} from '../core/homography';
import {
  orthogonalityDefect,
  poseFromHomography,
  refinePlanarPose,
  type CameraIntrinsics,
  type Pose,
} from '../core/pose';
import {
  DenseAligner,
  distortPoint,
  undistortPoint,
  type RadialDistortion,
} from '../core/densealign';
import type { CompiledTarget } from './target';

export type TrackerState = 'searching' | 'tracking';

export interface TrackerResult {
  state: TrackerState;
  /** Homography mapping compiled-target pixels to frame pixels (processing scale). */
  H: Mat3 | null;
  pose: Pose | null;
  inlierCount: number;
  /** Currently tracked points in frame coordinates, for debug overlays. */
  trackedPoints: Point2[];
  /** Projected target corners in frame coordinates when a pose is available. */
  corners: Point2[] | null;
}

export interface TrackerOptions {
  fastThreshold?: number;
  maxFrameFeatures?: number;
  framePyramidLevels?: number;
  minMatches?: number;
  minInliers?: number;
  /** Inlier threshold in (processing-scale) pixels. */
  ransacThreshold?: number;
  /** Replenish tracked points from the model when fewer than this survive. */
  replenishBelow?: number;
  maxTrackedPoints?: number;
  /** Run detection only every Nth frame while searching (saves CPU). */
  detectEveryN?: number;
  /** Minimum photometric correlation (NCC) to accept a detection. */
  minDetectNCC?: number;
  /** Minimum photometric correlation (NCC) to keep tracking. */
  minTrackNCC?: number;
  intrinsics?: CameraIntrinsics;
  /** Compute backend; defaults to the pure-TypeScript kernels. */
  kernels?: CVKernels;
}

export class ImageTracker {
  readonly width: number;
  readonly height: number;
  readonly intrinsics: CameraIntrinsics;

  private readonly target: CompiledTarget;
  private readonly kernels: CVKernels;
  private readonly opts: Required<Omit<TrackerOptions, 'intrinsics' | 'kernels'>>;

  private state: TrackerState = 'searching';
  private prevPyramid: PyramidLevel[] | null = null;
  private framePoints: Point2[] = []; // tracked points, frame coords
  private modelPoints: Point2[] = []; // matching points, compiled-target pixel coords
  private H: Mat3 | null = null;
  private prevH: Mat3 | null = null; // H of the frame before, for motion prediction
  private frameCounter = 0;
  private planeToFrame: Mat3 | null = null;
  private readonly initialFocal: number;
  private readonly aligner: DenseAligner;
  /** Sparse probe points (target px) for photometric validation. */
  private readonly probePoints: Point2[];
  /**
   * Radial lens distortion coefficient (Brown k1, normalized units),
   * self-calibrated photometrically. Geometry (H, pose, fits) lives in ideal
   * pixel coordinates; observations are undistorted on the way in and
   * projections distorted on the way out.
   */
  private k1 = 0;
  /** Gyro rotation homography of the previously processed interval. */
  private prevGyroH: Mat3 | null = null;
  // Calibration hysteresis: apply a step only after consecutive evaluations
  // agree on the direction, so measurement noise cannot random-walk f/k1
  // (they are weakly coupled and would otherwise co-drift).
  private focalStreak = 0;
  private k1Streak = 0;
  /** FAST threshold auto-tuned to scene contrast (drops in dim scenes). */
  private fastThreshold: number;

  constructor(target: CompiledTarget, frameWidth: number, frameHeight: number, options: TrackerOptions = {}) {
    this.target = target;
    this.width = frameWidth;
    this.height = frameHeight;
    this.kernels = options.kernels ?? jsKernels;
    this.opts = {
      fastThreshold: options.fastThreshold ?? 20,
      maxFrameFeatures: options.maxFrameFeatures ?? 500,
      framePyramidLevels: options.framePyramidLevels ?? 4,
      minMatches: options.minMatches ?? 12,
      minInliers: options.minInliers ?? 10,
      ransacThreshold: options.ransacThreshold ?? 3,
      replenishBelow: options.replenishBelow ?? 45,
      maxTrackedPoints: options.maxTrackedPoints ?? 80,
      detectEveryN: options.detectEveryN ?? 1,
      minDetectNCC: options.minDetectNCC ?? 0.55,
      minTrackNCC: options.minTrackNCC ?? 0.45,
    };
    this.intrinsics = options.intrinsics ?? defaultIntrinsics(frameWidth, frameHeight);
    this.initialFocal = this.intrinsics.fx;
    this.fastThreshold = this.opts.fastThreshold;
    this.aligner = new DenseAligner(target.gray, target.width, target.height);
    this.probePoints = [];
    const stride = Math.max(1, Math.floor(target.points.length / 2 / 128));
    for (let i = 0; i < target.points.length / 2; i += stride) {
      this.probePoints.push({ x: target.points[i * 2], y: target.points[i * 2 + 1] });
    }
  }

  /** Current distortion model (identity when k1 has not been calibrated). */
  get distortion(): RadialDistortion {
    return { k1: this.k1, cx: this.intrinsics.cx, cy: this.intrinsics.cy, f: this.intrinsics.fx };
  }

  /** Observed (distorted) frame point -> ideal point. */
  private undistort(p: Point2): Point2 {
    if (this.k1 === 0) return p;
    const out = { x: 0, y: 0 };
    undistortPoint(this.distortion, p.x, p.y, out);
    return out;
  }

  /** Ideal point -> observed (distorted) frame point. */
  private distort(p: Point2): Point2 {
    if (this.k1 === 0) return p;
    const out = { x: 0, y: 0 };
    distortPoint(this.distortion, p.x, p.y, out);
    return out;
  }

  /**
   * Process one grayscale frame at the tracker's processing resolution.
   * `gyroH` is an optional image-space rotation homography measured by the
   * gyroscope over the interval since the previous frame (ideal pixels).
   */
  processFrame(gray: Uint8Array, gyroH: Mat3 | null = null): TrackerResult {
    this.frameCounter++;
    const pyramid = this.kernels.buildPyramid(gray, this.width, this.height, this.opts.framePyramidLevels);

    if (this.state === 'tracking' && this.prevPyramid) {
      this.trackStep(pyramid, gyroH);
      // If tracking just broke (H cleared by lost()), try to re-acquire on
      // this very frame instead of leaving a search-latency gap.
      if (this.H === null) this.detectStep(pyramid);
    } else if (this.frameCounter % this.opts.detectEveryN === 0) {
      this.detectStep(pyramid);
    }

    this.prevGyroH = gyroH;
    this.prevPyramid = pyramid;
    return this.buildResult();
  }

  reset(): void {
    this.state = 'searching';
    this.H = null;
    this.prevH = null;
    this.framePoints = [];
    this.modelPoints = [];
    this.prevPyramid = null;
  }

  // ---------------------------------------------------------------- detect

  private detectStep(pyramid: PyramidLevel[]): void {
    const framePts: number[] = [];
    const descChunks: Uint32Array[] = [];

    for (const level of pyramid) {
      let kps = this.kernels.detectFast(level, this.fastThreshold, PATCH_BORDER);
      kps = selectSpread(
        kps,
        level.width,
        level.height,
        Math.ceil(this.opts.maxFrameFeatures / pyramid.length)
      );
      if (kps.length === 0) continue;
      descChunks.push(this.kernels.orientAndDescribe(level, kps));
      for (const kp of kps) framePts.push(kp.x * level.scale, kp.y * level.scale);
    }

    // Auto-tune the detector to scene contrast: dim scenes starve a fixed
    // threshold of corners (noisy 15-point fits), bright ones flood it.
    const total = framePts.length / 2;
    if (total < 150) this.fastThreshold = Math.max(8, this.fastThreshold - 2);
    else if (total > 450) this.fastThreshold = Math.min(30, this.fastThreshold + 2);

    if (total < this.opts.minMatches) return;

    const frameDescriptors = concatUint32(descChunks);
    const matches = this.kernels.matchDescriptors(this.target.descriptors, frameDescriptors, {
      maxDistance: 64,
      ratio: 0.85,
      crossCheck: true,
    });
    if (matches.length < this.opts.minMatches) return;

    const src: Point2[] = new Array(matches.length);
    const dstRaw: Point2[] = new Array(matches.length); // sensor (distorted) space, for LK
    const dst: Point2[] = new Array(matches.length); // ideal space, for fitting
    for (let i = 0; i < matches.length; i++) {
      const m = matches[i];
      src[i] = { x: this.target.points[m.a * 2], y: this.target.points[m.a * 2 + 1] };
      dstRaw[i] = { x: framePts[m.b * 2], y: framePts[m.b * 2 + 1] };
      dst[i] = this.undistort(dstRaw[i]);
    }

    const result = ransacHomography(src, dst, {
      threshold: this.opts.ransacThreshold,
      maxIterations: 400,
    });
    if (!result || result.inliers.length < this.opts.minInliers) return;
    if (!this.isPlausible(result.H)) return;

    // Photometric verification: geometry alone can be fooled by repetitive or
    // coincidental structure; the warped appearance must also agree.
    const inlierModel = result.inliers.map((i) => src[i]);
    const ncc = appearanceNCC(this.target, inlierModel, result.H, pyramid[0], 64, this.distortion);
    if (ncc < this.opts.minDetectNCC) return;

    // Subpixel dense refinement against the reference texture.
    const refined = this.aligner.align(result.H, pyramid[0].data, this.width, this.height, this.distortion);
    if (refined && this.isPlausible(refined.H)) result.H = refined.H;

    this.H = result.H;
    this.prevH = null; // fresh acquisition: no velocity estimate yet
    this.framePoints = result.inliers.map((i) => dstRaw[i]);
    this.modelPoints = result.inliers.map((i) => src[i]);
    this.capTrackedPoints();
    this.state = 'tracking';
  }

  // ----------------------------------------------------------------- track

  private trackStep(pyramid: PyramidLevel[], gyroH: Mat3 | null): void {
    // Motion prediction for the LK warm start and the fit prior. The visual
    // constant-velocity model is corrected with the gyroscope when available:
    // last interval's measured rotation is divided out of the visual motion
    // (leaving the translation-ish residual) and replaced by the *current*
    // interval's measured rotation. During fast rotation - exactly when the
    // image blurs and vision fails - the prior stays accurate.
    let predicted: Point2[] | undefined;
    let prior: Mat3 | null = this.H;
    let motion: Mat3 | null = null;
    if (this.H && this.prevH) {
      const prevInv = invert3(this.prevH);
      if (prevInv) {
        motion = matMul3(this.H, prevInv);
        if (gyroH && this.prevGyroH) {
          const gPrevInv = invert3(this.prevGyroH);
          if (gPrevInv) motion = matMul3(gyroH, matMul3(gPrevInv, motion));
        }
      }
    } else if (this.H && gyroH) {
      // First tracked frame after acquisition: no visual velocity yet, but
      // the gyro already knows the rotation.
      motion = gyroH;
    }
    if (this.H && motion) {
      const m = motion;
      // Motion lives in ideal space; LK guesses must be in sensor space.
      predicted = this.framePoints.map((p) => {
        const ideal = this.undistort(p);
        return this.distort(applyHomography(m, ideal.x, ideal.y));
      });
      prior = matMul3(m, this.H);
    }

    const flows = this.kernels.trackPyrLK(this.prevPyramid!, pyramid, this.framePoints, {
      windowRadius: 4,
      maxIterations: 12,
      maxError: 24,
      initialGuess: predicted,
    });

    // Forward-backward consistency check.
    const forwardPts: Point2[] = [];
    const forwardIdx: number[] = [];
    for (let i = 0; i < flows.length; i++) {
      if (flows[i].ok) {
        forwardPts.push({ x: flows[i].x, y: flows[i].y });
        forwardIdx.push(i);
      }
    }
    // The backward pass must undo the same (possibly large) motion, so give
    // it the original positions as its warm start.
    const backFlows = this.kernels.trackPyrLK(pyramid, this.prevPyramid!, forwardPts, {
      windowRadius: 4,
      maxIterations: 8,
      maxError: 32,
      initialGuess: forwardIdx.map((i) => this.framePoints[i]),
    });

    const nextFrame: Point2[] = [];
    const nextModel: Point2[] = [];
    for (let j = 0; j < forwardPts.length; j++) {
      const orig = this.framePoints[forwardIdx[j]];
      const back = backFlows[j];
      if (!back.ok) continue;
      const dx = back.x - orig.x;
      const dy = back.y - orig.y;
      if (dx * dx + dy * dy > 1.5 * 1.5) continue;
      nextFrame.push(forwardPts[j]);
      nextModel.push(this.modelPoints[forwardIdx[j]]);
    }

    if (nextFrame.length < this.opts.minInliers) {
      if (!this.denseRescue(prior, pyramid)) this.lost();
      return;
    }

    // Fit in ideal (undistorted) coordinates.
    const nextIdeal = nextFrame.map((p) => this.undistort(p));

    // Deterministic prior-guided fit first: RANSAC's random minimal samples
    // make the estimate wobble frame to frame even on a static scene, which
    // shows up as visible jitter. With last frame's H (advanced by the motion
    // model) as a prior, gating + iterated least squares is stable and cheap.
    // Full RANSAC remains as the fallback when the prior is way off.
    let result = prior ? this.fitGuided(nextModel, nextIdeal, prior) : null;
    if (!result) {
      result = ransacHomography(nextModel, nextIdeal, {
        threshold: this.opts.ransacThreshold,
        maxIterations: 150,
      });
    }
    if (!result || result.inliers.length < this.opts.minInliers || !this.isPlausible(result.H)) {
      if (!this.denseRescue(prior, pyramid)) this.lost();
      return;
    }

    // Photometric verification every frame: kills stale poses quickly when
    // the target disappears, is occluded, or motion blur wipes the texture,
    // instead of letting optical flow limp along on wrong content.
    // With few inliers the geometric evidence is weak, so demand stronger
    // photometric evidence before trusting the fit.
    const nccFloor =
      result.inliers.length < 20 ? Math.max(this.opts.minTrackNCC, 0.6) : this.opts.minTrackNCC;
    const ncc = appearanceNCC(
      this.target,
      result.inliers.map((i) => nextModel[i]),
      result.H,
      pyramid[0],
      64,
      this.distortion
    );
    if (ncc < nccFloor) {
      this.lost();
      return;
    }

    // Dense subpixel refinement: point-based estimates carry per-corner LK
    // noise; aligning the whole reference texture against the frame removes
    // the residual sub-pixel swimming.
    const refined = this.aligner.align(result.H, pyramid[0].data, this.width, this.height, this.distortion);
    if (refined && this.isPlausible(refined.H)) {
      result.H = refined.H;
      this.maybeCalibrateDistortion(refined.H, refined.err, pyramid);
    }

    // Temporal shape gate: no real hand motion changes the projected quad's
    // edge lengths by >15% in one frame against a static target. A fit that
    // does is a blur/occlusion artefact that slipped past the other gates.
    if (this.H && !this.shapeContinuous(this.H, result.H, 0.15)) {
      if (!this.denseRescue(prior, pyramid)) this.lost();
      return;
    }

    this.prevH = this.H;
    this.H = result.H;
    this.framePoints = result.inliers.map((i) => nextFrame[i]);
    this.modelPoints = result.inliers.map((i) => nextModel[i]);

    if (this.framePoints.length < this.opts.replenishBelow) this.replenish();
  }

  /**
   * Photometric self-calibration of radial distortion: try k1 one step up
   * and down; whichever makes the dense alignment residual smaller wins.
   * Only observable when the target reaches the frame periphery (the k1 r^2
   * term vanishes near the principal point), so gate on corner radius.
   */
  private maybeCalibrateDistortion(H: Mat3, currentErr: number, pyramid: PyramidLevel[]): void {
    // Offset cadence from the focal calibration (never both in one frame).
    if (this.frameCounter % 8 !== 4) return;
    const K = this.intrinsics;
    const corners = projectCorners(H, this.target.width, this.target.height);
    let rMax = 0;
    for (const c of corners) {
      rMax = Math.max(rMax, Math.hypot((c.x - K.cx) / K.fx, (c.y - K.cy) / K.fx));
    }
    if (rMax < 0.6) return; // k1 is unobservable away from the periphery

    const step = 0.02;
    let bestK = this.k1;
    let bestErr = currentErr;
    for (const k of [this.k1 + step, this.k1 - step]) {
      const d: RadialDistortion = { k1: k, cx: K.cx, cy: K.cy, f: K.fx };
      const r = this.aligner.align(H, pyramid[0].data, this.width, this.height, d);
      // Demand a clear (1.5%) improvement so noise cannot cast votes.
      if (r && r.err < bestErr * 0.985) {
        bestErr = r.err;
        bestK = k;
      }
    }
    if (bestK === this.k1) {
      this.k1Streak = 0;
      return;
    }
    const dir = bestK > this.k1 ? 1 : -1;
    this.k1Streak = Math.sign(this.k1Streak) === dir ? this.k1Streak + dir : dir;
    if (Math.abs(this.k1Streak) < 2) return;
    // Physically plausible range for phone cameras (ISPs pre-correct most of it).
    this.k1 = Math.min(0.08, Math.max(-0.15, this.k1 + dir * step * 0.5));
  }

  /**
   * Keep tracking through corner starvation with dense alignment alone.
   * When the target recedes, FAST corners die out long before the texture
   * does; aligning the reference against the frame from the motion prior
   * needs no corners at all, so the lock survives (validated by NCC every
   * frame, so this is genuine measurement, not blind coasting).
   */
  private denseRescue(prior: Mat3 | null, pyramid: PyramidLevel[]): boolean {
    if (!prior) return false;
    const refined = this.aligner.align(prior, pyramid[0].data, this.width, this.height, this.distortion);
    if (!refined || !this.isPlausible(refined.H)) return false;
    // The rescue must also respect temporal shape continuity.
    if (this.H && !this.shapeContinuous(this.H, refined.H, 0.2)) return false;
    const ncc = appearanceNCC(this.target, this.probePoints, refined.H, pyramid[0], 64, this.distortion);
    if (ncc < this.opts.minTrackNCC) return false;

    this.prevH = this.H;
    this.H = refined.H;
    // Re-seed the point set from the model so LK can resume next frame.
    this.framePoints = [];
    this.modelPoints = [];
    this.replenish();
    return true;
  }

  /**
   * Deterministic homography fit using a strong prior: gate correspondences
   * against the prior, least-squares fit, then tighten the gate and refit.
   * Returns null when the prior does not explain enough points (fast fallback
   * to RANSAC).
   */
  private fitGuided(model: Point2[], frame: Point2[], prior: Mat3): RansacResult | null {
    const thr = this.opts.ransacThreshold;
    let H = prior;
    let inliers: number[] = [];
    for (const gate of [thr * 2.5, thr]) {
      const Hinv = invert3(H);
      if (!Hinv) return null;
      const gate2 = gate * gate;
      inliers = [];
      for (let i = 0; i < model.length; i++) {
        const e = symmetricTransferError2(H, Hinv, model[i].x, model[i].y, frame[i].x, frame[i].y);
        if (e < gate2) inliers.push(i);
      }
      if (inliers.length < Math.max(this.opts.minInliers, model.length * 0.5)) return null;
      const fitted = computeHomography(
        inliers.map((i) => model[i]),
        inliers.map((i) => frame[i])
      );
      if (!fitted) return null;
      H = fitted;
    }
    return { H, inliers };
  }

  /**
   * Re-seed tracked points by projecting the target's keypoints through the
   * current homography. Bad seeds are pruned by the forward-backward check
   * and RANSAC within a frame or two.
   */
  private replenish(): void {
    if (!this.H) return;
    const existing = new Set(this.modelPoints.map((p) => `${p.x.toFixed(1)},${p.y.toFixed(1)}`));
    const margin = 8;
    const pts = this.target.points;
    // Re-seed gradually (<= 20 points per frame): a wholesale reset of the
    // point set applies any accumulated bias as one visible correction jump.
    const cap = Math.min(this.opts.maxTrackedPoints, this.framePoints.length + 20);
    for (let i = 0; i < pts.length / 2 && this.framePoints.length < cap; i++) {
      const mx = pts[i * 2];
      const my = pts[i * 2 + 1];
      const key = `${mx.toFixed(1)},${my.toFixed(1)}`;
      if (existing.has(key)) continue;
      // Project through H (ideal space), then into sensor space for LK.
      const p = this.distort(applyHomography(this.H, mx, my));
      if (p.x < margin || p.y < margin || p.x >= this.width - margin || p.y >= this.height - margin) {
        continue;
      }
      existing.add(key);
      this.framePoints.push(p);
      this.modelPoints.push({ x: mx, y: my });
    }
  }

  private capTrackedPoints(): void {
    const max = this.opts.maxTrackedPoints;
    if (this.framePoints.length <= max) return;
    // Uniform subsample keeps the spatial spread of the inlier set.
    const step = this.framePoints.length / max;
    const fp: Point2[] = [];
    const mp: Point2[] = [];
    for (let i = 0; i < max; i++) {
      const idx = Math.floor(i * step);
      fp.push(this.framePoints[idx]);
      mp.push(this.modelPoints[idx]);
    }
    this.framePoints = fp;
    this.modelPoints = mp;
  }

  private lost(): void {
    this.state = 'searching';
    this.H = null;
    this.prevH = null;
    this.framePoints = [];
    this.modelPoints = [];
  }

  // ------------------------------------------------------------------ misc

  /** Edge/diagonal lengths of the projected quad must move smoothly. */
  private shapeContinuous(prevH: Mat3, newH: Mat3, tol: number): boolean {
    const a = projectCorners(prevH, this.target.width, this.target.height);
    const b = projectCorners(newH, this.target.width, this.target.height);
    const pairs = [
      [0, 1], [1, 2], [2, 3], [3, 0], [0, 2], [1, 3],
    ];
    for (const [i, j] of pairs) {
      const la = Math.hypot(a[j].x - a[i].x, a[j].y - a[i].y);
      if (la < 1e-6) return false;
      const lb = Math.hypot(b[j].x - b[i].x, b[j].y - b[i].y);
      const ratio = lb / la;
      if (ratio < 1 - tol || ratio > 1 + tol) return false;
    }
    return true;
  }

  /** Sanity checks on H: projected target corners must form a convex, sane quad. */
  private isPlausible(H: Mat3): boolean {
    const corners = projectCorners(H, this.target.width, this.target.height);
    // Convexity + consistent winding via cross products.
    let sign = 0;
    for (let i = 0; i < 4; i++) {
      const a = corners[i];
      const b = corners[(i + 1) % 4];
      const c = corners[(i + 2) % 4];
      const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
      const s = Math.sign(cross);
      if (s === 0) return false;
      if (sign === 0) sign = s;
      else if (s !== sign) return false;
    }
    // Area must be non-trivial and not absurdly large.
    let area = 0;
    for (let i = 0; i < 4; i++) {
      const a = corners[i];
      const b = corners[(i + 1) % 4];
      area += a.x * b.y - b.x * a.y;
    }
    area = Math.abs(area) / 2;
    const frameArea = this.width * this.height;
    if (area < frameArea * 0.002 || area > frameArea * 12) return false;
    // Edge length ratio guard against extreme skew.
    const len = (p: Point2, q: Point2) => Math.hypot(q.x - p.x, q.y - p.y);
    const edges = [0, 1, 2, 3].map((i) => len(corners[i], corners[(i + 1) % 4]));
    const maxE = Math.max(...edges);
    const minE = Math.min(...edges);
    if (minE < 1e-3 || maxE / minE > 12) return false;
    // Opposite edges of a projected rectangle stay comparable under any
    // realistic viewing angle; kite/spike quads (blurred-fit artefacts) don't.
    const opp = (a: number, b: number) => Math.max(edges[a], edges[b]) / Math.max(1e-3, Math.min(edges[a], edges[b]));
    if (opp(0, 2) > 3 || opp(1, 3) > 3) return false;
    return true;
  }

  /**
   * Online focal self-calibration: hill-climb the focal length toward the
   * value that makes K^-1 H closest to a valid rotation. Fronto-parallel
   * views carry no information (the defect is flat in f), so only step when
   * the candidates actually separate; the slow EMA keeps it stable.
   */
  private calibrateFocal(planeToFrame: Mat3): void {
    // Decoupled cadence from the k1 calibration (never both in one frame).
    if (this.frameCounter % 8 !== 0) return;
    const K = this.intrinsics;
    const f = K.fx;
    const candidates = [f, f * 1.02, f / 1.02];
    const defects = candidates.map((fc) =>
      orthogonalityDefect(planeToFrame, { fx: fc, fy: fc, cx: K.cx, cy: K.cy })
    );
    const spread = Math.max(...defects) - Math.min(...defects);
    if (!isFinite(spread) || spread < 3e-4) {
      this.focalStreak = 0;
      return;
    }
    let best = 0;
    if (defects[1] < defects[best]) best = 1;
    if (defects[2] < defects[best]) best = 2;
    if (best === 0) {
      this.focalStreak = 0;
      return;
    }
    const dir = best === 1 ? 1 : -1;
    this.focalStreak = Math.sign(this.focalStreak) === dir ? this.focalStreak + dir : dir;
    if (Math.abs(this.focalStreak) < 2) return;

    const target = candidates[best];
    const fNew = Math.min(
      this.initialFocal * 1.6,
      Math.max(this.initialFocal * 0.6, 0.85 * f + 0.15 * target)
    );
    K.fx = fNew;
    K.fy = fNew;
  }

  private buildResult(): TrackerResult {
    if (!this.H) {
      this.planeToFrame = null;
      return {
        state: this.state,
        H: null,
        pose: null,
        inlierCount: 0,
        trackedPoints: [],
        corners: null,
      };
    }
    this.planeToFrame = matMul3(this.H, this.target.pixelFromPlane);
    if (this.state === 'tracking') this.calibrateFocal(this.planeToFrame);
    // Refine the decomposed pose against the measured homography: the raw
    // decomposition's orthonormalization moves the plane's reprojection by
    // several pixels under noise, which pose-anchored content shows as a
    // constant offset from the marker.
    let pose = poseFromHomography(this.planeToFrame, this.intrinsics);
    if (pose) {
      pose = refinePlanarPose(
        pose,
        this.intrinsics,
        this.planeToFrame,
        this.target.widthMeters,
        this.target.heightMeters
      );
    }
    return {
      state: this.state,
      H: this.H,
      pose,
      inlierCount: this.framePoints.length,
      trackedPoints: this.framePoints.slice(),
      // Corners go to the renderer, which composites over the real (distorted)
      // camera image - so project in ideal space, then distort.
      corners: projectCorners(this.H, this.target.width, this.target.height).map((c) =>
        this.distort(c)
      ),
    };
  }
}

/**
 * Default pinhole intrinsics when the camera is uncalibrated: assume a
 * horizontal field of view of ~64 degrees, principal point at the centre.
 */
export function defaultIntrinsics(width: number, height: number): CameraIntrinsics {
  const f = 0.8 * Math.max(width, height);
  return { fx: f, fy: f, cx: width / 2, cy: height / 2 };
}

/**
 * Photometric consistency of a hypothesised homography: normalized cross
 * correlation between target intensities at the given model points and frame
 * intensities at their warped locations. NCC is invariant to global gain and
 * offset, so lighting differences between the print/screen and the reference
 * image are tolerated. Returns a value in [-1, 1]; higher is better.
 */
export function appearanceNCC(
  target: Pick<CompiledTarget, 'gray' | 'width' | 'height'>,
  modelPoints: Point2[],
  H: Mat3,
  frame: { data: Uint8Array; width: number; height: number },
  maxSamples = 64,
  distortion?: RadialDistortion
): number {
  const n = modelPoints.length;
  if (n === 0) return -1;
  const stride = Math.max(1, Math.floor(n / maxSamples));
  const useDist = distortion !== undefined && distortion.k1 !== 0;
  const dpt = { x: 0, y: 0 };

  const tVals: number[] = [];
  const fVals: number[] = [];
  for (let i = 0; i < n; i += stride) {
    const mp = modelPoints[i];
    if (mp.x < 0 || mp.y < 0 || mp.x > target.width - 1 || mp.y > target.height - 1) continue;
    let p = applyHomography(H, mp.x, mp.y);
    if (useDist) {
      distortPoint(distortion!, p.x, p.y, dpt);
      p = dpt;
    }
    if (p.x < 1 || p.y < 1 || p.x > frame.width - 2 || p.y > frame.height - 2) continue;
    tVals.push(sampleBilinear(target.gray, target.width, target.height, mp.x, mp.y));
    fVals.push(sampleBilinear(frame.data, frame.width, frame.height, p.x, p.y));
  }
  const m = tVals.length;
  if (m < 8) return -1;

  let meanT = 0;
  let meanF = 0;
  for (let i = 0; i < m; i++) {
    meanT += tVals[i];
    meanF += fVals[i];
  }
  meanT /= m;
  meanF /= m;
  let covTF = 0;
  let varT = 0;
  let varF = 0;
  for (let i = 0; i < m; i++) {
    const dt = tVals[i] - meanT;
    const df = fVals[i] - meanF;
    covTF += dt * df;
    varT += dt * dt;
    varF += df * df;
  }
  if (varT < 1e-6 || varF < 1e-6) return -1;
  return covTF / Math.sqrt(varT * varF);
}

function concatUint32(chunks: Uint32Array[]): Uint32Array {
  let total = 0;
  for (const c of chunks) total += c.length;
  const out = new Uint32Array(total);
  let off = 0;
  for (const c of chunks) {
    out.set(c, off);
    off += c.length;
  }
  return out;
}
