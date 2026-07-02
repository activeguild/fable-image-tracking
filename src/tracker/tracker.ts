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
import { poseFromHomography, type CameraIntrinsics, type Pose } from '../core/pose';
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
      replenishBelow: options.replenishBelow ?? 30,
      maxTrackedPoints: options.maxTrackedPoints ?? 80,
      detectEveryN: options.detectEveryN ?? 1,
      minDetectNCC: options.minDetectNCC ?? 0.55,
      minTrackNCC: options.minTrackNCC ?? 0.45,
    };
    this.intrinsics = options.intrinsics ?? defaultIntrinsics(frameWidth, frameHeight);
  }

  /** Process one grayscale frame at the tracker's processing resolution. */
  processFrame(gray: Uint8Array): TrackerResult {
    this.frameCounter++;
    const pyramid = this.kernels.buildPyramid(gray, this.width, this.height, this.opts.framePyramidLevels);

    if (this.state === 'tracking' && this.prevPyramid) {
      this.trackStep(pyramid);
      // If tracking just broke (H cleared by lost()), try to re-acquire on
      // this very frame instead of leaving a search-latency gap.
      if (this.H === null) this.detectStep(pyramid);
    } else if (this.frameCounter % this.opts.detectEveryN === 0) {
      this.detectStep(pyramid);
    }

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
      let kps = this.kernels.detectFast(level, this.opts.fastThreshold, PATCH_BORDER);
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

    const total = framePts.length / 2;
    if (total < this.opts.minMatches) return;

    const frameDescriptors = concatUint32(descChunks);
    const matches = this.kernels.matchDescriptors(this.target.descriptors, frameDescriptors, {
      maxDistance: 64,
      ratio: 0.85,
      crossCheck: true,
    });
    if (matches.length < this.opts.minMatches) return;

    const src: Point2[] = new Array(matches.length);
    const dst: Point2[] = new Array(matches.length);
    for (let i = 0; i < matches.length; i++) {
      const m = matches[i];
      src[i] = { x: this.target.points[m.a * 2], y: this.target.points[m.a * 2 + 1] };
      dst[i] = { x: framePts[m.b * 2], y: framePts[m.b * 2 + 1] };
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
    const ncc = appearanceNCC(this.target, inlierModel, result.H, pyramid[0]);
    if (ncc < this.opts.minDetectNCC) return;

    this.H = result.H;
    this.prevH = null; // fresh acquisition: no velocity estimate yet
    this.framePoints = result.inliers.map((i) => dst[i]);
    this.modelPoints = result.inliers.map((i) => src[i]);
    this.capTrackedPoints();
    this.state = 'tracking';
  }

  // ----------------------------------------------------------------- track

  private trackStep(pyramid: PyramidLevel[]): void {
    // Constant-velocity prediction: assume the frame-to-frame homography
    // motion repeats, and warm-start optical flow there. This keeps points
    // locked on during fast pans that exceed the plain LK search range.
    let predicted: Point2[] | undefined;
    let prior: Mat3 | null = this.H;
    if (this.H && this.prevH) {
      const prevInv = invert3(this.prevH);
      if (prevInv) {
        const motion = matMul3(this.H, prevInv);
        predicted = this.framePoints.map((p) => applyHomography(motion, p.x, p.y));
        prior = matMul3(motion, this.H);
      }
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
      this.lost();
      return;
    }

    // Deterministic prior-guided fit first: RANSAC's random minimal samples
    // make the estimate wobble frame to frame even on a static scene, which
    // shows up as visible jitter. With last frame's H (advanced by the motion
    // model) as a prior, gating + iterated least squares is stable and cheap.
    // Full RANSAC remains as the fallback when the prior is way off.
    let result = prior ? this.fitGuided(nextModel, nextFrame, prior) : null;
    if (!result) {
      result = ransacHomography(nextModel, nextFrame, {
        threshold: this.opts.ransacThreshold,
        maxIterations: 150,
      });
    }
    if (!result || result.inliers.length < this.opts.minInliers || !this.isPlausible(result.H)) {
      this.lost();
      return;
    }

    // Photometric verification every frame: kills stale poses quickly when
    // the target disappears, is occluded, or motion blur wipes the texture,
    // instead of letting optical flow limp along on wrong content.
    const ncc = appearanceNCC(this.target, result.inliers.map((i) => nextModel[i]), result.H, pyramid[0]);
    if (ncc < this.opts.minTrackNCC) {
      this.lost();
      return;
    }

    this.prevH = this.H;
    this.H = result.H;
    this.framePoints = result.inliers.map((i) => nextFrame[i]);
    this.modelPoints = result.inliers.map((i) => nextModel[i]);

    if (this.framePoints.length < this.opts.replenishBelow) this.replenish();
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
    for (let i = 0; i < pts.length / 2 && this.framePoints.length < this.opts.maxTrackedPoints; i++) {
      const mx = pts[i * 2];
      const my = pts[i * 2 + 1];
      const key = `${mx.toFixed(1)},${my.toFixed(1)}`;
      if (existing.has(key)) continue;
      const p = applyHomography(this.H, mx, my);
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
    return true;
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
    const pose = poseFromHomography(this.planeToFrame, this.intrinsics);
    return {
      state: this.state,
      H: this.H,
      pose,
      inlierCount: this.framePoints.length,
      trackedPoints: this.framePoints.slice(),
      corners: projectCorners(this.H, this.target.width, this.target.height),
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
  maxSamples = 64
): number {
  const n = modelPoints.length;
  if (n === 0) return -1;
  const stride = Math.max(1, Math.floor(n / maxSamples));

  const tVals: number[] = [];
  const fVals: number[] = [];
  for (let i = 0; i < n; i += stride) {
    const mp = modelPoints[i];
    if (mp.x < 0 || mp.y < 0 || mp.x > target.width - 1 || mp.y > target.height - 1) continue;
    const p = applyHomography(H, mp.x, mp.y);
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
