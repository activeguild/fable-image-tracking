/**
 * Target compilation: turn a reference image into a multi-scale bank of
 * oriented FAST keypoints + ORB descriptors, expressed in level-0 pixel
 * coordinates of the compiled image. This is the offline half of natural
 * feature tracking (what 8th Wall / Vuforia call "image target compilation").
 */

import { selectSpread, type Keypoint } from '../core/fast';
import { DESCRIPTOR_WORDS, PATCH_BORDER } from '../core/orb';
import { jsKernels, type CVKernels } from '../core/kernels';
import type { Mat3 } from '../core/homography';

export interface CompiledTarget {
  /** Compiled (processing) resolution of the reference image. */
  width: number;
  height: number;
  /** Level-0 grayscale of the compiled image, kept for photometric checks. */
  gray: Uint8Array;
  /** Physical size the target is assumed to have in the world. */
  widthMeters: number;
  heightMeters: number;
  /** Keypoint positions in compiled-image pixels (level 0), 2 floats each. */
  points: Float32Array;
  /** ORB descriptors, DESCRIPTOR_WORDS words per keypoint. */
  descriptors: Uint32Array;
  /** Maps target-plane metres (x right, y up, origin at centre) to compiled-image pixels. */
  pixelFromPlane: Mat3;
}

export interface CompileOptions {
  widthMeters?: number;
  pyramidLevels?: number;
  fastThreshold?: number;
  maxFeaturesPerLevel?: number;
  /** Compute backend; must match the one used at runtime for best matching. */
  kernels?: CVKernels;
}

/**
 * Compile a grayscale reference image into a trackable target.
 * `gray` should already be at a sensible processing resolution (~300-500 px wide).
 */
export function compileTarget(
  gray: Uint8Array,
  width: number,
  height: number,
  options: CompileOptions = {}
): CompiledTarget {
  const {
    widthMeters = 0.2,
    pyramidLevels = 5,
    fastThreshold = 20,
    maxFeaturesPerLevel = 200,
    kernels = jsKernels,
  } = options;

  const pyramid = kernels.buildPyramid(gray, width, height, pyramidLevels);
  const allPoints: number[] = [];
  const allKeypoints: Keypoint[] = [];
  const descriptorChunks: Uint32Array[] = [];

  for (const level of pyramid) {
    let kps = kernels.detectFast(level, fastThreshold, PATCH_BORDER);
    kps = selectSpread(kps, level.width, level.height, maxFeaturesPerLevel);
    if (kps.length === 0) continue;
    descriptorChunks.push(kernels.orientAndDescribe(level, kps));
    for (const kp of kps) {
      allPoints.push(kp.x * level.scale, kp.y * level.scale);
      allKeypoints.push(kp);
    }
  }

  const n = allKeypoints.length;
  const descriptors = new Uint32Array(n * DESCRIPTOR_WORDS);
  let offset = 0;
  for (const chunk of descriptorChunks) {
    descriptors.set(chunk, offset);
    offset += chunk.length;
  }

  const heightMeters = (widthMeters * height) / width;
  const s = width / widthMeters; // pixels per metre
  // u = s*X + width/2 ; v = -s*Y + height/2  (plane y-up -> image y-down)
  const pixelFromPlane: Mat3 = [s, 0, width / 2, 0, -s, height / 2, 0, 0, 1];

  return {
    width,
    height,
    gray,
    widthMeters,
    heightMeters,
    points: new Float32Array(allPoints),
    descriptors,
    pixelFromPlane,
  };
}
