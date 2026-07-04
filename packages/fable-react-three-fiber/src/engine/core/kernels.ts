/**
 * The tracker's compute kernels behind a swappable interface. `jsKernels` is
 * the pure-TypeScript implementation (and the reference for WASM parity
 * tests); `createWasmKernels` (src/wasm/engine.ts) provides the fast path.
 */

import { buildPyramid, type GrayImage, type PyramidLevel } from './imageops';
import { computeOrientation, detectFast, type Keypoint } from './fast';
import { computeDescriptors } from './orb';
import { matchDescriptors, type Match, type MatchOptions } from './matcher';
import { trackPyrLK, type FlowOptions, type FlowResult } from './opticalflow';
import type { Point2 } from './homography';

export interface CVKernels {
  readonly name: 'js' | 'wasm';
  buildPyramid(
    gray: Uint8Array,
    width: number,
    height: number,
    numLevels: number,
    factor?: number
  ): PyramidLevel[];
  detectFast(image: GrayImage, threshold: number, border: number): Keypoint[];
  /** Computes orientations (written back into kps) and ORB descriptors. */
  orientAndDescribe(image: GrayImage, kps: Keypoint[]): Uint32Array;
  matchDescriptors(descA: Uint32Array, descB: Uint32Array, options?: MatchOptions): Match[];
  trackPyrLK(
    prevPyr: PyramidLevel[],
    nextPyr: PyramidLevel[],
    points: Point2[],
    options?: FlowOptions
  ): FlowResult[];
}

export const jsKernels: CVKernels = {
  name: 'js',
  buildPyramid,
  detectFast: (image, threshold, border) =>
    detectFast(image.data, image.width, image.height, threshold, border),
  orientAndDescribe(image, kps) {
    for (const kp of kps) {
      kp.angle = computeOrientation(image.data, image.width, image.height, kp.x, kp.y);
    }
    return computeDescriptors(image.data, image.width, image.height, kps);
  },
  matchDescriptors,
  trackPyrLK,
};
