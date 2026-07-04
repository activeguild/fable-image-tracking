/**
 * JS wrapper around the AssemblyScript kernels (assembly/index.ts). Owns the
 * WASM memory layout: three rotating pyramid arenas (so the previous frame's
 * pyramid stays valid while the next one is built) plus fixed scratch regions
 * for scores, descriptors, matches and optical-flow state.
 *
 * Works in both browsers and Node (tests): the caller supplies a compiled
 * WebAssembly.Module.
 */

import type { CVKernels } from '../core/kernels';
import { jsKernels } from '../core/kernels';
import type { GrayImage, PyramidLevel } from '../core/imageops';
import type { Keypoint } from '../core/fast';
import type { Match, MatchOptions } from '../core/matcher';
import type { FlowOptions, FlowResult } from '../core/opticalflow';
import { getPattern } from '../core/orb';

interface WasmExports {
  memory: WebAssembly.Memory;
  resize(srcPtr: number, sw: number, sh: number, dstPtr: number, dw: number, dh: number): void;
  integral(srcPtr: number, w: number, h: number, iiPtr: number): void;
  fastDetect(
    imgPtr: number, w: number, h: number, threshold: number, border: number,
    scoresPtr: number, candPtr: number, maxCand: number, outPtr: number, maxOut: number
  ): number;
  orientDescribe(
    imgPtr: number, w: number, h: number, iiPtr: number,
    kpsPtr: number, count: number, patternPtr: number, descPtr: number, anglesPtr: number
  ): void;
  matchDesc(
    aPtr: number, nA: number, bPtr: number, nB: number,
    maxDist: number, ratio: number, crossCheck: number,
    bestBPtr: number, bestBDistPtr: number, outPtr: number, maxOut: number
  ): number;
  lkLevel(
    prevPtr: number, pw: number, ph: number, nextPtr: number, nw: number, nh: number,
    ptsPtr: number, count: number, invScale: number, ratio: number, levelScale: number,
    statePtr: number, windowRadius: number, maxIter: number, epsilon: number, maxError: number,
    isFinal: number, frameW: number, frameH: number, outPtr: number
  ): void;
}

/** Max supported image: 512x512 (frames run at ~360px wide). */
const MAX_PIXELS = 512 * 512;
const MAX_KPS = 4096;
const MAX_FLOW_POINTS = 1024;
const MAX_CAND = 40000;

export function createWasmKernels(module: WebAssembly.Module): CVKernels {
  const instance = new WebAssembly.Instance(module, {});
  const ex = instance.exports as unknown as WasmExports;
  const memory = ex.memory;

  // ---- memory layout (bump-allocated once; the module never grows after) --
  let cursor = 1 << 16; // start 64KB in, safely past AS static data
  const alloc = (bytes: number): number => {
    const ptr = cursor;
    cursor = (cursor + bytes + 15) & ~15;
    return ptr;
  };
  const PYR_BYTES = MAX_PIXELS * 2 + 8192;
  const pyrArenas = [alloc(PYR_BYTES), alloc(PYR_BYTES), alloc(PYR_BYTES)];
  const scoresPtr = alloc(MAX_PIXELS * 4);
  const candPtr = alloc(MAX_CAND * 4);
  const cornersPtr = alloc(MAX_KPS * 12);
  const kpsPtr = alloc(MAX_KPS * 8);
  const anglesPtr = alloc(MAX_KPS * 4);
  const iiPtr = alloc(513 * 513 * 4);
  const descAPtr = alloc(MAX_KPS * 32);
  const descBPtr = alloc(MAX_KPS * 32);
  const matchOutPtr = alloc(MAX_KPS * 12);
  const bestBPtr = alloc(MAX_KPS * 4);
  const bestBDistPtr = alloc(MAX_KPS * 4);
  const patternPtr = alloc(1024);
  const ptsPtr = alloc(MAX_FLOW_POINTS * 8);
  const statePtr = alloc(MAX_FLOW_POINTS * 8);
  const flowOutPtr = alloc(MAX_FLOW_POINTS * 16);
  const scratchImgPtr = alloc(MAX_PIXELS);

  if (memory.buffer.byteLength < cursor) {
    memory.grow(Math.ceil((cursor - memory.buffer.byteLength) / 65536));
  }
  // Views stay valid because the module is compiled with max == initial memory.
  const u8 = new Uint8Array(memory.buffer);
  const f32 = new Float32Array(memory.buffer);
  const u32 = new Uint32Array(memory.buffer);
  const i32 = new Int32Array(memory.buffer);

  const pattern = getPattern();
  u8.set(new Uint8Array(pattern.buffer, pattern.byteOffset, pattern.length), patternPtr);

  let pyrRotation = 0;

  /** Pointer to image data: zero-copy for wasm-backed views, else a copy. */
  const ptrOf = (data: Uint8Array): number => {
    if (data.buffer === memory.buffer) return data.byteOffset;
    u8.set(data, scratchImgPtr);
    return scratchImgPtr;
  };
  const isWasmBacked = (levels: PyramidLevel[]): boolean =>
    levels.every((l) => l.data.buffer === memory.buffer);

  return {
    name: 'wasm',

    buildPyramid(gray, width, height, numLevels, factor = Math.SQRT1_2) {
      if (width * height > MAX_PIXELS) return jsKernels.buildPyramid(gray, width, height, numLevels, factor);
      const arena = pyrArenas[pyrRotation];
      pyrRotation = (pyrRotation + 1) % pyrArenas.length;
      let p = arena;
      u8.set(gray, p);
      const levels: PyramidLevel[] = [
        { data: u8.subarray(p, p + width * height), width, height, scale: 1 },
      ];
      p += (width * height + 15) & ~15;
      for (let l = 1; l < numLevels; l++) {
        const scale = Math.pow(1 / factor, l);
        const w = Math.round(width * Math.pow(factor, l));
        const h = Math.round(height * Math.pow(factor, l));
        if (w < 48 || h < 48) break;
        const prev = levels[l - 1];
        ex.resize(prev.data.byteOffset, prev.width, prev.height, p, w, h);
        levels.push({ data: u8.subarray(p, p + w * h), width: w, height: h, scale });
        p += (w * h + 15) & ~15;
      }
      return levels;
    },

    detectFast(image: GrayImage, threshold: number, border: number): Keypoint[] {
      if (image.width * image.height > MAX_PIXELS) return jsKernels.detectFast(image, threshold, border);
      const ptr = ptrOf(image.data);
      const n = ex.fastDetect(
        ptr, image.width, image.height, threshold, border,
        scoresPtr, candPtr, MAX_CAND, cornersPtr, MAX_KPS
      );
      const out: Keypoint[] = new Array(n);
      const base = cornersPtr >> 2;
      for (let i = 0; i < n; i++) {
        out[i] = { x: f32[base + i * 3], y: f32[base + i * 3 + 1], score: f32[base + i * 3 + 2], angle: 0 };
      }
      return out;
    },

    orientAndDescribe(image: GrayImage, kps: Keypoint[]): Uint32Array {
      if (kps.length === 0) return new Uint32Array(0);
      if (kps.length > MAX_KPS || image.width * image.height > MAX_PIXELS) {
        return jsKernels.orientAndDescribe(image, kps);
      }
      const ptr = ptrOf(image.data);
      ex.integral(ptr, image.width, image.height, iiPtr);
      const kbase = kpsPtr >> 2;
      for (let i = 0; i < kps.length; i++) {
        f32[kbase + i * 2] = kps[i].x;
        f32[kbase + i * 2 + 1] = kps[i].y;
      }
      ex.orientDescribe(ptr, image.width, image.height, iiPtr, kpsPtr, kps.length, patternPtr, descBPtr, anglesPtr);
      const abase = anglesPtr >> 2;
      for (let i = 0; i < kps.length; i++) kps[i].angle = f32[abase + i];
      return u32.slice(descBPtr >> 2, (descBPtr >> 2) + kps.length * 8);
    },

    matchDescriptors(descA, descB, options: MatchOptions = {}): Match[] {
      const { maxDistance = 64, ratio = 0.85, crossCheck = true } = options;
      const nA = descA.length / 8;
      const nB = descB.length / 8;
      if (nA > MAX_KPS || nB > MAX_KPS) return jsKernels.matchDescriptors(descA, descB, options);
      u32.set(descA, descAPtr >> 2);
      u32.set(descB, descBPtr >> 2);
      const n = ex.matchDesc(
        descAPtr, nA, descBPtr, nB, maxDistance, ratio, crossCheck ? 1 : 0,
        bestBPtr, bestBDistPtr, matchOutPtr, MAX_KPS
      );
      const out: Match[] = new Array(n);
      const base = matchOutPtr >> 2;
      for (let i = 0; i < n; i++) {
        out[i] = { a: i32[base + i * 3], b: i32[base + i * 3 + 1], dist: i32[base + i * 3 + 2] };
      }
      return out;
    },

    trackPyrLK(prevPyr, nextPyr, points, options: FlowOptions = {}): FlowResult[] {
      const { windowRadius = 4, maxIterations = 12, epsilon = 0.01, maxError = 24, initialGuess } = options;
      if (
        points.length === 0 ||
        points.length > MAX_FLOW_POINTS ||
        windowRadius > 8 ||
        !isWasmBacked(prevPyr) ||
        !isWasmBacked(nextPyr)
      ) {
        return jsKernels.trackPyrLK(prevPyr, nextPyr, points, options);
      }
      const numLevels = Math.min(prevPyr.length, nextPyr.length);
      const topScale = prevPyr[numLevels - 1].scale;
      const pbase = ptsPtr >> 2;
      const sbase = statePtr >> 2;
      for (let i = 0; i < points.length; i++) {
        f32[pbase + i * 2] = points[i].x;
        f32[pbase + i * 2 + 1] = points[i].y;
        const g = initialGuess?.[i];
        f32[sbase + i * 2] = g ? (g.x - points[i].x) / topScale : 0;
        f32[sbase + i * 2 + 1] = g ? (g.y - points[i].y) / topScale : 0;
      }
      for (let L = numLevels - 1; L >= 0; L--) {
        const prev = prevPyr[L];
        const next = nextPyr[L];
        const ratio = L > 0 ? prev.scale / prevPyr[L - 1].scale : 0;
        ex.lkLevel(
          prev.data.byteOffset, prev.width, prev.height,
          next.data.byteOffset, next.width, next.height,
          ptsPtr, points.length, 1 / prev.scale, ratio, prev.scale,
          statePtr, windowRadius, maxIterations, epsilon, maxError,
          L === 0 ? 1 : 0, prevPyr[0].width, prevPyr[0].height, flowOutPtr
        );
      }
      const out: FlowResult[] = new Array(points.length);
      const obase = flowOutPtr >> 2;
      for (let i = 0; i < points.length; i++) {
        const err = f32[obase + i * 4 + 2];
        out[i] = {
          x: f32[obase + i * 4],
          y: f32[obase + i * 4 + 1],
          err: err >= 1e29 ? Infinity : err,
          ok: f32[obase + i * 4 + 3] !== 0,
        };
      }
      return out;
    },
  };
}
