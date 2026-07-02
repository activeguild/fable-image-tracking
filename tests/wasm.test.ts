/**
 * WASM kernel parity tests: the AssemblyScript kernels must reproduce the
 * pure-TypeScript reference implementations. Requires public/tracker.wasm
 * (built automatically by the pretest script).
 */

import { beforeAll, describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { jsKernels, type CVKernels } from '../src/core/kernels';
import { createWasmKernels } from '../src/wasm/engine';
import { PATCH_BORDER, DESCRIPTOR_WORDS, hammingDistance } from '../src/core/orb';
import { compileTarget } from '../src/tracker/target';
import { ImageTracker } from '../src/tracker/tracker';
import { applyHomography, invert3, type Mat3 } from '../src/core/homography';
import { sampleBilinear } from '../src/core/imageops';
import { randomTexture, shiftImage } from './helpers';

let wasm: CVKernels;

beforeAll(() => {
  const path = fileURLToPath(new URL('../public/tracker.wasm', import.meta.url));
  const module = new WebAssembly.Module(readFileSync(path));
  wasm = createWasmKernels(module);
});

describe('wasm kernel parity', () => {
  const W = 240;
  const H = 200;
  const img = randomTexture(W, H, 71, 6);

  it('buildPyramid matches byte-for-byte', () => {
    const js = jsKernels.buildPyramid(img, W, H, 4);
    const ws = wasm.buildPyramid(img, W, H, 4);
    expect(ws.length).toBe(js.length);
    for (let l = 0; l < js.length; l++) {
      expect(ws[l].width).toBe(js[l].width);
      expect(ws[l].height).toBe(js[l].height);
      expect(ws[l].scale).toBeCloseTo(js[l].scale, 12);
      expect(Buffer.from(ws[l].data).equals(Buffer.from(js[l].data))).toBe(true);
    }
  });

  it('detectFast matches positions and scores', () => {
    const image = { data: img, width: W, height: H };
    const js = jsKernels.detectFast(image, 15, PATCH_BORDER);
    const ws = wasm.detectFast(image, 15, PATCH_BORDER);
    expect(js.length).toBeGreaterThan(20);
    expect(ws.length).toBe(js.length);
    for (let i = 0; i < js.length; i++) {
      expect(ws[i].x).toBeCloseTo(js[i].x, 3);
      expect(ws[i].y).toBeCloseTo(js[i].y, 3);
      expect(ws[i].score).toBeCloseTo(js[i].score, 3);
    }
  });

  it('orientAndDescribe matches angles and descriptor bits', () => {
    const image = { data: img, width: W, height: H };
    const kpsJs = jsKernels.detectFast(image, 15, PATCH_BORDER);
    const kpsWs = kpsJs.map((kp) => ({ ...kp }));
    const dJs = jsKernels.orientAndDescribe(image, kpsJs);
    const dWs = wasm.orientAndDescribe(image, kpsWs);
    expect(dWs.length).toBe(dJs.length);
    for (let i = 0; i < kpsJs.length; i++) {
      expect(kpsWs[i].angle).toBeCloseTo(kpsJs[i].angle, 4);
      const dist = hammingDistance(dJs, i * DESCRIPTOR_WORDS, dWs, i * DESCRIPTOR_WORDS);
      expect(dist, `descriptor ${i}`).toBeLessThanOrEqual(2);
    }
  });

  it('matchDescriptors returns identical matches for identical input', () => {
    const image = { data: img, width: W, height: H };
    const shifted = shiftImage(img, W, H, 6, -4);
    const kA = jsKernels.detectFast(image, 15, PATCH_BORDER);
    const kB = jsKernels.detectFast({ data: shifted, width: W, height: H }, 15, PATCH_BORDER);
    const dA = jsKernels.orientAndDescribe(image, kA);
    const dB = jsKernels.orientAndDescribe({ data: shifted, width: W, height: H }, kB);

    const js = jsKernels.matchDescriptors(dA, dB);
    const ws = wasm.matchDescriptors(dA, dB);
    expect(ws.length).toBe(js.length);
    for (let i = 0; i < js.length; i++) {
      expect(ws[i]).toEqual(js[i]);
    }
    expect(js.length).toBeGreaterThan(10);
  });

  it('trackPyrLK matches the reference flow', () => {
    const dx = 4.6;
    const dy = -3.1;
    const moved = shiftImage(img, W, H, dx, dy);
    const points = [];
    for (let y = 40; y <= 160; y += 30) {
      for (let x = 40; x <= 200; x += 40) points.push({ x, y });
    }

    const prevJs = jsKernels.buildPyramid(img, W, H, 3);
    const nextJs = jsKernels.buildPyramid(moved, W, H, 3);
    const js = jsKernels.trackPyrLK(prevJs, nextJs, points);

    const prevWs = wasm.buildPyramid(img, W, H, 3);
    const nextWs = wasm.buildPyramid(moved, W, H, 3);
    const ws = wasm.trackPyrLK(prevWs, nextWs, points);

    let tracked = 0;
    for (let i = 0; i < points.length; i++) {
      expect(ws[i].ok).toBe(js[i].ok);
      if (js[i].ok) {
        tracked++;
        expect(ws[i].x).toBeCloseTo(js[i].x, 2);
        expect(ws[i].y).toBeCloseTo(js[i].y, 2);
      }
    }
    expect(tracked).toBeGreaterThan(points.length * 0.7);
  });
});

describe('end-to-end tracking on wasm kernels', () => {
  const FRAME_W = 360;
  const FRAME_H = 270;
  const TARGET_SIZE = 256;
  const targetImg = randomTexture(TARGET_SIZE, TARGET_SIZE, 77, 9);

  function similarity(scale: number, angle: number, tx: number, ty: number): Mat3 {
    const c = Math.cos(angle) * scale;
    const s = Math.sin(angle) * scale;
    const cx = TARGET_SIZE / 2;
    const cy = TARGET_SIZE / 2;
    return [c, -s, tx - c * cx + s * cy, s, c, ty - s * cx - c * cy, 0, 0, 1];
  }

  function renderFrame(H: Mat3): Uint8Array {
    const Hinv = invert3(H)!;
    const frame = new Uint8Array(FRAME_W * FRAME_H);
    for (let y = 0; y < FRAME_H; y++) {
      for (let x = 0; x < FRAME_W; x++) {
        const p = applyHomography(Hinv, x, y);
        frame[y * FRAME_W + x] =
          p.x >= 0 && p.y >= 0 && p.x < TARGET_SIZE - 1 && p.y < TARGET_SIZE - 1
            ? sampleBilinear(targetImg, TARGET_SIZE, TARGET_SIZE, p.x, p.y) | 0
            : 48 + ((x * 31 + y * 17) % 23);
      }
    }
    return frame;
  }

  it('detects and tracks with corner error under 6 px', () => {
    const compiled = compileTarget(targetImg, TARGET_SIZE, TARGET_SIZE, {
      widthMeters: 0.2,
      kernels: wasm,
    });
    expect(compiled.points.length / 2).toBeGreaterThan(150);

    const tracker = new ImageTracker(compiled, FRAME_W, FRAME_H, { detectEveryN: 1, kernels: wasm });
    for (let f = 0; f < 6; f++) {
      const H = similarity(0.55 + f * 0.01, 0.12 + f * 0.02, 150 + f * 4, 110 + f * 2.5);
      const r = tracker.processFrame(renderFrame(H));
      expect(r.state, `frame ${f}`).toBe('tracking');
      let maxErr = 0;
      for (const [x, y] of [[0, 0], [TARGET_SIZE, 0], [TARGET_SIZE, TARGET_SIZE], [0, TARGET_SIZE]]) {
        const a = applyHomography(r.H!, x, y);
        const b = applyHomography(H, x, y);
        maxErr = Math.max(maxErr, Math.hypot(a.x - b.x, a.y - b.y));
      }
      expect(maxErr).toBeLessThan(6);
    }
  });
});
