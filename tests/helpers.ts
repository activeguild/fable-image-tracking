/** Shared synthetic-image helpers for tests (pure, no DOM). */

import { sampleBilinear } from '../src/core/imageops';

function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function seededRandom(seed: number): () => number {
  return mulberry32(seed);
}

/**
 * Smooth random texture: random low-resolution noise upsampled bilinearly.
 * Good input for optical flow and descriptor tests.
 */
export function randomTexture(w: number, h: number, seed = 42, cell = 6): Uint8Array {
  const rand = mulberry32(seed);
  const gw = Math.ceil(w / cell) + 2;
  const gh = Math.ceil(h / cell) + 2;
  const grid = new Float32Array(gw * gh);
  for (let i = 0; i < grid.length; i++) grid[i] = rand() * 255;
  const img = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const gx = x / cell;
      const gy = y / cell;
      const x0 = gx | 0;
      const y0 = gy | 0;
      const fx = gx - x0;
      const fy = gy - y0;
      const i = y0 * gw + x0;
      const top = grid[i] + (grid[i + 1] - grid[i]) * fx;
      const bot = grid[i + gw] + (grid[i + gw + 1] - grid[i + gw]) * fx;
      img[y * w + x] = (top + (bot - top) * fy) | 0;
    }
  }
  return img;
}

/** Translate an image by a subpixel offset using bilinear sampling. */
export function shiftImage(src: Uint8Array, w: number, h: number, dx: number, dy: number): Uint8Array {
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      out[y * w + x] = sampleBilinear(src, w, h, x - dx, y - dy) | 0;
    }
  }
  return out;
}
