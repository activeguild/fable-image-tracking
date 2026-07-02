/**
 * Basic image operations on tightly-packed grayscale buffers (Uint8Array, row-major).
 * Everything here is allocation-conscious: callers may pass a reusable `out` buffer.
 */

export interface GrayImage {
  data: Uint8Array;
  width: number;
  height: number;
}

/** Convert RGBA (canvas ImageData) to 8-bit grayscale using integer BT.601 weights. */
export function rgbaToGray(
  rgba: Uint8ClampedArray | Uint8Array,
  width: number,
  height: number,
  out?: Uint8Array
): Uint8Array {
  const n = width * height;
  const gray = out && out.length >= n ? out : new Uint8Array(n);
  for (let i = 0, j = 0; i < n; i++, j += 4) {
    gray[i] = (rgba[j] * 77 + rgba[j + 1] * 151 + rgba[j + 2] * 28) >> 8;
  }
  return gray;
}

/** Bilinear resize. Used to build scale pyramids with non-integer factors. */
export function resizeBilinear(
  src: Uint8Array,
  sw: number,
  sh: number,
  dw: number,
  dh: number
): Uint8Array {
  const dst = new Uint8Array(dw * dh);
  const xRatio = sw / dw;
  const yRatio = sh / dh;
  for (let y = 0; y < dh; y++) {
    const sy = Math.min((y + 0.5) * yRatio - 0.5, sh - 1.001);
    const y0 = Math.max(0, Math.floor(sy));
    const fy = sy - y0;
    const y1 = Math.min(y0 + 1, sh - 1);
    const r0 = y0 * sw;
    const r1 = y1 * sw;
    for (let x = 0; x < dw; x++) {
      const sx = Math.min((x + 0.5) * xRatio - 0.5, sw - 1.001);
      const x0 = Math.max(0, Math.floor(sx));
      const fx = sx - x0;
      const x1 = Math.min(x0 + 1, sw - 1);
      const a = src[r0 + x0];
      const b = src[r0 + x1];
      const c = src[r1 + x0];
      const d = src[r1 + x1];
      const top = a + (b - a) * fx;
      const bot = c + (d - c) * fx;
      dst[y * dw + x] = (top + (bot - top) * fy + 0.5) | 0;
    }
  }
  return dst;
}

/** Sample with bilinear interpolation and border clamping. Coordinates are pixel-centered. */
export function sampleBilinear(img: Uint8Array, w: number, h: number, x: number, y: number): number {
  if (x < 0) x = 0;
  else if (x > w - 1.001) x = w - 1.001;
  if (y < 0) y = 0;
  else if (y > h - 1.001) y = h - 1.001;
  const x0 = x | 0;
  const y0 = y | 0;
  const fx = x - x0;
  const fy = y - y0;
  const i = y0 * w + x0;
  const a = img[i];
  const b = img[i + 1];
  const c = img[i + w];
  const d = img[i + w + 1];
  const top = a + (b - a) * fx;
  const bot = c + (d - c) * fx;
  return top + (bot - top) * fy;
}

/**
 * Summed-area table with a 1-pixel zero border: ii has (w+1)*(h+1) entries and
 * ii[(y+1)*(w+1)+(x+1)] = sum of src[0..y][0..x]. Max value for a 4K-ish gray
 * image stays well below 2^32, so Uint32Array is safe.
 */
export function integralImage(src: Uint8Array, w: number, h: number): Uint32Array {
  const iw = w + 1;
  const ii = new Uint32Array(iw * (h + 1));
  for (let y = 0; y < h; y++) {
    let rowSum = 0;
    const srcRow = y * w;
    const dstRow = (y + 1) * iw;
    const prevRow = y * iw;
    for (let x = 0; x < w; x++) {
      rowSum += src[srcRow + x];
      ii[dstRow + x + 1] = rowSum + ii[prevRow + x + 1];
    }
  }
  return ii;
}

/** Inclusive box sum over [x0,x1]x[y0,y1] using an integral image from `integralImage`. */
export function boxSum(ii: Uint32Array, w: number, x0: number, y0: number, x1: number, y1: number): number {
  const iw = w + 1;
  return (
    ii[(y1 + 1) * iw + x1 + 1] -
    ii[y0 * iw + x1 + 1] -
    ii[(y1 + 1) * iw + x0] +
    ii[y0 * iw + x0]
  );
}

export interface PyramidLevel extends GrayImage {
  /** Multiply level coordinates by this to get level-0 coordinates. */
  scale: number;
}

/** Scale pyramid with an arbitrary per-level factor (default 1/sqrt(2)). */
export function buildPyramid(
  gray: Uint8Array,
  width: number,
  height: number,
  numLevels: number,
  factor = Math.SQRT1_2
): PyramidLevel[] {
  const levels: PyramidLevel[] = [{ data: gray, width, height, scale: 1 }];
  for (let l = 1; l < numLevels; l++) {
    const scale = Math.pow(1 / factor, l);
    const w = Math.round(width * Math.pow(factor, l));
    const h = Math.round(height * Math.pow(factor, l));
    if (w < 48 || h < 48) break;
    const prev = levels[l - 1];
    levels.push({ data: resizeBilinear(prev.data, prev.width, prev.height, w, h), width: w, height: h, scale });
  }
  return levels;
}
