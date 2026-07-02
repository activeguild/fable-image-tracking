/**
 * WASM kernels for the image tracker, written in AssemblyScript.
 *
 * These mirror the TypeScript implementations in src/core/ exactly (including
 * rounding behaviour), so the JS versions serve both as the fallback engine
 * and as the reference for parity tests. All functions operate on raw linear
 * memory; the JS wrapper (src/wasm/engine.ts) owns the memory layout.
 *
 * Note: JS `Math.round(x)` equals `floor(x + 0.5)`; AS `Math.round` rounds
 * half away from zero, so `Math.floor(x + 0.5)` is used everywhere instead.
 */

// ------------------------------------------------------------ image ops

/** Bilinear resize of an 8-bit grayscale image (mirror of resizeBilinear). */
export function resize(srcPtr: i32, sw: i32, sh: i32, dstPtr: i32, dw: i32, dh: i32): void {
  const xRatio: f64 = <f64>sw / <f64>dw;
  const yRatio: f64 = <f64>sh / <f64>dh;
  for (let y = 0; y < dh; y++) {
    let sy: f64 = (<f64>y + 0.5) * yRatio - 0.5;
    const syMax: f64 = <f64>sh - 1.001;
    if (sy > syMax) sy = syMax;
    let y0 = <i32>Math.floor(sy);
    if (y0 < 0) y0 = 0;
    const fy: f64 = sy - <f64>y0;
    let y1 = y0 + 1;
    if (y1 > sh - 1) y1 = sh - 1;
    const r0 = srcPtr + y0 * sw;
    const r1 = srcPtr + y1 * sw;
    for (let x = 0; x < dw; x++) {
      let sx: f64 = (<f64>x + 0.5) * xRatio - 0.5;
      const sxMax: f64 = <f64>sw - 1.001;
      if (sx > sxMax) sx = sxMax;
      let x0 = <i32>Math.floor(sx);
      if (x0 < 0) x0 = 0;
      const fx: f64 = sx - <f64>x0;
      let x1 = x0 + 1;
      if (x1 > sw - 1) x1 = sw - 1;
      const a: f64 = <f64>load<u8>(r0 + x0);
      const b: f64 = <f64>load<u8>(r0 + x1);
      const c: f64 = <f64>load<u8>(r1 + x0);
      const d: f64 = <f64>load<u8>(r1 + x1);
      const top: f64 = a + (b - a) * fx;
      const bot: f64 = c + (d - c) * fx;
      store<u8>(dstPtr + y * dw + x, <u8>(<i32>(top + (bot - top) * fy + 0.5)));
    }
  }
}

/** Summed-area table with a 1-px zero border (mirror of integralImage). */
export function integral(srcPtr: i32, w: i32, h: i32, iiPtr: i32): void {
  const iw = w + 1;
  memory.fill(iiPtr, 0, (iw * (h + 1)) << 2);
  for (let y = 0; y < h; y++) {
    let rowSum: u32 = 0;
    const srcRow = srcPtr + y * w;
    const dstRow = iiPtr + ((y + 1) * iw << 2);
    const prevRow = iiPtr + (y * iw << 2);
    for (let x = 0; x < w; x++) {
      rowSum += <u32>load<u8>(srcRow + x);
      store<u32>(dstRow + ((x + 1) << 2), rowSum + load<u32>(prevRow + ((x + 1) << 2)));
    }
  }
}

// @ts-ignore: decorator valid in AssemblyScript
@inline
function sampleBil(ptr: i32, w: i32, h: i32, x: f64, y: f64): f64 {
  if (x < 0) x = 0;
  else if (x > <f64>w - 1.001) x = <f64>w - 1.001;
  if (y < 0) y = 0;
  else if (y > <f64>h - 1.001) y = <f64>h - 1.001;
  const x0 = <i32>x;
  const y0 = <i32>y;
  const fx: f64 = x - <f64>x0;
  const fy: f64 = y - <f64>y0;
  const i = ptr + y0 * w + x0;
  const a: f64 = <f64>load<u8>(i);
  const b: f64 = <f64>load<u8>(i + 1);
  const c: f64 = <f64>load<u8>(i + w);
  const d: f64 = <f64>load<u8>(i + w + 1);
  const top: f64 = a + (b - a) * fx;
  const bot: f64 = c + (d - c) * fx;
  return top + (bot - top) * fy;
}

// ------------------------------------------------------------ FAST

// Bresenham circle of radius 3, clockwise from 12 o'clock.
const CIRCLE_DX = memory.data<i32>([0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1]);
const CIRCLE_DY = memory.data<i32>([-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3]);
const CIRCLE_OFFS = memory.data(64); // 16 x i32, filled per call (depends on width)

// @ts-ignore: decorator valid in AssemblyScript
@inline
function hasRun9(mask16: i32): bool {
  if (mask16 == 0) return false;
  const doubled = mask16 | (mask16 << 16);
  let run = 0;
  for (let i = 0; i < 32; i++) {
    if (doubled & (1 << i)) {
      run++;
      if (run >= 9) return true;
    } else {
      if (i >= 16) break;
      run = 0;
    }
  }
  return false;
}

/**
 * FAST-9 with NMS and subpixel refinement (mirror of detectFast).
 * scoresPtr: f32[w*h] scratch, candPtr: i32 scratch, outPtr: f32 triplets
 * (x, y, score). Returns the number of keypoints written.
 */
export function fastDetect(
  imgPtr: i32, w: i32, h: i32, threshold: i32, border: i32,
  scoresPtr: i32, candPtr: i32, maxCand: i32, outPtr: i32, maxOut: i32
): i32 {
  for (let i = 0; i < 16; i++) {
    store<i32>(CIRCLE_OFFS + (i << 2), load<i32>(CIRCLE_DY + (i << 2)) * w + load<i32>(CIRCLE_DX + (i << 2)));
  }
  let minB = border;
  if (minB < 3) minB = 3;

  memory.fill(scoresPtr, 0, (w * h) << 2);
  let candCount = 0;

  for (let y = minB; y < h - minB; y++) {
    const row = y * w;
    for (let x = minB; x < w - minB; x++) {
      const idx = row + x;
      const p = <i32>load<u8>(imgPtr + idx);
      const hi = p + threshold;
      const lo = p - threshold;

      let brighterQuad = 0;
      let darkerQuad = 0;
      for (let q = 0; q < 16; q += 4) {
        const v = <i32>load<u8>(imgPtr + idx + load<i32>(CIRCLE_OFFS + (q << 2)));
        if (v > hi) brighterQuad++;
        else if (v < lo) darkerQuad++;
      }
      if (brighterQuad < 2 && darkerQuad < 2) continue;

      let brightMask = 0;
      let darkMask = 0;
      for (let i = 0; i < 16; i++) {
        const v = <i32>load<u8>(imgPtr + idx + load<i32>(CIRCLE_OFFS + (i << 2)));
        if (v > hi) brightMask |= 1 << i;
        else if (v < lo) darkMask |= 1 << i;
      }
      if ((brightMask | darkMask) == 0) continue;
      if (!hasRun9(brightMask) && !hasRun9(darkMask)) continue;

      let score = 0;
      for (let i = 0; i < 16; i++) {
        const v = <i32>load<u8>(imgPtr + idx + load<i32>(CIRCLE_OFFS + (i << 2)));
        let d = v - p;
        if (d < 0) d = -d;
        d -= threshold;
        if (d > 0) score += d;
      }
      store<f32>(scoresPtr + (idx << 2), <f32>score);
      if (candCount < maxCand) {
        store<i32>(candPtr + (candCount << 2), idx);
        candCount++;
      }
    }
  }

  // 3x3 NMS + parabola subpixel refinement.
  let outCount = 0;
  for (let c = 0; c < candCount; c++) {
    if (outCount >= maxOut) break;
    const idx = load<i32>(candPtr + (c << 2));
    const sp = scoresPtr + (idx << 2);
    const s: f32 = load<f32>(sp);
    const wB = w << 2;
    if (
      s >= load<f32>(sp - 4) &&
      s > load<f32>(sp + 4) &&
      s >= load<f32>(sp - wB - 4) &&
      s >= load<f32>(sp - wB) &&
      s >= load<f32>(sp - wB + 4) &&
      s > load<f32>(sp + wB - 4) &&
      s > load<f32>(sp + wB) &&
      s > load<f32>(sp + wB + 4)
    ) {
      let ox: f64 = 0;
      let oy: f64 = 0;
      const sxm: f64 = <f64>load<f32>(sp - 4);
      const sxp: f64 = <f64>load<f32>(sp + 4);
      const sym: f64 = <f64>load<f32>(sp - wB);
      const syp: f64 = <f64>load<f32>(sp + wB);
      const sd: f64 = <f64>s;
      const dxDen: f64 = sxm - 2 * sd + sxp;
      if (dxDen < 0) {
        ox = (sxm - sxp) / (2 * dxDen);
        if (ox < -0.5) ox = -0.5;
        else if (ox > 0.5) ox = 0.5;
      }
      const dyDen: f64 = sym - 2 * sd + syp;
      if (dyDen < 0) {
        oy = (sym - syp) / (2 * dyDen);
        if (oy < -0.5) oy = -0.5;
        else if (oy > 0.5) oy = 0.5;
      }
      const base = outPtr + outCount * 12;
      store<f32>(base, <f32>(<f64>(idx % w) + ox));
      store<f32>(base + 4, <f32>(<f64>(idx / w) + oy));
      store<f32>(base + 8, s);
      outCount++;
    }
  }
  return outCount;
}

// ------------------------------------------------------------ ORB

// @ts-ignore: decorator valid in AssemblyScript
@inline
function boxSum5(iiPtr: i32, w: i32, x: i32, y: i32): u32 {
  const iw = w + 1;
  const x0 = x - 2;
  const y0 = y - 2;
  const x1 = x + 2;
  const y1 = y + 2;
  return (
    load<u32>(iiPtr + (((y1 + 1) * iw + x1 + 1) << 2)) -
    load<u32>(iiPtr + ((y0 * iw + x1 + 1) << 2)) -
    load<u32>(iiPtr + (((y1 + 1) * iw + x0) << 2)) +
    load<u32>(iiPtr + ((y0 * iw + x0) << 2))
  );
}

/**
 * Orientation (intensity centroid, radius 15) + rotated-BRIEF descriptors
 * (mirror of computeOrientation + computeDescriptors).
 * kpsPtr: f32 pairs (x, y); anglesPtr: f32 out; descPtr: u32 x 8 per kp.
 */
export function orientDescribe(
  imgPtr: i32, w: i32, h: i32, iiPtr: i32,
  kpsPtr: i32, count: i32, patternPtr: i32, descPtr: i32, anglesPtr: i32
): void {
  const maxX = w - 3;
  const maxY = h - 3;
  for (let k = 0; k < count; k++) {
    const kx: f64 = <f64>load<f32>(kpsPtr + (k << 3));
    const ky: f64 = <f64>load<f32>(kpsPtr + (k << 3) + 4);
    const cx = <i32>Math.floor(kx + 0.5);
    const cy = <i32>Math.floor(ky + 0.5);

    // Intensity centroid over a radius-15 disc.
    let m01: i32 = 0;
    let m10: i32 = 0;
    for (let dy = -15; dy <= 15; dy++) {
      const rowLimit = <i32>Math.floor(Math.sqrt(<f64>(225 - dy * dy)));
      const rowBase = imgPtr + (cy + dy) * w + cx;
      for (let dx = -rowLimit; dx <= rowLimit; dx++) {
        const v = <i32>load<u8>(rowBase + dx);
        m10 += dx * v;
        m01 += dy * v;
      }
    }
    const angle: f64 = Math.atan2(<f64>m01, <f64>m10);
    store<f32>(anglesPtr + (k << 2), <f32>angle);

    const cosA: f64 = Math.cos(angle);
    const sinA: f64 = Math.sin(angle);

    let word: u32 = 0;
    let bit = 0;
    let wordPtr = descPtr + (k << 5);
    for (let i = 0; i < 256; i++) {
      const p = patternPtr + (i << 2);
      const x1p = <f64>load<i8>(p);
      const y1p = <f64>load<i8>(p + 1);
      const x2p = <f64>load<i8>(p + 2);
      const y2p = <f64>load<i8>(p + 3);

      let ax = cx + <i32>Math.floor(cosA * x1p - sinA * y1p + 0.5);
      let ay = cy + <i32>Math.floor(sinA * x1p + cosA * y1p + 0.5);
      if (ax < 2) ax = 2;
      else if (ax > maxX) ax = maxX;
      if (ay < 2) ay = 2;
      else if (ay > maxY) ay = maxY;
      const v1 = boxSum5(iiPtr, w, ax, ay);

      let bx = cx + <i32>Math.floor(cosA * x2p - sinA * y2p + 0.5);
      let by = cy + <i32>Math.floor(sinA * x2p + cosA * y2p + 0.5);
      if (bx < 2) bx = 2;
      else if (bx > maxX) bx = maxX;
      if (by < 2) by = 2;
      else if (by > maxY) by = maxY;
      const v2 = boxSum5(iiPtr, w, bx, by);

      if (v1 < v2) word |= (1 << bit) as u32;
      bit++;
      if (bit == 32) {
        store<u32>(wordPtr, word);
        wordPtr += 4;
        word = 0;
        bit = 0;
      }
    }
  }
}

// ------------------------------------------------------------ matching

// @ts-ignore: decorator valid in AssemblyScript
@inline
function hamming8(aPtr: i32, bPtr: i32): i32 {
  let dist: i32 = 0;
  for (let i = 0; i < 32; i += 4) {
    dist += <i32>popcnt<u32>(load<u32>(aPtr + i) ^ load<u32>(bPtr + i));
  }
  return dist;
}

/**
 * Brute-force Hamming matching with ratio test and cross-check (mirror of
 * matchDescriptors). outPtr receives i32 triplets (a, b, dist); bestBPtr and
 * bestBDistPtr are i32[nB] scratch. Returns the number of matches.
 */
export function matchDesc(
  aPtr: i32, nA: i32, bPtr: i32, nB: i32,
  maxDist: i32, ratio: f64, crossCheck: i32,
  bestBPtr: i32, bestBDistPtr: i32, outPtr: i32, maxOut: i32
): i32 {
  if (nA == 0 || nB == 0) return 0;
  if (crossCheck) {
    for (let b = 0; b < nB; b++) {
      store<i32>(bestBPtr + (b << 2), -1);
      store<i32>(bestBDistPtr + (b << 2), 0x7fffffff);
    }
  }

  let outCount = 0;
  for (let a = 0; a < nA; a++) {
    const arow = aPtr + (a << 5);
    let best = 0x7fffffff;
    let second = 0x7fffffff;
    let bestB = -1;
    for (let b = 0; b < nB; b++) {
      const d = hamming8(arow, bPtr + (b << 5));
      if (d < best) {
        second = best;
        best = d;
        bestB = b;
      } else if (d < second) {
        second = d;
      }
      if (crossCheck && d < load<i32>(bestBDistPtr + (b << 2))) {
        store<i32>(bestBDistPtr + (b << 2), d);
        store<i32>(bestBPtr + (b << 2), a);
      }
    }
    if (bestB >= 0 && best <= maxDist && <f64>best < ratio * <f64>second && outCount < maxOut) {
      const base = outPtr + outCount * 12;
      store<i32>(base, a);
      store<i32>(base + 4, bestB);
      store<i32>(base + 8, best);
      outCount++;
    }
  }

  if (!crossCheck) return outCount;

  // Keep a->b only if a is also b's best partner (compact in place).
  let kept = 0;
  for (let m = 0; m < outCount; m++) {
    const base = outPtr + m * 12;
    const a = load<i32>(base);
    const b = load<i32>(base + 4);
    if (load<i32>(bestBPtr + (b << 2)) == a) {
      const dst = outPtr + kept * 12;
      store<i32>(dst, a);
      store<i32>(dst + 4, b);
      store<i32>(dst + 8, load<i32>(base + 8));
      kept++;
    }
  }
  return kept;
}

// ------------------------------------------------------------ Lucas-Kanade

const LK_TPL = memory.data(1600); // f32 window buffers (up to 20x20)
const LK_GX = memory.data(1600);
const LK_GY = memory.data(1600);

/**
 * One pyramid level of Bouguet-style LK for all points (mirror of the level
 * loop in trackPyrLK). The JS wrapper iterates levels coarse-to-fine.
 *
 * ptsPtr:   f32 pairs, level-0 coordinates in the previous image.
 * statePtr: f32 pairs (gx, gy) - the running flow guess in current-level
 *           coordinates; updated in place for the next (finer) level.
 * outPtr:   f32 quads (x, y, err, ok), written when isFinal != 0.
 */
export function lkLevel(
  prevPtr: i32, pw: i32, ph: i32,
  nextPtr: i32, nw: i32, nh: i32,
  ptsPtr: i32, count: i32,
  invScale: f64, ratio: f64, levelScale: f64,
  statePtr: i32, windowRadius: i32, maxIter: i32, epsilon: f64, maxError: f64,
  isFinal: i32, frameW: i32, frameH: i32, outPtr: i32
): void {
  const win = 2 * windowRadius + 1;
  const winArea = win * win;
  const margin: f64 = <f64>(windowRadius + 1);
  const eps2: f64 = epsilon * epsilon;

  for (let k = 0; k < count; k++) {
    const p0x: f64 = <f64>load<f32>(ptsPtr + (k << 3));
    const p0y: f64 = <f64>load<f32>(ptsPtr + (k << 3) + 4);
    let gx: f64 = <f64>load<f32>(statePtr + (k << 3));
    let gy: f64 = <f64>load<f32>(statePtr + (k << 3) + 4);
    const px: f64 = p0x * invScale;
    const py: f64 = p0y * invScale;

    let converged = false;
    let vx: f64 = 0;
    let vy: f64 = 0;
    let err: f64 = 1e30;

    const inPrev =
      px >= margin && py >= margin && px < <f64>pw - margin && py < <f64>ph - margin;

    if (inPrev) {
      let sxx: f64 = 0;
      let sxy: f64 = 0;
      let syy: f64 = 0;
      let idx = 0;
      for (let dy = -windowRadius; dy <= windowRadius; dy++) {
        for (let dx = -windowRadius; dx <= windowRadius; dx++) {
          const x: f64 = px + <f64>dx;
          const y: f64 = py + <f64>dy;
          const tv: f64 = sampleBil(prevPtr, pw, ph, x, y);
          const gxv: f64 =
            (sampleBil(prevPtr, pw, ph, x + 1, y) - sampleBil(prevPtr, pw, ph, x - 1, y)) * 0.5;
          const gyv: f64 =
            (sampleBil(prevPtr, pw, ph, x, y + 1) - sampleBil(prevPtr, pw, ph, x, y - 1)) * 0.5;
          store<f32>(LK_TPL + (idx << 2), <f32>tv);
          store<f32>(LK_GX + (idx << 2), <f32>gxv);
          store<f32>(LK_GY + (idx << 2), <f32>gyv);
          sxx += gxv * gxv;
          sxy += gxv * gyv;
          syy += gyv * gyv;
          idx++;
        }
      }

      const det: f64 = sxx * syy - sxy * sxy;
      if (det >= 1e-4) {
        const invDet: f64 = 1.0 / det;
        for (let it = 0; it < maxIter; it++) {
          const qx: f64 = px + gx + vx;
          const qy: f64 = py + gy + vy;
          if (qx < margin || qy < margin || qx >= <f64>nw - margin || qy >= <f64>nh - margin) {
            break;
          }
          let bx: f64 = 0;
          let by: f64 = 0;
          let absSum: f64 = 0;
          idx = 0;
          for (let dy = -windowRadius; dy <= windowRadius; dy++) {
            for (let dx = -windowRadius; dx <= windowRadius; dx++) {
              const dI: f64 =
                sampleBil(nextPtr, nw, nh, qx + <f64>dx, qy + <f64>dy) -
                <f64>load<f32>(LK_TPL + (idx << 2));
              bx += dI * <f64>load<f32>(LK_GX + (idx << 2));
              by += dI * <f64>load<f32>(LK_GY + (idx << 2));
              absSum += Math.abs(dI);
              idx++;
            }
          }
          err = absSum / <f64>winArea;
          const dxStep: f64 = (-bx * syy + by * sxy) * invDet;
          const dyStep: f64 = (-by * sxx + bx * sxy) * invDet;
          vx += dxStep;
          vy += dyStep;
          converged = true;
          if (dxStep * dxStep + dyStep * dyStep < eps2) break;
        }
      }
    }

    if (!isFinal) {
      store<f32>(statePtr + (k << 3), <f32>((gx + vx) * ratio));
      store<f32>(statePtr + (k << 3) + 4, <f32>((gy + vy) * ratio));
    } else {
      const base = outPtr + (k << 4);
      if (!converged) {
        store<f32>(base, <f32>p0x);
        store<f32>(base + 4, <f32>p0y);
        store<f32>(base + 8, 1e30);
        store<f32>(base + 12, 0);
      } else {
        const outX: f64 = (px + gx + vx) * levelScale;
        const outY: f64 = (py + gy + vy) * levelScale;
        const ok =
          err <= maxError && outX >= 0 && outY >= 0 && outX < <f64>frameW && outY < <f64>frameH;
        store<f32>(base, <f32>outX);
        store<f32>(base + 4, <f32>outY);
        store<f32>(base + 8, <f32>err);
        store<f32>(base + 12, ok ? 1.0 : 0.0);
      }
    }
  }
}
