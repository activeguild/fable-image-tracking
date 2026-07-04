/**
 * FAST-9/16 corner detector (Rosten & Drummond) with non-maximum suppression
 * and grid-based feature selection, implemented from scratch.
 */

export interface Keypoint {
  x: number;
  y: number;
  score: number;
  /** Orientation in radians, filled in by computeOrientation. */
  angle: number;
}

// The 16 pixels of a Bresenham circle of radius 3, clockwise from 12 o'clock.
const CIRCLE_DX = [0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1];
const CIRCLE_DY = [-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3];

/**
 * Detect FAST-9 corners. `border` keeps keypoints far enough from the edge for
 * downstream orientation/descriptor patches. Returns keypoints with NMS applied.
 */
export function detectFast(
  img: Uint8Array,
  width: number,
  height: number,
  threshold: number,
  border: number
): Keypoint[] {
  const offsets = new Int32Array(16);
  for (let i = 0; i < 16; i++) offsets[i] = CIRCLE_DY[i] * width + CIRCLE_DX[i];

  const minB = Math.max(border, 3);
  const scores = new Float32Array(width * height);
  const candidates: number[] = [];

  for (let y = minB; y < height - minB; y++) {
    const row = y * width;
    for (let x = minB; x < width - minB; x++) {
      const idx = row + x;
      const p = img[idx];
      const hi = p + threshold;
      const lo = p - threshold;

      // High-speed rejection using the 4 compass pixels: a 9-contiguous arc
      // always includes at least 2 of {0, 4, 8, 12} on the same side.
      let brighterQuad = 0;
      let darkerQuad = 0;
      for (let q = 0; q < 16; q += 4) {
        const v = img[idx + offsets[q]];
        if (v > hi) brighterQuad++;
        else if (v < lo) darkerQuad++;
      }
      if (brighterQuad < 2 && darkerQuad < 2) continue;

      // Full segment test: look for >= 9 contiguous pixels all brighter or all darker.
      let flags = 0; // 2 bits worth via two masks
      let brightMask = 0;
      let darkMask = 0;
      for (let i = 0; i < 16; i++) {
        const v = img[idx + offsets[i]];
        if (v > hi) brightMask |= 1 << i;
        else if (v < lo) darkMask |= 1 << i;
      }
      flags = brightMask | darkMask;
      if (flags === 0) continue;

      if (!hasContiguousRun(brightMask, 9) && !hasContiguousRun(darkMask, 9)) continue;

      // Corner score: sum of |difference| beyond threshold over the circle.
      let score = 0;
      for (let i = 0; i < 16; i++) {
        const d = Math.abs(img[idx + offsets[i]] - p) - threshold;
        if (d > 0) score += d;
      }
      scores[idx] = score;
      candidates.push(idx);
    }
  }

  // 3x3 non-maximum suppression over the sparse candidate set, with subpixel
  // refinement from a parabola fit over the corner-score neighbourhood.
  const result: Keypoint[] = [];
  for (const idx of candidates) {
    const s = scores[idx];
    if (
      s >= scores[idx - 1] &&
      s > scores[idx + 1] &&
      s >= scores[idx - width - 1] &&
      s >= scores[idx - width] &&
      s >= scores[idx - width + 1] &&
      s > scores[idx + width - 1] &&
      s > scores[idx + width] &&
      s > scores[idx + width + 1]
    ) {
      let ox = 0;
      let oy = 0;
      const dxDen = scores[idx - 1] - 2 * s + scores[idx + 1];
      if (dxDen < 0) ox = clampHalf((scores[idx - 1] - scores[idx + 1]) / (2 * dxDen));
      const dyDen = scores[idx - width] - 2 * s + scores[idx + width];
      if (dyDen < 0) oy = clampHalf((scores[idx - width] - scores[idx + width]) / (2 * dyDen));
      result.push({ x: (idx % width) + ox, y: ((idx / width) | 0) + oy, score: s, angle: 0 });
    }
  }
  return result;
}

function clampHalf(v: number): number {
  return v < -0.5 ? -0.5 : v > 0.5 ? 0.5 : v;
}

/** True if the 16-bit circular mask contains a run of at least `runLen` set bits. */
function hasContiguousRun(mask16: number, runLen: number): boolean {
  if (mask16 === 0) return false;
  const doubled = mask16 | (mask16 << 16); // unroll the wrap-around
  let run = 0;
  for (let i = 0; i < 32; i++) {
    if (doubled & (1 << i)) {
      run++;
      if (run >= runLen) return true;
    } else {
      if (i >= 16) break; // no run can start this late and reach runLen
      run = 0;
    }
  }
  return false;
}

/**
 * Keep at most `maxTotal` keypoints, spatially spread out with a grid cap so a
 * single high-texture region cannot monopolise the budget.
 */
export function selectSpread(
  kps: Keypoint[],
  width: number,
  height: number,
  maxTotal: number,
  cellsX = 8,
  cellsY = 6
): Keypoint[] {
  if (kps.length <= maxTotal) return kps.slice().sort((a, b) => b.score - a.score);
  const sorted = kps.slice().sort((a, b) => b.score - a.score);
  const cellCap = Math.max(2, Math.ceil((maxTotal * 1.2) / (cellsX * cellsY)));
  const counts = new Int32Array(cellsX * cellsY);
  const out: Keypoint[] = [];
  const overflow: Keypoint[] = [];
  for (const kp of sorted) {
    const cx = Math.min(cellsX - 1, (kp.x / width) * cellsX) | 0;
    const cy = Math.min(cellsY - 1, (kp.y / height) * cellsY) | 0;
    const c = cy * cellsX + cx;
    if (counts[c] < cellCap) {
      counts[c]++;
      out.push(kp);
      if (out.length >= maxTotal) return out;
    } else {
      overflow.push(kp);
    }
  }
  // Fill the remaining budget with the best rejected points.
  for (const kp of overflow) {
    out.push(kp);
    if (out.length >= maxTotal) break;
  }
  return out;
}

/**
 * Orientation by intensity centroid (as in ORB): the angle of the vector from
 * the patch centre to its centroid, over a disc of the given radius.
 */
export function computeOrientation(
  img: Uint8Array,
  width: number,
  _height: number,
  x: number,
  y: number,
  radius = 15
): number {
  x = Math.round(x);
  y = Math.round(y);
  let m01 = 0;
  let m10 = 0;
  const r2 = radius * radius;
  for (let dy = -radius; dy <= radius; dy++) {
    const rowLimit = Math.floor(Math.sqrt(r2 - dy * dy));
    const row = (y + dy) * width + x;
    for (let dx = -rowLimit; dx <= rowLimit; dx++) {
      const v = img[row + dx];
      m10 += dx * v;
      m01 += dy * v;
    }
  }
  return Math.atan2(m01, m10);
}
