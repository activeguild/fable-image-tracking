/**
 * Procedural sample tracking target. Drawn identically by the main app (which
 * compiles it into a feature bank) and by target.html (which displays it so a
 * second screen or a print-out can be tracked).
 *
 * Design goals for a good natural-feature target: high contrast, corner-rich,
 * asymmetric, structure at multiple scales, no repeating patterns.
 */

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

const PALETTE = ['#111827', '#1f2937', '#b45309', '#0f766e', '#7c2d12', '#334155'];

export function drawSampleTarget(ctx: CanvasRenderingContext2D, size: number): void {
  const rand = mulberry32(0xa11ce);
  const pick = <T,>(arr: T[]): T => arr[(rand() * arr.length) | 0];

  ctx.save();
  ctx.fillStyle = '#f4f1e8';
  ctx.fillRect(0, 0, size, size);

  const u = size / 100; // pattern unit

  // Thick asymmetric frame: strong corners at the target boundary.
  ctx.fillStyle = '#111827';
  ctx.fillRect(0, 0, size, 4 * u);
  ctx.fillRect(0, size - 4 * u, size, 4 * u);
  ctx.fillRect(0, 0, 4 * u, size);
  ctx.fillRect(size - 4 * u, 0, 4 * u, size);
  ctx.fillStyle = '#b45309';
  ctx.fillRect(0, 0, 14 * u, 14 * u); // one distinct corner breaks symmetry

  // Large-scale blocks (visible from far / low pyramid levels).
  for (let i = 0; i < 6; i++) {
    ctx.fillStyle = pick(PALETTE);
    const w = (12 + rand() * 20) * u;
    const h = (12 + rand() * 20) * u;
    const x = 6 * u + rand() * (size - 12 * u - w);
    const y = 6 * u + rand() * (size - 12 * u - h);
    ctx.save();
    ctx.translate(x + w / 2, y + h / 2);
    ctx.rotate((rand() - 0.5) * 0.9);
    if (rand() < 0.5) {
      ctx.fillRect(-w / 2, -h / 2, w, h);
    } else {
      ctx.beginPath();
      ctx.moveTo(-w / 2, h / 2);
      ctx.lineTo(0, -h / 2);
      ctx.lineTo(w / 2, h / 2);
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
  }

  // Medium circles and rings.
  for (let i = 0; i < 10; i++) {
    ctx.fillStyle = pick(PALETTE);
    ctx.strokeStyle = pick(PALETTE);
    const r = (3 + rand() * 6) * u;
    const x = (10 + rand() * 80) * u;
    const y = (10 + rand() * 80) * u;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    if (rand() < 0.5) {
      ctx.fill();
    } else {
      ctx.lineWidth = 1.6 * u;
      ctx.stroke();
    }
  }

  // Small speckle triangles: dense corner features for close-up tracking.
  for (let i = 0; i < 70; i++) {
    ctx.fillStyle = pick(PALETTE);
    const s = (1.2 + rand() * 2.4) * u;
    const x = (7 + rand() * 86) * u;
    const y = (7 + rand() * 86) * u;
    const a = rand() * Math.PI * 2;
    ctx.beginPath();
    ctx.moveTo(x + Math.cos(a) * s, y + Math.sin(a) * s);
    ctx.lineTo(x + Math.cos(a + 2.1) * s, y + Math.sin(a + 2.1) * s);
    ctx.lineTo(x + Math.cos(a + 4.2) * s, y + Math.sin(a + 4.2) * s);
    ctx.closePath();
    ctx.fill();
  }

  ctx.restore();
}

/** Render the sample target to a fresh canvas of the given size. */
export function createSampleTargetCanvas(size: number): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  drawSampleTarget(canvas.getContext('2d')!, size);
  return canvas;
}
