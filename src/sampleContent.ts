/**
 * Built-in sample content: a procedurally drawn square image, so the
 * image/video content modes work out of the box without picking a file.
 * (The sample video is a bundled asset: public/sample-video.mp4.)
 */

let cached: HTMLCanvasElement | null = null;

/** Pick the bundled sample video variant the browser can decode. */
export function sampleVideoUrl(): string {
  const probe = document.createElement('video');
  if (probe.canPlayType('video/mp4; codecs="avc1.42E01E"')) return '/sample-video.mp4';
  return '/sample-video.webm';
}

export function createSampleImageCanvas(size = 512): HTMLCanvasElement {
  if (cached && cached.width === size) return cached;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d')!;
  const u = size / 100;

  // Deep gradient backdrop.
  const bg = ctx.createLinearGradient(0, 0, size, size);
  bg.addColorStop(0, '#0f172a');
  bg.addColorStop(0.55, '#1e3a8a');
  bg.addColorStop(1, '#7c3aed');
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, size, size);

  // Glowing concentric rings.
  for (let i = 5; i >= 1; i--) {
    ctx.beginPath();
    ctx.arc(size * 0.5, size * 0.42, i * 7 * u, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(96, 165, 250, ${0.55 - i * 0.08})`;
    ctx.lineWidth = 1.6 * u;
    ctx.stroke();
  }
  const core = ctx.createRadialGradient(size * 0.5, size * 0.42, 0, size * 0.5, size * 0.42, 9 * u);
  core.addColorStop(0, '#fef3c7');
  core.addColorStop(1, 'rgba(251, 191, 36, 0)');
  ctx.fillStyle = core;
  ctx.beginPath();
  ctx.arc(size * 0.5, size * 0.42, 9 * u, 0, Math.PI * 2);
  ctx.fill();

  // Sparkles.
  const positions = [
    [18, 20], [80, 14], [88, 46], [14, 58], [26, 82], [72, 78], [58, 22], [36, 34],
  ];
  ctx.fillStyle = '#e0f2fe';
  for (const [px, py] of positions) {
    const r = 1.1 * u;
    ctx.save();
    ctx.translate(px * u, py * u);
    ctx.beginPath();
    for (let k = 0; k < 8; k++) {
      const a = (k * Math.PI) / 4;
      const rr = k % 2 === 0 ? r * 2.4 : r * 0.9;
      ctx.lineTo(Math.cos(a) * rr, Math.sin(a) * rr);
    }
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // Caption.
  ctx.fillStyle = '#f8fafc';
  ctx.font = `700 ${9 * u}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillText('Fable AR', size * 0.5, size * 0.78);
  ctx.fillStyle = 'rgba(226, 232, 240, 0.75)';
  ctx.font = `400 ${3.6 * u}px system-ui, sans-serif`;
  ctx.fillText('image tracking from scratch', size * 0.5, size * 0.85);

  cached = canvas;
  return canvas;
}
