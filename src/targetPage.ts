import { drawSampleTarget } from './sampleTarget';

const canvas = document.getElementById('target-canvas') as HTMLCanvasElement;

function render(): void {
  const size = Math.floor(Math.min(window.innerWidth, window.innerHeight) * 0.85);
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = size * dpr;
  canvas.height = size * dpr;
  canvas.style.width = `${size}px`;
  canvas.style.height = `${size}px`;
  const ctx = canvas.getContext('2d')!;
  ctx.scale(dpr, dpr);
  drawSampleTarget(ctx, size);
}

render();
window.addEventListener('resize', render);
