/**
 * App entry point: camera capture -> grayscale downscale -> ImageTracker ->
 * pose -> Three.js overlay, with a small HUD and debug point overlay.
 */

import { rgbaToGray } from './core/imageops';
import { compileTarget, type CompiledTarget } from './tracker/target';
import { ImageTracker, type TrackerResult } from './tracker/tracker';
import { ARRenderer } from './render/renderer';
import { createSampleTargetCanvas } from './sampleTarget';

const PROC_WIDTH = 360; // processing resolution (width); height follows aspect
const TARGET_COMPILE_SIZE = 384;
const TARGET_WIDTH_METERS = 0.2;

const container = document.getElementById('ar-container') as HTMLDivElement;
const video = document.getElementById('camera') as HTMLVideoElement;
const bgCanvas = document.getElementById('bg-canvas') as HTMLCanvasElement;
const glCanvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
const debugCanvas = document.getElementById('debug-canvas') as HTMLCanvasElement;
const statusEl = document.getElementById('status') as HTMLDivElement;
const startOverlay = document.getElementById('start-overlay') as HTMLDivElement;
const startButton = document.getElementById('start-button') as HTMLButtonElement;
const uploadInput = document.getElementById('target-upload') as HTMLInputElement;
const debugToggle = document.getElementById('debug-toggle') as HTMLInputElement;

let compiledTarget: CompiledTarget | null = null;
let tracker: ImageTracker | null = null;
let renderer: ARRenderer | null = null;

let procCanvas: HTMLCanvasElement;
let procCtx: CanvasRenderingContext2D;
let bgCtx: CanvasRenderingContext2D;
let grayBuffer: Uint8Array | undefined;
let procW = 0;
let procH = 0;

let fps = 0;
let fpsCounter = 0;
let fpsWindowStart = 0;
let lastFrameTime = 0;

function compileFromCanvas(canvas: HTMLCanvasElement): CompiledTarget {
  const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
  const { width, height } = canvas;
  const rgba = ctx.getImageData(0, 0, width, height).data;
  const gray = rgbaToGray(rgba, width, height);
  return compileTarget(gray, width, height, { widthMeters: TARGET_WIDTH_METERS });
}

function compileFromImage(img: HTMLImageElement): CompiledTarget {
  const scale = Math.min(1, 420 / img.naturalWidth);
  const w = Math.round(img.naturalWidth * scale);
  const h = Math.round(img.naturalHeight * scale);
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  canvas.getContext('2d')!.drawImage(img, 0, 0, w, h);
  return compileFromCanvas(canvas);
}

async function startCamera(): Promise<void> {
  const stream = await navigator.mediaDevices.getUserMedia({
    video: {
      facingMode: 'environment',
      width: { ideal: 1280 },
      height: { ideal: 720 },
    },
    audio: false,
  });
  video.srcObject = stream;
  await video.play();
  await new Promise<void>((resolve) => {
    if (video.videoWidth > 0) resolve();
    else video.addEventListener('loadedmetadata', () => resolve(), { once: true });
  });
}

function setupProcessing(): void {
  const vw = video.videoWidth;
  const vh = video.videoHeight;
  procW = PROC_WIDTH;
  procH = Math.round((vh / vw) * PROC_WIDTH);

  procCanvas = document.createElement('canvas');
  procCanvas.width = procW;
  procCanvas.height = procH;
  procCtx = procCanvas.getContext('2d', { willReadFrequently: true })!;

  debugCanvas.width = procW;
  debugCanvas.height = procH;

  tracker = new ImageTracker(compiledTarget!, procW, procH);

  renderer = new ARRenderer(container, bgCanvas, glCanvas, debugCanvas);
  renderer.setVideoSize(vw, vh);
  bgCtx = bgCanvas.getContext('2d')!;
  renderer.setIntrinsics(tracker.intrinsics, procW, procH);
  renderer.setTargetSize(compiledTarget!.widthMeters, compiledTarget!.heightMeters);
}

function drawDebug(result: TrackerResult): void {
  const ctx = debugCanvas.getContext('2d')!;
  ctx.clearRect(0, 0, procW, procH);
  if (!debugToggle.checked) return;

  if (result.corners) {
    ctx.strokeStyle = '#00e5a0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(result.corners[0].x, result.corners[0].y);
    for (let i = 1; i <= 4; i++) ctx.lineTo(result.corners[i % 4].x, result.corners[i % 4].y);
    ctx.stroke();
  }
  ctx.fillStyle = result.state === 'tracking' ? '#ffd166' : '#f87171';
  for (const p of result.trackedPoints) {
    ctx.fillRect(p.x - 1.5, p.y - 1.5, 3, 3);
  }
}

function updateStatus(result: TrackerResult): void {
  const stateJa = result.state === 'tracking' ? 'トラッキング中' : 'ターゲット検索中';
  statusEl.textContent =
    `${stateJa} | 追跡点: ${result.inlierCount} | ${fps.toFixed(0)} fps\n` +
    `ターゲット特徴点: ${compiledTarget ? compiledTarget.points.length / 2 : 0}`;
}

function loop(now: number): void {
  requestAnimationFrame(loop);
  if (!tracker || !renderer || video.readyState < 2) return;

  const timeSec = now / 1000;
  const dt = lastFrameTime > 0 ? Math.min(0.1, timeSec - lastFrameTime) : 1 / 60;
  lastFrameTime = timeSec;

  // Capture the frame once: the background canvas and the tracking input are
  // the same instant, so the overlay never lags behind the visible image.
  bgCtx.drawImage(video, 0, 0, bgCanvas.width, bgCanvas.height);
  procCtx.drawImage(video, 0, 0, procW, procH);
  const rgba = procCtx.getImageData(0, 0, procW, procH).data;
  grayBuffer = rgbaToGray(rgba, procW, procH, grayBuffer);

  const result = tracker.processFrame(grayBuffer);
  renderer.updatePose(result.pose, timeSec, dt * 1.2);
  drawDebug(result);

  fpsCounter++;
  if (timeSec - fpsWindowStart >= 1) {
    fps = fpsCounter / (timeSec - fpsWindowStart);
    fpsCounter = 0;
    fpsWindowStart = timeSec;
  }
  updateStatus(result);
}

async function start(): Promise<void> {
  startButton.disabled = true;
  statusEl.textContent = 'カメラ起動中...';
  try {
    if (!compiledTarget) {
      compiledTarget = compileFromCanvas(createSampleTargetCanvas(TARGET_COMPILE_SIZE));
    }
    await startCamera();
    setupProcessing();
    startOverlay.classList.add('hidden');
    requestAnimationFrame(loop);
  } catch (err) {
    statusEl.textContent = `カメラを起動できませんでした: ${(err as Error).message}`;
    startButton.disabled = false;
    alert(`カメラを起動できませんでした: ${(err as Error).message}\nHTTPS でアクセスしているか確認してください。`);
  }
}

startButton.addEventListener('click', () => void start());

uploadInput.addEventListener('change', () => {
  const file = uploadInput.files?.[0];
  if (!file) return;
  const url = URL.createObjectURL(file);
  const img = new Image();
  img.onload = () => {
    URL.revokeObjectURL(url);
    compiledTarget = compileFromImage(img);
    if (tracker && renderer) {
      // Rebuild the tracker against the new target while the camera runs.
      tracker = new ImageTracker(compiledTarget, procW, procH);
      renderer.setTargetSize(compiledTarget.widthMeters, compiledTarget.heightMeters);
    }
    const n = compiledTarget.points.length / 2;
    statusEl.textContent = `カスタムターゲットを登録しました（特徴点 ${n} 個）`;
  };
  img.src = url;
});

statusEl.textContent = 'ボタンを押してカメラを起動してください';
