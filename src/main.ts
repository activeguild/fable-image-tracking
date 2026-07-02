/**
 * App entry point. The heavy lifting (feature detection, matching, optical
 * flow) runs in a Web Worker with WASM kernels; the main thread only captures
 * frames, extrapolates the latest pose to the render timestamp and draws the
 * Three.js overlay at display rate.
 */

import { rgbaToGray } from './core/imageops';
import { PosePredictor } from './core/predictor';
import { defaultIntrinsics } from './tracker/tracker';
import type { ReadyMessage, ResultMessage } from './tracker/worker';
import { ARRenderer } from './render/renderer';
import { createSampleTargetCanvas } from './sampleTarget';

const PROC_WIDTH = 360; // processing resolution (width); height follows aspect
const TARGET_COMPILE_SIZE = 384;
const TARGET_WIDTH_METERS = 0.2;

const container = document.getElementById('ar-container') as HTMLDivElement;
const video = document.getElementById('camera') as HTMLVideoElement;
const glCanvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
const debugCanvas = document.getElementById('debug-canvas') as HTMLCanvasElement;
const statusEl = document.getElementById('status') as HTMLDivElement;
const startOverlay = document.getElementById('start-overlay') as HTMLDivElement;
const startButton = document.getElementById('start-button') as HTMLButtonElement;
const uploadInput = document.getElementById('target-upload') as HTMLInputElement;
const debugToggle = document.getElementById('debug-toggle') as HTMLInputElement;
const contentSelect = document.getElementById('content-select') as HTMLSelectElement;
const contentFile = document.getElementById('content-file') as HTMLInputElement;

const worker = new Worker(new URL('./tracker/worker.ts', import.meta.url), { type: 'module' });
const predictor = new PosePredictor();

let renderer: ARRenderer | null = null;
let procCanvas: HTMLCanvasElement;
let procCtx: CanvasRenderingContext2D;
let procW = 0;
let procH = 0;

let running = false;
let workerBusy = false;
let workerReady = false;
let engineName = '-';
let targetFeatures = 0;
const bufferPool: ArrayBuffer[] = [];

let lastResult: ResultMessage | null = null;
let lastFrameTime = 0;
let pendingTargetCanvas: HTMLCanvasElement | null = null;

// FPS accounting: render (rAF) and processing (worker results) separately.
let renderFps = 0;
let procFps = 0;
let renderCount = 0;
let procCount = 0;
let fpsWindowStart = 0;

function compileGrayFromCanvas(canvas: HTMLCanvasElement): { gray: Uint8Array; w: number; h: number } {
  const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
  const { width, height } = canvas;
  const rgba = ctx.getImageData(0, 0, width, height).data;
  return { gray: rgbaToGray(rgba, width, height), w: width, h: height };
}

function sendInit(canvas: HTMLCanvasElement): void {
  const { gray, w, h } = compileGrayFromCanvas(canvas);
  workerReady = false;
  worker.postMessage(
    {
      type: 'init',
      gray: gray.buffer,
      width: w,
      height: h,
      widthMeters: TARGET_WIDTH_METERS,
      procWidth: procW,
      procHeight: procH,
    },
    [gray.buffer]
  );
}

worker.onmessage = (event: MessageEvent<ReadyMessage | ResultMessage>) => {
  const msg = event.data;
  if (msg.type === 'ready') {
    workerReady = true;
    engineName = msg.engine;
    targetFeatures = msg.featureCount;
    renderer?.setTargetSize(msg.widthMeters, msg.heightMeters);
    return;
  }
  if (msg.type === 'result') {
    workerBusy = false;
    bufferPool.push(msg.buffer);
    procCount++;
    lastResult = msg;
    if (msg.pose) {
      predictor.addSample({ R: msg.pose.R, t: msg.pose.t }, msg.t / 1000);
    } else {
      predictor.clear();
    }
  }
};

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

  renderer = new ARRenderer(container, video, glCanvas, debugCanvas);
  renderer.setVideoSize(vw, vh);
  renderer.setIntrinsics(defaultIntrinsics(procW, procH), procW, procH);
  renderer.setDebugVisible(debugToggle.checked);
}

function captureAndSend(nowMs: number): void {
  procCtx.drawImage(video, 0, 0, procW, procH);
  const rgba = procCtx.getImageData(0, 0, procW, procH).data;
  const buffer = bufferPool.pop() ?? new ArrayBuffer(procW * procH);
  const gray = new Uint8Array(buffer);
  rgbaToGray(rgba, procW, procH, gray);
  worker.postMessage({ type: 'frame', buffer, t: nowMs }, [buffer]);
  workerBusy = true;
}

function drawDebug(): void {
  const ctx = debugCanvas.getContext('2d')!;
  ctx.clearRect(0, 0, procW, procH);
  if (!debugToggle.checked || !lastResult) return;
  const r = lastResult;

  if (r.corners) {
    ctx.strokeStyle = '#00e5a0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(r.corners[0].x, r.corners[0].y);
    for (let i = 1; i <= 4; i++) ctx.lineTo(r.corners[i % 4].x, r.corners[i % 4].y);
    ctx.stroke();
  }
  ctx.fillStyle = r.state === 'tracking' ? '#ffd166' : '#f87171';
  for (const p of r.points) {
    ctx.fillRect(p.x - 1.5, p.y - 1.5, 3, 3);
  }
}

function updateStatus(): void {
  const state = lastResult?.state === 'tracking' ? 'トラッキング中' : 'ターゲット検索中';
  const inliers = lastResult?.inlierCount ?? 0;
  statusEl.textContent =
    `${state} | 追跡点: ${inliers} | 描画 ${renderFps.toFixed(0)} fps / 処理 ${procFps.toFixed(0)} fps\n` +
    `エンジン: ${engineName} | ターゲット特徴点: ${targetFeatures}`;
}

function loop(now: number): void {
  requestAnimationFrame(loop);
  if (!running || !renderer || video.readyState < 2) return;

  const timeSec = now / 1000;
  const dt = lastFrameTime > 0 ? Math.min(0.1, timeSec - lastFrameTime) : 1 / 60;
  lastFrameTime = timeSec;

  // Feed the worker whenever it is idle (single frame in flight).
  if (workerReady && !workerBusy) captureAndSend(performance.now());

  // Render at display rate with the pose extrapolated to "now".
  const pose = predictor.predict(performance.now() / 1000);
  renderer.updatePose(pose, timeSec, dt * 1.2);
  drawDebug();

  renderCount++;
  if (timeSec - fpsWindowStart >= 1) {
    const span = timeSec - fpsWindowStart;
    renderFps = renderCount / span;
    procFps = procCount / span;
    renderCount = 0;
    procCount = 0;
    fpsWindowStart = timeSec;
  }
  updateStatus();
}

async function start(): Promise<void> {
  startButton.disabled = true;
  statusEl.textContent = 'カメラ起動中...';
  try {
    await startCamera();
    setupProcessing();
    sendInit(pendingTargetCanvas ?? createSampleTargetCanvas(TARGET_COMPILE_SIZE));
    startOverlay.classList.add('hidden');
    running = true;
    requestAnimationFrame(loop);
  } catch (err) {
    statusEl.textContent = `カメラを起動できませんでした: ${(err as Error).message}`;
    startButton.disabled = false;
    alert(`カメラを起動できませんでした: ${(err as Error).message}\nHTTPS でアクセスしているか確認してください。`);
  }
}

startButton.addEventListener('click', () => void start());

// ---------------------------------------------------------------- content UI

let contentVideo: HTMLVideoElement | null = null;
let contentVideoUrl: string | null = null;
let lastContentValue = 'cube';

function cleanupContentVideo(): void {
  if (contentVideo) {
    contentVideo.pause();
    contentVideo.removeAttribute('src');
    contentVideo = null;
  }
  if (contentVideoUrl) {
    URL.revokeObjectURL(contentVideoUrl);
    contentVideoUrl = null;
  }
}

debugToggle.addEventListener('change', () => renderer?.setDebugVisible(debugToggle.checked));

contentSelect.addEventListener('change', () => {
  if (contentSelect.value === 'cube') {
    cleanupContentVideo();
    renderer?.setContent({ type: 'cube' });
    lastContentValue = 'cube';
    return;
  }
  contentFile.accept = contentSelect.value === 'image' ? 'image/*' : 'video/*';
  contentFile.value = '';
  contentFile.click();
});

contentFile.addEventListener('cancel', () => {
  contentSelect.value = lastContentValue;
});

contentFile.addEventListener('change', () => {
  const file = contentFile.files?.[0];
  if (!file) {
    contentSelect.value = lastContentValue;
    return;
  }
  const url = URL.createObjectURL(file);
  if (contentSelect.value === 'image') {
    const img = new Image();
    img.onload = () => {
      URL.revokeObjectURL(url);
      cleanupContentVideo();
      renderer?.setContent({ type: 'image', source: img });
      lastContentValue = 'image';
    };
    img.onerror = () => {
      URL.revokeObjectURL(url);
      contentSelect.value = lastContentValue;
      alert('画像を読み込めませんでした');
    };
    img.src = url;
  } else {
    cleanupContentVideo();
    const vid = document.createElement('video');
    vid.loop = true;
    vid.muted = true;
    vid.playsInline = true;
    vid.addEventListener(
      'loadeddata',
      () => {
        renderer?.setContent({ type: 'video', source: vid });
        lastContentValue = 'video';
      },
      { once: true }
    );
    vid.addEventListener('error', () => {
      cleanupContentVideo();
      contentSelect.value = lastContentValue;
      alert('動画を読み込めませんでした');
    });
    vid.src = url;
    contentVideo = vid;
    contentVideoUrl = url;
    void vid.play();
  }
});

uploadInput.addEventListener('change', () => {
  const file = uploadInput.files?.[0];
  if (!file) return;
  const url = URL.createObjectURL(file);
  const img = new Image();
  img.onload = () => {
    URL.revokeObjectURL(url);
    // Clamp so the compiled image fits the WASM engine's 512x512 budget.
    const scale = Math.min(1, 420 / img.naturalWidth, 480 / img.naturalHeight);
    const w = Math.round(img.naturalWidth * scale);
    const h = Math.round(img.naturalHeight * scale);
    const canvas = document.createElement('canvas');
    canvas.width = w;
    canvas.height = h;
    canvas.getContext('2d')!.drawImage(img, 0, 0, w, h);
    if (running) {
      predictor.clear();
      sendInit(canvas);
      statusEl.textContent = 'カスタムターゲットを登録中...';
    } else {
      pendingTargetCanvas = canvas;
      statusEl.textContent = 'カスタムターゲットを選択しました（カメラ起動後に有効）';
    }
  };
  img.src = url;
});

statusEl.textContent = 'ボタンを押してカメラを起動してください';
