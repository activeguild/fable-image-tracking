/**
 * App entry point. The heavy lifting (feature detection, matching, optical
 * flow) runs in a Web Worker with WASM kernels.
 *
 * Display is frame-synchronized (the approach commercial engines use): each
 * captured camera frame is buffered at full resolution, and when the tracker
 * result for it arrives, the frame and the corners measured ON that frame are
 * shown in the same paint. Overlay and camera pixels always belong to the
 * same instant, so tracking latency delays the camera image slightly
 * (~one processing period) instead of appearing as marker slip.
 */

import { rgbaToGray } from './core/imageops';
import { PosePredictor } from './core/predictor';
import { QuadFilter } from './core/quadfilter';
import { defaultIntrinsics } from './tracker/tracker';
import type { ReadyMessage, ResultMessage } from './tracker/worker';
import { ARRenderer } from './render/renderer';
import { createSampleTargetCanvas } from './sampleTarget';
import { createSampleImageCanvas, sampleVideoUrl } from './sampleContent';
import { GyroCollector } from './gyro';
import { gyroHomography } from './core/imu';
import { applyHomography, type Point2 } from './core/homography';

const PROC_WIDTH = 360; // processing resolution (width); height follows aspect
const TARGET_COMPILE_SIZE = 384;
const TARGET_WIDTH_METERS = 0.2;

const container = document.getElementById('ar-container') as HTMLDivElement;
const video = document.getElementById('camera') as HTMLVideoElement;
const camCanvas = document.getElementById('cam-canvas') as HTMLCanvasElement;
const glCanvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
const debugCanvas = document.getElementById('debug-canvas') as HTMLCanvasElement;
const statusEl = document.getElementById('status') as HTMLDivElement;
const startOverlay = document.getElementById('start-overlay') as HTMLDivElement;
const startButton = document.getElementById('start-button') as HTMLButtonElement;
const uploadInput = document.getElementById('target-upload') as HTMLInputElement;
const debugToggle = document.getElementById('debug-toggle') as HTMLInputElement;
const imuToggle = document.getElementById('imu-toggle') as HTMLInputElement;
const contentSelect = document.getElementById('content-select') as HTMLSelectElement;

const worker = new Worker(new URL('./tracker/worker.ts', import.meta.url), { type: 'module' });
const predictor = new PosePredictor();
const quadFilter = new QuadFilter();
const gyro = new GyroCollector();
let gyroOn = false;
let lastCaptureMs = 0;

let renderer: ARRenderer | null = null;
let procCanvas: HTMLCanvasElement;
let procCtx: CanvasRenderingContext2D;
let procW = 0;
let procH = 0;
// Full-resolution copy of the frame currently being processed; blitted to the
// visible camera canvas when its tracker result arrives (frame sync). Safe as
// a single buffer because only one frame is ever in flight.
let frameBufCanvas: HTMLCanvasElement;
let frameBufCtx: CanvasRenderingContext2D;
let lastBlitMs = 0;

let running = false;
let workerBusy = false;
let workerReady = false;
let engineName = '-';
let targetFeatures = 0;
const bufferPool: ArrayBuffer[] = [];

let lastResult: ResultMessage | null = null;
let lastFrameTime = 0;
let lastFx = 1;
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
    renderer?.setTargetPixelSize(msg.targetWidthPx, msg.targetHeightPx);
    return;
  }
  if (msg.type === 'result') {
    workerBusy = false;
    bufferPool.push(msg.buffer);
    procCount++;
    lastResult = msg;
    updateConfidenceGate(msg);
    // A lost frame does NOT clear the filters: single-frame dropouts (motion
    // blur, brief occlusion) are bridged by coasting on the last measurement
    // until it goes stale (maxAge), instead of blinking the content off.
    if (msg.pose) predictor.addSample({ R: msg.pose.R, t: msg.pose.t }, msg.t / 1000);
    if (msg.corners) {
      // Confidence from the inlier count: blur/dim frames with few surviving
      // points get blended toward the motion prediction instead of trusted.
      const weight = Math.min(1, Math.max(0.2, msg.inlierCount / 50));
      quadFilter.addSample(msg.corners, msg.t / 1000, weight);
    }
    // Adopt the worker's self-calibrated focal length for the 3D camera.
    if (renderer && Math.abs(msg.fx - lastFx) / lastFx > 0.01) {
      lastFx = msg.fx;
      renderer.setIntrinsics({ fx: msg.fx, fy: msg.fx, cx: procW / 2, cy: procH / 2 }, procW, procH);
    }
    // Frame sync: show the processed frame together with the quad measured
    // on it. Evaluating the filter AT the sample time applies its smoothing
    // and glitch guards without any extrapolation (during dropouts the
    // sample is older and the filter coasts, with the gyro model if on).
    if (renderer) {
      renderer.drawCameraFrame(frameBufCanvas);
      lastBlitMs = performance.now();
      renderer.updatePlanarQuad(quadFilter.predict(msg.t / 1000, quadAdvance), procW, procH);
      drawDebug();
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

  frameBufCanvas = document.createElement('canvas');
  frameBufCanvas.width = vw;
  frameBufCanvas.height = vh;
  frameBufCtx = frameBufCanvas.getContext('2d')!;

  debugCanvas.width = procW;
  debugCanvas.height = procH;

  renderer = new ARRenderer(container, video, camCanvas, glCanvas, debugCanvas);
  renderer.setVideoSize(vw, vh);
  const K = defaultIntrinsics(procW, procH);
  lastFx = K.fx;
  renderer.setIntrinsics(K, procW, procH);
  renderer.setDebugVisible(debugToggle.checked);
  applyContent(contentSelect.value);
}

function captureAndSend(nowMs: number): void {
  // Buffer the full-resolution frame first, then derive the processing
  // image from that same buffer so display and measurements can never come
  // from different camera frames.
  frameBufCtx.drawImage(video, 0, 0);
  procCtx.drawImage(frameBufCanvas, 0, 0, procW, procH);
  const rgba = procCtx.getImageData(0, 0, procW, procH).data;
  const buffer = bufferPool.pop() ?? new ArrayBuffer(procW * procH);
  const gray = new Uint8Array(buffer);
  rgbaToGray(rgba, procW, procH, gray);
  const gyroDelta = gyroOn && lastCaptureMs > 0 ? gyro.delta(lastCaptureMs, nowMs) : null;
  lastCaptureMs = nowMs;
  worker.postMessage({ type: 'frame', buffer, t: nowMs, gyro: gyroDelta }, [buffer]);
  workerBusy = true;
}

// -------------------------------------------------------- confidence gate
// During violent motion the measurements degrade before they fail: rather
// than show content that is sliding off the target, fade it out while the
// tracked-point count is low and fade back once tracking is solid again.
// Hysteresis (hide < 18, show >= 32, both needing 2 consecutive results)
// keeps the opacity from pumping; single-frame dropouts are still bridged
// invisibly by the coasting logic.
const HIDE_BELOW_POINTS = 18;
const SHOW_ABOVE_POINTS = 32;
let contentShown = true;
let weakStreak = 0;
let strongStreak = 0;

function updateConfidenceGate(msg: ResultMessage): void {
  const weak = msg.state !== 'tracking' || msg.inlierCount < HIDE_BELOW_POINTS;
  const strong = msg.state === 'tracking' && msg.inlierCount >= SHOW_ABOVE_POINTS;
  weakStreak = weak ? weakStreak + 1 : 0;
  strongStreak = strong ? strongStreak + 1 : 0;
  if (contentShown && weakStreak >= 2) {
    contentShown = false;
    renderer?.setContentOpacity(0);
  } else if (!contentShown && strongStreak >= 2) {
    contentShown = true;
    renderer?.setContentOpacity(1);
  }
}

/**
 * Gyro-measured motion model for display-time quad prediction: rotate the
 * quad by the camera rotation actually measured between the two timestamps
 * instead of assuming constant velocity - no overshoot when the hand
 * reverses direction, and coasting stays glued during dropouts.
 */
function quadAdvance(corners: Point2[], fromSec: number, toSec: number): Point2[] {
  if (!gyroOn || !lastResult) return corners;
  const delta = gyro.delta(fromSec * 1000, toSec * 1000);
  if (!delta) return corners;
  const K = { fx: lastResult.fx, fy: lastResult.fx, cx: procW / 2, cy: procH / 2 };
  const Hg = gyroHomography(delta, K);
  if (!Hg) return corners;
  return corners.map((c) => applyHomography(Hg, c.x, c.y));
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
  const calib = lastResult ? ` | f:${lastResult.fx.toFixed(0)} k1:${lastResult.k1.toFixed(3)}` : '';
  statusEl.textContent =
    `${state} | 追跡点: ${inliers} | 描画 ${renderFps.toFixed(0)} fps / 処理 ${procFps.toFixed(0)} fps\n` +
    `エンジン: ${engineName} | IMU: ${gyroOn ? 'on' : 'off'} | 特徴点: ${targetFeatures}${calib}`;
}

function loop(now: number): void {
  requestAnimationFrame(loop);
  if (!running || !renderer || video.readyState < 2) return;

  const timeSec = now / 1000;
  const dt = lastFrameTime > 0 ? Math.min(0.1, timeSec - lastFrameTime) : 1 / 60;
  lastFrameTime = timeSec;

  // Feed the worker whenever it is idle (single frame in flight).
  if (workerReady && !workerBusy) captureAndSend(performance.now());

  // The camera canvas is normally painted on worker results (frame sync);
  // fall back to the live video only when the pipeline is not producing
  // frames (before init, re-registering a target, worker hiccup).
  if (!workerReady || performance.now() - lastBlitMs > 250) {
    renderer.drawCameraFrame(video);
  }

  // 3D content: hold the pose of the displayed (processed) frame; the
  // renderer's own smoothing converges on it between results.
  const sampleSec = lastResult ? lastResult.t / 1000 : timeSec;
  renderer.updatePose(predictor.predict(sampleSec), timeSec, dt * 1.2);

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
    // Two launch modes: with motion sensing (gyro fusion for fast-motion
    // priors and display prediction) or camera-only. The permission must be
    // requested inside the click gesture (iOS), so decide here.
    const gyroPromise = imuToggle.checked
      ? gyro.start().then((ok) => {
          gyroOn = ok;
        })
      : Promise.resolve();
    await startCamera();
    await gyroPromise;
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
let lastContentValue = 'image';

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

/**
 * The sample video is prefetched as a Blob at startup for two reasons:
 * - iOS Safari needs HTTP Range support to stream <video> sources, which the
 *   dev/static server lacks; a blob: URL sidesteps that.
 * - play() must be called synchronously inside the user gesture (in Low
 *   Power Mode iOS blocks even muted autoplay without one), so the media
 *   must already be available when the selector fires.
 */
let sampleVideoBlobUrl: string | null = null;
let sampleVideoFetch: Promise<string> | null = null;

function prefetchSampleVideo(): Promise<string> {
  if (!sampleVideoFetch) {
    sampleVideoFetch = fetch(sampleVideoUrl())
      .then((response) => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.blob();
      })
      .then((blob) => {
        sampleVideoBlobUrl = URL.createObjectURL(blob);
        return sampleVideoBlobUrl;
      });
  }
  return sampleVideoFetch;
}
void prefetchSampleVideo();

function startSampleVideo(url: string): void {
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
  vid.src = url; // shared cached blob URL; never revoked here
  contentVideo = vid;
  contentVideoUrl = null;
  vid.play()?.catch(() => showTapToPlay(vid));
}

/** Fallback when autoplay is blocked (e.g. iOS Low Power Mode). */
function showTapToPlay(vid: HTMLVideoElement): void {
  const hint = document.createElement('div');
  hint.className = 'tap-hint';
  hint.textContent = '画面をタップすると動画が再生されます';
  document.body.appendChild(hint);
  const resume = () => {
    hint.remove();
    if (contentVideo === vid) void vid.play();
  };
  document.addEventListener('pointerdown', resume, { once: true });
}

function applyContent(value: string): void {
  switch (value) {
    case 'video':
      if (sampleVideoBlobUrl) {
        // Synchronous path keeps the user-gesture context for play().
        startSampleVideo(sampleVideoBlobUrl);
      } else {
        prefetchSampleVideo()
          .then((url) => startSampleVideo(url))
          .catch(() => {
            contentSelect.value = lastContentValue;
            alert('動画を読み込めませんでした');
          });
      }
      break;
    case 'cube':
      cleanupContentVideo();
      renderer?.setContent({ type: 'cube' });
      lastContentValue = 'cube';
      break;
    default: // image
      cleanupContentVideo();
      renderer?.setContent({ type: 'image', source: createSampleImageCanvas() });
      lastContentValue = 'image';
  }
}

contentSelect.addEventListener('change', () => applyContent(contentSelect.value));

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
      quadFilter.clear();
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
