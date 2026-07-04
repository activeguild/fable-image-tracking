/**
 * Framework-agnostic controller around the tracking worker: camera capture,
 * frame-synchronized display and result fan-out. The React components are
 * thin wrappers over this class.
 *
 * Display is frame-synchronized (the approach commercial engines use): each
 * captured camera frame is buffered at full resolution and blitted to the
 * visible camera canvas exactly when the tracker result for it arrives, in
 * the same paint as the anchor update. Overlay and camera pixels always
 * belong to the same instant, so latency never appears as marker slip.
 */

import { rgbaToGray } from './core/imageops';
import { GyroCollector } from './gyro';
import type { ReadyMessage, ResultMessage } from './tracker/worker';

export type TargetSource = string | HTMLImageElement | HTMLCanvasElement;

export interface FableEngineOptions {
  /** Tracking target: image URL or an already-loaded image/canvas. */
  target: TargetSource;
  video: HTMLVideoElement;
  cameraCanvas: HTMLCanvasElement;
  /** URL of the WASM kernels (a package asset). Default '/tracker.wasm'. */
  wasmSrc?: string;
  /** Use the gyroscope as a motion prior (requests permission on iOS). */
  imu?: boolean;
  /**
   * Target width in scene units. Default 1, so 3D coordinates are relative
   * to the marker: 1 unit = one marker width. Monocular tracking has no
   * absolute scale, so this only sets the scene's scale convention; pass
   * the physical width in meters if you prefer metric units.
   */
  targetWidthMeters?: number;
  /** Processing resolution (width in px). Default 360. */
  procWidth?: number;
}

export interface FableTargetInfo {
  engine: string;
  featureCount: number;
  widthMeters: number;
  heightMeters: number;
  targetWidthPx: number;
  targetHeightPx: number;
}

/** Per-result tracking snapshot delivered to subscribers. */
export interface FableFrame {
  /** Capture timestamp of the processed frame (performance.now(), ms). */
  t: number;
  state: 'searching' | 'tracking';
  /**
   * Confidence-gated visibility: false while tracking is too weak to trust
   * (hysteresis on the tracked-point count), so content neither flickers on
   * single-frame dropouts nor slides around on bad measurements.
   */
  visible: boolean;
  pose: { R: number[]; t: [number, number, number] } | null;
  corners: { x: number; y: number }[] | null;
  points: { x: number; y: number }[];
  inlierCount: number;
  /** Self-calibrated focal length in processing pixels. */
  fx: number;
  /** Self-calibrated radial distortion coefficient. */
  k1: number;
  procWidth: number;
  procHeight: number;
}

const HIDE_BELOW_POINTS = 18;
const SHOW_ABOVE_POINTS = 32;

export class FableEngine {
  private readonly video: HTMLVideoElement;
  private readonly camCanvas: HTMLCanvasElement;
  private camCtx: CanvasRenderingContext2D | null = null;
  private readonly options: Required<Pick<FableEngineOptions, 'wasmSrc' | 'imu' | 'targetWidthMeters' | 'procWidth'>> &
    FableEngineOptions;

  private worker: Worker | null = null;
  private gyro = new GyroCollector();
  private gyroOn = false;
  private lastCaptureMs = 0;

  private procCanvas!: HTMLCanvasElement;
  private procCtx!: CanvasRenderingContext2D;
  private procW = 0;
  private procH = 0;
  private frameBufCanvas!: HTMLCanvasElement;
  private frameBufCtx!: CanvasRenderingContext2D;
  private lastBlitMs = 0;

  private running = false;
  private rafId = 0;
  private workerBusy = false;
  private workerReady = false;
  private readonly bufferPool: ArrayBuffer[] = [];

  private contentShown = false;
  private weakStreak = 0;
  private strongStreak = 0;

  private frameListeners = new Set<(frame: FableFrame) => void>();
  private readyListeners = new Set<(info: FableTargetInfo) => void>();
  targetInfo: FableTargetInfo | null = null;
  videoWidth = 0;
  videoHeight = 0;

  constructor(options: FableEngineOptions) {
    this.video = options.video;
    this.camCanvas = options.cameraCanvas;
    this.options = {
      wasmSrc: '/tracker.wasm',
      imu: false,
      targetWidthMeters: 1,
      procWidth: 360,
      ...options,
    };
  }

  onFrame(listener: (frame: FableFrame) => void): () => void {
    this.frameListeners.add(listener);
    return () => this.frameListeners.delete(listener);
  }

  onReady(listener: (info: FableTargetInfo) => void): () => void {
    this.readyListeners.add(listener);
    if (this.targetInfo) listener(this.targetInfo);
    return () => this.readyListeners.delete(listener);
  }

  /**
   * Requests camera (and optionally motion) permission, compiles the target
   * and starts the capture loop. Call from a user gesture on iOS.
   */
  async start(): Promise<void> {
    if (this.running) return;
    // Permission requests must start inside the user gesture on iOS.
    const gyroPromise = this.options.imu
      ? this.gyro.start().then((ok) => {
          this.gyroOn = ok;
        })
      : Promise.resolve();
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: 'environment', width: { ideal: 1280 }, height: { ideal: 720 } },
      audio: false,
    });
    this.video.srcObject = stream;
    await this.video.play();
    await new Promise<void>((resolve) => {
      if (this.video.videoWidth > 0) resolve();
      else this.video.addEventListener('loadedmetadata', () => resolve(), { once: true });
    });
    await gyroPromise;

    const vw = (this.videoWidth = this.video.videoWidth);
    const vh = (this.videoHeight = this.video.videoHeight);
    this.procW = this.options.procWidth;
    this.procH = Math.round((vh / vw) * this.procW);

    this.procCanvas = document.createElement('canvas');
    this.procCanvas.width = this.procW;
    this.procCanvas.height = this.procH;
    this.procCtx = this.procCanvas.getContext('2d', { willReadFrequently: true })!;
    this.frameBufCanvas = document.createElement('canvas');
    this.frameBufCanvas.width = vw;
    this.frameBufCanvas.height = vh;
    this.frameBufCtx = this.frameBufCanvas.getContext('2d')!;
    this.camCanvas.width = vw;
    this.camCanvas.height = vh;
    this.camCtx = this.camCanvas.getContext('2d')!;

    this.worker = new Worker(new URL('./tracker/worker.ts', import.meta.url), { type: 'module' });
    this.worker.onmessage = (event: MessageEvent<ReadyMessage | ResultMessage>) => this.onWorkerMessage(event.data);
    const targetCanvas = await resolveTarget(this.options.target);
    this.sendInit(targetCanvas);

    this.running = true;
    this.rafId = requestAnimationFrame(this.loop);
  }

  /** Stops the loop, camera stream and worker. The instance is not reusable. */
  stop(): void {
    this.running = false;
    cancelAnimationFrame(this.rafId);
    this.worker?.terminate();
    this.worker = null;
    const stream = this.video.srcObject as MediaStream | null;
    stream?.getTracks().forEach((track) => track.stop());
    this.video.srcObject = null;
  }

  private sendInit(canvas: HTMLCanvasElement): void {
    const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
    const rgba = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    const gray = rgbaToGray(rgba, canvas.width, canvas.height);
    this.workerReady = false;
    this.worker!.postMessage(
      {
        type: 'init',
        gray: gray.buffer,
        width: canvas.width,
        height: canvas.height,
        widthMeters: this.options.targetWidthMeters,
        procWidth: this.procW,
        procHeight: this.procH,
        wasmSrc: this.options.wasmSrc,
      },
      [gray.buffer]
    );
  }

  private onWorkerMessage(msg: ReadyMessage | ResultMessage): void {
    if (msg.type === 'ready') {
      this.workerReady = true;
      this.targetInfo = {
        engine: msg.engine,
        featureCount: msg.featureCount,
        widthMeters: msg.widthMeters,
        heightMeters: msg.heightMeters,
        targetWidthPx: msg.targetWidthPx,
        targetHeightPx: msg.targetHeightPx,
      };
      for (const listener of this.readyListeners) listener(this.targetInfo);
      return;
    }
    this.workerBusy = false;
    this.bufferPool.push(msg.buffer);
    this.updateConfidenceGate(msg);
    // Frame sync: show the processed frame in the same paint as the anchor
    // update that subscribers make from this callback.
    if (this.camCtx) {
      this.camCtx.drawImage(this.frameBufCanvas, 0, 0, this.videoWidth, this.videoHeight);
      this.lastBlitMs = performance.now();
    }
    const frame: FableFrame = {
      t: msg.t,
      state: msg.state,
      visible: this.contentShown,
      pose: msg.pose,
      corners: msg.corners,
      points: msg.points,
      inlierCount: msg.inlierCount,
      fx: msg.fx,
      k1: msg.k1,
      procWidth: this.procW,
      procHeight: this.procH,
    };
    for (const listener of this.frameListeners) listener(frame);
  }

  private updateConfidenceGate(msg: ResultMessage): void {
    const weak = msg.state !== 'tracking' || msg.inlierCount < HIDE_BELOW_POINTS;
    const strong = msg.state === 'tracking' && msg.inlierCount >= SHOW_ABOVE_POINTS;
    this.weakStreak = weak ? this.weakStreak + 1 : 0;
    this.strongStreak = strong ? this.strongStreak + 1 : 0;
    if (this.contentShown && this.weakStreak >= 2) this.contentShown = false;
    else if (!this.contentShown && this.strongStreak >= 2) this.contentShown = true;
  }

  private captureAndSend(nowMs: number): void {
    // Buffer the full-resolution frame first, then derive the processing
    // image from that same buffer so display and measurements can never
    // come from different camera frames.
    this.frameBufCtx.drawImage(this.video, 0, 0);
    this.procCtx.drawImage(this.frameBufCanvas, 0, 0, this.procW, this.procH);
    const rgba = this.procCtx.getImageData(0, 0, this.procW, this.procH).data;
    const buffer = this.bufferPool.pop() ?? new ArrayBuffer(this.procW * this.procH);
    const gray = new Uint8Array(buffer);
    rgbaToGray(rgba, this.procW, this.procH, gray);
    const gyroDelta = this.gyroOn && this.lastCaptureMs > 0 ? this.gyro.delta(this.lastCaptureMs, nowMs) : null;
    this.lastCaptureMs = nowMs;
    this.worker!.postMessage({ type: 'frame', buffer, t: nowMs, gyro: gyroDelta }, [buffer]);
    this.workerBusy = true;
  }

  private loop = (): void => {
    if (!this.running) return;
    this.rafId = requestAnimationFrame(this.loop);
    if (this.video.readyState < 2) return;
    // Feed the worker whenever it is idle (single frame in flight).
    if (this.workerReady && !this.workerBusy) this.captureAndSend(performance.now());
    // The camera canvas is normally painted on worker results (frame sync);
    // fall back to the live video only when the pipeline is not producing
    // frames (before init, worker hiccup).
    if (this.camCtx && (!this.workerReady || performance.now() - this.lastBlitMs > 250)) {
      this.camCtx.drawImage(this.video, 0, 0, this.videoWidth, this.videoHeight);
    }
  };
}

async function resolveTarget(target: TargetSource): Promise<HTMLCanvasElement> {
  if (typeof target === 'string') {
    const img = new Image();
    img.crossOrigin = 'anonymous';
    await new Promise<void>((resolve, reject) => {
      img.onload = () => resolve();
      img.onerror = () => reject(new Error(`failed to load target image: ${target}`));
      img.src = target;
    });
    return imageToCanvas(img);
  }
  if (target instanceof HTMLCanvasElement) return target;
  return imageToCanvas(target);
}

function imageToCanvas(img: HTMLImageElement): HTMLCanvasElement {
  // Clamp so the compiled target fits the WASM engine's memory budget.
  const scale = Math.min(1, 420 / img.naturalWidth, 480 / img.naturalHeight);
  const w = Math.round(img.naturalWidth * scale);
  const h = Math.round(img.naturalHeight * scale);
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  canvas.getContext('2d')!.drawImage(img, 0, 0, w, h);
  return canvas;
}
