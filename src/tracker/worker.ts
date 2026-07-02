/**
 * Tracking worker: owns target compilation and the ImageTracker, preferring
 * the WASM kernels (with the pure-TS kernels as fallback). The main thread
 * streams grayscale frames in (transferable buffers, one in flight) and
 * receives poses stamped with the frame capture time; buffers are transferred
 * back for reuse.
 */

import { compileTarget } from './target';
import { ImageTracker } from './tracker';
import { jsKernels, type CVKernels } from '../core/kernels';
import { createWasmKernels } from '../wasm/engine';

export interface InitMessage {
  type: 'init';
  gray: ArrayBuffer;
  width: number;
  height: number;
  widthMeters: number;
  procWidth: number;
  procHeight: number;
}

export interface FrameMessage {
  type: 'frame';
  buffer: ArrayBuffer;
  t: number; // capture time (performance.now() in the main thread, ms)
}

export interface ReadyMessage {
  type: 'ready';
  engine: string;
  featureCount: number;
  widthMeters: number;
  heightMeters: number;
  /** Compiled target size in pixels (the coordinate frame of `corners`). */
  targetWidthPx: number;
  targetHeightPx: number;
}

export interface ResultMessage {
  type: 'result';
  t: number;
  procMs: number;
  state: 'searching' | 'tracking';
  pose: { R: number[]; t: [number, number, number] } | null;
  corners: { x: number; y: number }[] | null;
  points: { x: number; y: number }[];
  inlierCount: number;
  /** Current (self-calibrated) focal length in processing pixels. */
  fx: number;
  /** Current (self-calibrated) radial distortion coefficient. */
  k1: number;
  buffer: ArrayBuffer;
}

let kernels: CVKernels = jsKernels;
let tracker: ImageTracker | null = null;

const kernelsReady = (async () => {
  try {
    const response = await fetch('/tracker.wasm');
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const module = await WebAssembly.compile(await response.arrayBuffer());
    kernels = createWasmKernels(module);
  } catch (err) {
    console.warn('[tracker-worker] WASM kernels unavailable, using JS fallback:', err);
  }
})();

self.onmessage = async (event: MessageEvent<InitMessage | FrameMessage>) => {
  const msg = event.data;

  if (msg.type === 'init') {
    await kernelsReady;
    const gray = new Uint8Array(msg.gray);
    const target = compileTarget(gray, msg.width, msg.height, {
      widthMeters: msg.widthMeters,
      kernels,
    });
    tracker = new ImageTracker(target, msg.procWidth, msg.procHeight, { kernels });
    const ready: ReadyMessage = {
      type: 'ready',
      engine: kernels.name,
      featureCount: target.points.length / 2,
      widthMeters: target.widthMeters,
      heightMeters: target.heightMeters,
      targetWidthPx: target.width,
      targetHeightPx: target.height,
    };
    self.postMessage(ready);
    return;
  }

  if (msg.type === 'frame') {
    if (!tracker) return;
    const gray = new Uint8Array(msg.buffer);
    const start = performance.now();
    const r = tracker.processFrame(gray);
    const result: ResultMessage = {
      type: 'result',
      t: msg.t,
      procMs: performance.now() - start,
      state: r.state,
      pose: r.pose ? { R: r.pose.R, t: r.pose.t } : null,
      corners: r.corners,
      points: r.trackedPoints,
      inlierCount: r.inlierCount,
      fx: tracker.intrinsics.fx,
      k1: tracker.distortion.k1,
      buffer: msg.buffer,
    };
    (self as unknown as Worker).postMessage(result, [msg.buffer]);
  }
};
