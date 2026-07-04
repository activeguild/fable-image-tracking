/**
 * Root component: starts the camera + tracking engine and layers the
 * frame-synchronized camera canvas under a transparent react-three-fiber
 * <Canvas>. Children (lights, <ImageTracker>, meshes) render inside the R3F
 * scene with the tracking context re-provided across the renderer boundary.
 */

import { Canvas } from '@react-three/fiber';
import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type CSSProperties,
  type ReactNode,
} from 'react';
import { FableContext, type FableContextValue } from './context';
import {
  FableEngine,
  type FableFrame,
  type FableTargetInfo,
  type TargetSource,
} from './engine/FableEngine';

export interface FableCanvasProps {
  /** Tracking target: image URL or an already-loaded image/canvas. */
  targetImage: TargetSource;
  /**
   * URL of the WASM kernels. Copy `assets/tracker.wasm` from this package to
   * your public directory. Defaults to '/tracker.wasm'; without it the
   * engine falls back to the (slower) pure-JS kernels.
   */
  wasmSrc?: string;
  /** Start the camera immediately. Default true. On iOS, `false` +
   * `startCamera()` from a tap handler avoids permission-prompt issues. */
  autoStart?: boolean;
  /** Use the gyroscope as a tracking motion prior. Default false. */
  imu?: boolean;
  /** Physical target width in meters (scales the 3D scene). Default 0.2. */
  targetWidthMeters?: number;
  style?: CSSProperties;
  className?: string;
  /** Device pixel ratio for the 3D canvas (react-three-fiber `dpr`). */
  dpr?: number | [number, number];
  onReady?: (info: FableTargetInfo) => void;
  onError?: (error: Error) => void;
  children?: ReactNode;
}

interface Rect {
  left: number;
  top: number;
  width: number;
  height: number;
}

export function FableCanvas({
  targetImage,
  wasmSrc,
  autoStart = true,
  imu = false,
  targetWidthMeters = 0.2,
  style,
  className,
  dpr,
  onReady,
  onError,
  children,
}: FableCanvasProps): ReactNode {
  const containerRef = useRef<HTMLDivElement>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const camCanvasRef = useRef<HTMLCanvasElement>(null);
  const engineRef = useRef<FableEngine | null>(null);
  const [started, setStarted] = useState(false);
  const [targetInfo, setTargetInfo] = useState<FableTargetInfo | null>(null);
  const [coverRect, setCoverRect] = useState<Rect | null>(null);
  // DOM layer between the camera canvas and the 3D canvas, where
  // <PlanarContent> mounts its media (via context, from anywhere in the
  // tree - including inside the R3F scene).
  const [overlayContainer, setOverlayContainer] = useState<HTMLDivElement | null>(null);

  // Latest-value refs so the stable startCamera callback never goes stale.
  const callbacksRef = useRef({ onReady, onError });
  callbacksRef.current = { onReady, onError };

  /** Cover-fit the camera image inside the container (like object-fit: cover). */
  const layout = useCallback(() => {
    const container = containerRef.current;
    const engine = engineRef.current;
    if (!container || !engine || engine.videoWidth === 0) return;
    const cw = container.clientWidth;
    const ch = container.clientHeight;
    if (cw === 0 || ch === 0) return;
    const scale = Math.max(cw / engine.videoWidth, ch / engine.videoHeight);
    const width = engine.videoWidth * scale;
    const height = engine.videoHeight * scale;
    setCoverRect({ left: (cw - width) / 2, top: (ch - height) / 2, width, height });
  }, []);

  const startCamera = useCallback(async () => {
    const engine = engineRef.current;
    if (!engine) return;
    try {
      await engine.start();
      setStarted(true);
      layout();
    } catch (err) {
      callbacksRef.current.onError?.(err as Error);
      throw err;
    }
  }, [layout]);

  useEffect(() => {
    const engine = new FableEngine({
      target: targetImage,
      video: videoRef.current!,
      cameraCanvas: camCanvasRef.current!,
      wasmSrc,
      imu,
      targetWidthMeters,
    });
    engineRef.current = engine;
    const offReady = engine.onReady((info) => {
      setTargetInfo(info);
      callbacksRef.current.onReady?.(info);
    });
    if (autoStart) {
      void engine
        .start()
        .then(() => {
          setStarted(true);
          layout();
        })
        .catch((err) => callbacksRef.current.onError?.(err as Error));
    }
    return () => {
      offReady();
      engine.stop();
      engineRef.current = null;
      setStarted(false);
      setTargetInfo(null);
    };
    // Recreate the engine when the target or camera options change.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [targetImage, wasmSrc, imu, targetWidthMeters, autoStart, layout]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver(layout);
    observer.observe(container);
    return () => observer.disconnect();
  }, [layout]);

  const contextValue = useMemo<FableContextValue>(
    () => ({
      engine: engineRef.current,
      targetInfo,
      started,
      startCamera,
      onFrame: (listener: (frame: FableFrame) => void) =>
        engineRef.current ? engineRef.current.onFrame(listener) : () => {},
      coverRect,
      overlayContainer,
    }),
    [targetInfo, started, startCamera, coverRect, overlayContainer]
  );

  const overlayStyle: CSSProperties = coverRect
    ? {
        position: 'absolute',
        left: coverRect.left,
        top: coverRect.top,
        width: coverRect.width,
        height: coverRect.height,
      }
    : { position: 'absolute', inset: 0 };

  return (
    <FableContext.Provider value={contextValue}>
      <div
        ref={containerRef}
        className={className}
        style={{ position: 'relative', overflow: 'hidden', background: '#000', ...style }}
      >
        {/* Frame source only; the visible image is the frame-synced canvas. */}
        <video ref={videoRef} autoPlay muted playsInline style={{ ...overlayStyle, visibility: 'hidden' }} />
        <canvas ref={camCanvasRef} style={overlayStyle} />
        <div
          ref={setOverlayContainer}
          style={{ position: 'absolute', inset: 0, overflow: 'hidden', pointerEvents: 'none' }}
        />
        <div style={overlayStyle}>
          <Canvas
            gl={{ alpha: true, antialias: true }}
            dpr={dpr}
            style={{ width: '100%', height: '100%' }}
          >
            {/* R3F children live in a separate React root: re-provide. */}
            <FableContext.Provider value={contextValue}>{children}</FableContext.Provider>
          </Canvas>
        </div>
      </div>
    </FableContext.Provider>
  );
}
