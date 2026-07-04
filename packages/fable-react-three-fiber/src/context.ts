import { createContext, useContext } from 'react';
import type { FableEngine, FableFrame, FableTargetInfo } from './engine/FableEngine';

export interface FableContextValue {
  engine: FableEngine | null;
  /** Compiled target metadata (null until the engine is ready). */
  targetInfo: FableTargetInfo | null;
  /** Whether the camera has been started. */
  started: boolean;
  /** Start the camera manually (for `autoStart={false}`; call in a gesture). */
  startCamera: () => Promise<void>;
  /** Subscribe to per-result tracking frames. Returns an unsubscriber. */
  onFrame: (listener: (frame: FableFrame) => void) => () => void;
  /**
   * Cover-fit placement of the camera image inside the container, in
   * container-local CSS pixels (null until the camera is running). Overlay
   * layers use it to map processing-frame coordinates to the screen.
   */
  coverRect: { left: number; top: number; width: number; height: number } | null;
}

export const FableContext = createContext<FableContextValue | null>(null);

/** Access the tracking engine state from anywhere inside <FableCanvas>. */
export function useFable(): FableContextValue {
  const ctx = useContext(FableContext);
  if (!ctx) throw new Error('useFable must be used inside <FableCanvas>');
  return ctx;
}
