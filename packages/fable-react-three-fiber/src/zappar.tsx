/**
 * Zappar compatibility layer: drop-in component names and props matching
 * @zappar/zappar-react-three-fiber, so existing Zappar apps migrate by
 * changing the import path (and swapping .zpt files for the original
 * target images).
 *
 * Unsupported Zappar features (face/instant tracking, sky effects,
 * user-facing camera) are accepted as props but ignored, with a one-time
 * console warning where silently ignoring would be surprising.
 */

import { useEffect, type ReactNode } from 'react';
import { FableCanvas, type FableCanvasProps } from './FableCanvas';
import { FableCamera, type FableCameraProps } from './FableCamera';
import { useFable } from './context';

const warned = new Set<string>();
function warnOnce(key: string, message: string): void {
  if (warned.has(key)) return;
  warned.add(key);
  console.warn(`[fable-react-three-fiber] ${message}`);
}

export interface ZapparCanvasProps extends FableCanvasProps {
  /** Accepted for compatibility; ignored. */
  legacyWebGL?: boolean;
  /** Accepted for compatibility; ignored. */
  colorManagement?: boolean;
}

/** Drop-in for Zappar's <ZapparCanvas>: a FableCanvas with the same shape. */
export function ZapparCanvas({ legacyWebGL, colorManagement, ...props }: ZapparCanvasProps): ReactNode {
  void legacyWebGL;
  void colorManagement;
  return <FableCanvas {...props} />;
}

export interface ZapparCameraProps extends FableCameraProps {
  /** Not supported: only the environment (rear) camera is available. */
  userFacing?: boolean;
  /** Accepted for compatibility; ignored (mirroring is not applied). */
  userCameraMirrorMode?: 'poses' | 'css' | 'none';
  /** Accepted for compatibility; ignored (mirroring is not applied). */
  rearCameraMirrorMode?: 'poses' | 'css' | 'none';
  /** Accepted for compatibility; ignored. */
  makeDefault?: boolean;
  /** Accepted for compatibility; ignored. */
  renderPriority?: number;
  /** Accepted for compatibility; ignored. */
  environmentMap?: boolean;
  /** Accepted for compatibility; ignored. */
  poseMode?: string;
  /** Accepted for compatibility; ignored. */
  poseAnchorOrigin?: unknown;
  /** Accepted for compatibility; ignored. */
  sources?: unknown;
  /** Accepted for compatibility; ignored. */
  pipeline?: unknown;
}

/** Drop-in for Zappar's <ZapparCamera>. */
export function ZapparCamera({ fov, onFirstFrame, userFacing, environmentMap, ...rest }: ZapparCameraProps): ReactNode {
  void rest;
  useEffect(() => {
    if (userFacing) {
      warnOnce('userFacing', 'userFacing is not supported - the environment (rear) camera is used.');
    }
    if (environmentMap) {
      warnOnce('environmentMap', 'environmentMap is not supported and was ignored.');
    }
  }, [userFacing, environmentMap]);
  return <FableCamera fov={fov} onFirstFrame={onFirstFrame} />;
}

/**
 * Drop-in for Zappar's <BrowserCompatibility>: renders its children only
 * when the browser cannot run the AR experience (no camera API / WebGL).
 */
export function BrowserCompatibility({ children }: { children?: ReactNode }): ReactNode {
  const supported =
    typeof navigator !== 'undefined' &&
    !!navigator.mediaDevices?.getUserMedia &&
    (() => {
      try {
        const canvas = document.createElement('canvas');
        return !!(canvas.getContext('webgl2') || canvas.getContext('webgl'));
      } catch {
        return false;
      }
    })();
  return supported ? null : <>{children}</>;
}

/**
 * Drop-in for Zappar's <Loader>: a minimal fullscreen spinner shown until
 * the target is compiled and the camera is running. Place it inside
 * <FableCanvas>/<ZapparCanvas>'s parent DOM (it is a DOM component).
 */
export function Loader(): ReactNode {
  const { targetInfo, started } = useFable();
  if (targetInfo && started) return null;
  return (
    <div
      style={{
        position: 'absolute',
        inset: 0,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        background: 'rgba(0,0,0,0.55)',
        color: '#fff',
        font: '14px system-ui',
        zIndex: 10,
        pointerEvents: 'none',
      }}
    >
      Loading...
    </div>
  );
}
