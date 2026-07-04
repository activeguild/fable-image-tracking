/**
 * Matches the three.js camera projection to the tracker's (self-calibrating)
 * pinhole intrinsics, so rendered content lines up with the camera image.
 * The camera itself stays at the origin: <ImageTracker> moves the anchor.
 */

import { useThree } from '@react-three/fiber';
import { useEffect, useRef, type ReactNode } from 'react';
import { PerspectiveCamera } from 'three';
import { useFable } from './context';

export interface FableCameraProps {
  /** Manual vertical field of view (degrees); overrides self-calibration. */
  fov?: number;
  /** Fires once when the first tracking result arrives. */
  onFirstFrame?: () => void;
}

export function FableCamera({ fov, onFirstFrame }: FableCameraProps): ReactNode {
  const camera = useThree((state) => state.camera);
  const { onFrame } = useFable();
  const callbacksRef = useRef({ fov, onFirstFrame });
  callbacksRef.current = { fov, onFirstFrame };

  useEffect(() => {
    if (!(camera instanceof PerspectiveCamera)) return;
    // The tracker pose is relative to the physical camera: put the virtual
    // camera at the origin (r3f defaults to z=5) with AR-scale clip planes.
    camera.position.set(0, 0, 0);
    camera.quaternion.identity();
    camera.near = 0.01;
    camera.far = 100;
    camera.updateProjectionMatrix();
    let lastFx = 0;
    let first = true;
    return onFrame((frame) => {
      if (first) {
        first = false;
        callbacksRef.current.onFirstFrame?.();
      }
      const manualFov = callbacksRef.current.fov;
      if (manualFov !== undefined) {
        if (camera.fov !== manualFov) {
          camera.fov = manualFov;
          camera.updateProjectionMatrix();
        }
        return;
      }
      // Adopt the self-calibrated focal length (in processing pixels).
      if (Math.abs(frame.fx - lastFx) / (lastFx || 1) > 0.01) {
        lastFx = frame.fx;
        camera.fov = 2 * Math.atan(frame.procHeight / (2 * frame.fx)) * (180 / Math.PI);
        camera.updateProjectionMatrix();
      }
    });
  }, [camera, onFrame]);

  return null;
}
