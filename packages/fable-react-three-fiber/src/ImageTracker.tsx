/**
 * Anchors its children to the tracked image target. The group's transform is
 * updated per tracking result (frame-synchronized with the camera canvas)
 * with One-Euro position smoothing and velocity-adaptive quaternion
 * smoothing: estimation noise is crushed while the target is steady, real
 * motion passes through immediately.
 *
 * Coordinate frame: the target center is the origin, x right, y up toward
 * the target's top edge, z out of the target toward the viewer, in meters
 * (target width = `targetWidthMeters` on <FableCanvas>).
 */

import { useEffect, useRef, type ReactNode } from 'react';
import { Group, Matrix4, Quaternion, Vector3 } from 'three';
import { useFable } from './context';
import { Vector3Filter } from './engine/core/filter';
import type { FableFrame } from './engine/FableEngine';

export interface ImageTrackerProps {
  /** Set false to hide the anchor and suspend callbacks. Default true. */
  enabled?: boolean;
  /** Fires when the target becomes visible (found / confidence recovered). */
  onFound?: (frame: FableFrame) => void;
  /** Fires on every tracking update while visible. */
  onUpdated?: (frame: FableFrame) => void;
  /** Fires when the target is lost (or confidence-gated out). */
  onLost?: () => void;
  children?: ReactNode;
}

export function ImageTracker({ enabled = true, onFound, onUpdated, onLost, children }: ImageTrackerProps): ReactNode {
  const groupRef = useRef<Group>(null);
  const { onFrame } = useFable();
  const callbacksRef = useRef({ enabled, onFound, onUpdated, onLost });
  callbacksRef.current = { enabled, onFound, onUpdated, onLost };

  useEffect(() => {
    const group = groupRef.current;
    if (!group) return;
    group.matrixAutoUpdate = false;
    group.visible = false;

    // One-Euro position filter + velocity-adaptive quaternion smoothing
    // (same scheme as the reference app renderer).
    const positionFilter = new Vector3Filter(0.6, 3.0, 1.0);
    const smoothedQuat = new Quaternion();
    let quatVelocity = 0;
    let hasPose = false;
    let lastUpdateTime = -1;
    let shown = false;

    const matrix = new Matrix4();
    const position = new Vector3();
    const quat = new Quaternion();
    const scale = new Vector3();

    const hide = () => {
      if (shown) {
        shown = false;
        group.visible = false;
        callbacksRef.current.onLost?.();
      }
      if (hasPose) {
        positionFilter.reset();
        hasPose = false;
      }
    };

    const off = onFrame((frame) => {
      if (!callbacksRef.current.enabled) {
        hide();
        return;
      }
      // The confidence gate hides weak tracking with hysteresis; a coasting
      // gap (visible but momentarily no pose) keeps the last transform.
      if (!frame.visible) {
        hide();
        return;
      }
      if (frame.pose) {
        const timeSec = frame.t / 1000;
        const { R, t } = frame.pose;
        // CV camera (x right, y down, z forward) -> three.js camera:
        // flip y and z (C = diag(1,-1,-1)).
        matrix.set(
          R[0], R[1], R[2], t[0],
          -R[3], -R[4], -R[5], -t[1],
          -R[6], -R[7], -R[8], -t[2],
          0, 0, 0, 1
        );
        matrix.decompose(position, quat, scale);
        const [fx, fy, fz] = positionFilter.filter([position.x, position.y, position.z], timeSec);
        if (!hasPose) {
          smoothedQuat.copy(quat);
          quatVelocity = 0;
          hasPose = true;
        } else {
          const dt = Math.min(0.1, Math.max(1e-3, timeSec - lastUpdateTime));
          const angle = smoothedQuat.angleTo(quat);
          quatVelocity += (angle / dt - quatVelocity) * Math.min(1, dt * 5);
          const cutoff = 0.5 + 3.0 * quatVelocity;
          const alpha = Math.min(1, 1 - Math.exp(-2 * Math.PI * cutoff * dt));
          smoothedQuat.slerp(quat, alpha);
        }
        lastUpdateTime = timeSec;
        group.matrix.compose(new Vector3(fx, fy, fz), smoothedQuat, new Vector3(1, 1, 1));
      }
      group.visible = true;
      if (!shown) {
        shown = true;
        callbacksRef.current.onFound?.(frame);
      }
      callbacksRef.current.onUpdated?.(frame);
    });
    return () => {
      off();
    };
  }, [onFrame]);

  return <group ref={groupRef}>{children}</group>;
}
