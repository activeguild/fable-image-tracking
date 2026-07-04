/**
 * Flat image/video content pinned by the measured homography (CSS matrix3d),
 * bypassing the 3-D pose and therefore any camera-intrinsics error: pinning
 * accuracy equals the tracker's own point-measurement accuracy. Use this for
 * media that lies on the target plane; use 3D meshes for volumetric content.
 *
 * Declare it anywhere inside <FableCanvas> - typically next to your 3D
 * content inside <ImageTracker>. It renders nothing in place: the media
 * element is mounted imperatively into FableCanvas's DOM overlay layer
 * (between the camera canvas and the 3D canvas), so the same component works
 * in the React DOM tree and inside the react-three-fiber scene.
 */

import { useEffect, useRef, type ReactNode } from 'react';
import { useFable } from './context';
import { QuadFilter } from './engine/core/quadfilter';
import { computeHomography, matMul3, type Mat3, type Point2 } from './engine/core/homography';

export interface PlanarContentProps {
  /** Media shown on the target: URL or an image/canvas/video element. */
  source: string | HTMLImageElement | HTMLCanvasElement | HTMLVideoElement;
}

export function PlanarContent({ source }: PlanarContentProps): ReactNode {
  const { onFrame, targetInfo, coverRect, overlayContainer } = useFable();

  // Latest layout/target info for the frame subscription without resubscribing.
  const layoutRef = useRef({ targetInfo, coverRect });
  layoutRef.current = { targetInfo, coverRect };

  useEffect(() => {
    if (!overlayContainer) return;

    let element: HTMLElement | null = null;
    let mediaW = 1;
    let mediaH = 1;
    let cancelled = false;
    const quadFilter = new QuadFilter();

    const attach = (el: HTMLImageElement | HTMLCanvasElement | HTMLVideoElement) => {
      if (cancelled) return;
      mediaW =
        el instanceof HTMLImageElement
          ? el.naturalWidth || 1
          : el instanceof HTMLVideoElement
            ? el.videoWidth || 16
            : el.width;
      mediaH =
        el instanceof HTMLImageElement
          ? el.naturalHeight || 1
          : el instanceof HTMLVideoElement
            ? el.videoHeight || 9
            : el.height;
      Object.assign(el.style, {
        position: 'absolute',
        left: '0',
        top: '0',
        width: `${mediaW}px`,
        height: `${mediaH}px`,
        maxWidth: 'none',
        transformOrigin: '0 0',
        pointerEvents: 'none',
        backfaceVisibility: 'hidden',
        willChange: 'transform, opacity',
        transition: 'opacity 0.12s linear',
        visibility: 'hidden',
      } satisfies Partial<CSSStyleDeclaration>);
      overlayContainer.appendChild(el);
      element = el;
    };

    if (typeof source === 'string') {
      const img = new Image();
      img.crossOrigin = 'anonymous';
      img.onload = () => attach(img);
      img.src = source;
    } else {
      attach(source);
    }

    const off = onFrame((frame) => {
      const el = element;
      if (!el) return;
      const { targetInfo: info, coverRect: rect } = layoutRef.current;
      if (frame.corners) {
        const weight = Math.min(1, Math.max(0.2, frame.inlierCount / 50));
        quadFilter.addSample(frame.corners, frame.t / 1000, weight);
      }
      const quad = quadFilter.predict(frame.t / 1000);
      if (!quad || !info || !rect) {
        el.style.visibility = 'hidden';
        return;
      }
      // Processing-frame coordinates -> container coordinates (cover fit).
      const sx = rect.width / frame.procWidth;
      const sy = rect.height / frame.procHeight;
      const screenCorners: Point2[] = quad.map((p) => ({
        x: rect.left + p.x * sx,
        y: rect.top + p.y * sy,
      }));
      const targetRect: Point2[] = [
        { x: 0, y: 0 },
        { x: info.targetWidthPx, y: 0 },
        { x: info.targetWidthPx, y: info.targetHeightPx },
        { x: 0, y: info.targetHeightPx },
      ];
      const targetToScreen = computeHomography(targetRect, screenCorners);
      if (!targetToScreen) {
        el.style.visibility = 'hidden';
        return;
      }
      // Contain-fit the media inside the target rectangle, then compose.
      const fit = Math.min(info.targetWidthPx / mediaW, info.targetHeightPx / mediaH);
      const ox = (info.targetWidthPx - mediaW * fit) / 2;
      const oy = (info.targetHeightPx - mediaH * fit) / 2;
      const mediaToTarget: Mat3 = [fit, 0, ox, 0, fit, oy, 0, 0, 1];
      const h = matMul3(targetToScreen, mediaToTarget);
      if (Math.abs(h[8]) < 1e-12) {
        el.style.visibility = 'hidden';
        return;
      }
      for (let i = 0; i < 9; i++) h[i] /= h[8];
      el.style.transform =
        `matrix3d(${h[0]},${h[3]},0,${h[6]},` +
        `${h[1]},${h[4]},0,${h[7]},` +
        `0,0,1,0,` +
        `${h[2]},${h[5]},0,1)`;
      // Confidence fade: hide while tracking is too weak to trust.
      el.style.opacity = frame.visible ? '1' : '0';
      el.style.visibility = 'visible';
    });

    return () => {
      cancelled = true;
      off();
      element?.remove();
    };
  }, [source, onFrame, overlayContainer]);

  return null;
}
