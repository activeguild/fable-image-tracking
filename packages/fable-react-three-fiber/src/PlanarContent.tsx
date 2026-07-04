/**
 * A plane mesh pinned to the target with homography accuracy.
 *
 * To the consumer this is a normal react-three-fiber mesh: place it inside
 * <ImageTracker> and pass any material as children (meshStandardMaterial,
 * video textures, shaders...). Under the hood the subdivided plane's
 * vertices are nudged every frame so their screen projection matches the
 * measured homography exactly - the same accuracy as compositing a DOM
 * element with CSS matrix3d, but with real depth (occlusion against other
 * 3D content works) and real materials.
 *
 * Sizes/offsets are in scene units (the same units as sibling meshes;
 * default scale: target height = 2).
 */

import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef, type ReactNode } from 'react';
import { Matrix4, Mesh, PlaneGeometry, Vector3 } from 'three';
import { useFable } from './context';
import { QuadFilter } from './engine/core/quadfilter';
import { computeHomography, type Point2 } from './engine/core/homography';
import type { FableFrame } from './engine/FableEngine';

export interface PlanarContentProps {
  /** Plane width in scene units. Default: the full target width. */
  width?: number;
  /** Plane height in scene units. Default: the full target height. */
  height?: number;
  /** Placement in the target plane, scene units (0,0 = target center). */
  offset?: { x?: number; y?: number };
  /** Material element(s), e.g. <meshStandardMaterial map={...} />. */
  children?: ReactNode;
}

const GRID = 12;

export function PlanarContent({ width, height, offset, children }: PlanarContentProps): ReactNode {
  const meshRef = useRef<Mesh>(null);
  const camera = useThree((state) => state.camera);
  const { onFrame, targetInfo } = useFable();

  const geometry = useMemo(() => new PlaneGeometry(1, 1, GRID, GRID), []);
  // Undeformed vertex grid in the plane's unit square, captured once.
  const base = useMemo(() => {
    const pos = geometry.attributes.position;
    const out = new Float32Array(pos.count * 2);
    for (let i = 0; i < pos.count; i++) {
      out[i * 2] = pos.getX(i);
      out[i * 2 + 1] = pos.getY(i);
    }
    return out;
  }, [geometry]);
  useEffect(() => () => geometry.dispose(), [geometry]);

  // Latest measurement; geometry updates in useFrame, which runs after the
  // tracker's message handler has set the anchor group's matrix.
  const frameRef = useRef<FableFrame | null>(null);
  const dirtyRef = useRef(false);
  const quadFilter = useMemo(() => new QuadFilter(), []);
  useEffect(
    () =>
      onFrame((frame) => {
        frameRef.current = frame;
        dirtyRef.current = true;
      }),
    [onFrame]
  );

  const propsRef = useRef({ width, height, offset, targetInfo });
  propsRef.current = { width, height, offset, targetInfo };

  const scratch = useMemo(
    () => ({
      ray: new Vector3(),
      origin: new Vector3(),
      normal: new Vector3(),
      point: new Vector3(),
      inv: new Matrix4(),
    }),
    []
  );

  useFrame(() => {
    const mesh = meshRef.current;
    const frame = frameRef.current;
    const { targetInfo: info, width: w0, height: h0, offset: place } = propsRef.current;
    if (!mesh || !frame || !info || !dirtyRef.current) return;
    dirtyRef.current = false;
    if (frame.corners) {
      const weight = Math.min(1, Math.max(0.2, frame.inlierCount / 50));
      quadFilter.addSample(frame.corners, frame.t / 1000, weight);
    }
    const quad = quadFilter.predict(frame.t / 1000);
    if (!quad) return; // stale: ImageTracker hides the group anyway

    // Homography from plane scene-units to processing-frame pixels, from
    // the four target corners. Target pixel (0,0) is the top-left, scene
    // y is up.
    const tw = info.widthMeters;
    const th = info.heightMeters;
    const planeCorners: Point2[] = [
      { x: -tw / 2, y: th / 2 },
      { x: tw / 2, y: th / 2 },
      { x: tw / 2, y: -th / 2 },
      { x: -tw / 2, y: -th / 2 },
    ];
    const H = computeHomography(planeCorners, quad);
    if (!H) return;

    // Anchor plane in world (= camera) space, from the smoothed group
    // matrix the tracker just applied. Inverting through the same matrix
    // below means smoothing never shifts the final screen position.
    mesh.updateWorldMatrix(true, false);
    const world = mesh.parent ? mesh.parent.matrixWorld : mesh.matrixWorld;
    scratch.origin.set(0, 0, 0).applyMatrix4(world);
    scratch.normal.set(0, 0, 1).transformDirection(world);
    scratch.inv.copy(world).invert();

    const w = w0 ?? tw;
    const h = h0 ?? th;
    const ox = place?.x ?? 0;
    const oy = place?.y ?? 0;
    const pos = mesh.geometry.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      const xs = ox + base[i * 2] * w;
      const ys = oy + base[i * 2 + 1] * h;
      // Where the homography puts this plane point on screen...
      const denom = H[6] * xs + H[7] * ys + H[8];
      if (Math.abs(denom) < 1e-12) return;
      const u = (H[0] * xs + H[1] * ys + H[2]) / denom;
      const v = (H[3] * xs + H[4] * ys + H[5]) / denom;
      // ...the camera ray through that pixel...
      scratch.ray
        .set((2 * u) / frame.procWidth - 1, 1 - (2 * v) / frame.procHeight, 0.5)
        .unproject(camera);
      const dn = scratch.ray.dot(scratch.normal);
      if (Math.abs(dn) < 1e-9) return; // grazing view: keep last geometry
      // ...intersected with the anchor plane, back in anchor-local coords.
      const s = scratch.origin.dot(scratch.normal) / dn;
      if (s <= 0) return;
      scratch.point.copy(scratch.ray).multiplyScalar(s).applyMatrix4(scratch.inv);
      pos.setXYZ(i, scratch.point.x, scratch.point.y, scratch.point.z);
    }
    pos.needsUpdate = true;
    mesh.geometry.computeBoundingSphere();
  });

  return (
    <mesh ref={meshRef} geometry={geometry} frustumCulled={false}>
      {children}
    </mesh>
  );
}
