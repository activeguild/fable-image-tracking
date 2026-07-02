/**
 * Three.js overlay renderer. The camera background is the live <video>
 * element rendered at display rate; alignment with the (slightly older)
 * tracker output is restored by extrapolating poses to the render timestamp
 * (see core/predictor.ts). A transparent WebGL canvas is layered on top with
 * a camera whose projection matches the tracker's pinhole intrinsics.
 *
 * Anchor children are split into user content (cube / image / video) and the
 * debug registration helpers (target outline, translucent plane, axes) that
 * visualise what the tracker has estimated; the latter can be toggled.
 */

import * as THREE from 'three';
import type { Pose, CameraIntrinsics } from '../core/pose';
import { Vector3Filter } from '../core/filter';

export type ContentSpec =
  | { type: 'cube' }
  | { type: 'image'; source: HTMLImageElement | HTMLCanvasElement }
  | { type: 'video'; source: HTMLVideoElement };

export class ARRenderer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private anchor = new THREE.Group();
  private contentGroup = new THREE.Group();
  private debugGroup = new THREE.Group();
  private spinTarget: THREE.Object3D | null = null;
  private content: ContentSpec = { type: 'cube' };
  private contentTexture: THREE.Texture | null = null;

  // One-Euro position filter: low cutoff kills hand-tremor jitter at rest,
  // high beta opens the filter wide as soon as the pose actually moves.
  private positionFilter = new Vector3Filter(0.6, 3.0, 1.0);
  private smoothedQuat = new THREE.Quaternion();
  private quatVelocity = 0; // low-passed angular velocity (rad/s)
  private hasPose = false;
  private lastPoseTime = -1;
  private lastUpdateTime = -1;
  /** Keep showing the last pose this long (seconds) to bridge 1-frame dropouts. */
  private readonly graceSeconds = 0.12;

  private videoWidth = 1280;
  private videoHeight = 720;
  private targetW = 0.2;
  private targetH = 0.2;

  constructor(
    private container: HTMLElement,
    private video: HTMLVideoElement,
    canvas: HTMLCanvasElement,
    private debugCanvas: HTMLCanvasElement
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.01, 100);
    this.scene.add(this.camera);

    this.anchor.matrixAutoUpdate = false;
    this.anchor.visible = false;
    this.anchor.add(this.contentGroup);
    this.anchor.add(this.debugGroup);
    this.scene.add(this.anchor);

    const hemi = new THREE.HemisphereLight(0xffffff, 0x555566, 2.2);
    this.scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(0.4, 1, 0.6);
    this.scene.add(dir);

    window.addEventListener('resize', () => this.layout());
  }

  /** Rebuild anchor content and debug helpers, sized to the physical target. */
  setTargetSize(widthMeters: number, heightMeters: number): void {
    this.targetW = widthMeters;
    this.targetH = heightMeters;
    this.rebuildDebugHelpers();
    this.rebuildContent();
  }

  /** Show or hide the registration debug helpers (green outline/plane/axes). */
  setDebugVisible(visible: boolean): void {
    this.debugGroup.visible = visible;
  }

  /** Switch the displayed content (cube, still image, or video). */
  setContent(spec: ContentSpec): void {
    this.content = spec;
    this.rebuildContent();
  }

  private rebuildDebugHelpers(): void {
    disposeChildren(this.debugGroup);

    const outline = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.PlaneGeometry(this.targetW, this.targetH)),
      new THREE.LineBasicMaterial({ color: 0x00e5a0 })
    );
    this.debugGroup.add(outline);

    const plane = new THREE.Mesh(
      new THREE.PlaneGeometry(this.targetW, this.targetH),
      new THREE.MeshBasicMaterial({
        color: 0x00e5a0,
        transparent: true,
        opacity: 0.12,
        side: THREE.DoubleSide,
      })
    );
    this.debugGroup.add(plane);

    this.debugGroup.add(new THREE.AxesHelper(this.targetW * 0.4));
  }

  private rebuildContent(): void {
    disposeChildren(this.contentGroup);
    this.spinTarget = null;
    if (this.contentTexture) {
      this.contentTexture.dispose();
      this.contentTexture = null;
    }

    if (this.content.type === 'cube') {
      const s = this.targetW * 0.22;
      const cube = new THREE.Mesh(
        new THREE.BoxGeometry(s, s, s),
        new THREE.MeshStandardMaterial({ color: 0x4f8cff, roughness: 0.35, metalness: 0.1 })
      );
      cube.position.set(0, 0, s * 0.5 + this.targetW * 0.05);
      this.contentGroup.add(cube);
      this.spinTarget = cube;
      return;
    }

    // Image / video: an unlit plane fitted inside the target rectangle,
    // floated a hair above it to avoid z-fighting with the debug plane.
    let texture: THREE.Texture;
    let mediaW: number;
    let mediaH: number;
    if (this.content.type === 'image') {
      texture = new THREE.Texture(this.content.source);
      texture.needsUpdate = true;
      mediaW = this.content.source.width;
      mediaH = this.content.source.height;
    } else {
      texture = new THREE.VideoTexture(this.content.source);
      mediaW = this.content.source.videoWidth || 16;
      mediaH = this.content.source.videoHeight || 9;
    }
    texture.colorSpace = THREE.SRGBColorSpace;
    this.contentTexture = texture;

    const fit = Math.min(this.targetW / mediaW, this.targetH / mediaH);
    const plane = new THREE.Mesh(
      new THREE.PlaneGeometry(mediaW * fit, mediaH * fit),
      new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide })
    );
    plane.position.set(0, 0, 0.002);
    this.contentGroup.add(plane);
  }

  setVideoSize(width: number, height: number): void {
    this.videoWidth = width;
    this.videoHeight = height;
    this.layout();
  }

  /** Match the virtual camera to the tracker's pinhole intrinsics. */
  setIntrinsics(K: CameraIntrinsics, procWidth: number, procHeight: number): void {
    const fovY = 2 * Math.atan(procHeight / (2 * K.fy)) * (180 / Math.PI);
    this.camera.fov = fovY;
    this.camera.aspect = procWidth / procHeight;
    this.camera.updateProjectionMatrix();
  }

  /**
   * Update the anchor from a CV-convention pose (x right, y down, z forward),
   * or pass null when tracking is lost this frame.
   */
  updatePose(pose: Pose | null, timeSec: number, spinDelta: number): void {
    if (pose) {
      this.lastPoseTime = timeSec;
      const { R, t } = pose;
      // CV camera -> three.js camera: flip y and z (C = diag(1,-1,-1)).
      const m = new THREE.Matrix4().set(
        R[0], R[1], R[2], t[0],
        -R[3], -R[4], -R[5], -t[1],
        -R[6], -R[7], -R[8], -t[2],
        0, 0, 0, 1
      );
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      m.decompose(pos, quat, scale);

      const [fx, fy, fz] = this.positionFilter.filter([pos.x, pos.y, pos.z], timeSec);

      if (!this.hasPose) {
        this.smoothedQuat.copy(quat);
        this.quatVelocity = 0;
        this.hasPose = true;
      } else {
        // One-Euro-style rotation smoothing: the cutoff frequency follows the
        // (low-passed) angular velocity, so estimation noise is crushed while
        // the target is steady but real rotation passes through immediately.
        const dt = Math.min(0.1, Math.max(1e-3, timeSec - this.lastUpdateTime));
        const angle = this.smoothedQuat.angleTo(quat);
        this.quatVelocity += (angle / dt - this.quatVelocity) * Math.min(1, dt * 5);
        const cutoff = 0.5 + 3.0 * this.quatVelocity;
        const alpha = Math.min(1, 1 - Math.exp(-2 * Math.PI * cutoff * dt));
        this.smoothedQuat.slerp(quat, alpha);
      }

      const sm = new THREE.Matrix4().compose(
        new THREE.Vector3(fx, fy, fz),
        this.smoothedQuat,
        new THREE.Vector3(1, 1, 1)
      );
      this.anchor.matrix.copy(sm);
      this.anchor.visible = true;
    } else if (this.lastPoseTime < 0 || timeSec - this.lastPoseTime > this.graceSeconds) {
      this.anchor.visible = false;
      if (this.hasPose) {
        this.positionFilter.reset();
        this.hasPose = false;
      }
    }

    this.lastUpdateTime = timeSec;
    if (this.spinTarget) this.spinTarget.rotation.z += spinDelta;
    this.renderer.render(this.scene, this.camera);
  }

  getDebugContext(): CanvasRenderingContext2D {
    return this.debugCanvas.getContext('2d')!;
  }

  /** Cover-fit the video, WebGL canvas and debug canvas to the container. */
  layout(): void {
    const cw = this.container.clientWidth;
    const ch = this.container.clientHeight;
    if (cw === 0 || ch === 0 || this.videoWidth === 0) return;
    const scale = Math.max(cw / this.videoWidth, ch / this.videoHeight);
    const w = this.videoWidth * scale;
    const h = this.videoHeight * scale;
    const left = (cw - w) / 2;
    const top = (ch - h) / 2;
    for (const el of [this.video, this.renderer.domElement, this.debugCanvas]) {
      el.style.position = 'absolute';
      el.style.left = `${left}px`;
      el.style.top = `${top}px`;
      el.style.width = `${w}px`;
      el.style.height = `${h}px`;
    }
    this.renderer.setSize(w, h, false);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  }
}

function disposeChildren(group: THREE.Group): void {
  for (const child of [...group.children]) {
    group.remove(child);
    child.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(material)) material.forEach((mat) => mat.dispose());
      else if (material) material.dispose();
    });
  }
}
