/**
 * Three.js overlay renderer. The camera background is the live <video>
 * element rendered at display rate; alignment with the (slightly older)
 * tracker output is restored by extrapolating poses to the render timestamp
 * (see core/predictor.ts). A transparent WebGL canvas is layered on top with
 * a camera whose projection matches the tracker's pinhole intrinsics.
 */

import * as THREE from 'three';
import type { Pose, CameraIntrinsics } from '../core/pose';
import { Vector3Filter } from '../core/filter';

export class ARRenderer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private anchor = new THREE.Group();
  private cube: THREE.Mesh;

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
    this.scene.add(this.anchor);

    const hemi = new THREE.HemisphereLight(0xffffff, 0x555566, 2.2);
    this.scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(0.4, 1, 0.6);
    this.scene.add(dir);

    this.cube = new THREE.Mesh(
      new THREE.BoxGeometry(1, 1, 1),
      new THREE.MeshStandardMaterial({ color: 0x4f8cff, roughness: 0.35, metalness: 0.1 })
    );
    this.anchor.add(this.cube);

    window.addEventListener('resize', () => this.layout());
  }

  /** Populate anchor content sized to the physical target. */
  setTargetSize(widthMeters: number, heightMeters: number): void {
    // Remove previous helpers except the cube.
    for (const child of [...this.anchor.children]) {
      if (child !== this.cube) this.anchor.remove(child);
    }

    const outline = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.PlaneGeometry(widthMeters, heightMeters)),
      new THREE.LineBasicMaterial({ color: 0x00e5a0 })
    );
    this.anchor.add(outline);

    const plane = new THREE.Mesh(
      new THREE.PlaneGeometry(widthMeters, heightMeters),
      new THREE.MeshBasicMaterial({
        color: 0x00e5a0,
        transparent: true,
        opacity: 0.12,
        side: THREE.DoubleSide,
      })
    );
    this.anchor.add(plane);

    const axes = new THREE.AxesHelper(widthMeters * 0.4);
    this.anchor.add(axes);

    const s = widthMeters * 0.22;
    this.cube.geometry.dispose();
    this.cube.geometry = new THREE.BoxGeometry(s, s, s);
    this.cube.position.set(0, 0, s * 0.5 + widthMeters * 0.05);
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
    this.cube.rotation.z += spinDelta;
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
