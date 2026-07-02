/**
 * DeviceMotion gyroscope collector (main thread). Buffers rotation-rate
 * samples so the capture loop can integrate the rotation between two frame
 * timestamps and ship it to the tracking worker as a motion prior.
 */

import { deviceRateToCamera, type GyroDelta } from './core/imu';

interface Sample {
  t: number; // performance.now() ms
  x: number; // camera-frame angular rate, rad/s
  y: number;
  z: number;
}

export class GyroCollector {
  private samples: Sample[] = [];
  private listening = false;
  available = false;

  /**
   * Ask for motion permission (iOS 13+ requires it inside a user gesture)
   * and start listening. Resolves to whether gyro data can be used.
   */
  async start(): Promise<boolean> {
    if (this.listening) return this.available;
    interface PermissionCapable {
      requestPermission?: () => Promise<'granted' | 'denied'>;
    }
    try {
      const dme = DeviceMotionEvent as unknown as PermissionCapable;
      if (typeof dme.requestPermission === 'function') {
        const state = await dme.requestPermission();
        if (state !== 'granted') return false;
      }
    } catch {
      return false;
    }
    window.addEventListener('devicemotion', (e) => this.onMotion(e), { passive: true });
    this.listening = true;
    return new Promise((resolve) => {
      // Consider gyro available once a real sample arrives (some desktops
      // fire events with null rotation rates).
      setTimeout(() => resolve(this.available), 600);
    });
  }

  private onMotion(e: DeviceMotionEvent): void {
    const r = e.rotationRate;
    if (!r || r.alpha === null || r.beta === null || r.gamma === null) return;
    const angle =
      (screen.orientation && typeof screen.orientation.angle === 'number'
        ? screen.orientation.angle
        : 0) as number;
    const cam = deviceRateToCamera(r.alpha, r.beta, r.gamma, angle);
    const t = performance.now();
    this.samples.push({ t, x: cam.x, y: cam.y, z: cam.z });
    this.available = true;
    // Keep half a second of history.
    const cutoff = t - 500;
    while (this.samples.length > 0 && this.samples[0].t < cutoff) this.samples.shift();
  }

  /**
   * Integrated camera-frame rotation between two performance.now()
   * timestamps (ms), or null when there are no samples in the window.
   */
  delta(t0Ms: number, t1Ms: number): GyroDelta | null {
    if (!this.available || t1Ms <= t0Ms) return null;
    let wx = 0;
    let wy = 0;
    let wz = 0;
    let covered = 0;
    for (let i = 0; i < this.samples.length; i++) {
      const s = this.samples[i];
      const next = this.samples[i + 1];
      const segStart = Math.max(t0Ms, s.t);
      const segEnd = Math.min(t1Ms, next ? next.t : t1Ms);
      if (segEnd <= segStart) continue;
      const dt = (segEnd - segStart) / 1000;
      wx += s.x * dt;
      wy += s.y * dt;
      wz += s.z * dt;
      covered += dt;
    }
    if (covered <= 0) return null;
    return { wx, wy, wz };
  }
}
