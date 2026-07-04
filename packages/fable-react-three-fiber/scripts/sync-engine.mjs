/**
 * Copies the tracking engine sources from the repository root into this
 * package, so the package stays self-contained (it can be moved to its own
 * repository) while the engine keeps a single source of truth here.
 *
 *   node scripts/sync-engine.mjs
 */
import { cpSync, mkdirSync, existsSync, copyFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const pkgRoot = join(dirname(fileURLToPath(import.meta.url)), '..');
const repoRoot = join(pkgRoot, '..', '..');
const engineDir = join(pkgRoot, 'src', 'engine');

const copies = [
  ['src/core', 'core'],
  ['src/tracker', 'tracker'],
  ['src/wasm', 'wasm'],
  ['src/gyro.ts', 'gyro.ts'],
];

mkdirSync(engineDir, { recursive: true });
for (const [from, to] of copies) {
  cpSync(join(repoRoot, from), join(engineDir, to), { recursive: true });
  console.log(`synced ${from} -> src/engine/${to}`);
}

// Prebuilt WASM kernels, shipped as a package asset the consumer copies to
// their public directory (or points to via the wasmSrc prop).
const wasm = join(repoRoot, 'public', 'tracker.wasm');
if (existsSync(wasm)) {
  mkdirSync(join(pkgRoot, 'assets'), { recursive: true });
  copyFileSync(wasm, join(pkgRoot, 'assets', 'tracker.wasm'));
  console.log('synced public/tracker.wasm -> assets/tracker.wasm');
} else {
  console.warn('public/tracker.wasm not found - run `npm run asbuild` at the repo root first');
}
