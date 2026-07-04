# @j1ngzoue/fable-react-three-fiber

[fable-image-tracking](../..) のトラッキングエンジン（スクラッチ実装のマーカーレス画像トラッキング）を
[React Three Fiber](https://github.com/pmndrs/react-three-fiber) で使うためのバインディングです。
API は [@j1ngzoue/8thwall-react-three-fiber](https://github.com/activeguild/8thwall-react-three-fiber) と同じ使い勝手を踏襲しています。

8th Wall と違い **外部エンジン（xr.js）もライセンスキーも不要**です。ターゲットは事前コンパイル済み JSON ではなく
**ただの画像**（URL / `<img>` / `<canvas>`）を渡すと、実行時に Worker 内で特徴点コンパイルされます。

## インストール

```bash
npm install @j1ngzoue/fable-react-three-fiber react react-dom three @react-three/fiber
```

WASM カーネル（無くても純 JS フォールバックで動作しますが、あると約 2 倍速）をパッケージから public へコピーします:

```bash
cp node_modules/@j1ngzoue/fable-react-three-fiber/assets/tracker.wasm public/
```

別の場所に置く場合は `<FableCanvas wasmSrc="/assets/tracker.wasm">` で指定できます。

## 使い方

```jsx
import { FableCanvas, FableCamera, ImageTracker } from '@j1ngzoue/fable-react-three-fiber';

export default function App() {
  return (
    <FableCanvas targetImage="/my-target.png" style={{ width: '100vw', height: '100vh' }}>
      <FableCamera />
      <ImageTracker onFound={() => console.log('found!')}>
        <mesh position={[0, 0, 0.03]}>
          <boxGeometry args={[0.06, 0.06, 0.06]} />
          <meshStandardMaterial color="hotpink" />
        </mesh>
      </ImageTracker>
      <hemisphereLight args={[0xffffff, 0x555566, 2.2]} />
      <directionalLight position={[1, 2, 3]} intensity={1.5} />
    </FableCanvas>
  );
}
```

カメラ映像は**フレーム同期表示**です: トラッカーが処理を終えたフレームを、そのフレームで計測した
アンカー姿勢と同じペイントで表示するため、遅延がマーカーずれとして見えません（商用エンジンと同方式）。

## コンポーネント

### `<FableCanvas>`

カメラとトラッキングエンジンを起動し、フレーム同期カメラキャンバスの上に透明な R3F `<Canvas>` を重ねるルートコンポーネント。

| prop | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `targetImage` | `string \| HTMLImageElement \| HTMLCanvasElement` | 必須 | トラッキングターゲット画像 |
| `wasmSrc` | `string` | `/tracker.wasm` | WASM カーネルの URL（404 なら JS フォールバック） |
| `autoStart` | `boolean` | `true` | マウント時にカメラを起動。iOS では `false` + `startCamera()` 推奨 |
| `imu` | `boolean` | `false` | ジャイロを運動事前値として使用（iOS は権限ダイアログ） |
| `targetWidthMeters` | `number` | `0.2` | ターゲットの物理幅（3D シーンのスケール基準） |
| `dpr` | `number \| [number, number]` | - | 3D キャンバスの devicePixelRatio |
| `onReady` | `(info) => void` | - | ターゲットコンパイル完了時 |
| `onError` | `(err) => void` | - | カメラ起動失敗など |

### `<FableCamera>`

three.js カメラをトラッカーの（自己校正される）ピンホール内部パラメータに一致させます。

| prop | 型 | 説明 |
| --- | --- | --- |
| `fov` | `number` | 垂直 FOV を手動指定（自己校正を上書き） |
| `onFirstFrame` | `() => void` | 最初のトラッキング結果が届いた時に一度だけ発火 |

### `<ImageTracker>`

子要素をターゲットにアンカーします。座標系はターゲット中心が原点、x 右・y 上・z 手前（メートル単位）。

| prop | 型 | 説明 |
| --- | --- | --- |
| `enabled` | `boolean` | `false` で非表示 + コールバック停止 |
| `onFound` | `(frame) => void` | ターゲット捕捉時（信頼度ゲート回復時も） |
| `onUpdated` | `(frame) => void` | 表示中の毎トラッキング更新 |
| `onLost` | `() => void` | ロスト時（信頼度ゲートで隠れた時も） |

### `useFable()`

`<FableCanvas>` 内（DOM 側・R3F シーン側どちらでも）でトラッキング状態にアクセスするフック。

```ts
const { engine, targetInfo, started, startCamera, onFrame } = useFable();
```

`onFrame` で毎結果（`state` / `pose` / `corners` / `inlierCount` / 自己校正済み `fx`, `k1` など）を購読できます。

## 開発

```bash
npm run sync-engine   # リポジトリルートからエンジンソースと tracker.wasm を取り込む
npm run build         # dist/ （ESM + 型定義 + Worker チャンク）
npm run dev:example   # example/ を https://localhost:4176 で起動
```

エンジン本体（`src/engine/`）は `sync-engine` がリポジトリルートの `src/` からコピーする生成物です。
エンジンの修正はリポジトリルート側で行ってください。
