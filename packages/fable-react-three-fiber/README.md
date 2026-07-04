# @j1ngzoue/fable-react-three-fiber

[fable-image-tracking](https://github.com/activeguild/fable-image-tracking) のトラッキングエンジン（スクラッチ実装のマーカーレス画像トラッキング）を
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
import { FableCanvas, FableCamera, ImageTracker, PlanarContent } from '@j1ngzoue/fable-react-three-fiber';

export default function App() {
  return (
    <FableCanvas style={{ width: '100vw', height: '100vh' }}>
      <FableCamera />
      <ImageTracker
        targetImage="/my-target.png"
        onVisible={(anchor) => console.log(`Visible ${anchor.id}`)}
        onNotVisible={(anchor) => console.log(`Not visible ${anchor.id}`)}
      >
        {/* 平面の画像・動画: 自動で高精度なホモグラフィ表示になる */}
        <PlanarContent source="/my-photo.png" />
        {/* 3D オブジェクト: ポーズにアンカーされる。単位は Zappar 互換
            （ターゲットの高さ = 2、上端 y=+1）: 0.6 のキューブ =
            正方形マーカー幅の 30% */}
        <mesh position={[0, 0, 0.3]}>
          <boxGeometry args={[0.6, 0.6, 0.6]} />
          <meshStandardMaterial color="hotpink" />
        </mesh>
      </ImageTracker>
      <hemisphereLight args={[0xffffff, 0x555566, 2.2]} />
      <directionalLight position={[1, 2, 3]} intensity={1.5} />
    </FableCanvas>
  );
}
```

> コンテンツは全部 `<ImageTracker>` の中に書けば OK です。
> `<PlanarContent>`（平面の画像・動画）は自動的に DOM レイヤーへルーティングされ、
> 計測ホモグラフィそのままの CSS matrix3d でピクセル精度で貼り付きます。
> mesh などの 3D オブジェクトは 6DoF ポーズにアンカーされます。ポーズは
> 再投影誤差最小化（Gauss-Newton）で毎フレーム精密化されるので、マーカーから
> 離れた位置に置いたコンテンツも安定します。

カメラ映像は**フレーム同期表示**です: トラッカーが処理を終えたフレームを、そのフレームで計測した
アンカー姿勢と同じペイントで表示するため、遅延がマーカーずれとして見えません（商用エンジンと同方式）。

## コンポーネント

### `<FableCanvas>`

カメラとトラッキングエンジンを起動し、フレーム同期カメラキャンバスの上に透明な R3F `<Canvas>` を重ねるルートコンポーネント。

| prop | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `targetImage` | `string \| HTMLImageElement \| HTMLCanvasElement` | - | トラッキングターゲット画像（`<ImageTracker targetImage>` で渡す場合は省略可） |
| `wasmSrc` | `string` | `/tracker.wasm` | WASM カーネルの URL（404 なら JS フォールバック） |
| `autoStart` | `boolean` | `true` | マウント時にカメラを起動。iOS では `false` + `startCamera()` 推奨 |
| `imu` | `boolean` | `false` | ジャイロを運動事前値として使用（iOS は権限ダイアログ） |
| `targetWidthMeters` | `number` | Zappar互換 | シーン単位でのターゲット幅。省略時は **Zappar と同じ「高さ = 2 単位」**（上端 y=+1・下端 y=-1、幅はアスペクト比なり）。明示すると幅基準（`1` = マーカー幅 1 単位、`0.2` = 実寸メートル等） |
| `dpr` | `number \| [number, number]` | - | 3D キャンバスの devicePixelRatio |
| `onReady` | `(info) => void` | - | ターゲットコンパイル完了時 |
| `onError` | `(err) => void` | - | カメラ起動失敗など |

### `<PlanarContent>`

ターゲット平面上の画像・動画を、計測ホモグラフィ（CSS matrix3d）でピクセル精度で貼り付けます。
（`offset` の単位はターゲット幅/高さで、3D 側のシーン単位とは独立です。）
`<FableCanvas>` 内ならどこに書いても動きます（通常は 3D コンテンツと並べて
`<ImageTracker>` 内に）。実体はカメラと 3D キャンバスの間の DOM レイヤーに
マウントされるため、カメラ内部パラメータの誤差の影響を受けません。
信頼度ゲート連動のフェード（トラッキングが弱い間は非表示）付き。

| prop | 型 | 説明 |
| --- | --- | --- |
| `source` | `string \| HTMLImageElement \| HTMLCanvasElement \| HTMLVideoElement` | 表示するメディア（URL または要素） |
| `offset` | `{ x?: number; y?: number }` | ターゲット平面内の配置オフセット（ターゲット幅/高さ単位）。`{ x: 1.15 }` でマーカーの右横。ホモグラフィは平面全体を写像するのでマーカー外でも精度は同じ |

動画を渡す場合は `muted` + `playsInline` を設定し、ユーザージェスチャ内で `play()` を呼んでください（iOS の自動再生制約）。

### `<FableCamera>`

three.js カメラをトラッカーの（自己校正される）ピンホール内部パラメータに一致させます。

| prop | 型 | 説明 |
| --- | --- | --- |
| `fov` | `number` | 垂直 FOV を手動指定（自己校正を上書き） |
| `onFirstFrame` | `() => void` | 最初のトラッキング結果が届いた時に一度だけ発火 |

### `<ImageTracker>`

子要素をターゲットにアンカーします。座標系はターゲット中心が原点、x 右・y 上・z 手前。
単位はデフォルトで **Zappar 互換（ターゲットの高さ = 2、上端 y=+1）**、`targetWidthMeters` で変更可。

| prop | 型 | 説明 |
| --- | --- | --- |
| `targetImage` | `string \| img \| canvas` | ターゲット画像（Zappar スタイル。`.zpt` 不要、普通の画像でOK） |
| `enabled` | `boolean` | `false` で非表示 + コールバック停止 |
| `onFound` | `(frame) => void` | ターゲット捕捉時（信頼度ゲート回復時も） |
| `onUpdated` | `(frame) => void` | 表示中の毎トラッキング更新 |
| `onLost` | `() => void` | ロスト時（信頼度ゲートで隠れた時も） |
| `onVisible` | `(anchor) => void` | Zappar 互換エイリアス（= onFound） |
| `onNotVisible` | `(anchor) => void` | Zappar 互換エイリアス（= onLost） |
| `onNewAnchor` | `(anchor) => void` | Zappar 互換: 初回検出時に一度だけ発火 |

### `useFable()`

`<FableCanvas>` 内（DOM 側・R3F シーン側どちらでも）でトラッキング状態にアクセスするフック。

```ts
const { engine, targetInfo, started, startCamera, onFrame } = useFable();
```

`onFrame` で毎結果（`state` / `pose` / `corners` / `inlierCount` / 自己校正済み `fx`, `k1` など）を購読できます。

## Zappar からの移行

`@zappar/zappar-react-three-fiber` とほぼ同じ形で書けます:

| Zappar | 本パッケージ | 備考 |
| --- | --- | --- |
| `<ZapparCanvas>` | `<FableCanvas>` | ライセンスキー不要 |
| `<ZapparCamera />` | `<FableCamera />` | `onFirstFrame` も同名 |
| `<ImageTracker targetImage="x.zpt">` | `<ImageTracker targetImage="x.png">` | **学習ファイル（.zpt）不要**。元画像をそのまま渡す（実行時に Worker で特徴点コンパイル） |
| `onVisible` / `onNotVisible` / `onNewAnchor` | 同名で使用可 | `anchor.id` 付き |
| 座標系（中心原点・y 上・z 手前） | 同じ | |
| 単位（高さ = 2、上端 y=+1） | デフォルトで同じ | 物理サイズ基準にしたい場合は `targetWidthMeters` を明示 |
| `<Loader>` / `<BrowserCompatibility>` | なし | `onReady` / `onError` で代替 |
| 顔・インスタントトラッキング | 非対応 | 画像トラッキング専用 |

追加機能: 平面メディアは `<PlanarContent>` に置き換えるとポーズ非経由（計測ホモグラフィ直貼り）になり、張り付き精度が上がります。

## 開発

```bash
npm run sync-engine   # リポジトリルートからエンジンソースと tracker.wasm を取り込む
npm run build         # dist/ （ESM + 型定義 + Worker チャンク）
npm run dev:example   # example/ を https://localhost:4176 で起動
```

エンジン本体（`src/engine/`）は `sync-engine` がリポジトリルートの `src/` からコピーする生成物です。
エンジンの修正はリポジトリルート側で行ってください。
