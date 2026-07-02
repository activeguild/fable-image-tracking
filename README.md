# Fable Image Tracking

8th Wall のような **image tracking（自然特徴点ベースの画像トラッキング WebAR）** を、OpenCV.js などの CV ライブラリを一切使わずスクラッチで実装した TypeScript プロジェクトです。カメラでトラッキングターゲット（画像）を映すと、その画像に 6DoF で張り付いた 3D コンテンツが表示されます。

## デモの動かし方

```bash
npm install
npm run dev   # --host 付きで起動するので同一ネットワークのスマホからもアクセス可
```

1. `target.html`（アプリ内リンク「サンプルターゲットを別画面で開く」）を別のディスプレイ・スマホで表示するか印刷する
2. アプリ（`index.html`）で「カメラを起動してはじめる」を押す
3. カメラでターゲットを映すと、コンテンツが画像に張り付く

表示コンテンツは HUD の「コンテンツ」セレクタで **キューブ / 画像（サンプル） / 動画（サンプル） / 自分のファイル** を切り替えられます。緑の枠・半透明プレーン・3 軸・黄色い特徴点は**トラッキング判定を可視化するデバッグ表示**で、「判定を表示」チェックでまとめて ON/OFF できます。

平面コンテンツ（画像・動画）は 3D ポーズを経由せず、**計測したホモグラフィで直接 CSS matrix3d ワープ**して合成します。ポーズ分解は焦点距離の仮定誤差の影響を受けますが、この経路は追跡点の計測精度そのままでターゲットに張り付きます（Zappar 等の商用 SDK と同じ発想）。3D コンテンツ（キューブ）向けには、`K⁻¹H` の直交性の破れを毎フレーム評価して焦点距離をオンライン自己校正します。

任意の画像をターゲットにすることもできます（スタート画面のファイル選択から。模様が複雑で非対称な画像ほど安定します）。

> **Note:** getUserMedia は secure context が必要なため、開発サーバは [`@vitejs/plugin-basic-ssl`](https://github.com/vitejs/vite-plugin-basic-ssl) による自己署名 HTTPS で起動します（`https://<PCのIP>:5173` にスマホからアクセス）。自己署名証明書の警告は「詳細設定 → アクセスする」で進んでください。

## パイプライン全体像

8th Wall / Vuforia / MindAR と同じ古典的な **detect → track** の 2 段構成を、
**Web Worker + WASM** で描画（60fps）と処理を分離して実行します。

```
[オフライン] ターゲットコンパイル（Worker 内）
  参照画像 → スケールピラミッド → FAST-9 コーナー検出 → 向き推定
          → rBRIEF(ORB) 記述子 → 特徴点バンク（マルチスケール）

[Worker / WASM] フレーム処理（メインスレッドから転送された縮小グレースケール）
  ├─ SEARCHING: FAST + ORB → ハミング距離マッチング(cross-check + ratio test)
  │             → RANSAC ホモグラフィ → 妥当性検査 → TRACKING へ
  └─ TRACKING:  ピラミッド Lucas-Kanade（等速運動予測でウォームスタート）
                → forward-backward チェック → 事前値ゲート付き決定論的フィット
                → 密なサブピクセル精密化（逆合成 Gauss-Newton 画像位置合わせ）
                → フォトメトリック検証（NCC）で見た目の整合を毎フレーム確認
                → 点が減ったらモデルから再投影で補充、破綻したら SEARCHING へ
  → H を分解して 6DoF ポーズ (H = K [r1 r2 t]) をキャプチャ時刻付きで返送

[メインスレッド] 毎 vsync (60fps)
  → 直近 2 サンプルの等速外挿で「描画時刻のポーズ」を予測
  → One-Euro（位置）+ 角速度適応平滑化（回転）
  → Three.js でライブカメラ映像に重ねて描画
```

## Web Worker + WASM

- CV カーネル（リサイズ / FAST / ORB 記述子 / ハミングマッチング / LK
  オプティカルフロー）は **AssemblyScript**（`assembly/index.ts`）で WASM 化。
  純 TS 実装（`src/core/`）と数値的に一致することをパリティテストで保証し、
  WASM が使えない環境では自動で TS 実装にフォールバックします。
- Node ベンチマーク（360x270）: 検出+記述子 33.7ms → **16.2ms**、
  LK 追跡（80 点 × 往復）7.5ms → **3.4ms**（約 2.1 倍）。
- トラッキングは Worker で非同期実行（in-flight 1 フレーム、バッファは
  Transferable を往復再利用）。メインスレッドはポーズを描画時刻へ外挿する
  ため、処理レートに関係なく 60fps で滑らかに追従します。
- ビルド: `npm run asbuild`（`asc` は純 JS コンパイラ）が
  `public/tracker.wasm` を生成し、dev / build / test の各スクリプトが
  自動実行します。

## 実装したアルゴリズム（すべてスクラッチ）

| モジュール | 内容 |
| --- | --- |
| `src/core/fast.ts` | FAST-9/16 コーナー検出（高速棄却・NMS・サブピクセル補間・グリッド分散選択）、intensity centroid による向き推定 |
| `src/core/orb.ts` | 回転補正付き BRIEF（ORB 相当）。固定シード乱数による 256 ペアのテストパターン、積分画像による 5x5 ボックス平滑化、popcount ハミング距離 |
| `src/core/matcher.ts` | 総当たりハミングマッチング + Lowe ratio test + cross-check |
| `src/core/homography.ts` | 正規化 DLT（Hartley 正規化 + 8x8 正規方程式をガウス消去で解く） |
| `src/core/ransac.ts` | 適応的反復回数の RANSAC、退化サンプル除去、インライアでの再フィット |
| `src/core/opticalflow.ts` | ピラミッド Lucas-Kanade（Bouguet 方式、バイリニア補間、反復解法） |
| `src/core/pose.ts` | ホモグラフィ分解による 6DoF ポーズ復元（H = K [r1 r2 t]、回転の直交化、正面制約） |
| `src/core/filter.ts` | One-Euro フィルタ（位置のジッタ除去） |
| `src/core/densealign.ts` | 逆合成 Gauss-Newton による密なホモグラフィ精密化（サブピクセル、ゲイン/バイアス照明不変） |
| `src/core/imageops.ts` | グレースケール変換、バイリニアリサイズ、積分画像、スケールピラミッド |
| `src/tracker/target.ts` | ターゲットコンパイル（マルチスケール特徴点バンク生成） |
| `src/tracker/tracker.ts` | detect/track ステートマシン、FB チェック、点の補充、H の妥当性検査、NCC によるフォトメトリック検証 |
| `src/tracker/worker.ts` | トラッキング Worker（WASM カーネル優先、TS フォールバック） |
| `src/core/kernels.ts` | CV カーネルの抽象化（TS 実装 = リファレンス） |
| `assembly/index.ts` | AssemblyScript 製 WASM カーネル（TS 実装と数値一致） |
| `src/wasm/engine.ts` | WASM メモリレイアウト管理とカーネルラッパー |
| `src/core/predictor.ts` | 等速外挿によるポーズ予測（描画時刻への補間） |
| `src/render/renderer.ts` | Three.js オーバーレイ（ピンホール内部パラメータと一致した射影、cover-fit レイアウト、速度適応ポーズ平滑化） |

外部依存はレンダリング用の **Three.js のみ**。トラッキングは全て自前実装です。

## テスト

```bash
npm test        # vitest（23 テスト）
npm run typecheck
```

- 各アルゴリズムの単体テスト（合成画像でのコーナー検出、既知ホモグラフィの復元、40% 外れ値下の RANSAC、サブピクセル並進のオプティカルフロー復元、既知ポーズの復元など）
- `tests/tracker.test.ts` は合成フレームに既知のホモグラフィでターゲットを描画し、検出 → 追跡の全パイプラインでコーナー誤差 4px 以内を検証する E2E テスト

また、Chromium のフェイクカメラ（y4m）にサンプルターゲットを流し込み、実ブラウザで「トラッキング中」まで到達することをスモークテストで確認済みです。

## チューニングポイント

- `PROC_WIDTH`（`src/main.ts`）: 処理解像度。上げると精度・ロバスト性が上がり、fps が下がる
- `fastThreshold` / `maxFrameFeatures`（`ImageTracker` オプション）: 特徴点の量と質
- `ransacThreshold`: インライア判定の厳しさ（処理解像度ピクセル）
- `defaultIntrinsics`: カメラの焦点距離は水平 FOV ~64° と仮定。デバイスごとに合わせると位置精度が向上

## 制限と今後の拡張

- **単一ターゲット**: 複数ターゲット対応はバンクを分けてマッチングを分岐すれば可能
- **カメラキャリブレーション**: 固定 FOV 仮定。WebXR Camera API や事前キャリブレーションで精度向上
- **WASM SIMD**: 現状はスカラー WASM。`i8x16.popcnt` 等の v128 化でマッチング・LK をさらに高速化できる
- 照明変化には ORB のバイナリテストである程度強いが、強い鏡面反射・モーションブラーには弱い

## プロジェクト構成

```
├── index.html            # ARアプリ本体
├── target.html           # サンプルターゲット表示ページ（別画面/印刷用）
├── src/
│   ├── core/             # CVアルゴリズム（純粋関数、DOM非依存）
│   ├── tracker/          # ターゲットコンパイル + トラッキングステートマシン
│   ├── render/           # Three.js オーバーレイレンダラ
│   ├── sampleTarget.ts   # 手続き生成のサンプルターゲット（アプリとtarget.htmlで共有）
│   └── main.ts           # カメラ・ループ・UI の結線
└── tests/                # vitest（コア + E2E）
```
