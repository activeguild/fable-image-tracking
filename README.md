# Fable Image Tracking

8th Wall のような **image tracking（自然特徴点ベースの画像トラッキング WebAR）** を、OpenCV.js などの CV ライブラリを一切使わずスクラッチで実装した TypeScript プロジェクトです。カメラでトラッキングターゲット（画像）を映すと、その画像に 6DoF で張り付いた 3D コンテンツが表示されます。

## デモの動かし方

```bash
npm install
npm run dev   # --host 付きで起動するので同一ネットワークのスマホからもアクセス可
```

1. `target.html`（アプリ内リンク「サンプルターゲットを別画面で開く」）を別のディスプレイ・スマホで表示するか印刷する
2. アプリ（`index.html`）で「カメラを起動してはじめる」を押す
3. カメラでターゲットを映すと、緑の枠と 3D キューブが画像に張り付く

任意の画像をターゲットにすることもできます（スタート画面のファイル選択から。模様が複雑で非対称な画像ほど安定します）。

> **Note:** getUserMedia は secure context が必要なため、開発サーバは [`@vitejs/plugin-basic-ssl`](https://github.com/vitejs/vite-plugin-basic-ssl) による自己署名 HTTPS で起動します（`https://<PCのIP>:5173` にスマホからアクセス）。自己署名証明書の警告は「詳細設定 → アクセスする」で進んでください。

## パイプライン全体像

8th Wall / Vuforia / MindAR と同じ古典的な **detect → track** の 2 段構成です。

```
[オフライン] ターゲットコンパイル
  参照画像 → スケールピラミッド → FAST-9 コーナー検出 → 向き推定
          → rBRIEF(ORB) 記述子 → 特徴点バンク（マルチスケール）

[ランタイム] 毎フレーム
  カメラ映像 → 縮小グレースケール(360px)
  ├─ SEARCHING: FAST + ORB → ハミング距離マッチング(cross-check + ratio test)
  │             → RANSAC ホモグラフィ → 妥当性検査 → TRACKING へ
  └─ TRACKING:  ピラミッド Lucas-Kanade で特徴点を追跡
                → forward-backward チェック → RANSAC で H を再推定
                → 点が減ったらモデルから再投影で補充、破綻したら SEARCHING へ
  → H を分解して 6DoF ポーズ (H = K [r1 r2 t])
  → One-Euro フィルタ + quaternion slerp で平滑化
  → Three.js でビデオ背景に重ねて描画
```

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
| `src/core/imageops.ts` | グレースケール変換、バイリニアリサイズ、積分画像、スケールピラミッド |
| `src/tracker/target.ts` | ターゲットコンパイル（マルチスケール特徴点バンク生成） |
| `src/tracker/tracker.ts` | detect/track ステートマシン、FB チェック、点の補充、H の妥当性検査 |
| `src/render/renderer.ts` | Three.js オーバーレイ（ピンホール内部パラメータと一致した射影、cover-fit レイアウト、ポーズ平滑化） |

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

- **シングルスレッド**: 検出は数十 ms かかるため探索中は fps が落ちる。Web Worker（+ WASM/SIMD）化が次の一手
- **単一ターゲット**: 複数ターゲット対応はバンクを分けてマッチングを分岐すれば可能
- **カメラキャリブレーション**: 固定 FOV 仮定。WebXR Camera API や事前キャリブレーションで精度向上
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
