# Unity Bullet 互換 物理エンジン (MMD/PMX 物理)

PmxEditor (極北P) の **PMX 2.1 仕様** に記述された物理演算を、**Bullet 2.75** の
挙動に合わせて Unity C# で再実装したものです。MMD モデルの剛体・Joint・SoftBody を
外部ネイティブライブラリ (BulletSharp 等) 無しで動かすことを目的にしています。

> **本プロジェクトは非公式・独立の個人プロジェクトです。**
> MikuMikuDance (樋口優氏)、PmxEditor (極北P氏)、Bullet Physics、およびモデル・モーションの
> 各作者とは、いかなる提携・承認・支援関係もありません。「MMD」「PMX」等は仕様や比較対象を
> 指すための名称として用いています。挙動の一致は目標であって保証ではありません。

## 特徴

- 依存ゼロの純 C# 実装 (Unity 標準アセンブリのみ)
- Bullet の Sequential-Impulse ソルバを踏襲
- 入力は **GLB (`extras.mmd`)** と **PMX バイナリ直読み** の 2 経路（同一の物理を駆動）
- MMD ↔ Unity のボーン同期（変換は単位スケールのみ。後述）

## 現在の到達状況

モデルA モデルで、MMD がベイクした VMD のスカート挙動と定量・目視の両方で比較しています。
**検証はこの 1 モデルに限られており、一般の MMD モデルでの忠実度は未知数**です
（後述「検証カバレッジについて」）。

| 指標 | 自前 | MMD |
|---|---|---|
| 平時の傾き 中央値 | **10.41°** | 11.39° |
| ターン12窓の傾きmax 比 (中央値) | **1.061** | 1.0 |
| 貫入 (定常, スカート×脚) | **~0.0002** | 0.002〜0.004 |
| 性能 (モデルA 300ステップ) | **0.65 ms/step** | — |

> ⚠ 上表は **2026-08-10 のオイラー順序修正 (ZYX→YXZ) より前**の実測値です。同修正で モデルA も
> 物理出力が変わりました（ユーザー実機では見た目の変化なしを確認済み）。テスト用 `.pmx` が
> 手元に無く `bonecheck` が SKIP されるため、**数値の取り直しは未了**です。

**動きの大きさ・タイミングという点では、目視で MMD に近い**ところまで来ています。
ただし「MMD と同等」ではありません。**静止時の微振動は MMD より明確に大きく**（下記「静止時のジッタ」）、
上表の数値も モデルA 1 モデルだけの、しかも直近の修正より前の実測です。

設計判断と「試して失敗した記録」は [docs/DESIGN.md](docs/DESIGN.md) を参照してください。

### 検証カバレッジについて (重要)

長らく **モデルA 1 モデルだけ**で忠実度を検証してきた結果、モデルA が「たまたま使っていない機能」の
欠陥を 2026-08-10 に 4 件まとめて踏みました（ばね定数 0 / mode2 が 0 個 / 複合回転が小さい /
揺れ物カーブが無い）。詳細は
[docs/investigations/2026-08-10-multi-model-defects.md](docs/investigations/2026-08-10-multi-model-defects.md)。

**新しい修正は、ばね定数・mode2・複合回転・揺れ物カーブを持つ別モデルでも必ず回してください。**

## 必要な外部データ (各自で用意)

モデルA などの MMD モデルは**再配布しないため、このリポジトリには含まれません**。各自で用意し、
以下のいずれかで指定してください（検証ハーネス・診断ツール・`BonePoseCsvPlayer` 共通）。

- **推奨**: リポジトリ直下に `testdata/` を作り、以下を置く（`testdata/` は `.gitignore` 済み）:
  - `testdata/modelA.pmx` — モデル本体
  - `testdata/modelA_bone_world_pose.csv` — MMDベイクのボーン世界姿勢CSV (30fps)
  - `testdata/ia.csv` — PMXエディタの構造エクスポート (剛体/Joint/ボーン照合用, pmxverify)
  - `testdata/modelA.glb` — glTF バイナリ (`extras.mmd` 付き。GLB経由入力の検証用, glbverify)
- または環境変数で指定: `MMD_TEST_PMX` / `MMD_TEST_BONECSV` / `MMD_TEST_PMXCSV` / `MMD_TEST_GLB`。
- どちらも無ければ、モデル依存の検証は自動で **SKIP** されます（他は動きます）。

### GLB (extras.mmd 付き) の作り方

GLB 経由の物理入力（`Assets/MmdPhysics/Pmx/GlbPhysicsReader.cs`）は、`extras.mmd` を含む GLB を要します。
[mmd2gltf-gui](https://github.com/masaka1024/) の変換器（標準ライブラリのみ・依存なし）で PMX から生成できます:

```bash
python -m mmd2gltf path/to/modelA.pmx -o testdata/modelA.glb
```

`extras.mmd` は既定で出力され、剛体/Joint を PMX raw のまま、ボーンを glTF ノードとして持ちます。
PMX 直読み経路と全項目・300ステップ物理がビット一致することを `glbverify` で確認済みです。

## 対象環境 / 言語バージョン (C# 9)

- **想定 Unity: Unity 6 (6000.0 LTS)**。Unity の C# バージョンは Unity 本体に固定されており
  (2021.2 以降は **C# 9**、Unity 6 も C# 9 系)、`.csproj` の `LangVersion` を上げる回避策は
  **Unity 非サポート**。したがって本エンジンのコードは **C# 9 の機能のみ**で書く。
- 使わない C# 10 以降の機能: パラメータなし構造体コンストラクタ、構造体のインスタンス
  フィールド初期化子、`record`/`record struct`、ファイルスコープ名前空間、`with` 式、
  `required`、生文字列リテラル、コレクション式 `[...]`、`field` キーワード等。
  - 例: 単位クォータニオン/単位行列が要る箇所は `new Quat()`/`new Matrix4x4()` に頼らず、
    必ず `Quat.Identity` / `Matrix4x4.Identity` を使う (C# 9 の既定 `new()` は全 0)。
- **検証**: `tools` の全 `.csproj` は `<LangVersion>9.0</LangVersion>` を
  設定し、`Assets/MmdPhysics/**` を Unity と同じ言語バージョンでコンパイル検証する。

## ファイル構成 (`Assets/MmdPhysics/`)

| ファイル | 対応する Bullet / PMX | 内容 |
|---|---|---|
| `Core/MathTypes.cs` | btVector3 / btQuaternion / btMatrix3x3 | Vec3, Quat, Matrix4x4, Aabb, Plane |
| `Core/Transform.cs` | btTransform | RigidTransform, Matrix3x3 |
| `Core/CollisionShape.cs` | btSphere/Box/CapsuleShape | 形状 0:球 1:箱 2:カプセル + 慣性 |
| `Core/RigidBody.cs` | btRigidBody | 質量/減衰/反発/摩擦, static/dynamic/kinematic, スリープ状態 |
| `Core/Collision.cs` | btGjkEpa | GJK+EPA ナローフェーズ + 永続マニフォールド |
| `Core/Constraints.cs` | btGeneric6Dof… 他 6 種 | Joint 全種を行ベース SI で求解 + ばね |
| `Core/PhysicsWorld.cs` | btDiscreteDynamicsWorld | 重力/積分/接触/Joint 統合ステップ + スリープ |
| `Core/SoftBody.cs` | btSoftBody | 質点-バネ (Rope / TriMesh), B-Link, Anchor, Pin |
| `Pmx/PmxPhysicsData.cs` | — | PMX 物理レコードの構造体 |
| `Pmx/PmxReader.cs` | — | PMX バイナリパーサ (全セクション対応) |
| `Pmx/GlbPhysicsReader.cs` | — | GLB の `extras.mmd` から同じモデルを構築 |
| `Pmx/MiniJson.cs` | — | 依存ゼロの最小 JSON パーサ (GLB 読取用) |
| `Pmx/PmxPhysicsBuilder.cs` | — | PMX → PhysicsWorld 変換 + ボーン紐付け + 補正姿勢 |
| `Unity/MmdPhysicsBehaviour.cs` | — | Unity MonoBehaviour ブリッジ |
| `Unity/MmdPhysicsBackendSwitch.cs` | — | PhysX (Unity 組込) との排他切替 |
| `DevTools/BonePoseCsvSource.cs` | — | ボーン姿勢CSVローダ (Unity非依存) |
| `DevTools/BonePoseCsvPlayer.cs` | — | MMDベイクCSV再生+MMDのスカートのゴースト重畳 (目視確認用) |

## Joint 対応 (PMX 仕様の対応表どおり)

| PMX Joint 種 | Bullet | 実装 |
|---|---|---|
| 0 ﾊﾞﾈ付6DOF | btGeneric6DofSpringConstraint | 6DOF + 移動/回転バネ |
| 1 6DOF | btGeneric6DofConstraint | 6DOF (バネ無効) |
| 2 P2P | btPoint2PointConstraint | 並進固定・回転フリー |
| 3 ConeTwist | btConeTwistConstraint | 並進固定・回転制限 |
| 4 Slider | btSliderConstraint | X 軸のみ並進/回転 |
| 5 Hinge | btHingeConstraint | X 軸のみ回転 |

すべて `Joint` クラス (行ベース 6DOF ソルバ) に統一し、種別ごとに制限を構成します。
MMD モデルの大多数は種 0 (ﾊﾞﾈ付6DOF) を使用します。

### 角度制限の「フリー」表現

PMX/Bullet の慣習どおり、**下限 > 上限 の軸は「制限なし(フリー)」**として扱います
(`IsFree(lo, hi) => lo > hi`)。モデル作者が意図的にこの指定をすることがあります。

### ばねの安定化クランプ (2026-08-10)

Bullet の 6DOF ばねは陽的 (前進オイラー) で、`k·dt²/m > 1` になると必ず発散します。
実在モデルには `spring = 100000` に対し質量 0.01 (= `k·dt²/m` 2778) のものがあり、
25 ステップで float が溢れて NaN になっていました。

そこでばねの力積を **`|err| · mEff / dt`（デッドビート = 安定限界）で頭打ち**にしています。
`mEff` はソルバ本体と同一の実効質量式で求めるため、**健全な範囲 (`k·dt²/m < 1`) では
一切発動せず、従来と 1 ビットも変わりません**。

## PMX 物理モード (mode) の扱い

| mode | PMX での意味 | 実装 |
|---|---|---|
| 0 | ボーン追従 | kinematic。ボーン姿勢を目標に駆動 |
| 1 | 物理演算 | 完全に動的 |
| 2 | 物理演算 + ボーン位置合わせ | 動的に**シミュレートし**、ボーンへの**出力時のみ**位置を親チェーンから再構成（回転は物理） |

mode2 は**書き戻し側の仕様**であって、シミュレーションを拘束するものではありません
（剛体側を固定すると、旋回時に遠心力を担う並進速度まで失われて髪が軸へ collapse します）。
`MmdPhysicsBehaviour.EnableBoneMergeMode`（既定 ON）で切替できます。mode2 の剛体を持たない
モデルでは補正計算ごとスキップするため無コストです。

## 使い方 (Unity)

`MmdPhysicsBehaviour` をモデルのルートに付け、Inspector で設定します。既定入力は **GLB** です。

```csharp
// Source    = Glb                     (既定)
// GlbPath   = "path/to/model.glb"     (extras.mmd 付き)
// ModelRoot = ボーン階層のルート Transform
// → LateUpdate で物理結果がボーンへ反映されます
```

コードから直接使う場合:

```csharp
var model   = BulletPhysics.Pmx.GlbPhysicsReader.LoadFile("model.glb", out float unitScale);
// あるいは BulletPhysics.Pmx.PmxReader.LoadFile("model.pmx");
var builder = BulletPhysics.Pmx.PmxPhysicsBuilder.Build(model);
builder.World.Gravity = new Vec3(0, -98f, 0);   // MMD スケール

// 毎フレーム
builder.World.StepSimulation(Time.fixedDeltaTime);
```

## 1 フレームの処理順 (Unity)

```
FixedUpdate : ボーン → kinematic 剛体へ目標姿勢を渡す → 物理ステップ
Update      : (Animator がこの後で姿勢を書く)
LateUpdate  : 起動直後の FK-rest リセット → 物理 → ボーンへ書き戻し
```

**書き戻しは必ず `LateUpdate`** で行います。`FixedUpdate` で書くと、揺れ物ボーンに
カーブを持つ AnimationClip では Animator に毎フレーム上書きされ、物理が一切見えなくなります
（レストポーズの定数キーだけでも起きます）。体のボーン (mode0) には書き戻さないので、
ダンスの動きはそのまま残ります。

## 時間刻み

**既定は `FixedTimeStep = 1/60`, `SubSteps = 1`（実効刻み 1/60）** です
(`MmdPhysicsBehaviour` の既定値。`PhysicsWorld` 単体の既定は 1/30 × 2 で実効刻みは同じ)。

当初は「MMDは 30fps で 1 描画フレーム = 1 物理ステップ」という想定でしたが、**これは誤りと判明**
しました。刻みを細かくするほどMMDのスカート挙動に一致し（12窓比の中央値 1/30:1.133 → 1/60:1.030
→ 1/120:0.978）、外部の MMD 互換実装も細刻み（Saba=1/120, libmmd=1/60）だったためです。

`1/60 × 1` は `1/30 × 2` と実効刻みが同一で忠実度も数値まで一致しますが、
**Unity の `Time.fixedDeltaTime` と一致するため、毎 FixedUpdate でちょうど 1 ステップ進み、
更新間隔が等間隔になります**（髪/スカートのコマ落ちが消える）。`AlignUnityFixedTimestep`
（既定 ON）が起動時に `Time.fixedDeltaTime` を自動で合わせます。

## 座標系

エンジンは PMX ネイティブ座標で計算し、境界 (`MmdPhysicsBehaviour`) で Unity へ変換します。
**変換は単位スケールのみで、軸反転はありません**（真の等長変換）。

```
MmdToUnityPos(v) = (v.x, v.y, v.z) * UnitScale
MmdToUnityRot(q) = (q.x, q.y, q.z, q.w)      // 恒等
```

mmd2gltf (PMX→glTF) と UniGLTF (glTF→Unity) の **ReverseZ が二重に掛かって相殺**するため、
Unity 側のボーンは既に PMX ネイティブ座標値になっています。かつてここに 3 回目の Z 反転が
あり、鏡映 (det = -1) になっていました（髪が正面へ反転して体を貫通する症状）。除去済みです。

`UnitScale` は GLB 読込時に `extras.mmd` の値（既定 0.08）で自動上書きされます。

> ★**この相殺は取り込みに UniGLTF を使うことが前提**です。Unity 標準の glTF インポーター（glTFast）は
> Z ではなく **X を反転**するため相殺せず、スケルトンだけが PMX に対して Y 軸 180° 回った状態になります。
> 剛体は `extras.mmd` の raw PMX 座標のまま構築されるので基準が食い違い、髪やスカートが体の正面へ出ます。
> **Unity も UniGLTF もこの食い違いにエラーを出しません。**
> `MmdPhysicsBehaviour.CheckImportConvention`（既定 ON）が起動時にボーン配置と PMX バインド位置を
> 突き合わせ、検出したら対処法つきで `LogError` します。判定はモデルのシーン配置（並進・回転・スケール）に
> 依存しません（`ModelRoot` のローカルへ落とし、重心を引いてから比較するため）。

### オイラー角の順序

PMX の剛体/Joint 回転は **YXZ 順 (R = Ry·Rx·Rz)** です
(Bullet の `btQuaternion(yaw, pitch, roll)` と同型)。`Quat.FromEulerYxz` を使ってください。
`Quat.FromEuler` は ZYX 順なので **PMX データには使わないこと**。

## 静止時のジッタ (未解決)

ほとんど静止した状態でも動的剛体に残留運動が残り、MMDより細かく震えます
(モデルA: `|v|` 平均 0.79 / `|w|` 平均 1.37)。拘束の位置誤差 (Baumgarte) を**実速度**として
打ち消しているため、毎ステップ運動エネルギーが供給され続けるのが原因です。
**ソルバ反復を 10→40 にしても改善しません**（収束不足ではない）。

対策候補を Inspector に出してあります。いずれも**効果がモデル依存**のため既定 OFF です。

| 設定 | 内容 | 実測 |
|---|---|---|
| `JointSplitImpulse` / `ContactSplitImpulse` | 位置補正を擬似速度へ分離 | モデルA は約 3 割減。**モデルBは 10 倍悪化** |
| `EnableSleeping` | Bullet 相当の非活性化 (linear<0.8 かつ angular<1.0 が 2 秒) | 残留がしきい値を超えるため**ほとんど発動しない**(101 体中 2 体) |

スリープはアイランド単位（動的剛体どうしを Joint と接触で連結し、全員が眠りたがるときだけ
まとめて眠らせる）で実装しています。**動いている kinematic に触れているアイランドは眠らせない**
ため、ダンス中に揺れ物が固まることはありません（モデルA の髪で ON/OFF の揺れ幅が完全一致することを確認済み）。

## Unity で目視確認する (セットアップ手順)

### 1. `MmdPhysicsBehaviour` の主な設定

| 項目 | 既定 / 推奨 | 説明 |
|---|---|---|
| `Source` | `Glb` | Unity 運用は GLB 経由。`Pmx` は直読み (検証用) |
| `GlbPath` / `PmxPath` | `.glb` / `.pmx` のパス | 「必要な外部データ」参照 |
| `ModelRoot` | ボーン階層のルート `Transform` | この配下の GameObject 名を PMX ボーン名で解決する |
| `UnitScale` | `0.08` | GLB 読込時に `extras.mmd` の値で自動上書きされる。位置換算のみ (回転は無関係) |
| `Gravity` | `98` | MMD スケール (≒ 9.8×10) |
| `FixedTimeStep` / `SubSteps` | `1/60` / `1` | 実効刻み 1/60。「時間刻み」節参照 |
| `AlignUnityFixedTimestep` | `true` | `Time.fixedDeltaTime` を自動で 1/60 に合わせる |
| `SolverIterations` | `10` | Bullet 既定と同じ |
| `PoseResetDelayFrames` | `2` | Animator のフレーム0適用後に物理を再整合 (初期貫入対策) |
| `EnableBoneMergeMode` | `true` | PMX mode2 を再現する |
| `DrawGizmos` | `false` | 剛体ギズモ。デバッグ時のみ ON (剛体は 100 個超あり重い) |

`UnitScale` は「Unity シーン上に置いたモデルの実寸 ÷ PMXネイティブ寸法」に相当します。
髪・スカートが胴体からズレて描かれるときは、まずここを疑ってください。

### 2. ボーン名解決が失敗したときの症状と確認

`ModelRoot` 配下の GameObject 名を `t.name → Transform` の辞書にし、PMX ボーン名で
引きます (`ResolveBones`)。**完全一致**が必要で、インポータがボーン名を
ローマ字化・改名していると一致しません。

- **症状A (ModelRoot 未設定 or 全滅)**: ボーン追従(kinematic)剛体が目標を得られず
  原点(0,0,0)へ吸い寄せられる。Gizmo を出すと**シアンの箱/カプセルが原点に固まる**。
- **症状B (一部不一致)**: 一致した部位だけ動き、不一致の部位は原点へ飛ぶ or
  バインド姿勢のまま。動的ボーンの物理結果が書き戻らず**メッシュが変形しない**。
- **切り分けの近道**: 後述の `BonePoseCsvPlayer` は `ModelRoot` を必要とせず
  CSV のボーン名で直接駆動するため、「物理が壊れているのか、名前解決が壊れているのか」を
  分離できます。

### 3. Gizmo で剛体を可視化する

`MmdPhysicsBehaviour.DrawGizmos = true` にし、Scene ビュー右上の **Gizmos トグルを ON**
にします (既定は OFF)。`OnDrawGizmos` が全剛体をワイヤーで描きます:

- **シアン** = mode0 ボーン追従 (kinematic)
- **緑** = mode1 物理 (dynamic)
- **黄** = mode2 物理+ボーン位置合わせ ← 異常ではなく、PMX の物理モードの表示

剛体が原点に固まる / モデルとズレる場合は §2 (名前解決) と `UnitScale` を見直します。

### 4. `BonePoseCsvPlayer` でMMDベイクCSVを再生し、MMDのスカートと重ねて見る

ヘッドレス検証(`HeadlessDriver`)と**同一入力・同一ロジック**でMMDでベイクした
ボーン姿勢CSVを再生し、自前物理の剛体(緑)に**MMDのスカート剛体をマゼンタのゴースト**で
重ねて表示するコンポーネントです (`Assets/MmdPhysics/DevTools/BonePoseCsvPlayer.cs`)。
`ModelRoot` は不要 (Unityボーンには書き戻さず、Gizmo で描くだけの目視専用)。

| 項目 | 推奨値 |
|---|---|
| `PmxPath` | modelA.pmx のパス (既定は空文字=何もしない) |
| `BoneCsvPath` | `modelA_bone_world_pose.csv` のパス (空/未存在ならゴースト無しで物理のみ) |
| `Gravity`/`FixedTimeStep`/`SubSteps`/`SolverIterations`/`WarmupSteps` | `98`/`1/30`/`1`/`10`/`60` (ヘッドレスと同一) |
| `UnitScale` | モデル配置に合わせる (単独で見るだけなら `1.0` でも可) |
| `DrawReferenceGhost` / `SkirtOnlyGhost` | `true` / `true` (MMDのスカートのみゴースト) |

操作は Inspector でコンポーネント名を**右クリック → ContextMenu** (Input/GUI 不使用):

- **Play / Pause** … 実時間再生 (`PlaybackFps=30` で等速)
- **Step Forward (+1) / Step Back (-1)** … コマ送り
- **Jump to Frame / Window Start / Window End** … `WindowStart=2440`, `WindowEnd=2470`

**コマ送り**: `Jump to Window Start` → `Step Forward` を連打。自前(緑)とMMD(マゼンタ)の
スカートの開きを1フレームずつ比較できます。物理は逆再生できないため、後退/ジャンプは
内部でフレーム0から再シミュレーションします (7000フレームでも一瞬)。

## Unity での GLB → 自作エンジン統合 (PhysX とのトグル並立)

[mmd2gltf-gui](https://github.com/masaka1024/) で書き出した GLB（`extras.mmd` 付き）を Unity に import し、
物理だけを自作エンジンへ差し替える運用です。**メッシュ／マテリアル（lilToon）／スキン／ベイク再生は
既存の mmd2gltf インポーターのまま**で、物理バックエンドだけを切り替えます。

### 手順
1. **GLB を import** し、既存の mmd2gltf インポーターでマテリアル(lilToon)等を復元する（従来どおり）。
2. モデルのルートに **`MmdPhysicsBehaviour`** を付け、Inspector で:
   - `Source = Glb`（既定）、`GlbPath` に import 元の `.glb` パス。
   - `ModelRoot` に import 済みスケルトンのルート Transform（ボーン名で解決）。
   - `UnitScale` は **extras.mmd の値で自動上書き**されるので触らなくてよい。
   - 起動時に **FK-rest 物理リセットが必ず呼ばれる**（初期のスカート沈み込み/暴れを防ぐ）。
     `PoseResetDelayFrames`（既定 2）で、Animator がフレーム0を適用した後に再整合します。
3. 同じルートに **`MmdPhysicsBackendSwitch`** を付ける。`Mode = Custom`（自作）/`PhysX`（既存）を
   Inspector か右クリック ContextMenu（`Use Custom` / `Use PhysX`）で切替。
   - `Custom` は PhysX 剛体をパーク（kinematic・衝突無効）+ ConfigurableJoint 無効化して排他にする。
   - `EnforceExclusive`（既定 ON）が**毎 FixedUpdate でパークを再主張**します。相手側が実行時に
     `isKinematic` を戻すケースがあり、初期化 1 回では排他を担保できないためです。

> 注: インポーター側 (`mmd2gltf-unity-physics-importer`) は 2026-08-10 に PhysX 経路を撤去し、
> 物理は本エンジンへ一本化されました。`MmdPhysicsBackendSwitch` は、外部から持ち込まれた
> Rigidbody が混ざった場合の保険として残っています（対象が無ければ空転します）。

### 目視で確認するポイント
- **起動直後にスカートが暴れない／脚に沈まない**か（FK-rest リセットが効いていれば静かに始まる）。
- ターン時の開き具合がMMD相当か。
- 揺れ物が**カクつかない**か（`AlignUnityFixedTimestep` が ON なら等間隔更新になる）。
- 静止ポーズで**細かく震えないか**（現状はMMDより震える。「静止時のジッタ」節）。

### うまく動かないときの確認項目
- **スカート/髪が原点へ吸われる・メッシュが変形しない** → ボーン名解決の失敗。`ModelRoot` 配下の
  GameObject 名が GLB のノード名（= extras.mmd のボーン名 = PMX ボーン名）と一致しているか確認。
- **剛体/Joint が 0 件・警告が出る** → その GLB に `extras.mmd` が無い（古いエクスポータ/`--no-extras`）。
- **再生 1 フレーム目から揺れ物がレストポーズで固まる** → AnimationClip が揺れ物ボーンの
  カーブを持っている。本エンジンは `LateUpdate` で書き戻すので通常は勝ちますが、他に
  Transform を書くスクリプトがある場合は競合します。
- **NaN エラーが大量に出る** → 物理が発散。`LateUpdate` の NaN ガードが最初の 1 回だけ
  ボーン名付きで `LogError` を出して書き戻しを止めます。`FixedTimeStep` を小さくするか
  `SubSteps` を増やすと改善することがあります。

## 既知の制限

- SoftBody のクラスタ / AeroModel は簡易対応 (PMX 仕様でも「精度・速度に問題あり・非対応も選択肢」と明記)。
- ConeTwist / Slider / Hinge のモーターは未対応 (仕様上も「暫定対応」)。
- 静止時のジッタがMMDより大きい（上記「静止時のジッタ」節。未解決）。
- 数値は Bullet 2.75 と厳密一致ではなく挙動互換を目標とします。

## 検証 (ハーネスの回し方)

`tools` に UnityEngine の最小シムを使った **Unity 非依存の検証ハーネス**を用意しています。

```bash
cd tools/compilecheck
dotnet run -c Release
```

- 全 `.csproj` は `<LangVersion>9.0</LangVersion>`（Unity=C# 9 と同一）でビルドされ、C# 10 以降の
  機能が混入すると**ハーネスの時点でビルドが落ちます**。
- モデルA モデル/CSV は各自用意（「必要な外部データ」参照）。**未指定なら モデルA 依存の項目は SKIP** され、
  Unity 非依存の項目だけが走ります（落ちません）。
- `tools/<name>/` 以下は各種の**診断ツール**。`cd <name> && dotnet run -c Release` で個別に実行できます。

| ツール | 用途 | PMX 要否 |
|---|---|---|
| `compilecheck` | C# 9 コンパイル検証 + 合成シナリオ | 不要 |
| `chainbug` | 合成チェーンの最小再現（刻み/質量分布の掃引） | 不要 |
| `restsim` | 静止（重力のみ）でのスカート貫入 | **要** |
| `bonecheck` | MMDベイクCSVとの傾き比較（忠実度の本丸） | **要** |
| `hairfid` | 髪のMMDフレーム突合 | **要** |
| `perf` | 位相別プロファイル | 不要 |

> ⚠ 現在 `testdata/modelA.pmx` が無いため `restsim` / `bonecheck` / `hairfid` は常に `[SKIP]` します。
> **忠実度の数値回帰が回せない状態**です。復旧が望まれます。
