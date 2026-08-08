# Unity Bullet 互換 物理エンジン (MMD/PMX 物理)

PmxEditor (極北P) の **PMX 2.1 仕様** に記述された物理演算を、**Bullet 2.75** の
挙動に合わせて Unity C# で再実装したものです。MMD モデルの剛体・Joint・SoftBody を
外部ネイティブライブラリ (BulletSharp 等) 無しで動かすことを目的にしています。

## 特徴

- 依存ゼロの純 C# 実装 (Unity 標準アセンブリのみ)
- Bullet の Sequential-Impulse ソルバを踏襲
- PMX バイナリ (2.0 / 2.1) から剛体・Joint・SoftBody を直接読み込み
- MMD ↔ Unity 座標変換込みのボーン同期

## 現在の到達状況 (IA.pmx で本家ベイクVMDと比較)

本家（MMD がベイクした VMD）のスカート挙動と、定量・目視の両方で比較しています。

| 指標 | 自前 | 本家 |
|---|---|---|
| 平時の傾き 中央値 | **10.41°** | 11.39° |
| ターン12窓の傾きmax 比 (中央値) | **1.061** | 1.0 |
| 貫入 (定常, スカート×脚) | **~0.0002** | 0.002〜0.004 |
| 性能 (IA 300ステップ) | **0.65 ms/step** | — |
| 測定ノイズフロア | **0** (無関係な変更で不変) | — |

平時はほぼ一致、ターンは +6% 程度の微過大に収束。初期状態は MMD 相当の物理リセット
（FK-rest）で整合させ、スカートが脚に沈む初期貫入は解消済み。**目視では本家とほぼ区別が
つかない水準**に到達しています。設計判断と「試して失敗した記録」は [DESIGN.md](DESIGN.md) 参照。

## 必要な外部データ (各自で用意)

IA などの MMD モデルは**再配布しないため、このリポジトリには含まれません**。各自で用意し、
以下のいずれかで指定してください（検証ハーネス・診断ツール・`BonePoseCsvPlayer` 共通）。

- **推奨**: リポジトリ直下に `testdata/` を作り、以下を置く（`testdata/` は `.gitignore` 済み）:
  - `testdata/IA.pmx` — モデル本体
  - `testdata/IA_bone_world_pose.csv` — 本家ベイクのボーン世界姿勢CSV (30fps)
  - `testdata/ia.csv` — PMXエディタの構造エクスポート (剛体/Joint/ボーン照合用, pmxverify)
  - `testdata/IA.glb` — glTF バイナリ (`extras.mmd` 付き。GLB経由入力 `GlbPhysicsReader` の検証用, glbverify)
- または環境変数で指定: `MMD_TEST_PMX` / `MMD_TEST_BONECSV` / `MMD_TEST_PMXCSV` / `MMD_TEST_GLB`。
- どちらも無ければ、モデル依存の検証は自動で **SKIP** されます（他は動きます）。

### GLB (extras.mmd 付き) の作り方
GLB 経由の物理入力（`Assets/Scripts/Pmx/GlbPhysicsReader.cs`）は、`extras.mmd` を含む GLB を要します。
[mmd2gltf-gui](https://github.com/masaka1024/) の変換器（標準ライブラリのみ・依存なし）で PMX から生成できます:

```bash
python -m mmd2gltf path/to/IA.pmx -o testdata/IA.glb
```

`extras.mmd` は既定で出力され、剛体/Joint を PMX raw のまま、ボーンを glTF ノードとして持ちます
（スケール `unitScale=0.08`・座標変換は当エンジンと一致）。PMX 直読み経路と全項目・300ステップ物理が
ビット一致することを `glbverify` で確認済みです。

## 対象環境 / 言語バージョン (C# 9)

- **想定 Unity: Unity 6 (6000.0 LTS)**。Unity の C# バージョンは Unity 本体に固定されており
  (2021.2 以降は **C# 9**、Unity 6 も C# 9 系)、`.csproj` の `LangVersion` を上げる回避策は
  **Unity 非サポート**。したがって本エンジンのコードは **C# 9 の機能のみ**で書く。
- 使わない C# 10 以降の機能: パラメータなし構造体コンストラクタ、構造体のインスタンス
  フィールド初期化子、`record`/`record struct`、ファイルスコープ名前空間、`with` 式、
  `required`、生文字列リテラル、コレクション式 `[...]`、`field` キーワード等。
  (C# 9 で使える target-typed `new()`、パターン、タプル代入などは可)。
  - 例: 単位クォータニオン/単位行列が要る箇所は `new Quat()`/`new Matrix4x4()` に頼らず、
    必ず `Quat.Identity` / `Matrix4x4.Identity` を使う (C# 9 の既定 `new()` は全 0)。
- **検証**: `scratchpad/compilecheck` の全 `.csproj` は `<LangVersion>9.0</LangVersion>` を
  設定し、`Assets/Scripts/**` を Unity と同じ言語バージョンでコンパイル検証する。
  これにより C# 10 以降の機能が混入するとハーネスの時点でビルドが落ちる。

## ファイル構成 (`Assets/Scripts/`)

| ファイル | 対応する Bullet / PMX | 内容 |
|---|---|---|
| `MathTypes.cs` | btVector3 / btQuaternion / btMatrix3x3 | Vec3, Quat, Matrix4x4, Aabb, Plane |
| `Transform.cs` | btTransform | RigidTransform, Matrix3x3 |
| `CollisionShape.cs` | btSphere/Box/CapsuleShape | 形状 0:球 1:箱 2:カプセル + 慣性 |
| `RigidBody.cs` | btRigidBody | 質量/減衰/反発/摩擦, static/dynamic/kinematic |
| `Collision.cs` | btGjkEpa | GJK+EPA ナローフェーズ + 永続マニフォールド |
| `Constraints.cs` | btGeneric6Dof… 他 6 種 | Joint 全種を行ベース SI で求解 |
| `PhysicsWorld.cs` | btDiscreteDynamicsWorld | 重力/積分/接触/Joint 統合ステップ |
| `SoftBody.cs` | btSoftBody | 質点-バネ (Rope / TriMesh), B-Link, Anchor, Pin |
| `Pmx/PmxPhysicsData.cs` | — | PMX 物理レコードの構造体 |
| `Pmx/PmxReader.cs` | — | PMX バイナリパーサ (全セクション対応) |
| `Pmx/PmxPhysicsBuilder.cs` | — | PMX → PhysicsWorld 変換 + ボーン紐付け |
| `Unity/MmdPhysicsBehaviour.cs` | — | Unity MonoBehaviour ブリッジ |
| `Unity/BonePoseCsvSource.cs` | — | ボーン姿勢CSVローダ (Unity非依存) |
| `Unity/BonePoseCsvPlayer.cs` | — | 本家CSV再生+本家スカートのゴースト重畳 (目視確認用) |

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

## 使い方 (Unity)

```csharp
// GameObject に MmdPhysicsBehaviour を付与し、
//   PmxPath   = "path/to/model.pmx"
//   ModelRoot = ボーン階層のルート Transform
// を設定するだけ。FixedUpdate で自動的に物理が進みます。
```

コードから直接使う場合:

```csharp
var model   = BulletPhysics.Pmx.PmxReader.LoadFile("model.pmx");
var builder = BulletPhysics.Pmx.PmxPhysicsBuilder.Build(model);
builder.World.Gravity = new Vec3(0, -98f, 0);   // MMD スケール

// 毎フレーム
builder.World.StepSimulation(Time.fixedDeltaTime);
```

## 時間刻み (リファレンス: 実効 1/60 = FixedTimeStep 1/30・SubSteps 2)

既定は `FixedTimeStep = 1/30`, `SubSteps = 2`（実効刻み **1/60**）です。`FixedTimeStep` は
30fps のボーン入力に合わせて 1/30 のまま、物理は `SubSteps` で細かく刻みます。

当初は「MMD 本家は 30fps で 1 描画フレーム = 1 物理ステップ」という想定で 30Hz・1サブを
リファレンスにしていましたが、**この想定は誤りと判明しました**。刻みを細かくするほど本家の
スカート挙動に一致し（12窓比の中央値 1/30:1.133 → 1/60:1.030 → 1/120:0.978）、外部の MMD
互換実装も細刻み（Saba=1/120, libmmd=1/60, MMD は物理最大60fps）だったため、**実効1/60 を
リファレンスに変更**しました。より忠実にしたい場合は `SubSteps=4`（実効1/120）も選べます。
（細刻み化は必ず `SubSteps` で行ってください。`FixedTimeStep` を下げる経路は 30fps 入力の
キネマティック補間が正しく効きません。詳細は [DESIGN.md](DESIGN.md) の「時間刻み」節。）
`MmdPhysicsBehaviour` では `FixedTimeStep`/`SubSteps` を Inspector から設定できます。

## 座標系

エンジンは PMX ネイティブ座標で計算し、境界 (`MmdPhysicsBehaviour`) で Unity へ変換します
(PMX/Unity はどちらも左手系ですが Z 向きが逆のため Z を反転)。

さらに、Unity 側のモデルが縮小配置される運用に合わせ、位置は `UnitScale`
(既定 0.08) で換算します (`MmdToUnityPos` で乗算、`UnityToMmdPos` で除算)。
回転はスケールに無関係なので変換しません。`UnitScale` は Inspector で調整可能です。

## Unity で目視確認する (セットアップ手順)

### 1. `MmdPhysicsBehaviour` でモデルを動かす

空の GameObject か、スキンメッシュのルートに `MmdPhysicsBehaviour` を付け、
Inspector で下表を設定します。

| 項目 | 推奨値 | 説明 |
|---|---|---|
| `PmxPath` | `.pmx` のパス (各自の環境。例 `testdata/IA.pmx` や絶対パス) | 「必要な外部データ」参照 |
| `ModelRoot` | ボーン階層のルート `Transform` | この配下の GameObject 名を PMX ボーン名で解決する |
| `UnitScale` | `0.08` (Unity 側モデルが約 1/12.5 縮小配置の場合) / `1.0` (PMXネイティブ寸法で配置の場合) | 位置換算のみ。回転は無関係 |
| `Gravity` | `98` | MMD スケール (≒ 9.8×10) |
| `FixedTimeStep` | `1/30` | リファレンス刻み。滑らかにしたいだけなら上げてよい |
| `SubSteps` | `1` | 同上 |
| `SolverIterations` | `10` | Bullet 既定と同じ |

`UnitScale` は「Unity シーン上に置いたモデルの実寸 ÷ PMXネイティブ寸法」に合わせます。
髪・スカートが胴体からズレて描かれるときは、まずここが原因を疑ってください。

### 2. ボーン名解決が失敗したときの症状と確認

`ModelRoot` 配下の GameObject 名を `t.name → Transform` の辞書にし、PMX ボーン名で
引きます (`ResolveBones`)。**完全一致**が必要で、インポータがボーン名を
ローマ字化・改名していると一致しません。

- **症状A (ModelRoot 未設定 or 全滅)**: ボーン追従(kinematic)剛体が目標を得られず
  原点(0,0,0)へ吸い寄せられる。Gizmo を出すと**シアンの箱/カプセルが原点に固まる**。
- **症状B (一部不一致)**: 一致した部位だけ動き、不一致の部位は原点へ飛ぶ or
  バインド姿勢のまま。動的ボーン(スカート/髪)の物理結果が書き戻らず**メッシュが変形しない**。
- **確認方法**: `_boneTransforms[i]` が null のボーンが不一致。名前を突き合わせるには、
  一時的に `ResolveBones` で未解決ボーンを `Debug.LogWarning` する行を足すのが確実です
  (PMXボーン名は日本語 `下半身` `スカート_0_0` 等。GameObject 名と一字一句合っているか)。
- **切り分けの近道**: 後述の `BonePoseCsvPlayer` は `ModelRoot` を必要とせず
  CSV のボーン名で直接駆動するため、「物理が壊れているのか、名前解決が壊れているのか」を
  分離できます。

### 3. Gizmo で剛体を可視化する

`MmdPhysicsBehaviour.DrawGizmos = true` (既定) の状態で、Scene ビュー右上の
**Gizmos トグルを ON** にします。`OnDrawGizmos` が全剛体をワイヤーで描きます:

- **シアン** = ボーン追従 (kinematic)、**緑** = 物理 (dynamic)、**黄** = 物理+Bone合わせ
- 形状は PMX ネイティブ寸法に `UnitScale` を掛けて位置と揃えて描画

剛体が原点に固まる / モデルとズレる場合は §2 (名前解決) と `UnitScale` を見直します。

### 4. `BonePoseCsvPlayer` で本家CSVを再生し、本家スカートと重ねて見る

ヘッドレス検証(`HeadlessDriver`)と**同一入力・同一ロジック**で本家ベイク済み
ボーン姿勢CSVを再生し、自前物理の剛体(緑)に**本家スカート剛体をマゼンタのゴースト**で
重ねて表示するコンポーネントです (`Assets/Scripts/Unity/BonePoseCsvPlayer.cs`)。
`ModelRoot` は不要 (Unityボーンには書き戻さず、Gizmo で描くだけの目視専用)。

設定:

| 項目 | 推奨値 |
|---|---|
| `PmxPath` | IA.pmx のパス (各自の環境。既定は空文字=何もしない) |
| `BoneCsvPath` | `IA_bone_world_pose.csv` のパス (既定は空文字。空/未存在ならゴースト無しで物理のみ) |
| `Gravity`/`FixedTimeStep`/`SubSteps`/`SolverIterations`/`WarmupSteps` | `98`/`1/30`/`1`/`10`/`60` (ヘッドレスと同一) |
| `UnitScale` | モデル配置に合わせる (単独で見るだけなら `1.0` でも可) |
| `DrawReferenceGhost` / `SkirtOnlyGhost` | `true` / `true` (本家スカートのみゴースト) |

操作は Inspector でコンポーネント名を**右クリック → ContextMenu** (Input/GUI 不使用):

- **Play / Pause** … 実時間再生 (`PlaybackFps=30` で等速)
- **Step Forward (+1) / Step Back (-1)** … コマ送り
- **Jump to Frame (Frame値へ)** … `Frame` に数値を入れてジャンプ
- **Jump to Window Start / End (窓先頭/末尾)** … `WindowStart=2440`, `WindowEnd=2470`

**コマ送り**: `Jump to Window Start` → `Step Forward` を連打。自前(緑)と本家(マゼンタ)の
スカートの開きを1フレームずつ比較できます。貫入は赤系で強調表示されます（下記「到達状況」参照）。
物理は逆再生できないため、後退/ジャンプは内部でフレーム0から再シミュレーションします
(7000フレームでも一瞬)。`CandidateFrames`（貫入が起きるフレーム候補）へのジャンプもあります。

## Unity での GLB → 自作エンジン統合 (PhysX とのトグル並立)

[mmd2gltf-gui](https://github.com/masaka1024/) で書き出した GLB（`extras.mmd` 付き）を Unity に import し、
物理だけを自作エンジンへ差し替える運用です。**メッシュ／マテリアル（lilToon）／スキン／ベイク再生は
既存の mmd2gltf インポーターのまま**で、物理バックエンドだけを切り替えます。

### 手順
1. **GLB を import** し、既存の mmd2gltf インポーターでマテリアル(lilToon)等を復元する（従来どおり）。
2. モデルのルートに **`MmdPhysicsBehaviour`** を付け、Inspector で:
   - `Source = Glb`、`GlbPath` に import 元の `.glb` パス（`extras.mmd` から剛体/Joint/ボーンを構築）。
   - `ModelRoot` に import 済みスケルトンのルート Transform（ボーン名で解決）。
   - `UnitScale` は **extras.mmd の値（既定0.08）で自動上書き**されるので触らなくてよい。
   - 起動時に **FK-rest 物理リセットが必ず呼ばれる**（初期のスカート沈み込み/暴れを防ぐ）。
3. 同じルートに **`MmdPhysicsBackendSwitch`** を付ける。`Mode = Custom`（自作）/`PhysX`（既存）を
   Inspector か右クリック ContextMenu（`Use Custom` / `Use PhysX`）で切替。
   - `Custom` は PhysX 剛体をパーク（kinematic・衝突無効）+ ConfigurableJoint 無効化して排他にする。
   - `PhysX` は自作エンジンを無効化し、剛体/Joint を元状態へ戻す。**両者は同時に動かない。**

### 目視で確認するポイント
- **起動直後にスカートが暴れない／脚に沈まない**か（FK-rest リセットが効いていれば静かに始まる）。
- ターン時の開き具合が本家相当か（到達値: 平時中央 10.41°/本家 11.39°、12窓比 1.061）。
- `Use Custom` ↔ `Use PhysX` を A/B し、PhysX 版が本家からどれだけずれるかを目視/数値比較する。

### うまく動かないときの確認項目
- **スカート/髪が原点へ吸われる・メッシュが変形しない** → ボーン名解決の失敗。`ModelRoot` 配下の
  GameObject 名が GLB のノード名（= extras.mmd のボーン名 = PMX ボーン名）と一致しているか確認。
- **剛体/Joint が 0 件・警告が出る** → その GLB に `extras.mmd` が無い（古いエクスポータ/`--no-extras`）。
  `python -m mmd2gltf model.pmx -o out.glb` で再出力する（`extras.mmd` は既定で付く）。
- **スカートと脚が両方物理で動いて競合する** → `MmdPhysicsBackendSwitch` が付いているか、`Mode` が
  正しいか確認（`Custom` なら PhysX 剛体はパークされる）。

## 既知の制限

- SoftBody のクラスタ / AeroModel は簡易対応 (PMX 仕様でも「精度・速度に問題あり・非対応も選択肢」と明記)。
- ConeTwist / Slider / Hinge のモーターは未対応 (仕様上も「暫定対応」)。
- 数値は Bullet 2.75 と厳密一致ではなく挙動互換を目標とします。

## 検証 (ハーネスの回し方)

`scratchpad/compilecheck` に UnityEngine の最小シムを使った **Unity 非依存の検証ハーネス**を
用意しています（合成4シナリオ + IA.pmx スモーク + 静止押し出し回帰 + キネマティック補間回帰 = 11項目）。

```bash
cd scratchpad/compilecheck
dotnet run -c Release
```

- 全 `.csproj` は `<LangVersion>9.0</LangVersion>`（Unity=C# 9 と同一）でビルドされ、C# 10 以降の
  機能が混入すると**ハーネスの時点でビルドが落ちます**。
- IA モデル/CSV は各自用意（下記「必要な外部データ」）。**未指定なら IA 依存の項目は SKIP** され、
  Unity 非依存の項目だけが走ります（落ちません）。
- `scratchpad/compilecheck/<name>/` 以下は各種の**診断ツール**（貫入・傾き・刻み掃引・物理リセット
  検証など）。`cd <name> && dotnet run -c Release` で個別に実行できます。
