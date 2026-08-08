# Unity Bullet 互換 物理エンジン (MMD/PMX 物理)

PmxEditor (極北P) の **PMX 2.1 仕様** に記述された物理演算を、**Bullet 2.75** の
挙動に合わせて Unity C# で再実装したものです。MMD モデルの剛体・Joint・SoftBody を
外部ネイティブライブラリ (BulletSharp 等) 無しで動かすことを目的にしています。

## 特徴

- 依存ゼロの純 C# 実装 (Unity 標準アセンブリのみ)
- Bullet の Sequential-Impulse ソルバを踏襲
- PMX バイナリ (2.0 / 2.1) から剛体・Joint・SoftBody を直接読み込み
- MMD ↔ Unity 座標変換込みのボーン同期

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

## 時間刻み (リファレンス: 30Hz・1サブ)

既定は `FixedTimeStep = 1/30`, `SubSteps = 1` です。MMD 本家は 30fps で
1 描画フレーム = 1 物理ステップとして Bullet を回しており、PMX エディタのベイク出力も
その系譜にあります。本プロジェクトはこの **30Hz・1サブを再現対象 (リファレンス)** とし、
回帰・検証も本番と同じ刻みで行う設計方針です。より滑らかにしたい場合は
`FixedTimeStep`/`SubSteps` を上げられます (細かい刻みほど貫入は浅く安定しますが本家挙動から離れます)。
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
| `PmxPath` | `.pmx` の絶対パス (または Assets 相対) | 例 `C:\...\IA1\IA.pmx` |
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
| `PmxPath` | IA.pmx の絶対パス |
| `BoneCsvPath` | `IA_bone_world_pose.csv` の絶対パス |
| `Gravity`/`FixedTimeStep`/`SubSteps`/`SolverIterations`/`WarmupSteps` | `98`/`1/30`/`1`/`10`/`60` (ヘッドレスと同一) |
| `UnitScale` | モデル配置に合わせる (単独で見るだけなら `1.0` でも可) |
| `DrawReferenceGhost` / `SkirtOnlyGhost` | `true` / `true` (本家スカートのみゴースト) |

操作は Inspector でコンポーネント名を**右クリック → ContextMenu** (Input/GUI 不使用):

- **Play / Pause** … 実時間再生 (`PlaybackFps=30` で等速)
- **Step Forward (+1) / Step Back (-1)** … コマ送り
- **Jump to Frame (Frame値へ)** … `Frame` に数値を入れてジャンプ
- **Jump to Window Start / End (窓先頭/末尾)** … `WindowStart=2440`, `WindowEnd=2470`

**窓6 (F2440〜F2470) のコマ送り**: `Jump to Window Start` → `Step Forward` を連打。
自前(緑)と本家(マゼンタ)のスカートの開きを1フレームずつ比較できます
(この窓で自前 92.2°・本家 62.9°)。物理は逆再生できないため、後退/ジャンプは
内部でフレーム0から再シミュレーションします (7000フレームでも一瞬)。

## 既知の制限

- SoftBody のクラスタ / AeroModel は簡易対応 (PMX 仕様でも「精度・速度に問題あり・非対応も選択肢」と明記)。
- ConeTwist / Slider / Hinge のモーターは未対応 (仕様上も「暫定対応」)。
- 数値は Bullet 2.75 と厳密一致ではなく挙動互換を目標とします。

## 検証

`scratchpad/compilecheck` に Unity 非依存のコンパイル検証と、自由落下・接地・
Joint 保持・バネ振動のランタイム検証を用意しています (UnityEngine の最小シムを使用)。
