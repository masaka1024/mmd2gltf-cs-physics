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

## 座標系

エンジンは PMX ネイティブ座標で計算し、境界 (`MmdPhysicsBehaviour`) で Unity へ変換します
(PMX/Unity はどちらも左手系ですが Z 向きが逆のため Z を反転)。

## 既知の制限

- SoftBody のクラスタ / AeroModel は簡易対応 (PMX 仕様でも「精度・速度に問題あり・非対応も選択肢」と明記)。
- ConeTwist / Slider / Hinge のモーターは未対応 (仕様上も「暫定対応」)。
- 数値は Bullet 2.75 と厳密一致ではなく挙動互換を目標とします。

## 検証

`scratchpad/compilecheck` に Unity 非依存のコンパイル検証と、自由落下・接地・
Joint 保持・バネ振動のランタイム検証を用意しています (UnityEngine の最小シムを使用)。
