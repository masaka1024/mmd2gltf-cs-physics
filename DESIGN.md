---
description: Unity Bullet互換物理エンジンプロジェクトの設計文書
metadata:
  type: project
---

# Unity Bullet互換物理エンジン設計

## 概要
PmxEditor(極北P)のBullet 2.75準拠物理(剛体・Joint・SoftBody)をUnity C#で再実装。
PMX 2.1仕様に準拠したMMD物理演算エンジン。外部ネイティブ依存なし。

## 実装状況 (2026-08-07 実測・コンパイル検証済み)
- Core Math (Vec3/Quat/Matrix4x4/Matrix3x3/RigidTransform) — 完了
- RigidBody (形状/質量/減衰/反発/摩擦, static/dynamic/kinematic) — 完了
- 衝突 (GJK+EPA ナローフェーズ, 永続マニフォールド) — 完了
- Joint 6種 (行ベース Sequential-Impulse 6DOF + バネ) — 完了
- PhysicsWorld (ブロードフェーズ/接触ソルバ/重力/固定ステップ積分/Joint) — 完了
- SoftBody (質点-バネ Rope/TriMesh, B-Link, Anchor, Pin) — 完了(簡易)
- PMX Reader (2.0/2.1 全セクションskip対応 + 剛体/Joint/SoftBody抽出) — 完了
- PmxPhysicsBuilder (PMX→World変換 + ボーン紐付け) — 完了
- Unity ブリッジ MmdPhysicsBehaviour (座標変換/ボーン同期) — 完了

## 残タスク / 今後
- SoftBody のクラスタ / AeroModel の本格対応
- Joint モーター (ConeTwist/Slider/Hinge) の対応
- インパルスモーフ (剛体への速度/トルク付加) のUnity層配線
- Bullet 2.75 との数値検証 (現状は挙動互換レベル)
- Box-Box 多点マニフォールドの安定化 (現状 GJK/EPA の単一点+蓄積)

## PmxEditor(PMX 2.1仕様)とのマッピング
- 剛体 -> RigidBody (btRigidBody)  形状 0:球 1:箱 2:カプセル
- 物理演算タイプ 0:ボーン追従(kinematic) 1:物理(dynamic) 2:物理+Bone位置合わせ
- Joint種類0: ﾊﾞﾈ付6DOF -> btGeneric6DofSpringConstraint
- Joint種類1: 6DOF -> btGeneric6DofConstraint
- Joint種類2: P2P -> btPoint2PointConstraint
- Joint種類3: ConeTwist -> btConeTwistConstraint
- Joint種類4: Slider -> btSliderConstraint
- Joint種類5: Hinge -> btHingeConstraint
- SoftBody -> SoftBody (btSoftBody: TriMesh/Rope)
- 衝突グループ: PMX の16bitは「衝突する相手グループ」のマスク(bit=1で衝突)。
  Bullet の (groupA & maskB) && (groupB & maskA) で判定

## 座標系
エンジンはPMXネイティブ座標で計算。Unity境界(MmdPhysicsBehaviour)でZ反転変換。
重力はMMDスケールで約 -98 (= -9.8 * 10) を推奨。
