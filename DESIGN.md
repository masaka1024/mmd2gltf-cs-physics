---
description: Unity Bullet互換物理エンジンプロジェクトの設計文書
metadata:
  type: project
---

# Unity Bullet互換物理エンジン設計

## 概要
PmxEditor(極北P)のBullet 2.75準拠物理(剛体・Joint・SoftBody)をUnity C#で再実装。
PMX 2.1仕様に準拠したMMD物理演算エンジン。外部ネイティブ依存なし。

## 実装状況 (2026-08-07 実測・コンパイル/ランタイム検証済み)
- Core Math (Vec3/Quat/Matrix4x4/Matrix3x3/RigidTransform) — 完了
- RigidBody (形状/質量/減衰/反発/摩擦, static/dynamic/kinematic) — 完了
- 衝突 (GJK+EPA ナローフェーズ, 永続マニフォールド) — 完了
- Joint 6種 (行ベース Sequential-Impulse 6DOF + バネ) — 完了
- PhysicsWorld (ブロードフェーズ/接触ソルバ/重力/固定ステップ積分/Joint) — 完了
- SoftBody (質点-バネ Rope/TriMesh, B-Link, Anchor, Pin) — 完了(簡易)
- PMX Reader (2.0/2.1 全セクションskip対応 + 剛体/Joint/SoftBody抽出) — 完了
- PmxPhysicsBuilder (PMX→World変換 + ボーン紐付け) — 完了
- Unity ブリッジ MmdPhysicsBehaviour (座標変換/単位スケール/ボーン同期) — 完了

## 不具合修正 (2026-08-07)
- A: Matrix3x3.FromQuat の転置バグ (Rᵀ→R)。慣性テンソル/軸/オイラー角が正常化
- B: 減衰を Bullet 2.75 の秒単位 pow(1-d,dt) に修正
- C: 衝突グループを正しい向き (bit=1で衝突) に反転・CollisionMask へ改名
- D: PersistentManifold 接触点をローカル座標から再投影 (幽霊接触の除去)
- E: キネマティック剛体をサブステップ間で補間 (等速時の速度破綻を解消)
- F: Unity 境界に単位スケール (UnitScale) 換算を追加

## 残タスク / 今後
- SoftBody のクラスタ / AeroModel の本格対応
- Joint モーター (ConeTwist/Slider/Hinge) の対応
- インパルスモーフ (剛体への速度/トルク付加) のUnity層配線
- Bullet 2.75 との数値検証 (現状は挙動互換レベル)
- Box-Box 多点マニフォールドの安定化 (現状 GJK/EPA の単一点+蓄積)
- G: 既定ステップ設定の MMD 本家寄せ (FixedTimeStep=1/30, SubSteps=1) は
  影響分析済み・採否は保留 (人間判断待ち)

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

## 時間刻み (リファレンス: 30Hz・1サブ)
既定は `FixedTimeStep = 1/30`, `SubSteps = 1`。
MMD本家は 30fps で 1描画フレーム = 1物理ステップとして Bullet を回しており、
PMXエディタのベイク出力もその系譜にある。回帰・検証を本番と同じ刻みで積むため、
この 30Hz・1サブを再現対象 (リファレンス) とみなす設計方針とする。
より滑らかにしたい場合は呼び出し側で FixedTimeStep/SubSteps を上げてよい
(細かい刻みほど貫入は浅く安定するが、本家挙動からは離れる)。

## 座標系
エンジンはPMXネイティブ座標で計算。Unity境界(MmdPhysicsBehaviour)でZ反転変換。
重力はMMDスケールで約 -98 (= -9.8 * 10) を推奨。
