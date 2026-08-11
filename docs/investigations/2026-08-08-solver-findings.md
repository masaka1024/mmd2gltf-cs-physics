# 髪の静止爆散 調査ノート (タスクA〜D)

modelA.pmx の髪(動的剛体53個)が静止(アニメOFF)でも bind から最大8動く問題の原因調査。
本体コードは無改変。計測ハーネスのみ追加。

## タスクA: PMXを外した最小再現 (chainbug, TASK=A)
kinematic固定点に N剛体を全DOFロック6DOFで直列。重力-98/dt=1/30/subs2/300step。理論値=全て0。

maxDrift:

| N＼iters | 10 | 20 | 40 | 100 |
|---|---|---|---|---|
| 1 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 2 | 0.0001 | 0.0000 | 0.0000 | 0.0000 |
| 3 | 0.0081 | 0.0004 | 0.0000 | 0.0000 |
| 4 | 0.0362 | 0.0062 | 0.0002 | 0.0000 |
| 5 | 0.0834 | 0.0224 | 0.0026 | 0.0000 |
| 6 | 0.1474 | 0.0491 | 0.0098 | 0.0001 |
| 8 | 0.3360 | 0.1298 | 0.0409 | 0.0032 |
| 10 | 0.6194 | 0.2471 | 0.0919 | 0.0143 |

質量分布(iters10): 先端ほど軽いと約半減 (N10: 均一0.619 / 先端軽0.249)。

**結論:** PMX/髪/接触/バネ/角度リミット 全て無関係のソルバ単体バグ。深さN≥3から漏れ、
髪深さ6で0.15。反復増で単調に0へ収束＝ウォームスタート無しのPGSがチェーンに
インパルスを伝播しきれない。スカート深さ3は通り髪深さ5〜6で壊れる仮説を定量確認。

## タスクB: サブステップ悪化の再検証 (chainbug TASK=B, restsim BETA/SUBSTEPS)
合成チェーン (iters10, maxDrift/maxSpeed):

| N=6 beta＼subs | 1 | 2 | 4 | 8 |
|---|---|---|---|---|
| 0.20 | 0.590/2.58 | 0.147/1.24 | 0.037/0.59 | 0.009/0.13 |
| 0.00 | 35.24/3.54 | 17.65/1.77 | 8.83/0.88 | 4.42/0.44 |

RestSim モデルA (maxSpeed/maxDrift):

| beta | subs1 | subs2 | subs4 | subs8 |
|---|---|---|---|---|
| default | 18.95/8.38 | 18.65/8.17 | 19.25/8.00 | **34.46**/7.74 |
| 0 | 18.94/37.3 | 18.49/21.5 | 18.42/13.1 | **81.31**/9.19 |

**結論(事前想定を2つ覆す):**
1. Baumgarte注入源説は否定。Beta=0でdriftは激増(位置保持そのもの)。
2. サブステップ悪化も合成チェーンでは否定(Beta0.2でsubs増→drift単調改善)。
残る別現象: モデルA RestSim でのみ maxSpeed が subs8 で急増(18→34, Beta非依存)。
合成に無くコアのチェーン未収束とは別のモデルA固有スパイク(角度リミットEuler分解/
キネマティック補間/接触のいずれかが細刻みで悪化)。要追跡。

## タスクC: 求解順序の影響 (chainbug TASK=C, restsim ORDER) ※既定は不変
合成チェーン maxDrift:

| order＼N | 4 | 6 | 8 | 10 |
|---|---|---|---|---|
| root2leaf | 0.0362 | 0.1474 | 0.3360 | 0.6194 |
| leaf2root | 0.0582 | 0.2081 | 0.4359 | 0.7596 |
| shuffle | 0.0497 | 0.1814 | 0.3911 | 0.6807 |

RestSim モデルA maxDrift: current 8.166 / root2leaf 7.960 / leaf2root 8.369

**結論:** root→leaf 最良・leaf→root 最悪で伝播方向依存は実在だが改善は小
(合成N10で約18%、モデルAで約5%)。「大きく改善」には非該当。順序は副次要因、
reorderは根本解にならない。

## タスクD: Bullet 2.75 実ソース (調査のみ, 実装しない)
※タグ 2.75 は bullet3 に存在せず 404。近接タグ 2.83 を使用
(warm-start/split-impulse の構造は 2.7x〜2.8x で安定)。
src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp,
btGeneric6DofConstraint.cpp を確認。

- **(1) ウォームスタート有り**: SIソルバは `SOLVER_USE_WARMSTARTING`(既定ON)時、
  毎ステップ `m_appliedImpulse = 前回impulse * m_warmstartingFactor` を再適用。接触・摩擦・ジョイント行。
- **(2) ジョイント行はステップ間で引き継ぐ**: 既定の getInfo2→SIソルバ経路は
  `internalSetAppliedImpulse` で累積を保存＝warm-startされる。
  旧式 btRotationalLimitMotor/btTranslationalLimitMotor(`m_useSolveConstraintObsolete=true`,
  非既定)のみ `buildJacobian()` で `m_accumulatedImpulse=0` を毎ステップ実行。
- **(3) ERPバイアスの分離**: split-impulse機構。combined時は実速度rhs
  (`m_rhs = penetrationImpulse+velocityImpulse`)。split時は分離
  (`m_rhs=velocityImpulse; m_rhsPenetration=penetrationImpulse`) し
  `resolveSplitPenetration...` で擬似push/turn速度へ独立適用。

**結論:** 自作エンジンは warm-start 無し = Bullet(既定)から乖離。warm-start は
「Bullet忠実化として必須」。前回の warm-start が発散したのは、バイアスを実速度rhsに
残したまま引き継いだため(Bulletは split-impulse でバイアスを擬似速度へ分離し安定化)。
自作エンジンには split-impulse基盤(PseudoVelocity/ApplyPushImpulse/SolveSplitImpulse)が
既存だが接触専用・UseSplitImpulse既定OFF。

## 総括 (原因の確定)
主因 = **ジョイント行のウォームスタート欠如による長チェーンのPGS未収束**(タスクAで
PMX非依存に最小再現、タスクDでBullet忠実性として必須と確認)。
副次 = 求解順序(root→leaf が僅かに良, タスクC)、モデルA固有の高subsスパイク(タスクB)。
提案する修正方針(要合意): (a) ジョイント行の warm-start 実装 + (b) それを安定化する
ため Baumgarte バイアスを split-impulse(擬似速度)へ分離。既定を変える変更のため未着手。
