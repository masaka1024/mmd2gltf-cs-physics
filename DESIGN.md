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

## 試して破棄した方針 (再挑戦しないための記録)

### ジョイント行の蓄積インパルス引き継ぎ (warm-start) — 破棄
Bullet準拠 (係数0.85=m_warmstartingFactor) でジョイント制約行のウォームスタートを実装したが悪化。
X角度リミット(±80°)超過 最大 90.9°→111.5°、平時中央値 11.05→11.49。
見立て: 角度リミット行は Euler分解で軸/側が毎フレームトグルするため、引き継いだ力積が
別軸/別側へ適用され復元でなく擾乱になる。
※接触の PersistentManifold ウォームスタートは有効なままで、そちらは変更しない。

### バネのモーター化 (2-b) — 実装せず打ち切り
バネをBullet同様のソルバ反復内モーターへ変える案は、対象モデル(IA)のスカートJointが
バネ未使用 (SpringAngular=SpringLinear=0, 制御は角度リミットのみ) のため効果ゼロと判明し、打ち切った。
(参考: 6DOF バネには粘性項が無いことを確認し、当方の `-SpringDamping*k*relVel` 粘性項は除去済み。)

### 角度行のインパルス軸を Bullet 風混合軸へ (破棄)
`Constraints.cs` の角度リミット行の軸を、A のフレーム列(直交)から
Bullet の `btGeneric6DofConstraint::calculateTransforms` 風の混合軸
(A.Z と B.X 由来、一般に非直交) へ変える案を試したが、**全体的に悪化**したため破棄した。
- IA.pmx で 平時中央値 11.05→13.09、窓の(自前/本家)傾きmax比の中央値 1.12→1.5 と悪化。
  ほぼ全窓で悪化 (窓1: 81.5→164.7 など)。ring2 の1フレームスパイクも消えなかった。
- 推測: 非直交軸は角度行同士がカップリングし、逐次インパルス(Gauss-Seidel 10反復)では
  直交軸より収束が悪い。加えて Bullet の `m_calculatedAxis[i]` と `matrixToEulerXYZ` の
  (x,y,z) 対応順序をソース未確認のまま実装したため、軸と誤差値の対応がずれていた可能性。
- 未確認: Bullet 2.75 が非直交軸をソルバでどう条件付けしているか。

## 既知の数値特性 (チューニング時の前提)
以降のパラメータ調整で混乱しないよう、実測で判明した数値特性を記録する
(単位はPMXネイティブ単位。1単位 ≒ 0.125m 目安)。

### 投機的接触は (b) 方式
接触生成には投機的マージン `SpeculativeMargin = 0.02` を用いるが、押し出し方は
本来の投機的接触 (b) である。すなわち非貫入 (`Distance > 0`) の接触では目標接近速度を
`-Distance/dt` とし、「このステップで表面へちょうど到達する接近」までは許して
貫入だけを止める。**クリアランスは膨らまない** (接地する球はマージン分浮かず真の接触面で静止)。
マージンの役割は「表面が触れる少し手前から接触を連続的に検出し、貫入 on/off の
振動を防ぐ先読み」であり、静止物体を押し離す力にはならない
(重力0・1ステップで非貫入接触への法線インパルスは0件。回帰テストで監視)。

### 初期条件感度 (背景ノイズとしての軌道分岐)
`SpeculativeMargin` を 0→0.02 に変えると、重力0・300ステップでの動的剛体の
平均変位が **0.2086 → 0.2649** (差 **0.076 unit ≒ 9.5mm**) 分岐する。
これは**クリアランス膨張ではなく**、検出バンド幅が広がることで
バインドポーズの初期貫入を解消する過渡により多くの投機拘束が働き、
軌道が分岐する**カオス的発散**である (差はマージンに対し劣線形。膨張なら線形のはず)。
**パラメータの効果を評価する際は、この程度 (~0.08 unit) の軌道分岐が
背景ノイズとして常に存在することを前提に判断すること。**

### 刻みによる貫入量 (30Hz・1サブ vs 60Hz・2サブ)
IA.pmx (剛体117) 300ステップ・重力-98 での貫入量:

| 刻み | 貫入 平均 (貫入接触のみ) | 貫入 最大 |
|---|---|---|
| 30Hz・1サブ (既定) | 0.0561 | 1.023 |
| 60Hz・2サブ | 0.0288 | 0.845 |

30Hz は1ステップの移動量が大きい分、平均貫入で約2倍・最大で+21% 深くなる
(想定内)。(b) 投機的接触により、深い貫入でも Baumgarte の暴発 (エネルギー注入) は
起きず NaN・速度爆発なしで回復する。EPA上限hitも0。

### リファレンス刻み
既定を 30Hz・1サブとするのは、MMD本家が 30fps で 1描画フレーム=1物理ステップとして
Bullet を回しており、その刻みを再現対象とみなす設計方針のため (「時間刻み」節参照)。
