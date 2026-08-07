// ===========================================================================
// Unity Bullet 互換物理エンジン – Joint Constraints
// PMX Joint 6 種を Bullet 相当の Sequential-Impulse で解く。
//   0: ﾊﾞﾈ付6DOF -> btGeneric6DofSpringConstraint
//   1: 6DOF      -> btGeneric6DofConstraint
//   2: P2P       -> btPoint2PointConstraint
//   3: ConeTwist -> btConeTwistConstraint
//   4: Slider    -> btSliderConstraint
//   5: Hinge     -> btHingeConstraint
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    public enum JointType
    {
        Spring6Dof = 0,
        Generic6Dof = 1,
        Point2Point = 2,
        ConeTwist = 3,
        Slider = 4,
        Hinge = 5,
    }

    /// <summary>制約ソルバの 1 行 (linear または angular)。</summary>
    internal struct ConstraintRow
    {
        public Vec3 Axis;         // ワールド軸 (単位)
        public bool Angular;      // true=回転行, false=並進行
        public Vec3 RelA, RelB;   // 重心からのアンカーオフセット (linear 用)
        public float LowerImpulse, UpperImpulse;
        public float TargetVel;   // 目標相対速度 (Baumgarte バイアス込み)
        public float EffMass;
        public float Accumulated;
    }

    /// <summary>
    /// 全 Joint 種の基盤となる 6DOF 制約。
    /// PMX の位置/回転/移動制限/回転制限/バネ定数をそのまま解釈する。
    /// </summary>
    public sealed class Joint
    {
        public string Name = string.Empty;
        public JointType Type = JointType.Spring6Dof;

        public RigidBody BodyA;
        public RigidBody BodyB;

        // 剛体ローカルに変換済みのジョイントフレーム。
        public RigidTransform FrameInA = RigidTransform.Identity;
        public RigidTransform FrameInB = RigidTransform.Identity;

        // 移動/回転の制限 (下限, 上限)。
        public Vec3 LinearLowerLimit;
        public Vec3 LinearUpperLimit;
        public Vec3 AngularLowerLimit;
        public Vec3 AngularUpperLimit;

        // バネ定数 (移動/回転)。0 で無効。
        public Vec3 SpringLinear;
        public Vec3 SpringAngular;

        // バネのダンピング比 (PMX には無いので既定値)。
        public float SpringDamping = 0.1f;

        // 位置補正係数 (Baumgarte)。
        public float Beta = 0.2f;
        public const float MaxCorrectionVel = 10f;

        // 内部状態。
        private readonly List<ConstraintRow> _rows = new(6);
        private RigidTransform _worldA, _worldB;
        private Vec3 _anchorA, _anchorB;
        private Vec3[] _axesA = new Vec3[3];

        // --- ファクトリ: PMX Joint 種から生成 ---

        /// <summary>PMX の raw パラメータから Joint を構築する。</summary>
        public static Joint FromPmx(
            JointType type, RigidBody a, RigidBody b,
            RigidTransform worldFrame,
            Vec3 linLo, Vec3 linHi, Vec3 angLo, Vec3 angHi,
            Vec3 springLin, Vec3 springAng)
        {
            var j = new Joint
            {
                Type = type,
                BodyA = a,
                BodyB = b,
                LinearLowerLimit = linLo,
                LinearUpperLimit = linHi,
                AngularLowerLimit = angLo,
                AngularUpperLimit = angHi,
                SpringLinear = springLin,
                SpringAngular = springAng,
            };

            // フレームを各剛体ローカルへ落とし込む。
            j.FrameInA = a != null ? a.WorldTransform.InverseTimes(worldFrame) : worldFrame;
            j.FrameInB = b != null ? b.WorldTransform.InverseTimes(worldFrame) : worldFrame;

            // Joint 種ごとの制限の解釈調整 (仕様の対応表に準拠)。
            switch (type)
            {
                case JointType.Point2Point:
                    // 各制限/バネ無効。並進のみ固定。
                    j.AngularLowerLimit = new Vec3(1); // lo>hi = 回転フリー
                    j.AngularUpperLimit = new Vec3(-1);
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    j.SpringLinear = j.SpringAngular = Vec3.Zero;
                    break;

                case JointType.Hinge:
                    // X 軸のみ回転可。並進固定。
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    j.AngularLowerLimit = new Vec3(angLo.x, 0, 0);
                    j.AngularUpperLimit = new Vec3(angHi.x, 0, 0);
                    break;

                case JointType.Slider:
                    // X 軸のみ並進/回転可。
                    j.LinearLowerLimit = new Vec3(linLo.x, 0, 0);
                    j.LinearUpperLimit = new Vec3(linHi.x, 0, 0);
                    j.AngularLowerLimit = new Vec3(angLo.x, 0, 0);
                    j.AngularUpperLimit = new Vec3(angHi.x, 0, 0);
                    break;

                case JointType.ConeTwist:
                    // 並進固定。回転は円錐 (Y,Z) + 捻り (X)。
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    break;

                case JointType.Generic6Dof:
                    // バネ無効。
                    j.SpringLinear = j.SpringAngular = Vec3.Zero;
                    break;

                case JointType.Spring6Dof:
                default:
                    break;
            }
            return j;
        }

        private static bool IsLocked(float lo, float hi) => lo == hi;
        private static bool IsFree(float lo, float hi) => lo > hi;

        // --- 準備: フレーム/アンカー/行を構築 ---
        public void Prepare(float dt)
        {
            _rows.Clear();
            if (BodyA == null || BodyB == null) return;

            _worldA = BodyA.WorldTransform * FrameInA;
            _worldB = BodyB.WorldTransform * FrameInB;
            _anchorA = _worldA.Origin;
            _anchorB = _worldB.Origin;

            var rA = _anchorA - BodyA.CenterOfMass;
            var rB = _anchorB - BodyB.CenterOfMass;

            var basisA = Matrix3x3.FromQuat(_worldA.Rotation);
            _axesA[0] = basisA.Column(0);
            _axesA[1] = basisA.Column(1);
            _axesA[2] = basisA.Column(2);

            var invDt = dt > 0 ? 1f / dt : 0f;
            var linDelta = _anchorB - _anchorA;

            // --- 並進 3 軸 ---
            for (int i = 0; i < 3; i++)
            {
                var axis = _axesA[i];
                float lo = LinearLowerLimit[i], hi = LinearUpperLimit[i];
                if (IsFree(lo, hi)) continue;

                float cur = linDelta.Dot(axis);
                float err, lower, upper;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; }
                else continue; // 制限内 → バネのみ (後段)

                AddLinearRow(axis, rA, rB, Clamp(err * Beta * invDt), lower, upper);
            }

            // --- 回転 3 軸 ---
            var qRel = _worldA.Rotation.Conjugated() * _worldB.Rotation;
            var euler = ToEulerXYZ(qRel.Normalized);
            for (int i = 0; i < 3; i++)
            {
                var axis = _axesA[i];
                float lo = AngularLowerLimit[i], hi = AngularUpperLimit[i];
                if (IsFree(lo, hi)) continue;

                float cur = euler[i];
                float err, lower, upper;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; }
                else continue;

                AddAngularRow(axis, Clamp(err * Beta * invDt), lower, upper);
            }
        }

        private static float Clamp(float v) =>
            Math.Max(-MaxCorrectionVel, Math.Min(MaxCorrectionVel, v));

        private void AddLinearRow(Vec3 axis, Vec3 rA, Vec3 rB, float targetVel, float lo, float hi)
        {
            var rAxn = Vec3.Cross(rA, axis);
            var rBxn = Vec3.Cross(rB, axis);
            float k = BodyA.InverseMass + BodyB.InverseMass
                    + rAxn.Dot(BodyA.InverseInertiaWorld * rAxn)
                    + rBxn.Dot(BodyB.InverseInertiaWorld * rBxn);
            _rows.Add(new ConstraintRow
            {
                Axis = axis, Angular = false, RelA = rA, RelB = rB,
                LowerImpulse = lo, UpperImpulse = hi,
                TargetVel = targetVel, EffMass = k > 0 ? 1f / k : 0f,
            });
        }

        private void AddAngularRow(Vec3 axis, float targetVel, float lo, float hi)
        {
            float k = axis.Dot(BodyA.InverseInertiaWorld * axis)
                    + axis.Dot(BodyB.InverseInertiaWorld * axis);
            _rows.Add(new ConstraintRow
            {
                Axis = axis, Angular = true,
                LowerImpulse = lo, UpperImpulse = hi,
                TargetVel = targetVel, EffMass = k > 0 ? 1f / k : 0f,
            });
        }

        // --- バネ (明示力積, サブステップ毎に 1 回) ---
        public void ApplySprings(float dt)
        {
            if (BodyA == null || BodyB == null) return;
            bool hasLin = SpringLinear.LengthSquared > 0;
            bool hasAng = SpringAngular.LengthSquared > 0;
            if (!hasLin && !hasAng) return;

            var rA = _anchorA - BodyA.CenterOfMass;
            var rB = _anchorB - BodyB.CenterOfMass;
            var linDelta = _anchorB - _anchorA;

            if (hasLin)
            {
                for (int i = 0; i < 3; i++)
                {
                    float k = SpringLinear[i];
                    if (k <= 0) continue;
                    var axis = _axesA[i];
                    float eq = ClampToLimit(0f, LinearLowerLimit[i], LinearUpperLimit[i]);
                    float err = linDelta.Dot(axis) - eq;
                    // Bullet の 6DOF バネには速度比例の粘性項が無いので付けない (force = -delta*k のみ)。
                    float impulse = (-k * err) * dt;
                    var P = axis * impulse;
                    BodyA.ApplyImpulse(-P, rA);
                    BodyB.ApplyImpulse(P, rB);
                }
            }

            if (hasAng)
            {
                var qRel = _worldA.Rotation.Conjugated() * _worldB.Rotation;
                var euler = ToEulerXYZ(qRel.Normalized);
                for (int i = 0; i < 3; i++)
                {
                    float k = SpringAngular[i];
                    if (k <= 0) continue;
                    var axis = _axesA[i];
                    float eq = ClampToLimit(0f, AngularLowerLimit[i], AngularUpperLimit[i]);
                    float err = euler[i] - eq;
                    // Bullet の 6DOF バネには速度比例の粘性項が無いので付けない (force = -delta*k のみ)。
                    float impulse = (-k * err) * dt;
                    var L = axis * impulse;
                    BodyA.ApplyTorqueImpulse(-L);
                    BodyB.ApplyTorqueImpulse(L);
                }
            }
        }

        private static float ClampToLimit(float v, float lo, float hi)
        {
            if (lo > hi) return v;      // free
            return Math.Max(lo, Math.Min(hi, v));
        }

        // --- 速度反復 (world から複数回呼ばれる) ---
        public void SolveVelocity()
        {
            for (int r = 0; r < _rows.Count; r++)
            {
                var row = _rows[r];
                float relVel = row.Angular
                    ? (BodyB.AngularVelocity - BodyA.AngularVelocity).Dot(row.Axis)
                    : (BodyB.VelocityAtPoint(_anchorB) - BodyA.VelocityAtPoint(_anchorA)).Dot(row.Axis);

                float dImpulse = (row.TargetVel - relVel) * row.EffMass;
                float old = row.Accumulated;
                row.Accumulated = Math.Max(row.LowerImpulse, Math.Min(row.UpperImpulse, old + dImpulse));
                dImpulse = row.Accumulated - old;

                if (row.Angular)
                {
                    var L = row.Axis * dImpulse;
                    BodyA.ApplyTorqueImpulse(-L);
                    BodyB.ApplyTorqueImpulse(L);
                }
                else
                {
                    var P = row.Axis * dImpulse;
                    BodyA.ApplyImpulse(-P, row.RelA);
                    BodyB.ApplyImpulse(P, row.RelB);
                }
                _rows[r] = row;
            }
        }

        // --- Euler XYZ 抽出 (Bullet の matrixToEulerXYZ 相当) ---
        internal static Vec3 ToEulerXYZ(Quat q)
        {
            var m = Matrix3x3.FromQuat(q);
            // 行列から XYZ オイラー角を復元。
            float m02 = m.Row0.z;
            float y, x, z;
            if (m02 < 1f - 1e-6f)
            {
                if (m02 > -1f + 1e-6f)
                {
                    y = (float)Math.Asin(Math.Clamp(m02, -1f, 1f));
                    x = (float)Math.Atan2(-m.Row1.z, m.Row2.z);
                    z = (float)Math.Atan2(-m.Row0.y, m.Row0.x);
                }
                else
                {
                    y = -(float)(Math.PI / 2);
                    x = -(float)Math.Atan2(m.Row1.x, m.Row1.y);
                    z = 0f;
                }
            }
            else
            {
                y = (float)(Math.PI / 2);
                x = (float)Math.Atan2(m.Row1.x, m.Row1.y);
                z = 0f;
            }
            return new Vec3(x, y, z);
        }
    }
}
