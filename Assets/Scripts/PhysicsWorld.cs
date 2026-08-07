// ===========================================================================
// Unity Bullet 互換物理エンジン – PhysicsWorld
// btDiscreteDynamicsWorld 相当。重力・積分・衝突・Joint を統合する。
// Sequential-Impulse ソルバ + Baumgarte 位置補正。
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    /// <summary>接触制約 (1 接触点 = 法線 + 2 摩擦)。</summary>
    internal struct ContactConstraint
    {
        public RigidBody A, B;
        public Vec3 RelA, RelB;
        public Vec3 Normal, Tangent1, Tangent2;
        public float NormalMass, TangentMass1, TangentMass2;
        public float NormalBias;       // Baumgarte + restitution
        public float Friction;
        public float NormalImpulse, TangentImpulse1, TangentImpulse2;
        public PersistentManifold Manifold; public int PointRef; // ウォームスタート書き戻し用
    }

    /// <summary>物理ワールド。剛体と Joint を保持しシミュレートする。</summary>
    public sealed class PhysicsWorld
    {
        public Vec3 Gravity = new(0f, -9.8f * 10f, 0f); // MMD スケール: 重力は約 98

        // ソルバ設定。
        public int SolverIterations = 10;
        public int SubSteps = 2;
        public float FixedTimeStep = 1f / 60f;

        public float PenetrationSlop = 0.005f;
        public float BaumgarteFactor = 0.2f;
        public float RestitutionThreshold = 1.0f;

        public readonly List<RigidBody> Bodies = new();
        public readonly List<Joint> Joints = new();

        private readonly Dictionary<long, PersistentManifold> _manifolds = new();
        private readonly List<ContactConstraint> _contacts = new();
        private float _accumulator;

        public void AddBody(RigidBody b)
        {
            b.Index = Bodies.Count;
            // static/kinematic の目標姿勢を現在姿勢で初期化 (未設定でのテレポート防止)。
            // ボーン追従はこの後 MmdPhysicsBehaviour が毎フレーム上書きする。
            if (b.IsStaticOrKinematic)
                b.KinematicTarget = b.WorldTransform;
            Bodies.Add(b);
        }

        public void AddJoint(Joint j) => Joints.Add(j);

        // --- 公開ステップ (可変 dt を固定ステップに分割) ---
        public void StepSimulation(float deltaTime)
        {
            _accumulator += deltaTime;
            int steps = 0;
            while (_accumulator >= FixedTimeStep && steps < 8)
            {
                InternalStep(FixedTimeStep);
                _accumulator -= FixedTimeStep;
                steps++;
            }
        }

        // --- 固定 1 ステップ ---
        private void InternalStep(float dt)
        {
            float sub = dt / SubSteps;
            for (int s = 0; s < SubSteps; s++)
                SubStep(sub);
        }

        private void SubStep(float dt)
        {
            IntegrateVelocities(dt);
            BroadphaseNarrowphase();
            BuildContactConstraints(dt);

            foreach (var j in Joints) j.Prepare(dt);
            foreach (var j in Joints) j.ApplySprings(dt);

            WarmStart();

            for (int it = 0; it < SolverIterations; it++)
            {
                SolveContacts();
                foreach (var j in Joints) j.SolveVelocity();
            }

            StoreImpulses();
            IntegratePositions(dt);
        }

        // --- 速度積分: 重力/力/減衰 ---
        private void IntegrateVelocities(float dt)
        {
            foreach (var b in Bodies)
            {
                if (b.IsStaticOrKinematic)
                {
                    // Kinematic: 目標姿勢へ移動する速度を算出 (接触応答に使用)。
                    if (b.IsKinematic)
                    {
                        var cur = b.WorldTransform;
                        var tgt = b.KinematicTarget;
                        b.LinearVelocity = (tgt.Origin - cur.Origin) / dt;
                        var dq = tgt.Rotation * cur.Rotation.Conjugated();
                        b.AngularVelocity = QuatToAngularVelocity(dq, dt);
                    }
                    continue;
                }

                b.LinearVelocity += (Gravity + b.TotalForce * b.InverseMass) * dt;
                b.AngularVelocity += (b.InverseInertiaWorld * b.TotalTorque) * dt;

                // Bullet 2.75 btRigidBody::applyDamping と同じ秒単位の減衰。
                b.LinearVelocity *= DampingFactor(b.LinearDamping, dt);
                b.AngularVelocity *= DampingFactor(b.AngularDamping, dt);

                b.ClearForces();
            }
        }

        // Bullet 2.75 の秒単位減衰係数。(1 - d)^dt。
        // d=1.0 は 0 除算・完全停止を避けるためクランプする。
        private static float DampingFactor(float damping, float dt)
        {
            float d = Math.Clamp(damping, 0f, 0.999f);
            return (float)Math.Pow(1f - d, dt);
        }

        // --- 位置積分 ---
        private void IntegratePositions(float dt)
        {
            foreach (var b in Bodies)
            {
                if (b.IsStaticOrKinematic)
                {
                    if (b.IsKinematic)
                    {
                        b.WorldTransform = b.KinematicTarget;
                        b.UpdateInertiaWorld();
                    }
                    continue;
                }

                var t = b.WorldTransform;
                t.Origin += b.LinearVelocity * dt;

                // クォータニオン積分: q += 0.5 * w * q * dt。
                var w = b.AngularVelocity;
                var spin = new Quat(w.x, w.y, w.z, 0f) * t.Rotation;
                t.Rotation = new Quat(
                    t.Rotation.x + spin.x * 0.5f * dt,
                    t.Rotation.y + spin.y * 0.5f * dt,
                    t.Rotation.z + spin.z * 0.5f * dt,
                    t.Rotation.w + spin.w * 0.5f * dt).Normalized;

                b.WorldTransform = t;
                b.UpdateInertiaWorld();
            }
        }

        private static Vec3 QuatToAngularVelocity(Quat dq, float dt)
        {
            dq = dq.Normalized;
            float angle = 2f * (float)Math.Acos(Math.Clamp(dq.w, -1f, 1f));
            if (angle < 1e-6f) return Vec3.Zero;
            if (angle > Math.PI) angle -= 2f * (float)Math.PI;
            var axis = new Vec3(dq.x, dq.y, dq.z);
            var len = axis.Length;
            if (len < 1e-9f) return Vec3.Zero;
            return axis / len * (angle / dt);
        }

        // --- ブロードフェーズ + ナローフェーズ ---
        private void BroadphaseNarrowphase()
        {
            int n = Bodies.Count;
            var aabbs = new Aabb[n];
            for (int i = 0; i < n; i++) aabbs[i] = Bodies[i].ComputeAabb();

            var seen = new HashSet<long>();
            for (int i = 0; i < n; i++)
            {
                for (int k = i + 1; k < n; k++)
                {
                    var a = Bodies[i]; var b = Bodies[k];
                    if (a.IsStaticOrKinematic && b.IsStaticOrKinematic) continue;
                    if (!ShouldCollide(a, b)) continue;
                    if (!aabbs[i].Intersects(ref aabbs[k])) continue;

                    long key = PairKey(a.Index, b.Index);
                    seen.Add(key);
                    if (!_manifolds.TryGetValue(key, out var m))
                    {
                        m = new PersistentManifold(a, b);
                        _manifolds[key] = m;
                    }
                    m.Refresh();
                    if (GjkEpa.Detect(a, b, out var cp))
                        m.AddPoint(cp);
                }
            }
            // 消えたペアを掃除。
            if (_manifolds.Count > seen.Count)
            {
                var dead = new List<long>();
                foreach (var kv in _manifolds) if (!seen.Contains(kv.Key)) dead.Add(kv.Key);
                foreach (var d in dead) _manifolds.Remove(d);
            }
        }

        /// <summary>
        /// PMX 衝突フィルタ。非衝突グループフラグ: 相手のグループbitが立っていれば衝突しない。
        /// 双方が「衝突許可」の時のみ衝突する。
        /// </summary>
        public static bool ShouldCollide(RigidBody a, RigidBody b)
        {
            bool aAllows = (a.NonCollisionMask & (1 << b.Group)) == 0;
            bool bAllows = (b.NonCollisionMask & (1 << a.Group)) == 0;
            return aAllows && bAllows;
        }

        private static long PairKey(int a, int b)
        {
            if (a > b) (a, b) = (b, a);
            return ((long)a << 32) | (uint)b;
        }

        public int DebugContactCount; // 診断用

        // --- 接触制約構築 ---
        private void BuildContactConstraints(float dt)
        {
            _contacts.Clear();
            int manifoldIdx = -1;
            foreach (var kv in _manifolds)
            {
                manifoldIdx++;
                var m = kv.Value;
                for (int p = 0; p < m.Points.Count; p++)
                {
                    var cp = m.Points[p];
                    var a = m.BodyA; var b = m.BodyB;

                    var rA = cp.PositionWorldA - a.CenterOfMass;
                    var rB = cp.PositionWorldB - b.CenterOfMass;
                    // EPA 法線は A→B 方向。ソルバ規約 (relVel=vB-vA, -P:A/+P:B) と一致。
                    var n = cp.Normal;

                    // 接線基底。
                    BuildTangentBasis(n, out var t1, out var t2);

                    var cc = new ContactConstraint
                    {
                        A = a, B = b, RelA = rA, RelB = rB,
                        Normal = n, Tangent1 = t1, Tangent2 = t2,
                        Friction = (float)Math.Sqrt(Math.Max(0, a.Friction) * Math.Max(0, b.Friction)),
                        NormalMass = EffectiveMass(a, b, rA, rB, n),
                        TangentMass1 = EffectiveMass(a, b, rA, rB, t1),
                        TangentMass2 = EffectiveMass(a, b, rA, rB, t2),
                        NormalImpulse = cp.NormalImpulse,
                        TangentImpulse1 = cp.TangentImpulse1,
                        TangentImpulse2 = cp.TangentImpulse2,
                        Manifold = m, PointRef = p,
                    };

                    // Baumgarte 位置補正 + 反発。
                    float pen = -cp.Distance - PenetrationSlop;
                    float biasVel = pen > 0 ? BaumgarteFactor * pen / dt : 0f;

                    var relN = (b.VelocityAtPoint(cp.PositionWorldB) - a.VelocityAtPoint(cp.PositionWorldA)).Dot(n);
                    float rest = (float)Math.Sqrt(Math.Max(0, a.Restitution) * Math.Max(0, b.Restitution));
                    float restBias = (-relN > RestitutionThreshold) ? rest * -relN : 0f;

                    cc.NormalBias = Math.Max(biasVel, restBias);
                    _contacts.Add(cc);
                }
            }
            DebugContactCount = _contacts.Count;
        }

        private static float EffectiveMass(RigidBody a, RigidBody b, Vec3 rA, Vec3 rB, Vec3 dir)
        {
            var rAxn = Vec3.Cross(rA, dir);
            var rBxn = Vec3.Cross(rB, dir);
            float k = a.InverseMass + b.InverseMass
                    + rAxn.Dot(a.InverseInertiaWorld * rAxn)
                    + rBxn.Dot(b.InverseInertiaWorld * rBxn);
            return k > 0 ? 1f / k : 0f;
        }

        private static void BuildTangentBasis(Vec3 n, out Vec3 t1, out Vec3 t2)
        {
            if (Math.Abs(n.x) >= 0.577f)
                t1 = new Vec3(n.y, -n.x, 0f).Normalized;
            else
                t1 = new Vec3(0f, n.z, -n.y).Normalized;
            t2 = Vec3.Cross(n, t1);
        }

        // 蓄積インパルスを manifold へ書き戻し、次フレームのウォームスタートに使う。
        private void StoreImpulses()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                if (c.Manifold == null || c.PointRef >= c.Manifold.Points.Count) continue;
                var cp = c.Manifold.Points[c.PointRef];
                cp.NormalImpulse = c.NormalImpulse;
                cp.TangentImpulse1 = c.TangentImpulse1;
                cp.TangentImpulse2 = c.TangentImpulse2;
                c.Manifold.Points[c.PointRef] = cp;
            }
        }

        private void WarmStart()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                var P = c.Normal * c.NormalImpulse
                      + c.Tangent1 * c.TangentImpulse1
                      + c.Tangent2 * c.TangentImpulse2;
                c.A.ApplyImpulse(-P, c.RelA);
                c.B.ApplyImpulse(P, c.RelB);
                _contacts[i] = c;
            }
        }

        // --- 接触速度求解 ---
        private void SolveContacts()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                var a = c.A; var b = c.B;

                // 摩擦 (法線インパルスに従属)。
                SolveFriction(ref c, a, b, c.Tangent1, c.TangentMass1, ref c.TangentImpulse1);
                SolveFriction(ref c, a, b, c.Tangent2, c.TangentMass2, ref c.TangentImpulse2);

                // 法線。
                var pA = a.CenterOfMass + c.RelA;
                var pB = b.CenterOfMass + c.RelB;
                float relN = (b.VelocityAtPoint(pB) - a.VelocityAtPoint(pA)).Dot(c.Normal);
                float dPn = (c.NormalBias - relN) * c.NormalMass;
                float oldN = c.NormalImpulse;
                c.NormalImpulse = Math.Max(0f, oldN + dPn);
                dPn = c.NormalImpulse - oldN;
                var Pn = c.Normal * dPn;
                a.ApplyImpulse(-Pn, c.RelA);
                b.ApplyImpulse(Pn, c.RelB);

                _contacts[i] = c;
            }
        }

        private static void SolveFriction(ref ContactConstraint c, RigidBody a, RigidBody b,
            Vec3 tangent, float mass, ref float accum)
        {
            var pA = a.CenterOfMass + c.RelA;
            var pB = b.CenterOfMass + c.RelB;
            float relT = (b.VelocityAtPoint(pB) - a.VelocityAtPoint(pA)).Dot(tangent);
            float dPt = -relT * mass;
            float maxF = c.Friction * c.NormalImpulse;
            float old = accum;
            accum = Math.Max(-maxF, Math.Min(maxF, old + dPt));
            dPt = accum - old;
            var Pt = tangent * dPt;
            a.ApplyImpulse(-Pt, c.RelA);
            b.ApplyImpulse(Pt, c.RelB);
        }
    }
}
