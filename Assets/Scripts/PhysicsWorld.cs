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
        // リファレンスは 30Hz・1サブ (MMD本家は 30fps で 1描画フレーム=1物理ステップ。
        // その刻みを再現対象とする設計方針)。
        public int SolverIterations = 10;
        public int SubSteps = 1;
        public float FixedTimeStep = 1f / 30f;

        public float PenetrationSlop = 0.005f;
        public float BaumgarteFactor = 0.2f;
        public float RestitutionThreshold = 1.0f;

        public readonly List<RigidBody> Bodies = new();
        public readonly List<Joint> Joints = new();

        private readonly Dictionary<long, PersistentManifold> _manifolds = new();
        private readonly List<ContactConstraint> _contacts = new();
        private readonly List<ContactPoint> _detectBuffer = new(2); // Detect の返り値受け取り
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

        private RigidTransform[] _kinStart; // キネマティック剛体の開始姿勢 (body index 単位)

        // --- 固定 1 ステップ ---
        private void InternalStep(float dt)
        {
            float sub = dt / SubSteps;

            // 開始姿勢を保存し、サブステップ間で開始→目標を補間する。
            if (_kinStart == null || _kinStart.Length < Bodies.Count)
                _kinStart = new RigidTransform[Bodies.Count];
            for (int i = 0; i < Bodies.Count; i++)
                if (Bodies[i].IsKinematic) _kinStart[i] = Bodies[i].WorldTransform;

            for (int s = 0; s < SubSteps; s++)
                SubStep(sub, (float)(s + 1) / SubSteps);
        }

        // frac: このサブステップ終端における 開始→目標 の補間割合 ((s+1)/N)。
        private void SubStep(float dt, float frac)
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                var b = Bodies[i];
                if (!b.IsKinematic) continue;
                b.KinematicStepTarget = InterpTransform(_kinStart[i], b.KinematicTarget, frac);
            }

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
                        // このサブステップの補間目標への差分から速度を算出。
                        var cur = b.WorldTransform;
                        var tgt = b.KinematicStepTarget;
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
                        b.WorldTransform = b.KinematicStepTarget;
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

        // 開始→目標を frac で補間 (位置は線形、回転は slerp)。
        private static RigidTransform InterpTransform(RigidTransform from, RigidTransform to, float frac)
        {
            return new RigidTransform(
                Quat.Slerp(from.Rotation, to.Rotation, frac),
                from.Origin + (to.Origin - from.Origin) * frac);
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
                    _detectBuffer.Clear();
                    GjkEpa.Detect(a, b, _detectBuffer);
                    for (int di = 0; di < _detectBuffer.Count; di++)
                        m.AddPoint(_detectBuffer[di]);
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
        /// PMX 衝突フィルタ。16bitフィールドは「衝突する相手グループ」のビットマスク
        /// (bit=1 で衝突する)。Bullet の (groupA & maskB) && (groupB & maskA) と同じ。
        /// </summary>
        public static bool ShouldCollide(RigidBody a, RigidBody b)
        {
            return (b.CollisionMask & (1 << a.Group)) != 0
                && (a.CollisionMask & (1 << b.Group)) != 0;
        }

        private static long PairKey(int a, int b)
        {
            if (a > b) (a, b) = (b, a);
            return ((long)a << 32) | (uint)b;
        }

        public int DebugContactCount; // 診断用

        // マニフォールドを決定的順序で解くための再利用バッファ (毎ステップのアロケーションを避ける)。
        private readonly List<PersistentManifold> _sortedManifolds = new();
        private static int CompareManifold(PersistentManifold x, PersistentManifold y) =>
            PairKey(x.BodyA.Index, x.BodyB.Index).CompareTo(PairKey(y.BodyA.Index, y.BodyB.Index));

        // --- 接触制約構築 ---
        private void BuildContactConstraints(float dt)
        {
            _contacts.Clear();

            // Dictionary の列挙順は挿入/削除履歴に依存し非決定的なので、剛体indexの組(PairKey)の
            // 昇順に並べ替えてから制約を構築する。これにより「無関係な剛体の増減」で接触の解く順序が
            // 変わって結果が揺れる (Gauss-Seidel の順序依存) 現象を排除する。式・パラメータは不変、順序のみ。
            _sortedManifolds.Clear();
            foreach (var kv in _manifolds) _sortedManifolds.Add(kv.Value);
            _sortedManifolds.Sort(CompareManifold);

            foreach (var m in _sortedManifolds)
            {
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

                    // 法線の目標接近速度 (NormalBias) を決める。
                    var relN = (b.VelocityAtPoint(cp.PositionWorldB) - a.VelocityAtPoint(cp.PositionWorldA)).Dot(n);
                    float rest = (float)Math.Sqrt(Math.Max(0, a.Restitution) * Math.Max(0, b.Restitution));
                    float restBias = (-relN > RestitutionThreshold) ? rest * -relN : 0f;

                    if (cp.Distance <= 0f)
                    {
                        // 貫入: Baumgarte 位置補正 + 反発 (従来どおり)。
                        float pen = -cp.Distance - PenetrationSlop;
                        float biasVel = pen > 0 ? BaumgarteFactor * pen / dt : 0f;
                        cc.NormalBias = Math.Max(biasVel, restBias);
                    }
                    else
                    {
                        // 非貫入 (投機的接触): このステップで表面へちょうど到達する接近
                        // (-Distance/dt) までは許し、それを超える接近だけを止める。押し戻さない。
                        // 反発は押し戻す向きなので、より接近を許す (小さい) 方を採る。
                        float speculative = -cp.Distance / dt;
                        cc.NormalBias = Math.Min(speculative, restBias);
                    }
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

        // 検証用の読み取り専用診断フック。null (既定) の間は何もせず、挙動・性能に影響しない。
        // 回帰テスト (非貫入押し出しの検出など) が接触の Distance/法線インパルスを参照するために使う。
        public System.Collections.Generic.List<(string a, string b, float dist, float ni)> DebugContacts;

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
                DebugContacts?.Add((c.A.Name, c.B.Name, cp.Distance, c.NormalImpulse));
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
