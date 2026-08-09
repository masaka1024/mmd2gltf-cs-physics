// ===========================================================================
// 位相別プロファイル (PhysicsWorld.ProfileEnabled による直接計測)。
// 差分プロファイル(構成を変えて引き算)は交絡が大きい(Jointを外すと剛体が落ちて接触が消える等)ため、
// エンジン内に既定OFFの計装を入れて位相ごとの実時間を直接積む。
// 実行: dotnet run -c Release   (MMD_TEST_PMX でモデル指定 / ITERS,SUBSTEPS で構成掃引)
// ===========================================================================
using System;
using System.Diagnostics;
using System.Linq;
using BulletPhysics;
using BulletPhysics.Pmx;

static class Perf
{
    const int Warm = 30, Iter = 300;

    static int Main()
    {
        string pmx = TestData.PmxPath();
        if (pmx == null) { Console.WriteLine("[SKIP] no pmx"); return 0; }
        var model = PmxReader.LoadFile(pmx);
        int iters = int.TryParse(Environment.GetEnvironmentVariable("ITERS"), out var it) ? it : 10;
        int subs = int.TryParse(Environment.GetEnvironmentVariable("SUBSTEPS"), out var ss) ? ss : 2;
        int ftsDiv = int.TryParse(Environment.GetEnvironmentVariable("FTS_DIV"), out var fd) ? fd : 30;

        var b = PmxPhysicsBuilder.Build(model);
        var w = b.World;
        w.Gravity = new Vec3(0, -98f, 0); w.SolverIterations = iters; w.SubSteps = subs; w.FixedTimeStep = 1f / ftsDiv;
        foreach (var body in b.Bodies) if (body.Mode == PhysicsMode.BoneFollow) { body.KinematicTarget = body.WorldTransform; body.KinematicStepTarget = body.WorldTransform; }
        for (int i = 0; i < Warm; i++) w.StepSimulation(1f / 30f);

        // 全体時間 (計装OFF = 本来の速度)
        var ts = new double[Iter]; var sw = new Stopwatch();
        for (int i = 0; i < Iter; i++) { sw.Restart(); w.StepSimulation(1f / 30f); sw.Stop(); ts[i] = sw.Elapsed.TotalMilliseconds; }
        var srt = (double[])ts.Clone(); Array.Sort(srt);
        double avg = ts.Average(), med = srt[Iter / 2], p95 = srt[(int)(Iter * 0.95)], max = srt[Iter - 1];

        // GC/アロケーション計測 (Unityでの「カクつき」の主因になりやすい)
        long a0 = GC.GetTotalAllocatedBytes(true);
        int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1);
        for (int i = 0; i < Iter; i++) w.StepSimulation(1f / 30f);
        long a1 = GC.GetTotalAllocatedBytes(true);
        double allocPerStep = (a1 - a0) / (double)Iter;
        int gc0 = GC.CollectionCount(0) - g0, gc1 = GC.CollectionCount(1) - g1;

        // 位相別 (計装ON: Stopwatch分のオーバーヘッドが乗るため割合として読む)
        PhysicsWorld.ProfReset(); PhysicsWorld.ProfileEnabled = true;
        for (int i = 0; i < Iter; i++) w.StepSimulation(1f / 30f);
        PhysicsWorld.ProfileEnabled = false;

        Console.WriteLine($"[perf] {System.IO.Path.GetFileName(pmx)} 剛体={model.RigidBodies.Count} Joint={model.Joints.Count} 30Hz SubSteps={subs} iters={iters}");
        Console.WriteLine($"[全体] 1ステップ 中央={med:F3}ms 平均={avg:F3} p95={p95:F3} 最大={max:F3}   (30Hz予算33.3ms に対し 平均{avg / 33.3:P1})");
        Console.WriteLine($"[GC]   1ステップあたり確保={allocPerStep:N0}B  →  30Hzで {allocPerStep * 30 / 1024:N0}KB/秒   gen0回収={gc0}回/{Iter}step gen1={gc1}回");
        double n = PhysicsWorld.ProfSubSteps;
        double tot = PhysicsWorld.ProfBroad + PhysicsWorld.ProfBuild + PhysicsWorld.ProfPrepare + PhysicsWorld.ProfSpring
                   + PhysicsWorld.ProfWarm + PhysicsWorld.ProfSolveContact + PhysicsWorld.ProfSolveJoint
                   + PhysicsWorld.ProfIntegrate + PhysicsWorld.ProfStore;
        void L(string name, double ms) => Console.WriteLine($"   {name,-34} {ms / Iter,7:F3}ms/step  {ms / tot,6:P1}");
        Console.WriteLine($"[位相別] 計装合計={tot / Iter:F3}ms/step (サブステップ{n / Iter:F0}回/step, 接触{PhysicsWorld.ProfContacts / n:F0}件/サブ, マニフォールド{PhysicsWorld.ProfManifolds / n:F0}件)");
        L("ソルバ: 接触 (SolveContacts×iters)", PhysicsWorld.ProfSolveContact);
        L("ソルバ: Joint (SolveVelocity×iters)", PhysicsWorld.ProfSolveJoint);
        L("Joint Prepare (行構築)", PhysicsWorld.ProfPrepare);
        L("ブロードフェーズ+ナローフェーズ", PhysicsWorld.ProfBroad);
        L("接触制約の構築", PhysicsWorld.ProfBuild);
        L("ウォームスタート", PhysicsWorld.ProfWarm);
        L("インパルス書戻し", PhysicsWorld.ProfStore);
        L("速度積分", PhysicsWorld.ProfIntegrate);
        L("バネ", PhysicsWorld.ProfSpring);
        // --- 緩いAABB(球バウンド立方体) vs タイトAABB(形状+回転) の候補ペア数比較 ---
        // 現状の ComputeAabb は BoundingRadius を辺とする立方体で、薄い板や細長カプセルでは大幅に膨張する。
        {
            var bodies = w.Bodies; int nb = bodies.Count;
            Aabb Tight(RigidBody rb)
            {
                var t = rb.WorldTransform; var c = t.Origin;
                if (rb.Shape is SphereShape sp) { var r0 = new Vec3(sp.Radius + sp.Margin); return new Aabb(c - r0, c + r0); }
                if (rb.Shape is CapsuleShape cap)
                {
                    var p0 = t.TransformPoint(new Vec3(0, cap.HalfHeight, 0));
                    var p1 = t.TransformPoint(new Vec3(0, -cap.HalfHeight, 0));
                    float r1 = cap.Radius + cap.Margin;
                    var lo = new Vec3(Math.Min(p0.x, p1.x) - r1, Math.Min(p0.y, p1.y) - r1, Math.Min(p0.z, p1.z) - r1);
                    var hi = new Vec3(Math.Max(p0.x, p1.x) + r1, Math.Max(p0.y, p1.y) + r1, Math.Max(p0.z, p1.z) + r1);
                    return new Aabb(lo, hi);
                }
                if (rb.Shape is BoxShape bx)
                {
                    var m = Matrix3x3.FromQuat(t.Rotation); var h = bx.HalfExtents; float mg = bx.Margin;
                    float ex = Math.Abs(m.Row0.x) * h.x + Math.Abs(m.Row0.y) * h.y + Math.Abs(m.Row0.z) * h.z + mg;
                    float ey = Math.Abs(m.Row1.x) * h.x + Math.Abs(m.Row1.y) * h.y + Math.Abs(m.Row1.z) * h.z + mg;
                    float ez = Math.Abs(m.Row2.x) * h.x + Math.Abs(m.Row2.y) * h.y + Math.Abs(m.Row2.z) * h.z + mg;
                    return new Aabb(new Vec3(c.x - ex, c.y - ey, c.z - ez), new Vec3(c.x + ex, c.y + ey, c.z + ez));
                }
                return rb.ComputeAabb();
            }
            var loose = new Aabb[nb]; var tight = new Aabb[nb];
            for (int i = 0; i < nb; i++) { loose[i] = bodies[i].ComputeAabb(); tight[i] = Tight(bodies[i]); }
            int cand = 0, lo2 = 0, ti = 0, real = 0;
            var buf = new System.Collections.Generic.List<ContactPoint>();
            for (int i = 0; i < nb; i++)
                for (int k = i + 1; k < nb; k++)
                {
                    var a = bodies[i]; var b2 = bodies[k];
                    if (a.IsStaticOrKinematic && b2.IsStaticOrKinematic) continue;
                    if (!PhysicsWorld.ShouldCollide(a, b2)) continue;
                    cand++;
                    bool L2 = loose[i].Intersects(ref loose[k]); if (L2) lo2++;
                    bool T = tight[i].Intersects(ref tight[k]); if (T) ti++;
                    if (L2) { buf.Clear(); GjkEpa.Detect(a, b2, buf); if (buf.Count > 0) real++; }
                }
            Console.WriteLine($"\n[AABB効率] 候補ペア(Group/Mask後)={cand} → 緩いAABB通過={lo2} / タイトAABB通過={ti} / 実接触={real}");
            Console.WriteLine($"          ナローフェーズ空振り 現状={lo2 - real}/サブ → タイト化={ti - real}/サブ");
        }

        Console.WriteLine($"\n[規模] 総当たりペア={model.RigidBodies.Count * (model.RigidBodies.Count - 1) / 2}/サブ, Joint解決={model.Joints.Count * iters * subs:N0}/step, 接触解決={(long)(PhysicsWorld.ProfContacts / n) * iters * subs:N0}/step");
        return 0;
    }
}
