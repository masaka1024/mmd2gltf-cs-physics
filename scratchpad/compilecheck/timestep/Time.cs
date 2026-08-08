// 診断のみ(実装・既定値変更なし): 時間刻み仮説の検証。FixedTimeStep/SubSteps の設定値だけを
// 変えて測る。反復数は10固定。CSV駆動は毎フレーム 1/30 秒を StepSimulation に渡し、内部で
// FixedTimeStep 刻みに分割させる(=本家の stepSimulation(dt,maxSub,fixedTimeStep) 相当)。
// 併せて (a)同一実効刻みでの分割方法差、(b)キネマティック補間の効き、(c)減衰の刻み不変性 を検算。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class TimeStep
{
    static readonly string PmxPath = TestData.PmxPath();
    const float FRAME = 1f / 30f;   // CSVフレーム間隔(30fps)。毎フレームこの dt を渡す。
    const float RefCalmMed = 11.39f;
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static List<SkirtMeasure.TurnWindow> wins; static bool[] inWin; static List<SkirtJoint> skirt; static float[] refWinMax;

    class Res { public float calmMed, calmP90, ratioMed; public float winMax; public double penMean, penMax; }

    static Res Run(float fts, int sub)
    {
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        world.SolverIterations = 10; world.FixedTimeStep = fts; world.SubSteps = sub;
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(FRAME);
        var calm = new List<float>(); var winPeak = new float[wins.Count];
        double penSum = 0, penMax = 0; long penCnt = 0;
        for (int f = 0; f < F; f++)
        {
            Apply(f); dbg.Clear(); world.StepSimulation(FRAME);
            float fmax = 0;
            foreach (var sj in skirt) { float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb)); if (!inWin[f]) calm.Add(t); if (t > fmax) fmax = t; }
            for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
            foreach (var d in dbg) if (d.dist < 0f) { double p = -d.dist; penSum += p; penCnt++; if (p > penMax) penMax = p; }
        }
        var st = SkirtMeasure.Stats(calm);
        var ratios = new List<float>(); for (int w = 0; w < wins.Count; w++) if (refWinMax[w] > 0) ratios.Add(winPeak[w] / refWinMax[w]); ratios.Sort();
        return new Res { calmMed = st.med, calmP90 = st.p90, winMax = winPeak.Max(), penMean = penCnt > 0 ? penSum / penCnt : 0, penMax = penMax, ratioMed = SkirtMeasure.Percentile(ratios.ToArray(), 50) };
    }

    static (double avg, double p95, double max) Timing(float fts, int sub, int frames)
    {
        var b = PmxPhysicsBuilder.Build(model); var w = b.World; w.Gravity = new Vec3(0, -98f, 0);
        w.SolverIterations = 10; w.FixedTimeStep = fts; w.SubSteps = sub;
        var ms = new List<double>(frames);
        for (int i = 0; i < frames; i++) { var sw = Stopwatch.StartNew(); w.StepSimulation(FRAME); sw.Stop(); if (i >= 5) ms.Add(sw.Elapsed.TotalMilliseconds); }
        ms.Sort(); return (ms.Average(), ms[(int)(0.95 * (ms.Count - 1))], ms[ms.Count - 1]);
    }

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(PmxPath); F = csv.FrameCount;
        skirt = SkirtMeasure.ExtractVerticalJoints(model);
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, FRAME);
        wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        inWin = new bool[F]; foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;
        refWinMax = new float[wins.Count];
        for (int f = 0; f < F; f++) for (int w = 0; w < wins.Count; w++)
            if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30))
            { float m = 0; foreach (var sj in skirt) if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb)) { float t = SkirtMeasure.TiltDeg(pb.Rotation, cb.Rotation); if (t > m) m = t; } if (m > refWinMax[w]) refWinMax[w] = m; }

        L("==================== 時間刻み仮説の検証 (反復10固定, 本家平時11.39/比目標1.0) ====================");
        var cfgs = new (string label, float fts, int sub)[]
        {
            ("1/30  (FTS=1/30, Sub=1) 現行", 1f/30f, 1),
            ("1/60  (FTS=1/30, Sub=2)",      1f/30f, 2),
            ("1/60  (FTS=1/60, Sub=1)",      1f/60f, 1),
            ("1/120 (FTS=1/30, Sub=4)",      1f/30f, 4),
            ("1/120 (FTS=1/120,Sub=1)",      1f/120f,1),
            ("1/240 (FTS=1/120,Sub=2)",      1f/120f,2),
        };
        L("\n  設定                         | 平時中央 | 平時p90 | 12窓比中央 | 窓ピークmax | 貫入平均 | 貫入最大 | 性能avg/p95/max(ms)");
        float best = 1e9f; string bestLabel = "";
        foreach (var c in cfgs)
        {
            var r = Run(c.fts, c.sub);
            var t = Timing(c.fts, c.sub, 300);
            L($"  {c.label,-28} | {r.calmMed,7:F3} | {r.calmP90,6:F2} | {r.ratioMed,9:F3} | {r.winMax,9:F2} | {r.penMean,7:F4} | {r.penMax,6:F3} | {t.avg:F2}/{t.p95:F2}/{t.max:F2}");
            float d = Math.Abs(r.ratioMed - 1.0f); if (d < best) { best = d; bestLabel = c.label; }
        }
        L($"\n  → 12窓比が最も1.0に近い刻み: {bestLabel}");

        // 同一実効刻みでの分割差
        L("\n  [同一実効刻みでの分割方法差]");
        L("   1/60 : (FTS=1/30,Sub=2) と (FTS=1/60,Sub=1) の差 → 上表2行を比較");
        L("   1/120: (FTS=1/30,Sub=4) と (FTS=1/120,Sub=1) の差 → 上表4-5行目を比較");

        // (b) キネマティック補間の効きを直接確認: 1体kinematicを1フレーム動かし、フレーム内の
        //     中間到達位置を観測する。SubSteps経路は補間される/FTS(accumulator)経路は?
        L("\n  [キネマティック補間の検証] 1体を1フレームで x:0→1 へ動かし、各内部ステップ後の x を観測");
        KinProbe(1f/30f, 2, "FTS=1/30,Sub=2");
        KinProbe(1f/60f, 1, "FTS=1/60,Sub=1");
        KinProbe(1f/120f, 1, "FTS=1/120,Sub=1");
        KinProbe(1f/30f, 4, "FTS=1/30,Sub=4");

        // (c) 減衰の刻み不変性検算: d=0.9 の剛体を重力0で1秒回し、実効残存速度比を刻み別に比較。
        L("\n  [減衰の刻み不変性] 移動減衰d=0.9, 重力0, 初速1, 1秒後の速度 (刻み不変なら全て≒0.1)");
        foreach (var c in cfgs) L($"   {c.label,-28} : v(1s)={DampProbe(c.fts, c.sub):F5}");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "timestep_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    // kinematic 1体を KinematicTarget x=1 へ。1フレーム(1/30)ステップし、内部分割ごとの
    // 実位置を観測できないので、代わりに「1フレーム後の最終x」と「フレーム後の残存速度」を出す。
    // さらに 1/30 を N 個の小フレームに分けて渡した場合の軌跡で補間の有無を判定する。
    static void KinProbe(float fts, int sub, string label)
    {
        var w = new PhysicsWorld { Gravity = Vec3.Zero, SolverIterations = 10, FixedTimeStep = fts, SubSteps = sub };
        var k = new RigidBody(new BoxShape(new Vec3(0.5f, 0.5f, 0.5f))) { Mode = PhysicsMode.BoneFollow };
        k.SetMassProps(0f); k.WorldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); w.AddBody(k);
        k.KinematicTarget = new RigidTransform(Quat.Identity, new Vec3(1, 0, 0));
        w.StepSimulation(FRAME);
        L($"    {label,-16}: 1フレーム後 x={k.WorldTransform.Origin.x:F4} 残存velx={k.LinearVelocity.x:F3} (実効刻み={fts / sub:F5}s×{(int)Math.Round(FRAME / (fts / sub))}分割)");
    }

    static float DampProbe(float fts, int sub)
    {
        var w = new PhysicsWorld { Gravity = Vec3.Zero, SolverIterations = 10, FixedTimeStep = fts, SubSteps = sub };
        var b = new RigidBody(new SphereShape(0.5f)) { Mode = PhysicsMode.Dynamic, LinearDamping = 0.9f };
        b.SetMassProps(1f); b.WorldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); w.AddBody(b);
        b.LinearVelocity = new Vec3(1, 0, 0);
        for (int i = 0; i < 30; i++) w.StepSimulation(FRAME); // 30フレーム=1秒
        return b.LinearVelocity.x;
    }
}
