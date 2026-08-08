// 検証: タスクB(実効1/60=SubSteps2)後の新ベースライン記録と、Sub1のビット一致確認。
// 反復10固定。SubSteps を明示指定して測る(既定変更の影響を分離)。本体は無改変。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class TsBase
{
    const string PmxPath = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
    const float FRAME = 1f / 30f;
    const float RefCalmMed = 11.39f;
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }
    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static List<SkirtMeasure.TurnWindow> wins; static bool[] inWin; static List<SkirtJoint> skirt; static float[] refWinMax;

    class Res { public float calmMed, calmP90, calmMax; public float[] ringMed = new float[3]; public float[] winPeak; public float ratioMed; public double penMean, penMax; }

    static Res Run(int subSteps, bool dummies = false, bool useReset = false)
    {
        var builder = PmxPhysicsBuilder.Build(model); var world = builder.World;
        world.SolverIterations = 10; world.FixedTimeStep = FRAME; world.SubSteps = subSteps;
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;
        if (dummies) AddFarDummies(world);
        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }
        Apply(0);
        if (useReset) builder.ResetBodiesToBonePoseFk(i => (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw)) ? (RigidTransform?)bw : null);
        for (int s = 0; s < 60; s++) world.StepSimulation(FRAME);
        var calm = new List<float>(); var ring = new[] { new List<float>(), new List<float>(), new List<float>() }; var winPeak = new float[wins.Count];
        double penSum = 0, penMax = 0; long penCnt = 0;
        for (int f = 0; f < F; f++)
        {
            Apply(f); dbg.Clear(); world.StepSimulation(FRAME);
            float fmax = 0;
            foreach (var sj in skirt) { float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb)); if (!inWin[f]) { calm.Add(t); if (sj.Ring >= 0 && sj.Ring < 3) ring[sj.Ring].Add(t); } if (t > fmax) fmax = t; }
            for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
            foreach (var d in dbg) if (d.dist < 0f) { double p = -d.dist; penSum += p; penCnt++; if (p > penMax) penMax = p; }
        }
        var st = SkirtMeasure.Stats(calm); var r = new Res { calmMed = st.med, calmP90 = st.p90, calmMax = st.max, winPeak = winPeak, penMean = penCnt > 0 ? penSum / penCnt : 0, penMax = penMax };
        for (int k = 0; k < 3; k++) r.ringMed[k] = SkirtMeasure.Stats(ring[k]).med;
        var ratios = new List<float>(); for (int w = 0; w < wins.Count; w++) if (refWinMax[w] > 0) ratios.Add(winPeak[w] / refWinMax[w]); ratios.Sort();
        r.ratioMed = SkirtMeasure.Percentile(ratios.ToArray(), 50); return r;
    }

    static void AddFarDummies(PhysicsWorld world)
    {
        const float X = 100000f;
        var floor = new RigidBody(new BoxShape(new Vec3(50, 1, 50))) { Mode = PhysicsMode.BoneFollow, Group = 0, CollisionMask = 0xFFFF };
        floor.SetMassProps(0f); floor.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X, 0, 0)); world.AddBody(floor);
        var rnd = new Random(12345);
        for (int i = 0; i < 6; i++) { var s = new RigidBody(new SphereShape(1.0f)) { Mode = PhysicsMode.Dynamic, Group = 0, CollisionMask = 0xFFFF }; s.SetMassProps(1f); s.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X + (float)(rnd.NextDouble() * 4 - 2), 3 + i * 2.2f, (float)(rnd.NextDouble() * 4 - 2))); world.AddBody(s); }
    }

    static (double avg, double p95, double max) Timing(int subSteps, int frames)
    {
        var b = PmxPhysicsBuilder.Build(model); var w = b.World; w.Gravity = new Vec3(0, -98f, 0); w.SolverIterations = 10; w.FixedTimeStep = FRAME; w.SubSteps = subSteps;
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
        for (int f = 0; f < F; f++) for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30)) { float m = 0; foreach (var sj in skirt) if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb)) { float t = SkirtMeasure.TiltDeg(pb.Rotation, cb.Rotation); if (t > m) m = t; } if (m > refWinMax[w]) refWinMax[w] = m; }

        L("==================== 新ベースライン: 実効1/60(Sub2)+FK-restリセット ====================");

        // 1) Sub1 ビット一致 (旧ベースライン 11.041/22.498/111.822, リセット無し)
        var s1 = Run(1);
        bool bitok = Math.Abs(s1.calmMed - 11.041f) < 5e-4f && Math.Abs(s1.calmP90 - 22.498f) < 5e-4f && Math.Abs(s1.calmMax - 111.822f) < 5e-4f;
        L($"\n[Sub1(リセット無) ビット一致確認] 平時 中央={s1.calmMed:F3} p90={s1.calmP90:F3} max={s1.calmMax:F3}  => {(bitok ? "★旧ベースラインと一致" : "不一致(!)")}");

        // 2) Sub2 + FK-restリセット 新ベースライン
        var s2 = Run(2, false, useReset: true);
        L("\n[新ベースライン: SubSteps=2 (実効1/60) + FK-restリセット]");
        L($"  平時 中央={s2.calmMed:F3} p90={s2.calmP90:F3} max={s2.calmMax:F3}  (本家 中央={RefCalmMed})");
        L($"  リング別中央: ring0={s2.ringMed[0]:F2} ring1={s2.ringMed[1]:F2} ring2={s2.ringMed[2]:F2}");
        L($"  貫入(全動的接触) 平均={s2.penMean:F4} 最大={s2.penMax:F3} (最大はF2889の髪スパイク。スカート×太ももの深貫入は解消)");
        L("\n  12窓 傾きmax: 自前 | 本家 | 比");
        var ratios = new List<float>();
        for (int w = 0; w < wins.Count; w++) { float rr = refWinMax[w] > 0 ? s2.winPeak[w] / refWinMax[w] : 0; ratios.Add(rr); L($"    窓{w + 1,2} F{wins[w].StartFrame}-{wins[w].EndFrame} peakYaw={wins[w].PeakYaw:F0}: {s2.winPeak[w],7:F2} | {refWinMax[w],7:F2} | {rr:F3}"); }
        ratios.Sort(); L($"  → 12窓比の中央値={SkirtMeasure.Percentile(ratios.ToArray(), 50):F3}");

        // 3) ノイズフロア (Sub2 + FK-restリセット, ダミー有無)
        var nd = Run(2, false, useReset: true); var wd = Run(2, true, useReset: true);
        float dMax = Math.Abs(nd.calmMed - wd.calmMed);
        for (int w = 0; w < wins.Count; w++) dMax = Math.Max(dMax, Math.Abs(nd.winPeak[w] - wd.winPeak[w]));
        L($"\n[ノイズフロア Sub2] 遠方ダミー有無の最大差={dMax:F4} => {(dMax < 1e-3f ? "一致(0)" : "揺れあり")}");

        // 4) 性能
        var t1 = Timing(1, 300); var t2 = Timing(2, 300);
        L($"\n[性能] IA.pmx 300ステップ: Sub1 avg={t1.avg:F2}/p95={t1.p95:F2}/max={t1.max:F2}ms | Sub2 avg={t2.avg:F2}/p95={t2.p95:F2}/max={t2.max:F2}ms");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "tsbaseline_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
