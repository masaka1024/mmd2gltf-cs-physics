// 測定(調整なし): Split Impulse (接触の貫入回復を実速度から分離) の効果検証。
//   比較: UseSplitImpulse = false(従来 Baumgarte-in-velocity) vs true(新方式)。
//   指標: 平時傾き(全体/リング別), 12窓の自前/MMD傾きmaxと比, 貫入量, 反復数掃引での窓ピーク,
//        遠方ダミー剛体でのノイズフロア, 性能。
//   仮説: エネルギー注入が原因なら、Split後は「反復数依存が弱まる」はず(決め手)。
// 本体は不変(すべて public API から)。物理パラメータは変更しない(Split の on/off と反復数掃引のみ)。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Split
{
    static readonly string PmxPath = TestData.PmxPath();
    const float DT = 1f / 30f;
    static StringBuilder O = new(); static void L(string s = "") => O.AppendLine(s);

    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static List<SkirtMeasure.TurnWindow> wins; static bool[] inWin; static List<SkirtJoint> skirt;
    static float[] refWinMax; // MMDの窓別 傾きmax

    class Res
    {
        public float calmMed, calmP90, calmMax;
        public float[] ringMed = new float[3];
        public float[] winPeak;      // 窓別 自前 frame-max
        public double penMean, penMax; public long penCount;
    }

    static Res Run(bool split, int iters, bool dummies = false)
    {
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        world.UseSplitImpulse = split;
        if (iters > 0) world.SolverIterations = iters;
        var dbg = new List<(string a, string b, float dist, float ni)>();
        world.DebugContacts = dbg;
        if (dummies) AddFarDummies(world);

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(DT);

        var calm = new List<float>(); var ring = new List<float>[3] { new(), new(), new() };
        var winPeak = new float[wins.Count];
        double penSum = 0, penMax = 0; long penCnt = 0;
        for (int f = 0; f < F; f++)
        {
            Apply(f);
            dbg.Clear();
            world.StepSimulation(DT);
            float fmax = 0;
            foreach (var sj in skirt)
            {
                float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb));
                if (!inWin[f]) { calm.Add(t); if (sj.Ring >= 0 && sj.Ring < 3) ring[sj.Ring].Add(t); }
                if (t > fmax) fmax = t;
            }
            for (int w = 0; w < wins.Count; w++)
                if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
            foreach (var d in dbg) if (d.dist < 0f) { double p = -d.dist; penSum += p; penCnt++; if (p > penMax) penMax = p; }
        }
        var st = SkirtMeasure.Stats(calm);
        var r = new Res { calmMed = st.med, calmP90 = st.p90, calmMax = st.max, winPeak = winPeak, penMean = penCnt > 0 ? penSum / penCnt : 0, penMax = penMax, penCount = penCnt };
        for (int k = 0; k < 3; k++) r.ringMed[k] = SkirtMeasure.Stats(ring[k]).med;
        return r;
    }

    static void AddFarDummies(PhysicsWorld world)
    {
        const float X = 100000f;
        var floor = new RigidBody(new BoxShape(new Vec3(50, 1, 50))) { Mode = PhysicsMode.BoneFollow, Group = 0, CollisionMask = 0xFFFF };
        floor.SetMassProps(0f); floor.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X, 0, 0)); world.AddBody(floor);
        var rnd = new Random(12345);
        for (int i = 0; i < 6; i++)
        {
            var s = new RigidBody(new SphereShape(1.0f)) { Mode = PhysicsMode.Dynamic, Group = 0, CollisionMask = 0xFFFF };
            s.SetMassProps(1f);
            s.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X + (float)(rnd.NextDouble() * 4 - 2), 3 + i * 2.2f, (float)(rnd.NextDouble() * 4 - 2)));
            world.AddBody(s);
        }
    }

    static (double avg, double p95, double max) Timing(bool split, int steps)
    {
        var b = PmxPhysicsBuilder.Build(model); var w = b.World; w.UseSplitImpulse = split; w.Gravity = new Vec3(0, -98f, 0);
        var ms = new List<double>(steps);
        for (int i = 0; i < steps; i++) { var sw = Stopwatch.StartNew(); w.StepSimulation(w.FixedTimeStep); sw.Stop(); if (i >= 5) ms.Add(sw.Elapsed.TotalMilliseconds); }
        ms.Sort();
        return (ms.Average(), ms[(int)(0.95 * (ms.Count - 1))], ms[ms.Count - 1]);
    }

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(PmxPath); F = csv.FrameCount;
        skirt = SkirtMeasure.ExtractVerticalJoints(model);
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, DT);
        wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        inWin = new bool[F];
        foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        // MMDの窓別 傾きmax (CSVボーン姿勢から直接)
        refWinMax = new float[wins.Count];
        for (int f = 0; f < F; f++)
            for (int w = 0; w < wins.Count; w++)
                if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30))
                {
                    float m = 0;
                    foreach (var sj in skirt)
                        if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb))
                        { float t = SkirtMeasure.TiltDeg(pb.Rotation, cb.Rotation); if (t > m) m = t; }
                    if (m > refWinMax[w]) refWinMax[w] = m;
                }

        var off = Run(false, 0);
        var on = Run(true, 0);

        L("==================== Split Impulse 効果検証 ====================");
        L("\n[平時 スカート傾き] (全体 中央/p90/max, リング別中央)");
        L($"  従来(off): 中央={off.calmMed:F3} p90={off.calmP90:F3} max={off.calmMax:F3} | ring0={off.ringMed[0]:F2} ring1={off.ringMed[1]:F2} ring2={off.ringMed[2]:F2}");
        L($"  Split(on): 中央={on.calmMed:F3} p90={on.calmP90:F3} max={on.calmMax:F3} | ring0={on.ringMed[0]:F2} ring1={on.ringMed[1]:F2} ring2={on.ringMed[2]:F2}");

        L("\n[12窓 傾きmax] 自前(off) | 自前(on) | MMD | 比(off/MMD) | 比(on/MMD)");
        var ratOff = new List<float>(); var ratOn = new List<float>();
        for (int w = 0; w < wins.Count; w++)
        {
            float ro = refWinMax[w] > 0 ? off.winPeak[w] / refWinMax[w] : 0;
            float rn = refWinMax[w] > 0 ? on.winPeak[w] / refWinMax[w] : 0;
            ratOff.Add(ro); ratOn.Add(rn);
            L($"  窓{w + 1,2} F{wins[w].StartFrame}: {off.winPeak[w],7:F2} | {on.winPeak[w],7:F2} | {refWinMax[w],7:F2} | {ro:F3} | {rn:F3}");
        }
        ratOff.Sort(); ratOn.Sort();
        L($"  比の中央値: off={SkirtMeasure.Percentile(ratOff.ToArray(), 50):F3}  on={SkirtMeasure.Percentile(ratOn.ToArray(), 50):F3}");

        L("\n[貫入量] (全動的接触, 貫入接触のみ)");
        L($"  従来(off): 平均={off.penMean:F4} 最大={off.penMax:F4} (件数={off.penCount})");
        L($"  Split(on): 平均={on.penMean:F4} 最大={on.penMax:F4} (件数={on.penCount})");

        L("\n[反復数掃引] 窓ピーク(全窓の最大) — 仮説の決め手: Splitで反復依存が弱まるか");
        L("  iters |  off(従来) |  on(Split) ");
        foreach (int it in new[] { 1, 3, 10, 20 })
        {
            var ro = Run(false, it); var rn = Run(true, it);
            float offMax = ro.winPeak.Max(); float onMax = rn.winPeak.Max();
            L($"   {it,3}  |  {offMax,8:F2}  |  {onMax,8:F2}");
        }

        L("\n[ノイズフロア] 遠方ダミー剛体の有無 (Split on)");
        var nd = Run(true, 0, dummies: false); var wd = Run(true, 0, dummies: true);
        float dMax = 0; for (int w = 0; w < wins.Count; w++) dMax = Math.Max(dMax, Math.Abs(nd.winPeak[w] - wd.winPeak[w]));
        dMax = Math.Max(dMax, Math.Abs(nd.calmMed - wd.calmMed));
        L($"  窓ピーク/平時中央の最大差={dMax:F4} => {(dMax < 1e-3f ? "一致(ノイズフロア≈0)" : "揺れあり")}");

        var toff = Timing(false, 300); var ton = Timing(true, 300);
        L("\n[性能] modelA.pmx 300ステップ StepSimulation");
        L($"  従来(off): avg={toff.avg:F3}ms p95={toff.p95:F3}ms max={toff.max:F3}ms");
        L($"  Split(on): avg={ton.avg:F3}ms p95={ton.p95:F3}ms max={ton.max:F3}ms");

        string tag = Environment.GetEnvironmentVariable("SPLIT_TAG") ?? "run";
        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, $"split_{tag}.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
