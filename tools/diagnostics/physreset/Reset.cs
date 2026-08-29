// タスク2(測定): 物理開始時の剛体をボーン姿勢へ整合させる(ResetBodiesToBonePose)効果を測る。
//   初期貫入(F0-120)が解消したか / 定常・全編・傾き統計への影響 / warmup必要数 / ノイズフロア。
// 反復10・実効1/60(Sub2)固定。本体無改変(Resetは本体の公開API)。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class PhysReset
{
    static readonly string Pmx1 = TestData.PmxPath();
    static readonly string Pmx2 = TestData.PmxPath();
    // ★2026-08-29: RefCalmMed を「真OFF (純ソルバ)」基準へ。旧既定参照 (補正OFF+整え込み) では 11.39。
    const float FRAME = 1f / 30f; const float RefCalmMed = 10.87f; const string Leg = "左太もも";
    static readonly string[] Pairs = { "スカート_0_5", "スカート_1_5", "スカート_2_5" };
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }
    static List<ContactPoint> buf = new();
    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static List<SkirtMeasure.TurnWindow> wins; static bool[] inWin; static List<SkirtJoint> skirt; static float[] refWinMax;

    static float Depth(RigidBody a, RigidBody b) { buf.Clear(); GjkEpa.Detect(a, b, buf); float d = 0; foreach (var cp in buf) { float p = -cp.Distance; if (p > d) d = p; } return d; }

    class Res { public float calmMed, calmP90, calmMax, ratioMed; public double penMean, penMax; public float[][] pairDepth; }

    // mode: 0=リセット無, 1=CSVリセット(MMD物理姿勢・過拘束の対照), 2=FK-restリセット(本体汎用ヘルパ)
    static Res Run(int mode, int warmup, bool dummies = false)
    {
        var builder = PmxPhysicsBuilder.Build(model); var world = builder.World;
        world.SolverIterations = 10; world.FixedTimeStep = FRAME; world.SubSteps = 2;
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;
        if (dummies) AddFarDummies(world);
        var leg = builder.Bodies.First(b => b.Name == Leg); var tg = Pairs.Select(p => builder.Bodies.First(b => b.Name == p)).ToArray();
        var driven = new List<(BoneLink l, string bone)>();
        foreach (var l in builder.BoneLinks) if (l.Mode == PhysicsMode.BoneFollow && l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[l.BoneIndex])) driven.Add((l, model.BoneNames[l.BoneIndex]));
        void Apply(int f) { foreach (var (l, bn) in driven) if (csv.TryGet(f, bn, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }

        Apply(0);
        if (mode == 1)
            builder.ResetBodiesToBonePose(i => (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw)) ? (RigidTransform?)bw : null);
        else if (mode == 2)
        {
            // 本体の汎用 FK-rest ヘルパ (物理ボーンのCSV姿勢は無視され親から前計算される)。
            builder.ResetBodiesToBonePoseFk(i =>
                (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw))
                    ? (RigidTransform?)bw : null);
        }
        for (int s = 0; s < warmup; s++) world.StepSimulation(FRAME);

        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }
        var calm = new List<float>(); var winPeak = new float[wins.Count];
        double penSum = 0, penMax = 0; long penCnt = 0;
        var pd = new float[Pairs.Length][]; for (int t = 0; t < Pairs.Length; t++) pd[t] = new float[F];
        for (int f = 0; f < F; f++)
        {
            Apply(f); world.StepSimulation(FRAME);
            float fmax = 0;
            foreach (var sj in skirt) { float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb)); if (!inWin[f]) calm.Add(t); if (t > fmax) fmax = t; }
            for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
            for (int t = 0; t < Pairs.Length; t++) { float d = Depth(tg[t], leg); pd[t][f] = d; if (d > 0) { penSum += d; penCnt++; if (d > penMax) penMax = d; } }
        }
        var st = SkirtMeasure.Stats(calm);
        var ratios = new List<float>(); for (int w = 0; w < wins.Count; w++) if (refWinMax[w] > 0) ratios.Add(winPeak[w] / refWinMax[w]); ratios.Sort();
        return new Res { calmMed = st.med, calmP90 = st.p90, calmMax = st.max, ratioMed = SkirtMeasure.Percentile(ratios.ToArray(), 50), penMean = penCnt > 0 ? penSum / penCnt : 0, penMax = penMax, pairDepth = pd };
    }

    static void AddFarDummies(PhysicsWorld world)
    {
        const float X = 100000f;
        var floor = new RigidBody(new BoxShape(new Vec3(50, 1, 50))) { Mode = PhysicsMode.BoneFollow, Group = 0, CollisionMask = 0xFFFF };
        floor.SetMassProps(0f); floor.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X, 0, 0)); world.AddBody(floor);
        var rnd = new Random(12345);
        for (int i = 0; i < 6; i++) { var s = new RigidBody(new SphereShape(1.0f)) { Mode = PhysicsMode.Dynamic, Group = 0, CollisionMask = 0xFFFF }; s.SetMassProps(1f); s.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(X + (float)(rnd.NextDouble() * 4 - 2), 3 + i * 2.2f, (float)(rnd.NextDouble() * 4 - 2))); world.AddBody(s); }
    }

    static readonly (string name, int lo, int hi)[] Segs = { ("F0-120", 0, 120), ("F121-1000", 121, 1000), ("F1001-3000", 1001, 3000), ("F3001-5000", 3001, 5000), ("F5001-7000", 5001, 6999) };
    static double SegMean(float[] a, int lo, int hi) { double m = 0; int n = 0; for (int f = lo; f <= hi && f < a.Length; f++) { m += a[f]; n++; } return n > 0 ? m / n : 0; }

    static int Main()
    {
        string pmx = File.Exists(Pmx1) ? Pmx1 : Pmx2;
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(pmx); F = csv.FrameCount;
        skirt = SkirtMeasure.ExtractVerticalJoints(model);
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, FRAME);
        wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        inWin = new bool[F]; foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;
        refWinMax = new float[wins.Count];
        for (int f = 0; f < F; f++) for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30)) { float m = 0; foreach (var sj in skirt) if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb)) { float t = SkirtMeasure.TiltDeg(pb.Rotation, cb.Rotation); if (t > m) m = t; } if (m > refWinMax[w]) refWinMax[w] = m; }

        L("========== 物理リセット(全剛体をボーン姿勢へ整合) の効果 (Sub2, 反復10) ==========");
        L("列: [0]リセット無 | [1]CSVリセット(MMD物理姿勢) | [2]FK-restリセット(スカート=親駆動)");
        var m0 = Run(0, 60); var m1 = Run(1, 60); var m2 = Run(2, 60);

        L("\n[区間別 平均 貫入] スカート×左太もも ([0]無 | [1]CSV | [2]FK-rest)");
        for (int t = 0; t < Pairs.Length; t++)
        {
            L($"  {Pairs[t]}:");
            foreach (var s in Segs) L($"    {s.name,-12}: {SegMean(m0.pairDepth[t], s.lo, s.hi),7:F4} | {SegMean(m1.pairDepth[t], s.lo, s.hi),7:F4} | {SegMean(m2.pairDepth[t], s.lo, s.hi),7:F4}");
        }
        L("\n[全編 貫入統計] スカート×左太もも (全7001F, [0]|[1]|[2])");
        for (int t = 0; t < Pairs.Length; t++)
            L($"  {Pairs[t]}: 平均 {m0.pairDepth[t].Average():F4}|{m1.pairDepth[t].Average():F4}|{m2.pairDepth[t].Average():F4}  最大 {m0.pairDepth[t].Max():F3}|{m1.pairDepth[t].Max():F3}|{m2.pairDepth[t].Max():F3}  >0.5 {m0.pairDepth[t].Count(x => x > 0.5f)}|{m1.pairDepth[t].Count(x => x > 0.5f)}|{m2.pairDepth[t].Count(x => x > 0.5f)}F");

        L("\n[傾き統計] ([0]無 | [1]CSV | [2]FK-rest, 参照(真OFF) 平時中央=10.87, 12窓比目標1.0)");
        L($"  平時 中央 {m0.calmMed:F3}|{m1.calmMed:F3}|{m2.calmMed:F3}  p90 {m0.calmP90:F2}|{m1.calmP90:F2}|{m2.calmP90:F2}  max {m0.calmMax:F2}|{m1.calmMax:F2}|{m2.calmMax:F2}");
        L($"  12窓比 中央 {m0.ratioMed:F3}|{m1.ratioMed:F3}|{m2.ratioMed:F3}");

        L("\n[ウォームアップ必要数] FK-restリセット有りで warmup 0/10/60 の初期(F0-120)貫入と傾き");
        L("  warmup | 0_5 | 1_5 | 2_5 (F0-120貫入平均) | 平時中央 | 12窓比");
        foreach (int wu in new[] { 0, 10, 60 })
        {
            var r = Run(2, wu);
            L($"   {wu,5}  | {SegMean(r.pairDepth[0], 0, 120):F3} | {SegMean(r.pairDepth[1], 0, 120):F3} | {SegMean(r.pairDepth[2], 0, 120):F3} | {r.calmMed:F3} | {r.ratioMed:F3}");
        }

        L("\n[ノイズフロア] FK-restリセット・遠方ダミー有無で 平時中央/12窓比 が一致するか");
        var nd = Run(2, 60, false); var wd = Run(2, 60, true);
        float dMax = Math.Abs(nd.calmMed - wd.calmMed);
        L($"  平時中央 {nd.calmMed:F4} vs {wd.calmMed:F4} (差 {dMax:F4}) / 12窓比 {nd.ratioMed:F4} vs {wd.ratioMed:F4} => {(dMax < 1e-3f && Math.Abs(nd.ratioMed - wd.ratioMed) < 1e-3f ? "一致(ノイズフロア0)" : "揺れ")}");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "physreset_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
