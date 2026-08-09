// タスク1(調査のみ): スカート×左太もも の貫入を「全7001フレーム・同一分母」で測り直す。
// タスクCの平均は自前=貫入フレームのみ/本家=全フレーム と分母が不整合だった。ここでは
// 自前・本家とも全フレーム(貫入なし=0)で統計し、区間別・分布・時系列CSVを出す。
// 自前=物理の実剛体をDetect、本家=CSVボーン*offsetで剛体復元してDetect(同一narrowphase)。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class PenRange
{
    static readonly string Pmx1 = TestData.PmxPath();
    static readonly string Pmx2 = TestData.PmxPath();
    const float FRAME = 1f / 30f;
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }
    static readonly string[] Targets = { "スカート_0_5", "スカート_1_5", "スカート_2_5" };
    const string Leg = "左太もも";
    static List<ContactPoint> buf = new();

    static BoneCsv csv; static PmxPhysicsModel model; static int F; static bool[] inWin;

    static float Depth(RigidBody a, RigidBody b) { buf.Clear(); GjkEpa.Detect(a, b, buf); float d = 0; foreach (var cp in buf) { float p = -cp.Distance; if (p > d) d = p; } return d; }

    static int Main()
    {
        string pmx = File.Exists(Pmx1) ? Pmx1 : Pmx2;
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(pmx); F = csv.FrameCount;
        var skirtJ = SkirtMeasure.ExtractVerticalJoints(model);
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, FRAME);
        var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        inWin = new bool[F]; foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        // ---- 自前パス (物理 実効1/60) ----
        var builder = PmxPhysicsBuilder.Build(model); var world = builder.World;
        world.SolverIterations = 10; world.FixedTimeStep = FRAME; world.SubSteps = 2;
        RigidBody Body(string n) => builder.Bodies.First(b => b.Name == n);
        var leg = Body(Leg); var tg = Targets.Select(Body).ToArray();
        var driven = new List<(BoneLink l, string bone)>();
        foreach (var l in builder.BoneLinks) if (l.Mode == PhysicsMode.BoneFollow && l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[l.BoneIndex])) driven.Add((l, model.BoneNames[l.BoneIndex]));
        void Apply(int f) { foreach (var (l, bn) in driven) if (csv.TryGet(f, bn, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        var self = new float[Targets.Length][]; for (int t = 0; t < Targets.Length; t++) self[t] = new float[F];
        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(FRAME);
        for (int f = 0; f < F; f++) { Apply(f); world.StepSimulation(FRAME); for (int t = 0; t < Targets.Length; t++) self[t][f] = Depth(tg[t], leg); }

        // ---- 本家パス (CSVボーン*offsetで剛体復元) ----
        var b2 = PmxPhysicsBuilder.Build(model);
        RigidBody Body2(string n) => b2.Bodies.First(x => x.Name == n);
        var leg2 = Body2(Leg); var tg2 = Targets.Select(Body2).ToArray();
        (string bone, RigidTransform off) LinkOf(RigidBody rb) { int i = b2.Bodies.IndexOf(rb); var ln = b2.BoneLinks[i]; string bn = (ln.BoneIndex >= 0 && ln.BoneIndex < model.BoneNames.Count) ? model.BoneNames[ln.BoneIndex] : ""; return (bn, ln.BodyOffsetFromBone); }
        var legLink = LinkOf(leg2); var tgLink = tg2.Select(LinkOf).ToArray();
        void Place(RigidBody rb, (string bone, RigidTransform off) lk, int f) { if (csv.TryGet(f, lk.bone, out var bw)) rb.WorldTransform = bw * lk.off; }
        var refD = new float[Targets.Length][]; for (int t = 0; t < Targets.Length; t++) refD[t] = new float[F];
        for (int f = 0; f < F; f++) { Place(leg2, legLink, f); for (int t = 0; t < Targets.Length; t++) { Place(tg2[t], tgLink[t], f); refD[t][f] = Depth(tg2[t], leg2); } }

        // ---- 出力 ----
        L("==================== スカート×左太もも 貫入 全編再測定 (同一分母=全7001F) ====================");
        L("タスクC訂正: 自前平均は「貫入フレームのみ」で割っていた(分母不整合)。以下は全フレーム基準。");

        // 時系列CSV
        var sb = new StringBuilder("frame,time," + string.Join(",", Targets.Select(t => $"self_{t.Replace("スカート_", "")},ref_{t.Replace("スカート_", "")}")) + "\n");
        for (int f = 0; f < F; f++) { sb.Append($"{f},{f / 30.0:F4}"); for (int t = 0; t < Targets.Length; t++) sb.Append($",{self[t][f]:F5},{refD[t][f]:F5}"); sb.Append('\n'); }
        string csvOut = Path.Combine(AppContext.BaseDirectory, "penrange_timeseries.csv");
        File.WriteAllText(csvOut, sb.ToString(), new UTF8Encoding(false));
        L($"\n[時系列CSV] {csvOut} ({F}行, 列=frame,time,self/ref×3ペア)");

        var segs = new (string name, int lo, int hi)[] { ("F0-120", 0, 120), ("F121-1000", 121, 1000), ("F1001-3000", 1001, 3000), ("F3001-5000", 3001, 5000), ("F5001-7000", 5001, F - 1) };
        for (int t = 0; t < Targets.Length; t++)
        {
            L($"\n---------- {Targets[t]} × {Leg} ----------");
            Report("自前", self[t]);
            Report("本家", refD[t]);
            L("  区間別 平均 (自前 | 本家):");
            foreach (var s in segs) L($"    {s.name,-12}: {SegMean(self[t], s.lo, s.hi):F4} | {SegMean(refD[t], s.lo, s.hi):F4}");
            int selfDeep = self[t].Count(x => x > 0.5f), selfShallow = self[t].Count(x => x < 0.1f);
            int refDeep = refD[t].Count(x => x > 0.5f), refShallow = refD[t].Count(x => x < 0.1f);
            L($"  貫入>0.5: 自前={selfDeep}F ({100.0 * selfDeep / F:F1}%) | 本家={refDeep}F ({100.0 * refDeep / F:F1}%)");
            L($"  貫入<0.1: 自前={selfShallow}F ({100.0 * selfShallow / F:F1}%) | 本家={refShallow}F ({100.0 * refShallow / F:F1}%)");
            // 自前の深い(>0.5)フレームが窓内か
            var deepFrames = Enumerable.Range(0, F).Where(f => self[t][f] > 0.5f).ToList();
            int inW = deepFrames.Count(f => inWin[f]);
            L($"  自前>0.5 の窓内={inW}/{deepFrames.Count}  先頭10F: {string.Join(",", deepFrames.Take(10))}");
        }

        // ---- タスク2用: 推奨候補フレーム (スカート×左太もも の全編max/中盤max/最浅) ----
        var gmax = new float[F]; for (int f = 0; f < F; f++) { float m = 0; for (int t = 0; t < Targets.Length; t++) if (self[t][f] > m) m = self[t][f]; gmax[f] = m; }
        int AllMax(int lo, int hi) { int bf = lo; float bv = -1; for (int f = lo; f <= hi && f < F; f++) if (gmax[f] > bv) { bv = gmax[f]; bf = f; } return bf; }
        int gAll = AllMax(0, F - 1); int gMid = AllMax(2900, 3100);
        int shallow = -1; for (int f = 3500; f <= 4500; f++) if (gmax[f] < 0.01f) { shallow = f; break; }
        L("\n[タスク2 推奨候補] スカート×左太もも 基準:");
        L($"  全編最深フレーム = F{gAll} (深さ {gmax[gAll]:F3})");
        L($"  中盤(F2900-3100)最深 = F{gMid} (深さ {gmax[gMid]:F3})");
        L($"  最浅区間の代表 = F{shallow} (深さ {(shallow >= 0 ? gmax[shallow] : 0):F3})");
        L("  (全編で最も深いのは髪スパイク F2889=2.15。スカート主対象では上記)");

        // ---- ウォームアップ十分性: スカート_1_5×左太もも の初期貫入がwarmup数で変わるか ----
        L("\n---------- ウォームアップ掃引 (スカート_1_5×左太もも, F0-150の貫入) ----------");
        L("  warmup |  F0   |  F30  |  F60  | F120  | F150  | F0-150最大");
        foreach (int wu in new[] { 60, 300, 1000, 3000 })
        {
            var bb = PmxPhysicsBuilder.Build(model); var w2 = bb.World; w2.SolverIterations = 10; w2.FixedTimeStep = FRAME; w2.SubSteps = 2;
            var lg = bb.Bodies.First(x => x.Name == Leg); var sk = bb.Bodies.First(x => x.Name == "スカート_1_5");
            var drv = new List<(BoneLink l, string bone)>();
            foreach (var l in bb.BoneLinks) if (l.Mode == PhysicsMode.BoneFollow && l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[l.BoneIndex])) drv.Add((l, model.BoneNames[l.BoneIndex]));
            void Ap(int f) { foreach (var (l, bn) in drv) if (csv.TryGet(f, bn, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
            Ap(0); for (int s = 0; s < wu; s++) w2.StepSimulation(FRAME);
            float d0 = 0, d30 = 0, d60 = 0, d120 = 0, d150 = 0, mx = 0;
            for (int f = 0; f <= 150; f++) { Ap(f); w2.StepSimulation(FRAME); float d = Depth(sk, lg); if (d > mx) mx = d; if (f == 0) d0 = d; if (f == 30) d30 = d; if (f == 60) d60 = d; if (f == 120) d120 = d; if (f == 150) d150 = d; }
            L($"   {wu,5}  | {d0,5:F3} | {d30,5:F3} | {d60,5:F3} | {d120,5:F3} | {d150,5:F3} | {mx,6:F3}");
        }
        L("  → warmupを増やしても初期貫入が減らないなら「フレーム0近傍の脚ポーズが原因(整定では解けない)」、");
        L("    減るなら「60ステップのwarmup不足」。");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "penrange_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    static void Report(string tag, float[] a)
    {
        var s = (float[])a.Clone(); Array.Sort(s);
        double mean = 0; foreach (var x in a) mean += x; mean /= a.Length;
        L($"  {tag}: 全編 平均={mean:F4} 中央={SkirtMeasure.Percentile(s, 50):F4} p90={SkirtMeasure.Percentile(s, 90):F4} 最大={s[s.Length - 1]:F4}");
    }
    static double SegMean(float[] a, int lo, int hi) { double m = 0; int n = 0; for (int f = lo; f <= hi && f < a.Length; f++) { m += a[f]; n++; } return n > 0 ? m / n : 0; }
}
