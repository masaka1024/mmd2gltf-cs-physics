// 診断のみ(実装・パラメータ変更なし): ソルバ収束特性の切り分け。
//   タスク1: 反復数掃引(1..20)で 12窓比の中央値 と 平時中央値 をMMDと比較。
//   タスク2: island 分割(joint連結成分)の解析 と Joint解く順序の掃引。
// 順序切替は builder.World.Joints (public List) を外部から並べ替えるだけで、本体は無改変。
// (「各反復で往復」だけは SubStep 内の一時パッチが要るため別途。ここでは静的4順序を測る。)
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class SolverDiag
{
    static readonly string PmxPath = TestData.PmxPath();
    const float DT = 1f / 30f;
    const float RefCalmMed = 11.39f; // MMD 平時傾き中央値 (Python参照値)
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static List<SkirtMeasure.TurnWindow> wins; static bool[] inWin; static List<SkirtJoint> skirt;
    static float[] refWinMax;

    class Res { public float calmMed, calmP90, calmMax, ratioMed; public float[] winPeak; public double penMean, penMax; }

    // orderMode: 0=現状(PMX順) 1=逆順 2=リング昇順(ring0→2) 3=リング降順(ring2→0)
    static Res Run(int iters, int orderMode)
    {
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        if (iters > 0) world.SolverIterations = iters;
        if (orderMode == 4) world.AlternateJointDir = true; else ReorderJoints(world, model, orderMode);
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(DT);
        var calm = new List<float>(); var winPeak = new float[wins.Count];
        double penSum = 0, penMax = 0; long penCnt = 0;
        for (int f = 0; f < F; f++)
        {
            Apply(f); dbg.Clear(); world.StepSimulation(DT);
            float fmax = 0;
            foreach (var sj in skirt) { float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb)); if (!inWin[f]) calm.Add(t); if (t > fmax) fmax = t; }
            for (int w = 0; w < wins.Count; w++) if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
            foreach (var d in dbg) if (d.dist < 0f) { double p = -d.dist; penSum += p; penCnt++; if (p > penMax) penMax = p; }
        }
        var st = SkirtMeasure.Stats(calm);
        var ratios = new List<float>();
        for (int w = 0; w < wins.Count; w++) if (refWinMax[w] > 0) ratios.Add(winPeak[w] / refWinMax[w]);
        ratios.Sort();
        return new Res { calmMed = st.med, calmP90 = st.p90, calmMax = st.max, winPeak = winPeak, penMean = penCnt > 0 ? penSum / penCnt : 0, penMax = penMax, ratioMed = SkirtMeasure.Percentile(ratios.ToArray(), 50) };
    }

    static int RingOf(Joint j)
    {
        int bi = j.BodyB != null ? j.BodyB.BoneIndex : -1;
        if (bi < 0 || bi >= model.BoneNames.Count) return 99;
        foreach (var p in model.BoneNames[bi].Split('_')) if (int.TryParse(p, out int r)) return r;
        return 99;
    }

    static void ReorderJoints(PhysicsWorld world, PmxPhysicsModel model, int mode)
    {
        if (mode == 0) return;
        var js = world.Joints.ToList();
        List<Joint> ordered;
        if (mode == 1) { ordered = js; ordered.Reverse(); }
        else if (mode == 2) ordered = js.OrderBy(RingOf).ToList();      // 安定ソート
        else ordered = js.OrderByDescending(RingOf).ToList();
        world.Joints.Clear(); world.Joints.AddRange(ordered);
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
        refWinMax = new float[wins.Count];
        for (int f = 0; f < F; f++)
            for (int w = 0; w < wins.Count; w++)
                if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30))
                {
                    float m = 0; foreach (var sj in skirt) if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb)) { float t = SkirtMeasure.TiltDeg(pb.Rotation, cb.Rotation); if (t > m) m = t; }
                    if (m > refWinMax[w]) refWinMax[w] = m;
                }

        L("==================== ソルバ収束特性の切り分け ====================");
        L($"MMD 平時中央={RefCalmMed}  (12窓のMMD傾きmaxはCSVから算出)");

        // ---- タスク1: 反復数掃引 ----
        L("\n---------- タスク1: 反復数掃引 (order=現状) ----------");
        L("  iters | 平時中央 | 平時p90 | 12窓比中央 | 貫入平均 | 貫入最大");
        float bestRatioDist = 1e9f; int bestIter = 10;
        for (int it = 1; it <= 20; it++)
        {
            var r = Run(it, 0);
            L($"   {it,3}  | {r.calmMed,7:F3} | {r.calmP90,6:F2} | {r.ratioMed,9:F3} | {r.penMean,7:F4} | {r.penMax,7:F3}");
            float d = Math.Abs(r.ratioMed - 1.0f);
            if (d < bestRatioDist) { bestRatioDist = d; bestIter = it; }
        }
        L($"  → 12窓比が最も1.0に近い反復数: {bestIter}");

        // ---- タスク2-1..3: island 分割 (joint連結成分) ----
        L("\n---------- タスク2: island 分割 (joint による動的剛体の連結成分) ----------");
        var builder = PmxPhysicsBuilder.Build(model);
        int nb = builder.Bodies.Count;
        var uf = new int[nb]; for (int i = 0; i < nb; i++) uf[i] = i;
        int Find(int x) { while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; } return x; }
        void Union(int a, int b) { uf[Find(a)] = Find(b); }
        bool Dyn(RigidBody b) => !b.IsStaticOrKinematic;
        int jointEdges = 0;
        foreach (var pj in model.Joints)
        {
            if (pj.RigidBodyAIndex < 0 || pj.RigidBodyBIndex < 0) continue;
            var a = builder.Bodies[pj.RigidBodyAIndex]; var b = builder.Bodies[pj.RigidBodyBIndex];
            if (Dyn(a) && Dyn(b)) { Union(a.Index, b.Index); jointEdges++; } // 静的/kinematic は island 境界
        }
        var comp = new Dictionary<int, List<int>>();
        for (int i = 0; i < nb; i++) if (Dyn(builder.Bodies[i])) { int r = Find(i); if (!comp.ContainsKey(r)) comp[r] = new List<int>(); comp[r].Add(i); }
        int dynCount = comp.Values.Sum(v => v.Count);
        L($"  動的剛体数={dynCount}  joint辺(両端動的)={jointEdges}  island数={comp.Count}");
        // スカート島
        var skirtIdx = new HashSet<int>();
        for (int i = 0; i < nb; i++) if (builder.Bodies[i].Name.StartsWith("スカート") && Dyn(builder.Bodies[i])) skirtIdx.Add(i);
        var skirtRoots = skirtIdx.Select(Find).Distinct().ToList();
        L($"  スカート動的剛体数={skirtIdx.Count}  それらが属する island 数={skirtRoots.Count} ({(skirtRoots.Count == 1 ? "★全て1つのislandに連結" : "複数islandに分割")})");
        L("  island サイズ上位: " + string.Join(", ", comp.Values.OrderByDescending(v => v.Count).Take(6).Select(v => v.Count + "体")));
        L("  (脚/胴コライダーは kinematic のため island 境界。スカートは縦36+横52 Jointで相互連結)");

        // ---- タスク2 診断: Joint 解く順序の掃引 (iters=10) ----
        L("\n---------- タスク2 診断: Joint解く順序 掃引 (iters=10) ----------");
        L("  順序        | 窓ピークmax | 12窓比中央 | 平時中央");
        string[] names = { "現状(PMX順)", "逆順", "リング昇順0→2", "リング降順2→0", "往復(反復毎に順↔逆)" };
        for (int mode = 0; mode < 5; mode++)
        {
            var r = Run(10, mode);
            L($"  {names[mode],-14} | {r.winPeak.Max(),9:F2} | {r.ratioMed,9:F3} | {r.calmMed,7:F3}");
        }
        L("  (ノイズフロアは0なので、順序間の差はすべて実質的な収束先の差)");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "solverdiag_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
