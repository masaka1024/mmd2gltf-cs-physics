// タスクA(調査・修正なし): 貫入の時系列を出し、Unityで「どこを見ればいいか」を確定する。
//   modelA.pmx を CSV 駆動で7001フレーム再生し、フレームごとの最大貫入・上位貫入・窓内外内訳・
//   常習剛体ランキングを出力。現行(実効1/60=Sub2)と旧(1/30=Sub1)で最深箇所を比較。
// 貫入深さ = 接触の -Distance (Distance<0 が貫入)。DebugContacts フックから取得(本体無改変)。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Pen
{
    static readonly string PmxPath = TestData.PmxPath();
    static readonly string PmxPath2 = TestData.PmxPath();
    const float FRAME = 1f / 30f;
    const float DeepStore = 0.1f;   // これ以上の貫入だけ個別保持(尾の分析用)。平均0.03に対し十分深い。
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    struct Deep { public int frame; public float depth; public string a, b; public bool win; }

    static BoneCsv csv; static PmxPhysicsModel model; static int F; static bool[] inWin;

    // 戻り: perFrameMax[frame], perFramePair[frame], 深い接触リスト
    static (float[] maxD, (string a, string b)[] pair, List<Deep> deep) Run(int subSteps)
    {
        var builder = PmxPhysicsBuilder.Build(model); var world = builder.World;
        world.SolverIterations = 10; world.FixedTimeStep = FRAME; world.SubSteps = subSteps;
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;
        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(FRAME);
        var maxD = new float[F]; var pair = new (string, string)[F]; var deep = new List<Deep>();
        for (int f = 0; f < F; f++)
        {
            Apply(f); dbg.Clear(); world.StepSimulation(FRAME);
            float fm = 0; string pa = "", pb = "";
            foreach (var d in dbg) if (d.dist < 0f)
            {
                float dep = -d.dist;
                if (dep > fm) { fm = dep; pa = d.a; pb = d.b; }
                if (dep > DeepStore) deep.Add(new Deep { frame = f, depth = dep, a = d.a, b = d.b, win = inWin[f] });
            }
            maxD[f] = fm; pair[f] = (pa, pb);
        }
        return (maxD, pair, deep);
    }

    static int Main()
    {
        string pmx = File.Exists(PmxPath) ? PmxPath : PmxPath2;
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(pmx); F = csv.FrameCount;
        var skirt = SkirtMeasure.ExtractVerticalJoints(model);
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, FRAME);
        var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        inWin = new bool[F]; foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        L("==================== 貫入の時系列分析 (実効1/60=Sub2 現行) ====================");
        var cur = Run(2);   // 現行
        var old = Run(1);   // 旧(1/30)

        // 1) フレームごと最大貫入 CSV
        var sb = new StringBuilder("frame,time,maxDepth,bodyA,bodyB\n");
        for (int f = 0; f < F; f++) sb.Append($"{f},{f / 30.0:F4},{cur.maxD[f]:F5},{cur.pair[f].Item1},{cur.pair[f].Item2}\n");
        string csvOut = Path.Combine(AppContext.BaseDirectory, "penetration_perframe.csv");
        File.WriteAllText(csvOut, sb.ToString(), new UTF8Encoding(false));
        L($"\n[1] フレーム毎最大貫入CSV: {csvOut} ({F}行)");

        // 2) 貫入深さ 上位20 (個別接触)
        L("\n[2] 貫入深さ 上位20 (frame, time, 深さ, 剛体ペア, 窓)");
        foreach (var d in cur.deep.OrderByDescending(x => x.depth).Take(20))
            L($"   F{d.frame,5} t={d.frame / 30.0,6:F2}s 深さ={d.depth:F4}  {d.a} × {d.b}  {(d.win ? "[窓内]" : "")}");

        // 3) 深い貫入(>DeepStore)の窓内外内訳
        int inCnt = cur.deep.Count(d => d.win), outCnt = cur.deep.Count - inCnt;
        int winFrames = 0; for (int f = 0; f < F; f++) if (inWin[f]) winFrames++;
        L($"\n[3] 深い貫入(>{DeepStore})の窓内外内訳: 窓内={inCnt} 窓外={outCnt} (総{cur.deep.Count}件)");
        L($"    参考: 窓内フレーム割合={100.0 * winFrames / F:F1}% ({winFrames}/{F}) → 窓内に偏るなら旋回で深くなる");
        // 深さ帯別
        L("    深さ帯別件数: " + string.Join("  ", new[] { (0.1f, 0.2f), (0.2f, 0.5f), (0.5f, 1.0f), (1.0f, 99f) }.Select(r => $"[{r.Item1}-{r.Item2})={cur.deep.Count(d => d.depth >= r.Item1 && d.depth < r.Item2)}")));

        // 4) 常習剛体ランキング (深い接触への関与回数)
        var freq = new Dictionary<string, int>();
        void Bump(string n) { freq[n] = freq.GetValueOrDefault(n) + 1; }
        foreach (var d in cur.deep) { Bump(d.a); Bump(d.b); }
        L("\n[4] 深い貫入の常習剛体 上位12 (関与回数)");
        foreach (var kv in freq.OrderByDescending(k => k.Value).Take(12)) L($"   {kv.Key,-16} {kv.Value}回");
        // ペア種別
        int ss = cur.deep.Count(d => d.a.StartsWith("スカート") && d.b.StartsWith("スカート"));
        int sOther = cur.deep.Count(d => d.a.StartsWith("スカート") ^ d.b.StartsWith("スカート"));
        int oo = cur.deep.Count - ss - sOther;
        L($"    ペア種別: スカート同士={ss} スカート×非スカート={sOther} その他={oo}");

        // 5) 現行 vs 旧 の最深箇所比較
        int cf = Array.IndexOf(cur.maxD, cur.maxD.Max()); int of = Array.IndexOf(old.maxD, old.maxD.Max());
        L("\n[5] 現行(Sub2) vs 旧(Sub1) の最大貫入の発生箇所");
        L($"   現行: F{cf} t={cf / 30.0:F2}s 深さ={cur.maxD[cf]:F4} {cur.pair[cf].Item1}×{cur.pair[cf].Item2} {(inWin[cf] ? "[窓内]" : "[窓外]")}");
        L($"   旧  : F{of} t={of / 30.0:F2}s 深さ={old.maxD[of]:F4} {old.pair[of].Item1}×{old.pair[of].Item2} {(inWin[of] ? "[窓内]" : "[窓外]")}");
        L($"   → 発生フレーム {(cf == of ? "同じ" : "異なる")} / 剛体ペア {((cur.pair[cf].Item1 == old.pair[of].Item1 && cur.pair[cf].Item2 == old.pair[of].Item2) ? "同じ" : "異なる")}");
        // 現行の最深フレームで旧はどうだったか(同じ箇所の深さ推移)
        L($"   現行最深F{cf} での旧(Sub1)の最大貫入={old.maxD[cf]:F4} ({old.pair[cf].Item1}×{old.pair[cf].Item2})");
        L($"   旧最深F{of} での現行(Sub2)の最大貫入={cur.maxD[of]:F4} ({cur.pair[of].Item1}×{cur.pair[of].Item2})");
        // 全体統計
        L($"\n   全体: 現行 平均={Avg(cur.maxD):F4} 最大={cur.maxD.Max():F4} | 旧 平均={Avg(old.maxD):F4} 最大={old.maxD.Max():F4}");

        // Unity 用に「深い貫入フレーム候補」を出す(タスクBのジャンプ配列用)
        var cand = cur.deep.OrderByDescending(d => d.depth).Select(d => d.frame).Distinct().Take(12).ToList();
        L("\n[Unity用] 貫入が深いフレーム候補(重複除去・深い順12): " + string.Join(", ", cand));

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "penetration_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    static double Avg(float[] a) { double s = 0; foreach (var x in a) s += x; return s / a.Length; }
}
