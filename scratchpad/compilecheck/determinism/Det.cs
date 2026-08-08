// 検証(調整なし): 接触の解く順序を決定化した効果を測る。
//   検証1: スカート・髪と衝突しない「遠方ダミー動的剛体群」の有無で、スカート統計
//           (平時/12窓ピーク)が一致するか。修正前は揺れ、修正後は一致するはず。
//   検証2: 同一バイナリ2回で完全一致 (従来どおり)。
//   性能:   IA.pmx 300ステップの avg/p95/max。修正前後で比較 (ソート追加の劣化を測る)。
// 本体は不変 (すべて public API から)。ダミーは AABB が遠方でIA剛体と交差しないため非衝突。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Det
{
    const string PmxPath = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
    const float DT = 1f / 30f;
    static StringBuilder O = new(); static void L(string s = "") => O.AppendLine(s);

    // スカート統計: 平時 median/p90/max と 12窓の frame-max ピーク。
    struct SkirtStats { public float calmMed, calmP90, calmMax; public float[] winPeak; }

    static SkirtStats RunSkirt(BoneCsv csv, PmxPhysicsModel model, List<SkirtMeasure.TurnWindow> wins, bool[] inWin, bool withDummies)
    {
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        var skirt = SkirtMeasure.ExtractVerticalJoints(model);

        if (withDummies) AddFarDummies(world);

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        int F = csv.FrameCount;
        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(DT);

        var calm = new List<float>(); var winPeak = new float[wins.Count];
        for (int f = 0; f < F; f++)
        {
            Apply(f); world.StepSimulation(DT);
            float fmax = 0;
            foreach (var sj in skirt)
            {
                float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb));
                if (!inWin[f]) calm.Add(t);
                if (t > fmax) fmax = t;
            }
            for (int w = 0; w < wins.Count; w++)
                if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30) && fmax > winPeak[w]) winPeak[w] = fmax;
        }
        var st = SkirtMeasure.Stats(calm);
        return new SkirtStats { calmMed = st.med, calmP90 = st.p90, calmMax = st.max, winPeak = winPeak };
    }

    // スカート/髪と衝突しない遠方ダミー: x=100000 に動的球6+キネマティック床1。互いに弾んで
    // マニフォールドを毎ステップ生成/消滅させ、Dictionaryの挿入/削除履歴を撹乱する。
    // AABBが遠方なのでIA剛体とは決して交差しない (=スカート/髪へ物理的影響なし)。
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

    static (double avg, double p95, double max) Timing(PmxPhysicsModel model, int steps)
    {
        var b = PmxPhysicsBuilder.Build(model);
        var w = b.World; w.Gravity = new Vec3(0, -98f, 0);
        var ms = new List<double>(steps);
        for (int i = 0; i < steps; i++)
        {
            var sw = Stopwatch.StartNew(); w.StepSimulation(w.FixedTimeStep); sw.Stop();
            if (i >= 5) ms.Add(sw.Elapsed.TotalMilliseconds); // 先頭はJIT暖機を除く
        }
        ms.Sort();
        double avg = ms.Average();
        double p95 = ms[(int)(0.95 * (ms.Count - 1))];
        return (avg, p95, ms[ms.Count - 1]);
    }

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP]"); return 0; }
        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        int F = csv.FrameCount;
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, DT);
        var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        var inWin = new bool[F];
        foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        string tag = Environment.GetEnvironmentVariable("DET_TAG") ?? "run";
        L($"==================== 決定性検証 [{tag}] ====================");

        // 検証1: ダミー有無でスカート一致するか
        var baseA = RunSkirt(csv, model, wins, inWin, withDummies: false);
        var baseB = RunSkirt(csv, model, wins, inWin, withDummies: true);
        float dCalm = Math.Max(Math.Abs(baseA.calmMed - baseB.calmMed), Math.Max(Math.Abs(baseA.calmP90 - baseB.calmP90), Math.Abs(baseA.calmMax - baseB.calmMax)));
        float dWinMax = 0; int wi = -1;
        for (int w = 0; w < wins.Count; w++) { float d = Math.Abs(baseA.winPeak[w] - baseB.winPeak[w]); if (d > dWinMax) { dWinMax = d; wi = w; } }
        L("\n[検証1] 無関係な遠方ダミー剛体(動的球6+床)の有無でスカート統計が一致するか:");
        L($"  平時 中央/p90/max: ダミー無 {baseA.calmMed:F3}/{baseA.calmP90:F3}/{baseA.calmMax:F3}  ダミー有 {baseB.calmMed:F3}/{baseB.calmP90:F3}/{baseB.calmMax:F3}  最大差={dCalm:F4}");
        L($"  12窓ピーク 最大差={dWinMax:F4}" + (wi >= 0 ? $" (窓{wi + 1}: {baseA.winPeak[wi]:F3} vs {baseB.winPeak[wi]:F3})" : ""));
        L("  窓別 (ダミー無 | ダミー有 | 差):");
        for (int w = 0; w < wins.Count; w++)
            L($"    窓{w + 1,2} F{wins[w].StartFrame}: {baseA.winPeak[w],7:F3} | {baseB.winPeak[w],7:F3} | {Math.Abs(baseA.winPeak[w] - baseB.winPeak[w]):F4}");
        bool identical = dCalm < 1e-3f && dWinMax < 1e-3f;
        L($"  => {(identical ? "★一致 (ノイズフロア≈0)" : $"揺れあり (平時{dCalm:F3}°, 窓{dWinMax:F3}°)")}");

        // 検証2: 同一条件2回で完全一致
        var r1 = RunSkirt(csv, model, wins, inWin, false);
        var r2 = RunSkirt(csv, model, wins, inWin, false);
        float d2 = Math.Abs(r1.calmMed - r2.calmMed);
        for (int w = 0; w < wins.Count; w++) d2 = Math.Max(d2, Math.Abs(r1.winPeak[w] - r2.winPeak[w]));
        L($"\n[検証2] 同一条件2回の最大差={d2:F6} => {(d2 == 0f ? "完全一致" : "不一致(!)")}");

        // 新ベースライン (ダミー無)
        L("\n[新ベースライン] スカート統計 (以降の比較基準):");
        L($"  平時 中央={baseA.calmMed:F3} p90={baseA.calmP90:F3} max={baseA.calmMax:F3}");
        for (int w = 0; w < wins.Count; w++)
            L($"  窓{w + 1,2} F{wins[w].StartFrame}-{wins[w].EndFrame} peakYaw={wins[w].PeakYaw:F0}: {baseA.winPeak[w]:F3}");

        // 性能
        var t = Timing(model, 300);
        L($"\n[性能] IA.pmx 300ステップ StepSimulation: avg={t.avg:F3}ms p95={t.p95:F3}ms max={t.max:F3}ms");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, $"det_{tag}.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
