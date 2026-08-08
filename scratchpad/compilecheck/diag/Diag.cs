// ===========================================================================
// 診断 (調整なし): オーバースイングの原因切り分け。
//  タスク1: 代表窓(1/4/6)の時系列CSV + 4指標(位相/立ち上がり/減衰/オーバーシュート)。
//  タスク2: SolverIterations 1/3/10/20 での自前傾き統計 (バネのソルバ反復不感性の確認)。
// 本体は不変。CSV/PMX 未検出なら SKIP。
// ===========================================================================
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Diag
{
    static readonly string PmxPath = TestData.PmxPath();
    static string OutDir;

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP] CSV/PMX 未検出"); return 0; }
        OutDir = AppContext.BaseDirectory;

        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        var joints = SkirtMeasure.ExtractVerticalJoints(model);
        int F = csv.FrameCount, nj = joints.Count;
        float dt = 1f / 30f;

        // 参照(本家) 傾き/ヨー
        var refTilt = new float[F][]; var refYaw = new float[F][]; var refFmax = new float[F];
        for (int f = 0; f < F; f++)
        {
            refTilt[f] = new float[nj]; refYaw[f] = new float[nj]; float mx = 0;
            for (int j = 0; j < nj; j++)
            {
                var pj = joints[j];
                if (csv.TryGet(f, pj.ParentBone, out var p) && csv.TryGet(f, pj.ChildBone, out var c))
                { refTilt[f][j] = SkirtMeasure.TiltDeg(p.Rotation, c.Rotation); refYaw[f][j] = SkirtMeasure.YawOfRelDeg(p.Rotation, c.Rotation); if (refTilt[f][j] > mx) mx = refTilt[f][j]; }
            }
            refFmax[f] = mx;
        }
        var yawRate = new float[F];
        for (int f = 1; f < F; f++)
            if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1))
                yawRate[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, dt);
        var wins = SkirtMeasure.DetectTurnWindows(yawRate, 360f);

        // 既定(10反復)で物理を回し、時系列を取得
        var drv = new HeadlessDriver();
        drv.Run(csv, model, joints);
        var physTilt = drv.PhysTilt; var physYaw = drv.PhysRelYaw; var physFmax = drv.PhysFrameMaxTilt;

        // ---- タスク1: 代表窓の時系列CSV + 4指標 ----
        float steadyMed = SkirtMeasure.Stats(Flatten(refTilt, nj)).med; // 本家中央値を平時基準に
        var log = new StringBuilder();
        int[] targets = { 1, 4, 6 };
        foreach (int wi in targets)
        {
            var w = wins[wi - 1];
            int s = Math.Max(0, w.StartFrame - 30), e = Math.Min(F - 1, w.EndFrame + 90);
            WriteWindowCsv(wi, w, s, e, yawRate, refTilt, physTilt, refYaw, physYaw, joints, nj);
            Metrics(log, wi, w, s, e, yawRate, physFmax, refFmax, steadyMed);
        }

        // ---- タスク2: SolverIterations 掃引 ----
        log.AppendLine();
        log.AppendLine("========== タスク2: SolverIterations 掃引 (自前傾き, バネのソルバ反復不感性) ==========");
        log.AppendLine("  iters | 全体med | 全体p90 | 全体max | 窓1max | 窓6max");
        foreach (int it in new[] { 1, 3, 10, 20 })
        {
            var d2 = new HeadlessDriver { SolverIterationsOverride = it };
            d2.Run(csv, model, joints);
            var all = Flatten(d2.PhysTilt, nj);
            var st = SkirtMeasure.Stats(all);
            float w1 = WinMax(wins[0], d2.PhysFrameMaxTilt, F), w6 = WinMax(wins[5], d2.PhysFrameMaxTilt, F);
            log.AppendLine($"  {it,5} | {st.med,7:F2} | {st.p90,7:F2} | {st.max,7:F1} | {w1,6:F1} | {w6,6:F1}");
        }
        log.AppendLine("  (自前バネは ApplySprings でサブステップ毎に1回だけ適用=反復非依存。");
        log.AppendLine("   傾きが反復数にほぼ不感なら『バネがソルバ反復に乗っていない』仮説を裏付ける。)");

        File.WriteAllText(Path.Combine(OutDir, "diag_metrics.txt"), log.ToString(), new UTF8Encoding(false));
        Console.Write(log.ToString());
        Console.WriteLine($"\n時系列CSV: {OutDir} に window*.csv を出力");
        return 0;
    }

    static void WriteWindowCsv(int wi, SkirtMeasure.TurnWindow w, int s, int e,
        float[] yawRate, float[][] refTilt, float[][] physTilt, float[][] refYaw, float[][] physYaw,
        List<SkirtJoint> joints, int nj)
    {
        var sb = new StringBuilder();
        sb.AppendLine("frame,time,bodyYawRate,tilt_self_r0,tilt_ref_r0,tilt_self_r1,tilt_ref_r1,tilt_self_r2,tilt_ref_r2,yaw_self_r0,yaw_ref_r0,yaw_self_r1,yaw_ref_r1,yaw_self_r2,yaw_ref_r2");
        for (int f = s; f <= e; f++)
        {
            float[] ts = RingMax(physTilt[f], joints, nj), tr = RingMax(refTilt[f], joints, nj);
            float[] ys = RingMaxAbs(physYaw[f], joints, nj), yr = RingMaxAbs(refYaw[f], joints, nj);
            sb.AppendLine($"{f},{f / 30.0:F3},{yawRate[f]:F2}," +
                $"{ts[0]:F2},{tr[0]:F2},{ts[1]:F2},{tr[1]:F2},{ts[2]:F2},{tr[2]:F2}," +
                $"{ys[0]:F2},{yr[0]:F2},{ys[1]:F2},{yr[1]:F2},{ys[2]:F2},{yr[2]:F2}");
        }
        File.WriteAllText(Path.Combine(OutDir, $"window{wi}.csv"), sb.ToString(), new UTF8Encoding(false));
    }

    static void Metrics(StringBuilder log, int wi, SkirtMeasure.TurnWindow w, int s, int e,
        float[] yawRate, float[] physFmax, float[] refFmax, float steadyMed)
    {
        // ヨー角速度ピーク位置
        int yawPk = s; for (int f = s; f <= e; f++) if (Math.Abs(yawRate[f]) > Math.Abs(yawRate[yawPk])) yawPk = f;
        int pkS = PeakFrame(physFmax, s, e), pkR = PeakFrame(refFmax, s, e);
        int phase = CrossCorrLag(physFmax, refFmax, s, e, 30); // +なら自前が本家より遅い
        int riseS = pkS - yawPk, riseR = pkR - yawPk;
        int decS = DecayFrames(physFmax, pkS, e, steadyMed), decR = DecayFrames(refFmax, pkR, e, steadyMed);
        int ovS = Overshoots(physFmax, pkS, e, steadyMed), ovR = Overshoots(refFmax, pkR, e, steadyMed);

        log.AppendLine($"========== 窓{wi} (開始F{w.StartFrame}, ヨーpeak={w.PeakYaw:F0}°/s) ==========");
        log.AppendLine($"  傾きpeak: 自前={physFmax[pkS]:F1}°@F{pkS}  本家={refFmax[pkR]:F1}°@F{pkR}");
        log.AppendLine($"  1.位相(相互相関の最大位置, +で自前が遅い): {phase:+0;-0;0} フレーム");
        log.AppendLine($"  2.立ち上がり(ヨーpeak→傾きpeak): 自前={riseS} / 本家={riseR} フレーム");
        log.AppendLine($"  3.減衰(傾きpeak→平時中央値{steadyMed:F1}°復帰): 自前={(decS < 0 ? "未復帰" : decS.ToString())} / 本家={(decR < 0 ? "未復帰" : decR.ToString())} フレーム");
        log.AppendLine($"  4.オーバーシュート回数(peak後の再極大): 自前={ovS} / 本家={ovR}");
    }

    // ---- helpers ----
    static List<float> Flatten(float[][] a, int nj) { var r = new List<float>(a.Length * nj); foreach (var row in a) foreach (var v in row) r.Add(v); return r; }
    static float[] RingMax(float[] vals, List<SkirtJoint> joints, int nj) { var m = new float[3]; for (int j = 0; j < nj; j++) m[joints[j].Ring] = Math.Max(m[joints[j].Ring], vals[j]); return m; }
    static float[] RingMaxAbs(float[] vals, List<SkirtJoint> joints, int nj) { var m = new float[3]; for (int j = 0; j < nj; j++) m[joints[j].Ring] = Math.Max(m[joints[j].Ring], Math.Abs(vals[j])); return m; }
    static float WinMax(SkirtMeasure.TurnWindow w, float[] fm, int F) { float m = 0; for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) m = Math.Max(m, fm[f]); return m; }
    static int PeakFrame(float[] a, int s, int e) { int p = s; for (int f = s; f <= e; f++) if (a[f] > a[p]) p = f; return p; }
    static int DecayFrames(float[] a, int pk, int e, float med) { for (int f = pk; f <= e; f++) if (a[f] <= med) return f - pk; return -1; }
    static int Overshoots(float[] a, int pk, int e, float med)
    {
        int c = 0; // peak後、med超で再度極大になる回数
        for (int f = pk + 1; f < e; f++)
            if (a[f] > med && a[f] > a[f - 1] && a[f] >= a[f + 1]) c++;
        return c;
    }
    static int CrossCorrLag(float[] self, float[] refv, int s, int e, int maxLag)
    {
        double best = double.NegativeInfinity; int bestLag = 0;
        for (int lag = -maxLag; lag <= maxLag; lag++)
        {
            double dot = 0; int n = 0;
            for (int f = s; f <= e; f++)
            {
                int g = f - lag; if (g < s || g > e) continue;
                dot += self[f] * refv[g]; n++;
            }
            if (n > 0) { dot /= n; if (dot > best) { best = dot; bestLag = lag; } }
        }
        return bestLag;
    }
}
