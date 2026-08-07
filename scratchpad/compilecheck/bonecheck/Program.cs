// タスク3 検証先行: 本家参照値(CSVのスカートボーン姿勢)だけを自前の計測コードで算出し、
// Python側の値 (中央値11.39°, p90 25.66°, ring別, ターン12窓, 671.7°/s, 窓1→57.3°, 窓4→59.1°)
// を再現できるか確認する。物理はまだ回さない。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace BoneCheck
{
    static class Program
    {
        const string PmxPath = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
        static StringBuilder O = new();
        static void L(string s) { O.AppendLine(s); }

        static int Main()
        {
            string csvPath = BoneCsv.FindPath();
            if (csvPath == null || !File.Exists(PmxPath))
            {
                Console.WriteLine("[SKIP] CSV か PMX 未検出。pass 扱い。");
                return 0;
            }

            var csv = BoneCsv.Load(csvPath);
            var model = PmxReader.LoadFile(PmxPath);
            var joints = SkirtMeasure.ExtractVerticalJoints(model);

            L($"縦スカートJoint数={joints.Count}  ring別=" +
                string.Join(", ", joints.GroupBy(j => j.Ring).OrderBy(g => g.Key).Select(g => $"ring{g.Key}×{g.Count()}")));

            // 参照値が CSV に揃っているか確認。
            var missing = new List<string>();
            foreach (var j in joints)
            {
                if (!csv.HasBone(j.ParentBone)) missing.Add(j.ParentBone);
                if (!csv.HasBone(j.ChildBone)) missing.Add(j.ChildBone);
            }
            L($"CSV欠損ボーン(縦Joint親子): {(missing.Count == 0 ? "なし" : string.Join(",", missing.Distinct()))}");

            int F = csv.FrameCount;
            float dt = 1f / 30f;

            // --- 全フレーム×全Joint の本家傾き (スイングのみ) ---
            var perRing = new Dictionary<int, List<float>> { { 0, new() }, { 1, new() }, { 2, new() } };
            var all = new List<float>(joints.Count * F);
            // フレーム毎の「全Joint最大傾き」(窓の傾きmax用)。
            var frameMaxTilt = new float[F];

            for (int f = 0; f < F; f++)
            {
                float fmax = 0f;
                foreach (var j in joints)
                {
                    if (!csv.TryGet(f, j.ParentBone, out var p) || !csv.TryGet(f, j.ChildBone, out var c)) continue;
                    float t = SkirtMeasure.TiltDeg(p.Rotation, c.Rotation);
                    all.Add(t);
                    perRing[j.Ring].Add(t);
                    if (t > fmax) fmax = t;
                }
                frameMaxTilt[f] = fmax;
            }

            // --- 下半身 ヨー角速度 & ターン窓 ---
            var yaw = new float[F];
            for (int f = 1; f < F; f++)
            {
                if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1))
                    yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, dt);
            }
            var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
            float maxYaw = yaw.Length == 0 ? 0 : yaw.Max(v => Math.Abs(v));

            // --- 平時統計 (全フレーム & ターン窓除外の両方) ---
            var inWindow = new bool[F];
            foreach (var w in wins) for (int f = w.StartFrame; f <= w.EndFrame; f++) inWindow[f] = true;

            L("\n== 傾き統計 (本家参照, 全フレーム) med/p90/max ==");
            PrintStats("全体", all);
            foreach (var r in new[] { 0, 1, 2 }) PrintStats($"ring{r}", perRing[r]);

            // ターン窓除外版 (frameMaxTilt を使うのではなく、各Jointの各フレーム値を窓除外で集計)
            var allCalm = new List<float>();
            var ringCalm = new Dictionary<int, List<float>> { { 0, new() }, { 1, new() }, { 2, new() } };
            for (int f = 0; f < F; f++)
            {
                if (inWindow[f]) continue;
                foreach (var j in joints)
                {
                    if (!csv.TryGet(f, j.ParentBone, out var p) || !csv.TryGet(f, j.ChildBone, out var c)) continue;
                    float t = SkirtMeasure.TiltDeg(p.Rotation, c.Rotation);
                    allCalm.Add(t); ringCalm[j.Ring].Add(t);
                }
            }
            L("\n== 傾き統計 (本家参照, ターン窓除外=平時) med/p90/max ==");
            PrintStats("全体", allCalm);
            foreach (var r in new[] { 0, 1, 2 }) PrintStats($"ring{r}", ringCalm[r]);

            // --- ターン窓一覧 ---
            L($"\n== ターン窓 (|ヨー角速度|>360°/s): {wins.Count}窓  ヨー最大={maxYaw:F1}°/s ==");
            L("  #  開始F   時刻s   ヨーpeak°/s  傾きmax(窓内)  傾きmax(窓+30F)");
            for (int i = 0; i < wins.Count; i++)
            {
                var w = wins[i];
                float tmW = 0, tmE = 0;
                for (int f = w.StartFrame; f <= w.EndFrame; f++) tmW = Math.Max(tmW, frameMaxTilt[f]);
                for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) tmE = Math.Max(tmE, frameMaxTilt[f]);
                L($"  {i + 1,2}  {w.StartFrame,5}  {w.StartFrame / 30.0,6:F2}  {w.PeakYaw,9:F1}   {tmW,10:F1}   {tmE,10:F1}");
            }

            // --- Python 値との対決 ---
            L("\n== Python 参照値との対決 ==");
            var s = SkirtMeasure.Stats(all); var sc = SkirtMeasure.Stats(allCalm);
            L($"  中央値: 全フレーム={s.med:F2}  窓除外={sc.med:F2}   (Python=11.39)");
            L($"  p90   : 全フレーム={s.p90:F2}  窓除外={sc.p90:F2}   (Python=25.66)");
            L($"  ring別中央値(窓除外): ring0={SkirtMeasure.Stats(ringCalm[0]).med:F2} ring1={SkirtMeasure.Stats(ringCalm[1]).med:F2} ring2={SkirtMeasure.Stats(ringCalm[2]).med:F2}  (Python 10.06/12.78/11.81)");
            L($"  ターン窓数={wins.Count} (Python=12)   ヨー最大={maxYaw:F1} (Python=671.7)");
            var w1 = wins.FirstOrDefault(w => w.StartFrame >= 840 && w.StartFrame <= 855);
            var w4 = wins.FirstOrDefault(w => w.StartFrame >= 1955 && w.StartFrame <= 1970);
            L($"  窓@~847: 開始F={w1.StartFrame} 傾きmax(窓+30F)={WinTiltMax(w1, frameMaxTilt, F):F1} (Python 57.3)");
            L($"  窓@~1962: 開始F={w4.StartFrame} 傾きmax(窓+30F)={WinTiltMax(w4, frameMaxTilt, F):F1} (Python 59.1)");

            File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "verify_out.txt"), O.ToString(), new UTF8Encoding(false));
            Console.Write(O.ToString());
            return 0;
        }

        static float WinTiltMax(SkirtMeasure.TurnWindow w, float[] frameMaxTilt, int F)
        {
            float m = 0;
            for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) m = Math.Max(m, frameMaxTilt[f]);
            return m;
        }

        static void PrintStats(string label, IEnumerable<float> vals)
        {
            var st = SkirtMeasure.Stats(vals);
            L($"  {label,-6}: med={st.med,6:F2}  p90={st.p90,6:F2}  max={st.max,6:F2}");
        }
    }
}
