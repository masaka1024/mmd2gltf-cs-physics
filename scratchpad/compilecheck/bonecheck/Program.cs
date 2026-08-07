// タスク2+3: 本家(CSV)と自作物理(ヘッドレス再生)を同じ物差しで比較する。
// まず本家参照値の再現を検証し、続いて物理を7001フレーム回して並記する。
// 数字を合わせにいく調整は行わない。合っていなくてもそのまま出す。
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
        static void L(string s = "") { O.AppendLine(s); }

        static int Main()
        {
            string csvPath = BoneCsv.FindPath();
            if (csvPath == null || !File.Exists(PmxPath))
            {
                Console.WriteLine("[SKIP] CSV か PMX 未検出。合成モードは SyntheticTurn (タスク4) を参照。pass 扱い。");
                return 0;
            }

            var csv = BoneCsv.Load(csvPath);
            var model = PmxReader.LoadFile(PmxPath);
            var joints = SkirtMeasure.ExtractVerticalJoints(model);
            int F = csv.FrameCount, nj = joints.Count;
            float dt = 1f / 30f;

            // ---- 本家参照 傾き & rel-yaw ----
            var refTilt = new float[F][];
            var refRelYaw = new float[F][];
            var refFrameMax = new float[F];
            for (int f = 0; f < F; f++)
            {
                refTilt[f] = new float[nj]; refRelYaw[f] = new float[nj];
                float fmax = 0;
                for (int j = 0; j < nj; j++)
                {
                    var pj = joints[j];
                    if (csv.TryGet(f, pj.ParentBone, out var p) && csv.TryGet(f, pj.ChildBone, out var c))
                    {
                        refTilt[f][j] = SkirtMeasure.TiltDeg(p.Rotation, c.Rotation);
                        refRelYaw[f][j] = SkirtMeasure.YawOfRelDeg(p.Rotation, c.Rotation);
                        if (refTilt[f][j] > fmax) fmax = refTilt[f][j];
                    }
                }
                refFrameMax[f] = fmax;
            }

            // ---- 下半身 ヨー & ターン窓 (両者共通) ----
            var yaw = new float[F];
            for (int f = 1; f < F; f++)
                if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1))
                    yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, dt);
            var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
            float maxYaw = yaw.Max(v => Math.Abs(v));

            // ---- 検証: 本家参照値が Python と一致するか (物理へ進む前の門番) ----
            L("========== 検証: 本家参照値の再現 (Python対決) ==========");
            var refAll = Flatten(refTilt);
            var refRing = RingSplit(refTilt, joints);
            var sAll = SkirtMeasure.Stats(refAll);
            L($"  中央値={sAll.med:F2}(P11.39)  p90={sAll.p90:F2}(P25.66)");
            L($"  ring別中央値: ring0={SkirtMeasure.Stats(refRing[0]).med:F2}(P10.06) ring1={SkirtMeasure.Stats(refRing[1]).med:F2}(P12.78) ring2={SkirtMeasure.Stats(refRing[2]).med:F2}(P11.81)");
            L($"  ターン窓={wins.Count}(P12)  ヨー最大={maxYaw:F1}(P671.7)");
            L($"  窓@~847 傾きmax={WinMax(FindWin(wins, 847), refFrameMax, F):F1}(P57.3)  窓@~1962 傾きmax={WinMax(FindWin(wins, 1962), refFrameMax, F):F1}(P59.1)");
            bool gateOk = Math.Abs(sAll.med - 11.39f) < 0.2f && wins.Count == 12;
            L($"  => 物差し健全性: {(gateOk ? "OK (物理比較へ進む)" : "NG (要調査)")}");

            // ---- 物理を回す ----
            var drv = new HeadlessDriver();
            drv.Run(csv, model, joints);
            var physTilt = drv.PhysTilt; var physRelYaw = drv.PhysRelYaw; var physFrameMax = drv.PhysFrameMaxTilt;

            L();
            L("========== 入出力ボーン ==========");
            int skirtBones = joints.Select(j => j.ChildBone).Distinct().Count();
            L($"  入力(駆動)ユニークボーン数={drv.InputBoneCount}  参照スカートボーン数={skirtBones}");
            L($"  BoneFollowだがCSV欠損: {(drv.MissingDrivenBones.Count == 0 ? "なし" : string.Join(",", drv.MissingDrivenBones))}");
            L($"  ウォームアップ={drv.WarmupSteps}step  7001フレーム再生時間={drv.RunSeconds:F1}s");

            // ---- 1) 平時統計 (全フレーム, 自前 vs 本家) ----
            L();
            L("========== 1) 平時統計 (傾き, 全フレーム)  自前物理 / 本家 ==========");
            L("  対象  |  med(自/本)  |  p90(自/本)  |  max(自/本)");
            PrintPair("全体", Flatten(physTilt), refAll);
            var physRing = RingSplit(physTilt, joints);
            for (int r = 0; r < 3; r++) PrintPair($"ring{r}", physRing[r], refRing[r]);

            // ---- 2) ターンイベント (窓ごと 自前/本家) ----
            L();
            L("========== 2) ターンイベント (瞬間ヨー角速度>360°/s の窓) ==========");
            L("  #  開始F  時刻s  ヨーpeak  傾きmax:自前  傾きmax:本家 (窓+30F)");
            for (int i = 0; i < wins.Count; i++)
            {
                var w = wins[i];
                float mp = WinMax(w, physFrameMax, F), mr = WinMax(w, refFrameMax, F);
                L($"  {i + 1,2}  {w.StartFrame,5} {w.StartFrame / 30.0,6:F2} {w.PeakYaw,8:F1}   {mp,10:F1}   {mr,10:F1}");
            }

            // ---- 3) 窓1・窓4 対決 ----
            L();
            L("========== 3) 窓1・窓4 の本家対決 ==========");
            var W1 = FindWin(wins, 847); var W4 = FindWin(wins, 1962);
            L($"  窓1(開始~847): 自前={WinMax(W1, physFrameMax, F):F1}  本家={WinMax(W1, refFrameMax, F):F1}  (Python本家=57.3)");
            L($"  窓4(開始~1962): 自前={WinMax(W4, physFrameMax, F):F1}  本家={WinMax(W4, refFrameMax, F):F1}  (Python本家=59.1)");

            // ---- 4) ヨー遅れ (取付相対ヨー, ターン窓中の最大|ヨー|) ----
            L();
            L("========== 4) ヨー遅れ (取付相対ヨー角, ターン窓中の最大|deg|) ==========");
            L("  本家はターン中もヨー遅れ1〜3°の完全共回転。大きければ共回転できていない。");
            float refYawLag = MaxRelYawInWindows(refRelYaw, wins, F);
            float physYawLag = MaxRelYawInWindows(physRelYaw, wins, F);
            L($"  ターン窓中の最大|取付相対ヨー|: 自前={physYawLag:F1}°  本家={refYawLag:F1}°");

            File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "compare_out.txt"), O.ToString(), new UTF8Encoding(false));
            Console.Write(O.ToString());
            return 0;
        }

        // ---- helpers ----
        static List<float> Flatten(float[][] a)
        {
            var r = new List<float>(a.Length * (a.Length > 0 ? a[0].Length : 0));
            foreach (var row in a) foreach (var v in row) r.Add(v);
            return r;
        }
        static List<float>[] RingSplit(float[][] tilt, List<SkirtJoint> joints)
        {
            var r = new[] { new List<float>(), new List<float>(), new List<float>() };
            for (int f = 0; f < tilt.Length; f++)
                for (int j = 0; j < joints.Count; j++)
                    r[joints[j].Ring].Add(tilt[f][j]);
            return r;
        }
        static void PrintPair(string label, IEnumerable<float> mine, IEnumerable<float> theirs)
        {
            var a = SkirtMeasure.Stats(mine); var b = SkirtMeasure.Stats(theirs);
            L($"  {label,-6}| {a.med,6:F2}/{b.med,6:F2} | {a.p90,6:F2}/{b.p90,6:F2} | {a.max,6:F1}/{b.max,6:F1}");
        }
        static SkirtMeasure.TurnWindow FindWin(List<SkirtMeasure.TurnWindow> wins, int approxStart)
            => wins.OrderBy(w => Math.Abs(w.StartFrame - approxStart)).First();
        static float WinMax(SkirtMeasure.TurnWindow w, float[] frameMax, int F)
        {
            float m = 0;
            for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) m = Math.Max(m, frameMax[f]);
            return m;
        }
        static float MaxRelYawInWindows(float[][] relYaw, List<SkirtMeasure.TurnWindow> wins, int F)
        {
            float m = 0;
            foreach (var w in wins)
                for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++)
                    foreach (var v in relYaw[f]) m = Math.Max(m, Math.Abs(v));
            return m;
        }
    }
}
