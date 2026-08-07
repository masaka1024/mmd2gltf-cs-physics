// 診断(調整なし): スカートJointの並進拘束の飽和(MaxCorrectionVel)とジョイントの伸びを測る。
// 並進はロック(lo=hi=0)。補正速度 = |stretch·axis|*Beta*invDt。MaxCorrectionVel=10 に張り付くか。
// 伸び = |anchorB - anchorA| (本来0)。本家は CSVボーン*offset で剛体を復元して並記。
// 本体は不変(すべて public 状態から計算)。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Stretch
{
    const string PmxPath = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
    const float DT = 1f / 30f;
    const float Beta = 0.2f;       // 本体 Joint.Beta 既定と同値
    const float InvDt = 30f;
    const float MaxCorr = 10f;     // 本体 Joint.MaxCorrectionVel

    static StringBuilder O = new(); static void L(string s = "") => O.AppendLine(s);

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP]"); return 0; }
        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        var joints = SkirtMeasure.ExtractVerticalJoints(model);
        int F = csv.FrameCount;

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        Joint JOf(SkirtJoint sj) => world.Joints.First(j =>
            ReferenceEquals(j.BodyA, builder.Bodies[sj.ParentRb]) && ReferenceEquals(j.BodyB, builder.Bodies[sj.ChildRb]));
        var jm = joints.Select(sj => (sj, jo: JOf(sj), offA: builder.BoneLinks[sj.ParentRb].BodyOffsetFromBone, offB: builder.BoneLinks[sj.ChildRb].BodyOffsetFromBone)).ToList();

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        // ターン窓
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, DT);
        var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        var inWin = new bool[F];
        foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(DT);

        int nj = jm.Count;
        // 集計: 窓内/窓外
        var rawWin = new List<float>(); var rawCalm = new List<float>();  // クランプ前の生補正速度|raw|
        var strWin = new List<float>(); var strCalm = new List<float>();  // 自前 伸び
        var refStrWin = new List<float>(); var refStrCalm = new List<float>();
        long clampWin = 0, clampCalm = 0, axisWin = 0, axisCalm = 0;      // クランプ張り付き(axis)数/総axis数
        var perJointClamp = new long[nj]; var perJointMaxTilt = new float[nj];
        // F2454 上流クランプ確認
        string f2454up = "";

        for (int f = 0; f < F; f++)
        {
            Apply(f);
            world.StepSimulation(DT);
            bool win = inWin[f];
            for (int c = 0; c < nj; c++)
            {
                var (sj, jo, offA, offB) = jm[c];
                var wA = jo.BodyA.WorldTransform * jo.FrameInA;
                var wB = jo.BodyB.WorldTransform * jo.FrameInB;
                var lin = wB.Origin - wA.Origin;
                float stretch = lin.Length;
                // tilt (相関用)
                float tilt = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb));
                if (tilt > perJointMaxTilt[c]) perJointMaxTilt[c] = tilt;
                // 並進ロック3軸の生補正速度とクランプ
                var basisA = Matrix3x3.FromQuat(wA.Rotation);
                for (int ax = 0; ax < 3; ax++)
                {
                    // ロック軸のみ (skirt は全ロック)
                    if (sj_lin_free(jo, ax)) continue;
                    var axis = basisA.Column(ax);
                    float cur = lin.Dot(axis);
                    float raw = Math.Abs(cur) * Beta * InvDt; // |(-cur)*Beta*invDt|
                    bool hit = raw >= MaxCorr;
                    if (win) { axisWin++; if (hit) { clampWin++; perJointClamp[c]++; } rawWin.Add(raw); }
                    else { axisCalm++; if (hit) { clampCalm++; perJointClamp[c]++; } rawCalm.Add(raw); }
                }
                // 本家 伸び (CSVボーン*offset で剛体復元)
                float refStretch = float.NaN;
                if (csv.TryGet(f, sj.ParentBone, out var pb) && csv.TryGet(f, sj.ChildBone, out var cb))
                {
                    var refA = (pb * offA) * jo.FrameInA;
                    var refB = (cb * offB) * jo.FrameInB;
                    refStretch = (refB.Origin - refA.Origin).Length;
                }
                if (win) { strWin.Add(stretch); if (!float.IsNaN(refStretch)) refStrWin.Add(refStretch); }
                else { strCalm.Add(stretch); if (!float.IsNaN(refStretch)) refStrCalm.Add(refStretch); }

                if (f == 2454 && sj.Ring <= 1) // 上流 (ring0,ring1)
                {
                    float mraw = 0; var b2 = Matrix3x3.FromQuat(wA.Rotation);
                    for (int ax = 0; ax < 3; ax++) { float cur = lin.Dot(b2.Column(ax)); mraw = Math.Max(mraw, Math.Abs(cur) * Beta * InvDt); }
                    f2454up += $"[{sj.JointName} 伸び={stretch:F2} 最大生補正速度={mraw:F1}{(mraw >= MaxCorr ? "★飽和" : "")}] ";
                }
            }
        }

        float Mx(List<float> a) => a.Count > 0 ? a.Max() : 0;
        float Pc(List<float> a, double p) { if (a.Count == 0) return 0; a.Sort(); return SkirtMeasure.Percentile(a.ToArray(), p); }

        L("========== 1) 並進ロック行のクランプ(MaxCorrectionVel=10)発動 ==========");
        L($"  窓内: クランプ張り付き={clampWin}/{axisWin} ({100.0 * clampWin / Math.Max(1, axisWin):F2}%)  生補正速度 max={Mx(rawWin):F1} p99={Pc(rawWin, 99):F1} p90={Pc(rawWin, 90):F1}");
        L($"  平時: クランプ張り付き={clampCalm}/{axisCalm} ({100.0 * clampCalm / Math.Max(1, axisCalm):F2}%)  生補正速度 max={Mx(rawCalm):F1} p99={Pc(rawCalm, 99):F1} p90={Pc(rawCalm, 90):F1}");
        L("  (張り付き条件: |stretch·axis|*6 >= 10, つまり伸び成分 >= 1.67 unit)");

        L("\n========== 2) ジョイントの伸び |anchorB-anchorA| (本来0) ==========");
        L($"  窓内: 自前 max={Mx(strWin):F2} p90={Pc(strWin, 90):F2} 中央={Pc(strWin, 50):F2} | 本家 max={Mx(refStrWin):F2} p90={Pc(refStrWin, 90):F2} 中央={Pc(refStrWin, 50):F2}");
        L($"  平時: 自前 max={Mx(strCalm):F2} p90={Pc(strCalm, 90):F2} 中央={Pc(strCalm, 50):F2} | 本家 max={Mx(refStrCalm):F2} p90={Pc(refStrCalm, 90):F2} 中央={Pc(refStrCalm, 50):F2}");

        L("\n========== 3) 相関: クランプ多いJoint vs 傾き過大Joint ==========");
        var order = Enumerable.Range(0, nj).OrderByDescending(c => perJointClamp[c]).Take(6);
        L("  クランプ発動 上位6 Joint (clamp回数, 最大傾き°):");
        foreach (var c in order) L($"    {jm[c].sj.JointName}: clamp={perJointClamp[c]} maxTilt={perJointMaxTilt[c]:F1}");
        var order2 = Enumerable.Range(0, nj).OrderByDescending(c => perJointMaxTilt[c]).Take(6);
        L("  最大傾き 上位6 Joint (最大傾き°, clamp回数):");
        foreach (var c in order2) L($"    {jm[c].sj.JointName}: maxTilt={perJointMaxTilt[c]:F1} clamp={perJointClamp[c]}");
        L($"\n  F2454 上流(ring0/1)並進: {f2454up}");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "stretch_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    static bool sj_lin_free(Joint j, int ax) => j.LinearLowerLimit[ax] > j.LinearUpperLimit[ax];
}
