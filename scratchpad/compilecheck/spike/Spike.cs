// ===========================================================================
// 診断(調整なし): 窓6の ring2 ピークの正体特定。
//  スパイク(1フレームで跳ねて戻る + ToEulerXYZ の y≈±90° 特異点) か、
//  連続したむち打ち(数フレームで滑らかに上下) か を波形と数値で判定する。
// 本体は不変。内部 _rows は反射で読む(読み取り専用)。
// ===========================================================================
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Reflection;
using System.Collections;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Spike
{
    const string PmxPath = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
    const int F0 = 2435, F1 = 2470; // 記録区間 (窓6 ピーク F2454 近傍)

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP] CSV/PMX 未検出"); return 0; }
        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        var skJoints = SkirtMeasure.ExtractVerticalJoints(model);
        var ring2 = skJoints.Where(j => j.Ring == 2).ToList();

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;

        // SkirtJoint -> World Joint(実体) を参照一致で対応付け。
        Joint JointOf(SkirtJoint sj) => world.Joints.First(j =>
            ReferenceEquals(j.BodyA, builder.Bodies[sj.ParentRb]) && ReferenceEquals(j.BodyB, builder.Bodies[sj.ChildRb]));
        var r2joints = ring2.Select(sj => (sj, jo: JointOf(sj))).ToList();

        // 駆動用 BoneFollow リンク (7ボーン)。
        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count
                && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));

        // reflection: Joint._rows と ConstraintRow のフィールド。
        var rowsField = typeof(Joint).GetField("_rows", BindingFlags.NonPublic | BindingFlags.Instance);
        var crType = rowsField.FieldType.GetGenericArguments()[0];
        var fAng = crType.GetField("Angular"); var fTgt = crType.GetField("TargetVel");
        var fAcc = crType.GetField("Accumulated"); var fLo = crType.GetField("LowerImpulse"); var fUp = crType.GetField("UpperImpulse");

        void Apply(int f) { foreach (var (link, bone) in driven) if (csv.TryGet(f, bone, out var bw)) link.Body.KinematicTarget = bw * link.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }
        float TiltBone(SkirtJoint sj) => SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb));

        // ウォームアップ + F1 まで駆動。
        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(1f / 30f);

        int nCols = r2joints.Count;
        var tilt = new float[F1 + 1, nCols];
        var eulY = new float[F1 + 1, nCols];
        var eulX = new float[F1 + 1, nCols];
        var eulZ = new float[F1 + 1, nCols];
        var angv = new float[F1 + 1, nCols];
        var rowInfo = new string[F1 + 1, nCols];

        for (int f = 0; f <= F1; f++)
        {
            Apply(f);
            world.StepSimulation(1f / 30f);
            if (f < F0) continue;
            for (int c = 0; c < nCols; c++)
            {
                var (sj, jo) = r2joints[c];
                tilt[f, c] = TiltBone(sj);
                var wA = jo.BodyA.WorldTransform * jo.FrameInA;
                var wB = jo.BodyB.WorldTransform * jo.FrameInB;
                var qRel = (wA.Rotation.Conjugated() * wB.Rotation).Normalized;
                var e = Joint.ToEulerXYZ(qRel); // rad
                eulX[f, c] = e.x * 180f / MathF.PI; eulY[f, c] = e.y * 180f / MathF.PI; eulZ[f, c] = e.z * 180f / MathF.PI;
                angv[f, c] = jo.BodyB.AngularVelocity.Length;
                // rows (反射)
                var rows = (IList)rowsField.GetValue(jo);
                int nl = 0, lim = 0, lockd = 0; float maxAcc = 0, maxTgt = 0;
                foreach (var r in rows)
                {
                    if (!(bool)fAng.GetValue(r)) continue; nl++;
                    float lo = (float)fLo.GetValue(r), up = (float)fUp.GetValue(r);
                    if (lo < -1e17f && up > 1e17f) lockd++; else lim++;
                    maxAcc = MathF.Max(maxAcc, MathF.Abs((float)fAcc.GetValue(r)));
                    maxTgt = MathF.Max(maxTgt, MathF.Abs((float)fTgt.GetValue(r)));
                }
                rowInfo[f, c] = $"angRows={nl}(lock{lockd}/lim{lim}) maxTgt={maxTgt:F2} maxAcc={maxAcc:F3}";
            }
        }

        // F2454 で傾き最大の ring2 列 = スパイカー。
        int pk = 2454; int col = 0; float best = -1;
        for (int c = 0; c < nCols; c++) if (tilt[pk, c] > best) { best = tilt[pk, c]; col = c; }
        var spiker = r2joints[col].sj;

        var o = new StringBuilder();
        o.AppendLine($"スパイカー ring2 列: '{spiker.JointName}' (親={spiker.ParentBone} 子={spiker.ChildBone})  F2454傾き={best:F1}°");
        o.AppendLine("frame time  tilt   eulX    eulY    eulZ   |eulY-90| childAngVel  rows");
        for (int f = F0; f <= F1; f++)
        {
            float dy = 90f - MathF.Abs(eulY[f, col]);
            o.AppendLine($"{f} {f / 30.0,6:F2} {tilt[f, col],6:F1} {eulX[f, col],7:F1} {eulY[f, col],7:F1} {eulZ[f, col],7:F1} {dy,7:F1} {angv[f, col],9:F2}   {rowInfo[f, col]}");
        }

        // ---- タスク2: 自前(A列) vs Bullet風(A.Z,B.X混合)軸 の乖離 (スパイカーJoint) ----
        // 注: Bullet 2.75 の厳密なコードは未確認(ソース無し)。依頼の式に基づく再現で比較。
        // 角度リミットの角度自体は A^-1*B の Euler で共通。差は「角インパルスを撃つ軸方向」。
        o.AppendLine("\n[タスク2] 角インパルス軸の乖離: 自前=A列, Bullet風=(A.Z,B.X)から直交化。角度差(度)");
        o.AppendLine("frame  tilt   axis0差  axis1差  axis2差  (大きいほど撃つ方向がずれる)");
        // 再走ではなくフレーム毎に再構成できないため、記録済み姿勢を使えないので簡易に再実行する。
        var b2 = PmxPhysicsBuilder.Build(model); var w2 = b2.World;
        var joSp = w2.Joints.First(j => ReferenceEquals(j.BodyA, b2.Bodies[spiker.ParentRb]) && ReferenceEquals(j.BodyB, b2.Bodies[spiker.ChildRb]));
        var driven2 = new List<(BoneLink link, string bone)>();
        foreach (var link in b2.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven2.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply2(int f) { foreach (var (link, bone) in driven2) if (csv.TryGet(f, bone, out var bw)) link.Body.KinematicTarget = bw * link.BodyOffsetFromBone; }
        Apply2(0); for (int s = 0; s < 60; s++) w2.StepSimulation(1f / 30f);
        for (int f = 0; f <= F1; f++)
        {
            Apply2(f); w2.StepSimulation(1f / 30f);
            if (f < F0) continue;
            var wA = joSp.BodyA.WorldTransform * joSp.FrameInA;
            var wB = joSp.BodyB.WorldTransform * joSp.FrameInB;
            var bA = Matrix3x3.FromQuat(wA.Rotation); var bB = Matrix3x3.FromQuat(wB.Rotation);
            var selfA = new[] { bA.Column(0), bA.Column(1), bA.Column(2) };
            var axis0 = bB.Column(0); var axis2 = bA.Column(2);
            var ax1 = Vec3.Cross(axis2, axis0).Normalized;
            var ax0 = Vec3.Cross(ax1, axis2).Normalized;
            var ax2 = Vec3.Cross(axis0, ax1).Normalized;
            var bull = new[] { ax0, ax1, ax2 };
            float[] diff = new float[3];
            for (int i = 0; i < 3; i++)
                diff[i] = (float)(Math.Acos(Math.Clamp(selfA[i].Dot(bull[i]), -1, 1)) * 180.0 / Math.PI);
            o.AppendLine($"{f} {tilt[f, col],6:F1} {diff[0],8:F1} {diff[1],8:F1} {diff[2],8:F1}");
        }
        // 参考: 全ring2列の傾きのF2452-2456
        o.AppendLine("\n[参考] 全ring2列 傾き F2452-2456 (どの列がいつ跳ねるか):");
        o.Append("frame ");
        for (int c = 0; c < nCols; c++) o.Append($"c{r2joints[c].sj.Col,-2} ");
        o.AppendLine();
        for (int f = 2452; f <= 2456; f++)
        {
            o.Append($"{f}  ");
            for (int c = 0; c < nCols; c++) o.Append($"{tilt[f, c],4:F0} ");
            o.AppendLine();
        }

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "spike_out.txt"), o.ToString(), new UTF8Encoding(false));
        Console.Write(o.ToString());
        return 0;
    }
}
