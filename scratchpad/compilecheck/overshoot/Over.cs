// 診断: 縦スカートJointの角度リミット超過量(主にX ±80°)の統計を全フレームで測る。
// 各ジョイントの Euler(rigid frame) が lo/hi をどれだけ超えたか。max/p90/超過フレーム数。
// 本体は不変。ウォームスタート有無の効果を、この指標で比較する。
using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Over
{
    static readonly string PmxPath = TestData.PmxPath();

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP]"); return 0; }
        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        var skJoints = SkirtMeasure.ExtractVerticalJoints(model);

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        Joint JointOf(SkirtJoint sj) => world.Joints.First(j =>
            ReferenceEquals(j.BodyA, builder.Bodies[sj.ParentRb]) && ReferenceEquals(j.BodyB, builder.Bodies[sj.ChildRb]));
        var jmap = skJoints.Select(sj => (sj, jo: JointOf(sj))).ToList();

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(1f / 30f);

        int F = csv.FrameCount;
        var overX = new List<float>();          // X軸リミット超過量(度), >0のみ
        var overAny = new List<float>();         // 全3軸のうち最大超過(度)
        int frameExceedX = 0;                    // Xで超過した (frame×joint) 数
        float R2D = 180f / MathF.PI;

        for (int f = 0; f < F; f++)
        {
            Apply(f);
            world.StepSimulation(1f / 30f);
            foreach (var (sj, jo) in jmap)
            {
                var wA = jo.BodyA.WorldTransform * jo.FrameInA;
                var wB = jo.BodyB.WorldTransform * jo.FrameInB;
                var qRel = (wA.Rotation.Conjugated() * wB.Rotation).Normalized;
                var e = Joint.ToEulerXYZ(qRel); // rad
                float[] eu = { e.x, e.y, e.z };
                float amax = 0;
                for (int ax = 0; ax < 3; ax++)
                {
                    float lo = jo.AngularLowerLimit[ax], hi = jo.AngularUpperLimit[ax];
                    if (lo > hi) continue; // free
                    float ov = MathF.Max(0f, MathF.Max(eu[ax] - hi, lo - eu[ax]));
                    if (ov > amax) amax = ov;
                    if (ax == 0 && ov > 1e-4f) { overX.Add(ov * R2D); frameExceedX++; }
                }
                if (amax > 1e-4f) overAny.Add(amax * R2D);
            }
        }

        float Pct(List<float> a, double p) { if (a.Count == 0) return 0; a.Sort(); return SkirtMeasure.Percentile(a.ToArray(), p); }
        Console.WriteLine($"X角度リミット(±80°)超過: 最大={ (overX.Count>0?overX.Max():0):F1}° p90={Pct(overX,90):F1}° 超過(frame×joint)数={frameExceedX}");
        Console.WriteLine($"全軸リミット超過(最大値): 最大={ (overAny.Count>0?overAny.Max():0):F1}° p90={Pct(overAny,90):F1}° 件数={overAny.Count}");
        return 0;
    }
}
