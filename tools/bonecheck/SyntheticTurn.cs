// ===========================================================================
// タスク4: 合成ターンのフォールバック。
// CSV が無い環境でもパイプラインを回すための「環境確認用」モード。
// 下半身を 180°/0.3秒 で回し、1.5秒休止、方向を交互に4回。他はバインド姿勢のまま。
// 同じ計測(スカート傾き)を適用する。
// **これは環境確認専用であり、MMDとの比較には一切使わない** (合成入力なので正解値が無い)。
// ===========================================================================
using System;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace BoneCheck
{
    public static class SyntheticTurn
    {
        // 環境確認用に固定した合成ターンの角度スケジュール(度, 世界Y軸まわり)。
        static float[] BuildSchedule(out int frames)
        {
            int turnF = 9;     // 0.3s @30Hz
            int restF = 45;    // 1.5s @30Hz
            int reps = 4;
            frames = reps * (turnF + restF);
            var ang = new float[frames];
            float cur = 0f; int idx = 0;
            for (int k = 0; k < reps; k++)
            {
                float dir = (k % 2 == 0) ? +180f : -180f;
                for (int i = 0; i < turnF; i++) { cur += dir / turnF; ang[idx++] = cur; }
                for (int i = 0; i < restF; i++) ang[idx++] = cur;
            }
            return ang;
        }

        public static bool Run(PmxPhysicsModel model)
        {
            var joints = SkirtMeasure.ExtractVerticalJoints(model);
            var builder = PmxPhysicsBuilder.Build(model);
            var world = builder.World;

            int lowerIdx = model.BoneNames.IndexOf("下半身");
            if (lowerIdx < 0) { Console.WriteLine("[SKIP] 下半身ボーンが無い。合成モード不可。"); return true; }
            var lowerPos = model.BonePositions[lowerIdx];

            var lowerLinks = builder.BoneLinks
                .Where(l => l.Mode == PhysicsMode.BoneFollow && l.BoneIndex == lowerIdx).ToList();

            var sched = BuildSchedule(out int F);

            // ウォームアップ (角度0)。
            SetLower(lowerLinks, lowerPos, 0f);
            for (int s = 0; s < 60; s++) world.StepSimulation(1f / 30f);

            var all = new List<float>();
            var ring = new[] { new List<float>(), new List<float>(), new List<float>() };
            bool nan = false; float maxSpeed = 0;
            for (int f = 0; f < F; f++)
            {
                SetLower(lowerLinks, lowerPos, sched[f]);
                world.StepSimulation(1f / 30f);
                for (int j = 0; j < joints.Count; j++)
                {
                    var pj = joints[j];
                    float t = SkirtMeasure.TiltDeg(builder.Bodies[pj.ParentRb].WorldTransform.Rotation,
                                                    builder.Bodies[pj.ChildRb].WorldTransform.Rotation);
                    if (float.IsNaN(t) || float.IsInfinity(t)) nan = true;
                    all.Add(t); ring[pj.Ring].Add(t);
                }
                foreach (var rb in builder.Bodies) { float sp = rb.LinearVelocity.Length; if (sp > maxSpeed) maxSpeed = sp; }
            }

            var s0 = SkirtMeasure.Stats(all);
            Console.WriteLine("== 合成ターン (環境確認用・MMD比較不可) ==");
            Console.WriteLine($"  フレーム数={F} 下半身180°/0.3s×交互4回  NaN/Inf={nan}  maxSpeed={maxSpeed:F1}");
            Console.WriteLine($"  傾き 全体 med={s0.med:F1} p90={s0.p90:F1} max={s0.max:F1}");
            for (int r = 0; r < 3; r++)
            {
                var sr = SkirtMeasure.Stats(ring[r]);
                Console.WriteLine($"  傾き ring{r} med={sr.med:F1} p90={sr.p90:F1} max={sr.max:F1}");
            }
            bool ok = !nan && maxSpeed < 500f;
            Console.WriteLine(ok ? "  [PASS] パイプライン健全 (NaNなし/爆発なし)" : "  [FAIL] NaN か爆発");
            return ok;
        }

        static void SetLower(List<BoneLink> links, Vec3 lowerPos, float angleDeg)
        {
            var rot = Quat.FromAxisAngle(Vec3.YAxis, angleDeg * (float)Math.PI / 180f);
            var boneWorld = new RigidTransform(rot, lowerPos);
            foreach (var l in links)
                l.Body.KinematicTarget = boneWorld * l.BodyOffsetFromBone;
        }
    }
}
