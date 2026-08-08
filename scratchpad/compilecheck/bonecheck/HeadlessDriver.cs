// ===========================================================================
// ヘッドレス再生ドライバ (タスク2)。
// CSVの入力ボーン(7本)を BoneFollow 剛体に与え、スカート(dynamic)を物理で動かす。
// 座標はPMXネイティブなので変換・スケールは不要。既定=30Hz・1サブ。
// MmdPhysicsBehaviour.PushBonesToKinematic と同一ロジック:
//   KinematicTarget = boneWorld * BodyOffsetFromBone
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace BoneCheck
{
    public sealed class HeadlessDriver
    {
        // ウォームアップ既定: スカートがバインド姿勢からフレーム0の平衡へ沈み込むのに約1-2秒かかる。
        // 30Hzで 60ステップ(=2秒)を既定とする(計測開始時の初期過渡を除くため)。
        public int WarmupSteps = 60;

        // 診断用: >0 のとき World.SolverIterations を上書き (既定-1=本体既定10のまま)。
        public int SolverIterationsOverride = -1;

        public readonly List<string> MissingDrivenBones = new(); // BoneFollowだがCSVに無いボーン

        public int InputBoneCount { get; private set; }   // 実際に駆動できたユニーク入力ボーン数
        public double RunSeconds { get; private set; }

        // 出力: [frame][jointIdx]。
        // PhysTilt/PhysRelYaw は「ボーン空間」(剛体からボーン姿勢を復元して測る=本家と同じ物理量)。
        // PhysTiltRigid は旧方式「剛体相対」(検算用: ring1/2 で新旧一致するはず)。
        public float[][] PhysTilt;
        public float[][] PhysTiltRigid;
        public float[][] PhysRelYaw;
        public float[] PhysFrameMaxTilt;

        public void Run(BoneCsv csv, PmxPhysicsModel model, List<SkirtJoint> joints)
        {
            var builder = PmxPhysicsBuilder.Build(model); // 既定 World (30Hz・1サブ, gravity -98)
            var world = builder.World;
            if (SolverIterationsOverride > 0) world.SolverIterations = SolverIterationsOverride; // 診断用
            // 計測トグル (既定OFF=挙動不変)。VMD統計を OFF/ON で本家比較するため env で切替。
            if (System.Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
            if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;

            // BoneFollow リンク: bone名 → CSV姿勢で駆動。CSVに無いものは記録して据え置き。
            var driven = new List<(BoneLink link, string bone)>();
            var uniqueBones = new HashSet<string>();
            foreach (var link in builder.BoneLinks)
            {
                if (link.Mode != PhysicsMode.BoneFollow) continue;
                string bone = (link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count)
                    ? model.BoneNames[link.BoneIndex] : null;
                if (bone == null || !csv.HasBone(bone))
                {
                    if (bone != null) MissingDrivenBones.Add(bone);
                    continue;
                }
                driven.Add((link, bone));
                uniqueBones.Add(bone);
            }
            InputBoneCount = uniqueBones.Count;
            MissingDrivenBones.Sort();

            int F = csv.FrameCount, nj = joints.Count;
            PhysTilt = new float[F][];
            PhysTiltRigid = new float[F][];
            PhysRelYaw = new float[F][];
            PhysFrameMaxTilt = new float[F];

            var sw = System.Diagnostics.Stopwatch.StartNew();

            // 物理開始前に FK-rest で全剛体をボーン姿勢へ整合させる (MMDの物理リセット相当)。
            // 物理ボーン(スカート等)のCSV姿勢はFKヘルパが無視し親から前計算する。
            ApplyPose(driven, csv, 0);
            builder.ResetBodiesToBonePoseFk(i =>
                (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw))
                    ? (RigidTransform?)bw : null);

            // ウォームアップ: フレーム0姿勢で空回し。
            for (int s = 0; s < WarmupSteps; s++) world.StepSimulation(1f / 30f);

            // 計測本番。
            for (int f = 0; f < F; f++)
            {
                ApplyPose(driven, csv, f);
                world.StepSimulation(1f / 30f);

                var tilt = new float[nj];
                var tiltRigid = new float[nj];
                var relyaw = new float[nj];
                float fmax = 0f;
                for (int j = 0; j < nj; j++)
                {
                    var pj = joints[j];
                    // 剛体相対 (旧方式・検算用)。
                    var prRb = builder.Bodies[pj.ParentRb].WorldTransform.Rotation;
                    var crRb = builder.Bodies[pj.ChildRb].WorldTransform.Rotation;
                    tiltRigid[j] = SkirtMeasure.TiltDeg(prRb, crRb);
                    // ボーン空間 (剛体からボーン姿勢を復元。PullPhysicsToBones と同一式)。
                    var pr = RecoverBoneRot(builder, pj.ParentRb);
                    var cr = RecoverBoneRot(builder, pj.ChildRb);
                    tilt[j] = SkirtMeasure.TiltDeg(pr, cr);
                    relyaw[j] = SkirtMeasure.YawOfRelDeg(pr, cr);
                    if (tilt[j] > fmax) fmax = tilt[j];
                }
                PhysTilt[f] = tilt;
                PhysTiltRigid[f] = tiltRigid;
                PhysRelYaw[f] = relyaw;
                PhysFrameMaxTilt[f] = fmax;
            }
            sw.Stop();
            RunSeconds = sw.Elapsed.TotalSeconds;
        }

        private static void ApplyPose(List<(BoneLink link, string bone)> driven, BoneCsv csv, int frame)
        {
            foreach (var (link, bone) in driven)
            {
                if (csv.TryGet(frame, bone, out var boneWorld))
                    link.Body.KinematicTarget = boneWorld * link.BodyOffsetFromBone;
            }
        }

        // 剛体姿勢からボーン姿勢を復元 (MmdPhysicsBehaviour.PullPhysicsToBones と同一式)。
        // BoneLinks[i] は Bodies[i] (剛体index i) に対応する。
        private static Quat RecoverBoneRot(PmxPhysicsBuilder builder, int rigidIdx)
        {
            var link = builder.BoneLinks[rigidIdx];
            var boneWorld = link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
            return boneWorld.Rotation;
        }
    }
}
