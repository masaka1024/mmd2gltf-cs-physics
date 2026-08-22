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
        // PhysTilt/PhysRelYaw は「ボーン空間」(剛体からボーン姿勢を復元して測る=MMDと同じ物理量)。
        // PhysTiltRigid は旧方式「剛体相対」(検算用: ring1/2 で新旧一致するはず)。
        public float[][] PhysTilt;
        public float[][] PhysTiltRigid;
        public float[][] PhysRelYaw;
        public float[] PhysFrameMaxTilt;

        public void Run(BoneCsv csv, PmxPhysicsModel model, List<SkirtJoint> joints)
        {
            if (int.TryParse(System.Environment.GetEnvironmentVariable("WARMUP"), out var _wu) && _wu >= 0) WarmupSteps = _wu; // ばらつき見積(診断)
            // ★慣性は剛体構築時に確定するので Build より前に設定する (Bullet 2.75 は 0.04, 既定0=従来)。
            if (float.TryParse(System.Environment.GetEnvironmentVariable("INERTIAMARGIN"), out var _im) && _im >= 0f)
                CapsuleShape.InertiaMargin = _im;
            // ★形状マージンも剛体構築時に確定するので Build より前 (タスク38)。
            if (System.Environment.GetEnvironmentVariable("CMARGIN") == "1") CollisionShape.BulletShapeMargin = true;
            var builder = PmxPhysicsBuilder.Build(model); // 既定 World (30Hz・1サブ, gravity -98)
            var world = builder.World;
            if (SolverIterationsOverride > 0) world.SolverIterations = SolverIterationsOverride; // 診断用
            // 計測トグル (既定OFF=挙動不変)。VMD統計を OFF/ON でMMD比較するため env で切替。
            if (System.Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
            if (System.Environment.GetEnvironmentVariable("WARMSTART_ANG") == "1") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
            if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
            if (System.Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
            if (int.TryParse(System.Environment.GetEnvironmentVariable("ITERS"), out var _it) && _it > 0) world.SolverIterations = _it; // 反復掃引(診断)
            // 鎖のたわみ対策 (UE5移植版からの逆輸入)。既定 0 = 従来とビット不変。
            if (int.TryParse(System.Environment.GetEnvironmentVariable("JOINTITERS"), out var _ji) && _ji > 0) world.JointVelocityIterations = _ji;
            if (float.TryParse(System.Environment.GetEnvironmentVariable("JOINTMAXCORR"), out var _jm) && _jm > 0f) world.JointMaxCorrectionVel = _jm;
            if (int.TryParse(System.Environment.GetEnvironmentVariable("SUBSTEPS"), out var _ss) && _ss > 0) world.SubSteps = _ss; // substep掃引(診断)
            if (int.TryParse(System.Environment.GetEnvironmentVariable("FTS_DIV"), out var _fd) && _fd > 0) world.FixedTimeStep = 1f / _fd; // 刻み掃引(診断, 1/N を正確に)
            if (System.Environment.GetEnvironmentVariable("JOINTS_FIRST") == "1") world.SolveJointsFirst = true; // Bullet同順(ジョイント→接触)
            if (System.Environment.GetEnvironmentVariable("SPLIT") == "1") world.UseSplitImpulse = true; // 接触の貫入回復を擬似速度側(参考)
            if (float.TryParse(System.Environment.GetEnvironmentVariable("CWFAC"), out var _cwf)) world.ContactWarmStartFactor = _cwf; // 接触warm-start係数
            if (int.TryParse(System.Environment.GetEnvironmentVariable("LEVER"), out var _lv)) Joint.LinearLeverMode = _lv; // 線形レバーアーム 0/1/2
            // ★2026-08-21 追加: ジョイント位置補正係数の掃引 (既定0.2)。未設定=無変更。
            //   LEVER=1 の下で翻り量(ゲイン)がどう動くかを測るため。
            if (float.TryParse(System.Environment.GetEnvironmentVariable("JBETA"),
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out var _jb) && _jb >= 0f)
                foreach (var j in world.Joints) j.Beta = _jb;
            if (System.Environment.GetEnvironmentVariable("MIXAXES") == "1") Joint.AngularMixedAxes = true; // 角度リミット行=Bullet混合軸
            // タスク22: 角度抽出を Bullet 2.75 実挙動 (R_B⁻¹R_A のオイラー + 角度行の軸反転) へ。
            //   restosc の ANGCONV と同名。MIXAXES と違い、こちらは角度そのものの規約。
            if (System.Environment.GetEnvironmentVariable("ANGCONV") == "1") Joint.BulletAngleConvention = true;
            // タスク32: ばねを Bullet 2.75 のモーター行として解く。
            // ★タスク37: 既定が ON。env は明示されたときだけ上書きする (未設定で false にしない)。
            {
                var _sm = System.Environment.GetEnvironmentVariable("SPRINGMOTOR");
                if (_sm != null) Joint.SpringAsMotorRow = _sm == "1";
            }
            if (System.Environment.GetEnvironmentVariable("ROTEXP") == "1") PhysicsWorld.BulletRotationIntegration = true;
            if (System.Environment.GetEnvironmentVariable("CTHRESH") == "1") GjkEpa.BulletContactThreshold = true;
            if (System.Environment.GetEnvironmentVariable("CRHS") == "1") world.ContactRhsBullet = true;   // ★タスク48
            if (System.Environment.GetEnvironmentVariable("CMAN") == "1") PersistentManifold.BulletManifoldPoints = true;   // ★タスク51
            if (float.TryParse(System.Environment.GetEnvironmentVariable("WARMFAC"), out var _wf)) Joint.WarmStartFactor = _wf;

            // ★実効フラグのエコー (env の値ではなくエンジンから読み戻した値)。
            //   モードを足すたびに env の配線を忘れる事故が続いたので、常時出す (restosc と同じ流儀)。
            {
                float _bmin = float.MaxValue, _bmax = float.MinValue;
                foreach (var j in world.Joints) { if (j.Beta < _bmin) _bmin = j.Beta; if (j.Beta > _bmax) _bmax = j.Beta; }
                System.Console.WriteLine("[実効] bonecheck  FixedTimeStep=1/" + (1f / world.FixedTimeStep).ToString("F2")
                    + "  SubSteps=" + world.SubSteps + "  Iters=" + world.SolverIterations
                    + "  Joint.Beta=" + (world.Joints.Count == 0 ? "-" : (_bmin == _bmax ? _bmin.ToString("G6") : _bmin.ToString("G6") + "~" + _bmax.ToString("G6")))
                    + "  ContactBaumgarte=" + world.BaumgarteFactor.ToString("G6"));
                System.Console.WriteLine("[実効] bonecheck  JSplit=" + world.UseJointSplitImpulse
                    + "  Split=" + world.UseSplitImpulse
                    + "  JWarm=" + world.UseJointWarmStart + "/" + world.UseJointWarmStartAngular
                    + "  LeverMode=" + Joint.LinearLeverMode
                    + "  MixedAxes=" + Joint.AngularMixedAxes
                    + "  AngConv=" + Joint.BulletAngleConvention
                    + "  SpringMotor=" + Joint.SpringAsMotorRow
                    + "  RotExp=" + PhysicsWorld.BulletRotationIntegration
                    + "  CThresh=" + GjkEpa.BulletContactThreshold
                    + "  CMan=" + PersistentManifold.BulletManifoldPoints
                    + "  CMargin=" + CollisionShape.BulletShapeMargin
                    + "  MaxCorrVel=" + Joint.MaxCorrectionVel.ToString("G6"));
            }

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
            ApplyPose(builder, model, csv, 0);
            builder.ResetBodiesToBonePoseFk(i =>
                (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw))
                    ? (RigidTransform?)bw : null);

            // ウォームアップ: フレーム0姿勢で空回し。
            for (int s = 0; s < WarmupSteps; s++) world.StepSimulation(1f / 30f);

            // 計測本番。
            int alignMode = int.TryParse(System.Environment.GetEnvironmentVariable("ALIGN"), out var _am) ? _am : 0; // 2=補正フィードバック
            for (int f = 0; f < F; f++)
            {
                ApplyPose(builder, model, csv, f);
                world.StepSimulation(1f / 30f);
                // 補正層再現(段階2): aligned姿勢(位置=親チェーン再構成/回転=物理)を剛体へ書き戻し=次stepへ影響。
                if (alignMode == 2)
                {
                    float _alpha = float.TryParse(System.Environment.GetEnvironmentVariable("ALPHA"), out var _av) ? _av : 0f;
                    var aligned = builder.ComputeAlignedBonePoses(bi =>
                        (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(f, model.BoneNames[bi], out var dw)) ? (RigidTransform?)dw : null, _alpha, true);
                    foreach (var link in builder.BoneLinks)
                        if (link.Mode != PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < aligned.Length && aligned[link.BoneIndex].HasValue)
                        { link.Body.WorldTransform = aligned[link.BoneIndex].Value * link.BodyOffsetFromBone; link.Body.UpdateInertiaWorld(); }
                }

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

        // 駆動式は共通ヘルパ ApplyKinematicTargets に集約 (2026-08-09 hairfid誤配置事故の再発防止)。
        private static void ApplyPose(PmxPhysicsBuilder builder, PmxPhysicsModel model, BoneCsv csv, int frame)
        {
            builder.ApplyKinematicTargets(bi =>
                (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(frame, model.BoneNames[bi], out var bw)) ? (RigidTransform?)bw : null);
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
