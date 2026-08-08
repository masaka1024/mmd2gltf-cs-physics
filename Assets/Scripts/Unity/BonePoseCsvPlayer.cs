// ===========================================================================
// ボーン姿勢CSV 再生コンポーネント (目視確認用)。
// 本家ベイク済みのボーン世界姿勢CSVで PMX 物理を駆動し、ヘッドレス(HeadlessDriver)と
// 同一入力・同一ロジックで同じ動きを Unity 上に再現する。さらに、CSVに含まれる本家の
// スカートボーン姿勢から「本家スカート剛体」をゴースト(別色)で重ね描きし、自前物理との
// ズレを目視で比較できるようにする。
//
// 操作は Inspector の右クリック(ContextMenu)で行う (Input/GUI 非依存):
//   Play / Pause / Step Forward(+1) / Step Back(-1) / Jump to Frame / 窓先頭/末尾へ
// 特に窓6 (F2440〜F2470) をコマ送りで確認するため、WindowStart/End とジャンプを用意。
//
// 注意: 物理は逆再生できないため、後退やジャンプは「フレーム0から目標まで再シミュレーション」
//   する (7000フレームでも一瞬)。前進1フレームは差分で進むso安価。
// ===========================================================================
using System.Collections.Generic;
using UnityEngine;
using BulletPhysics.Pmx;

namespace BulletPhysics.Unity
{
    public sealed class BonePoseCsvPlayer : MonoBehaviour
    {
        [Header("入力")]
        [Tooltip("読み込む .pmx ファイルの絶対パス")]
        public string PmxPath = "";
        [Tooltip("本家ベイク済みボーン世界姿勢CSVの絶対パス")]
        public string BoneCsvPath = @"C:\Users\masa_\AppData\Local\Temp\IA_bone_world_pose.csv";

        [Header("Solver (リファレンス=30Hz・1サブ)")]
        public float Gravity = 98f;
        public int SolverIterations = 10;
        public int SubSteps = 1;
        public float FixedTimeStep = 1f / 30f;
        [Tooltip("計測開始前にフレーム0姿勢で空回しするステップ数 (バインド姿勢からの沈み込み過渡を除く)")]
        public int WarmupSteps = 60;

        [Header("表示")]
        [Tooltip("エンジン(PMXネイティブ単位) -> Unity 配置スケール")]
        public float UnitScale = 0.08f;
        public bool DrawSelf = true;
        [Tooltip("本家スカート剛体をゴースト(マゼンタ)で重ね描き")]
        public bool DrawReferenceGhost = true;
        [Tooltip("ゴーストをスカート(CSVに含まれるボーン)に限定")]
        public bool SkirtOnlyGhost = true;

        [Header("再生")]
        public bool Playing = false;
        [Tooltip("再生速度 (実時間の何倍速でフレームを進めるか。30=等速)")]
        public float PlaybackFps = 30f;
        [Tooltip("現在ワールドが到達しているフレーム (表示専用。移動は下の ContextMenu で)")]
        public int Frame = 0;

        [Header("窓6 コマ送り (自前92.2°/本家62.9°の確認)")]
        public int WindowStart = 2440;
        public int WindowEnd = 2470;

        private PmxPhysicsBuilder _builder;
        private PmxPhysicsModel _model;
        private BonePoseCsvSource _csv;
        private List<(BoneLink link, string bone)> _driven = new();
        private int _simFrame = -1;   // ワールドが到達しているフレーム
        private float _accum;

        void Start() { Reload(); }

        [ContextMenu("Reload (PMX+CSV 再読込)")]
        public void Reload()
        {
            if (string.IsNullOrEmpty(PmxPath)) { Debug.LogWarning("[CsvPlayer] PmxPath 未設定"); return; }
            _model = PmxReader.LoadFile(PmxPath);
            _csv = BonePoseCsvSource.Load(BoneCsvPath);
            if (_csv == null) Debug.LogWarning($"[CsvPlayer] CSVが読めません: {BoneCsvPath}");
            RewindTo(Frame);
        }

        private void BuildWorld()
        {
            _builder = PmxPhysicsBuilder.Build(_model);
            var w = _builder.World;
            w.Gravity = new Vec3(0f, -Gravity, 0f);
            w.SolverIterations = SolverIterations;
            w.SubSteps = SubSteps;
            w.FixedTimeStep = FixedTimeStep;

            _driven.Clear();
            if (_csv == null) return;
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode != PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || link.BoneIndex >= _model.BoneNames.Count) continue;
                string b = _model.BoneNames[link.BoneIndex];
                if (_csv.HasBone(b)) _driven.Add((link, b));
            }
        }

        private void ApplyPose(int f)
        {
            foreach (var (link, b) in _driven)
                if (_csv.TryGet(f, b, out var bw)) link.Body.KinematicTarget = bw * link.BodyOffsetFromBone;
        }

        /// <summary>ワールドを作り直し、フレーム0のウォームアップから target まで再シミュレーションする。</summary>
        public void RewindTo(int target)
        {
            if (_model == null) return;
            int last = _csv != null ? _csv.FrameCount - 1 : 0;
            target = System.Math.Clamp(target, 0, System.Math.Max(0, last));
            BuildWorld();
            ApplyPose(0);
            for (int s = 0; s < WarmupSteps; s++) _builder.World.StepSimulation(FixedTimeStep);
            for (int f = 0; f <= target; f++) { ApplyPose(f); _builder.World.StepSimulation(FixedTimeStep); }
            _simFrame = target;
            Frame = target;
        }

        [ContextMenu("Step Forward (+1)")]
        public void StepForward()
        {
            if (_builder == null || _csv == null) return;
            if (_simFrame + 1 >= _csv.FrameCount) { Playing = false; return; }
            int f = _simFrame + 1;
            ApplyPose(f);
            _builder.World.StepSimulation(FixedTimeStep);
            _simFrame = f;
            Frame = f;
        }

        [ContextMenu("Step Back (-1)")]
        public void StepBack()
        {
            if (_simFrame > 0) RewindTo(_simFrame - 1);
        }

        [ContextMenu("Play")] public void Play() { Playing = true; }
        [ContextMenu("Pause")] public void Pause() { Playing = false; }
        [ContextMenu("Jump to Frame (Frame値へ)")] public void JumpToFrame() { RewindTo(Frame); }
        [ContextMenu("Jump to Window Start (窓先頭)")] public void JumpWindowStart() { RewindTo(WindowStart); }
        [ContextMenu("Jump to Window End (窓末尾)")] public void JumpWindowEnd() { RewindTo(WindowEnd); }

        void FixedUpdate()
        {
            if (!Playing || _builder == null || _csv == null) return;
            _accum += Time.fixedDeltaTime * PlaybackFps;
            int steps = (int)_accum;
            _accum -= steps;
            for (int i = 0; i < steps; i++)
            {
                if (_simFrame + 1 >= _csv.FrameCount) { Playing = false; break; }
                StepForward();
            }
        }

        // ---- 座標変換 (MmdPhysicsBehaviour と同一) ----
        private Vector3 MmdToUnityPos(Vec3 v) => new(v.x * UnitScale, v.y * UnitScale, -v.z * UnitScale);
        private static Quaternion MmdToUnityRot(Quat q) => new(-q.x, -q.y, q.z, q.w);

        void OnDrawGizmos()
        {
            if (_builder == null) return;

            // 自前物理の剛体 (BoneFollow=cyan, Dynamic=green, Bone合わせ=yellow)。
            if (DrawSelf)
                foreach (var body in _builder.Bodies)
                {
                    Gizmos.color = body.Mode == PhysicsMode.BoneFollow ? Color.cyan
                        : (body.Mode == PhysicsMode.Dynamic ? Color.green : Color.yellow);
                    DrawShape(body.Shape, MmdToUnityPos(body.WorldTransform.Origin), MmdToUnityRot(body.WorldTransform.Rotation));
                }

            // 本家ゴースト: CSVのボーン姿勢 * オフセット で剛体位置を復元しマゼンタで重ね描き。
            if (DrawReferenceGhost && _csv != null && _simFrame >= 0)
            {
                Gizmos.color = Color.magenta;
                for (int i = 0; i < _builder.BoneLinks.Count; i++)
                {
                    var link = _builder.BoneLinks[i];
                    if (link.BoneIndex < 0 || link.BoneIndex >= _model.BoneNames.Count) continue;
                    string bn = _model.BoneNames[link.BoneIndex];
                    if (SkirtOnlyGhost && !bn.StartsWith("スカート")) continue;
                    if (!_csv.TryGet(_simFrame, bn, out var bw)) continue;
                    var refWorld = bw * link.BodyOffsetFromBone;
                    DrawShape(link.Body.Shape, MmdToUnityPos(refWorld.Origin), MmdToUnityRot(refWorld.Rotation));
                }
            }
        }

        private void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            Gizmos.matrix = UnityEngine.Matrix4x4.TRS(pos, rot, Vector3.one * UnitScale);
            switch (shape)
            {
                case SphereShape s:
                    Gizmos.DrawWireSphere(Vector3.zero, s.Radius);
                    break;
                case BoxShape b:
                    Gizmos.DrawWireCube(Vector3.zero, new Vector3(b.HalfExtents.x, b.HalfExtents.y, b.HalfExtents.z) * 2f);
                    break;
                case CapsuleShape c:
                    Gizmos.DrawWireSphere(new Vector3(0, c.HalfHeight, 0), c.Radius);
                    Gizmos.DrawWireSphere(new Vector3(0, -c.HalfHeight, 0), c.Radius);
                    break;
            }
            Gizmos.matrix = m;
        }
    }
}
