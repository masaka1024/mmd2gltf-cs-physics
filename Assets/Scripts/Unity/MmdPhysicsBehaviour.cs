// ===========================================================================
// Unity Bullet 互換物理エンジン – Unity ブリッジ
// PMX を読み込み、物理ワールドを毎フレーム進め、ボーン Transform と同期する。
// エンジンは PMX ネイティブ座標 (左手系 Y-up) で動作し、境界で Unity 座標へ変換。
// ===========================================================================

using System.Collections.Generic;
using UnityEngine;
using BulletPhysics.Pmx;

namespace BulletPhysics.Unity
{
    /// <summary>MMD/PMX 物理を Unity 上で駆動するコンポーネント。</summary>
    public sealed class MmdPhysicsBehaviour : MonoBehaviour
    {
        [Tooltip("読み込む .pmx ファイルの絶対 or Assets 相対パス")]
        public string PmxPath = "";

        [Tooltip("剛体 BoneIndex -> ボーン Transform のマップ (ボーン名で解決)")]
        public Transform ModelRoot;

        [Tooltip("エンジン(PMXネイティブ単位) -> Unity 配置スケール。Unity側モデルが縮小配置される運用向け")]
        public float UnitScale = 0.08f;

        [Header("Solver")]
        public float Gravity = 98f;         // MMD スケール重力 (約 9.8 * 10)
        public int SolverIterations = 10;
        // リファレンスは実効 1/60 (FixedTimeStep=1/30 は 30fps 入力に合わせ、SubSteps=2 で刻む)。
        // 詳細は DESIGN.md「リファレンス刻み」節。より忠実にしたい場合は SubSteps を 4 (=1/120) に。
        public int SubSteps = 2;
        public float FixedTimeStep = 1f / 30f;

        [Header("Debug")]
        public bool DrawGizmos = true;

        private PmxPhysicsBuilder _builder;
        private PmxPhysicsModel _model;
        private Transform[] _boneTransforms;   // BoneIndex -> Transform

        void Start()
        {
            if (string.IsNullOrEmpty(PmxPath)) return;
            LoadPmx(PmxPath);
        }

        public void LoadPmx(string path)
        {
            _model = PmxReader.LoadFile(path);
            _builder = PmxPhysicsBuilder.Build(_model);
            _builder.World.Gravity = new Vec3(0f, -Gravity, 0f);
            _builder.World.SolverIterations = SolverIterations;
            _builder.World.SubSteps = SubSteps;
            _builder.World.FixedTimeStep = FixedTimeStep;
            ResolveBones();
            ResetPhysicsToBones();
        }

        /// <summary>物理開始/リセット時に、全剛体を現在のボーン姿勢へ整合させる
        /// (MMD の物理演算リセット相当)。フレーム0で脚が曲がっていても動的剛体がバインド位置に
        /// 取り残されて貫入するのを防ぐ。LoadPmx 後および任意のタイミングで呼べる。</summary>
        public void ResetPhysicsToBones()
        {
            if (_builder == null) return;
            // FK-rest で統一: 物理ボーン(スカート/髪)はスケルトンの姿勢を使わず親から前計算する。
            // スケルトンの物理ボーンに前フレームの物理結果が残っていても正しい開始状態になる。
            _builder.ResetBodiesToBonePoseFk(BoneWorldOrNull);
        }

        // ボーンindex → ワールド姿勢 (MMD座標)。Transform 未解決なら null (バインド維持)。
        private RigidTransform? BoneWorldOrNull(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return null;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        private void ResolveBones()
        {
            _boneTransforms = new Transform[_model.BoneNames.Count];
            if (ModelRoot == null) return;
            var map = new Dictionary<string, Transform>();
            foreach (var t in ModelRoot.GetComponentsInChildren<Transform>())
                map[t.name] = t;
            for (int i = 0; i < _model.BoneNames.Count; i++)
                if (map.TryGetValue(_model.BoneNames[i], out var tr))
                    _boneTransforms[i] = tr;
        }

        void FixedUpdate()
        {
            if (_builder == null) return;

            // 1. ボーン追従剛体に目標姿勢を渡す (物理前)。
            PushBonesToKinematic();

            // 2. 物理ステップ。
            _builder.World.StepSimulation(Time.fixedDeltaTime);

            // 3. 物理剛体 -> ボーンへ反映 (物理後)。
            PullPhysicsToBones();
        }

        private void PushBonesToKinematic()
        {
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode != PhysicsMode.BoneFollow) continue;
                var boneWorld = BoneWorld(link.BoneIndex);
                link.Body.KinematicTarget = boneWorld * link.BodyOffsetFromBone;
            }
        }

        private void PullPhysicsToBones()
        {
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode == PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || _boneTransforms == null ||
                    link.BoneIndex >= _boneTransforms.Length) continue;
                var tr = _boneTransforms[link.BoneIndex];
                if (tr == null) continue;

                // body = bone * offset  ->  bone = body * offset^-1
                var boneWorld = link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
                tr.position = MmdToUnityPos(boneWorld.Origin);
                tr.rotation = MmdToUnityRot(boneWorld.Rotation);
            }
        }

        private RigidTransform BoneWorld(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return RigidTransform.Identity;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        // --- 座標変換 (MMD 左手系 <-> Unity 左手系, Z 反転 + 単位スケール) ---
        // PMX/MMD と Unity はどちらも左手系だが Z の向きが逆。位置は UnitScale で換算。
        // (回転はスケール無関係なので static のまま)
        public Vector3 MmdToUnityPos(Vec3 v) => new(v.x * UnitScale, v.y * UnitScale, -v.z * UnitScale);
        public Vec3 UnityToMmdPos(Vector3 v) => new(v.x / UnitScale, v.y / UnitScale, -v.z / UnitScale);
        public static Quaternion MmdToUnityRot(Quat q) => new(-q.x, -q.y, q.z, q.w);
        public static Quat UnityToMmdRot(Quaternion q) => new(-q.x, -q.y, q.z, q.w);

        void OnDrawGizmos()
        {
            if (!DrawGizmos || _builder == null) return;
            foreach (var body in _builder.Bodies)
            {
                Gizmos.color = body.Mode == PhysicsMode.BoneFollow
                    ? Color.cyan
                    : (body.Mode == PhysicsMode.Dynamic ? Color.green : Color.yellow);
                var p = MmdToUnityPos(body.WorldTransform.Origin);
                var rot = MmdToUnityRot(body.WorldTransform.Rotation);
                DrawShape(body.Shape, p, rot);
            }
        }

        private void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            // 形状サイズは PMX ネイティブ単位なので UnitScale を掛けて位置と揃える。
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
