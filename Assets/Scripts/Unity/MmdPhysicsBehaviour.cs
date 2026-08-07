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

        [Header("Solver")]
        public float Gravity = 98f;         // MMD スケール重力 (約 9.8 * 10)
        public int SolverIterations = 10;
        public int SubSteps = 2;

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
            ResolveBones();
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

        // --- 座標変換 (MMD 左手系 <-> Unity 左手系, Z 反転) ---
        // PMX/MMD と Unity はどちらも左手系だが Z の向きが逆。
        public static Vector3 MmdToUnityPos(Vec3 v) => new(v.x, v.y, -v.z);
        public static Vec3 UnityToMmdPos(Vector3 v) => new(v.x, v.y, -v.z);
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

        private static void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            Gizmos.matrix = UnityEngine.Matrix4x4.TRS(pos, rot, Vector3.one);
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
