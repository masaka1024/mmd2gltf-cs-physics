// ===========================================================================
// Unity Bullet 互換物理エンジン – PMX -> PhysicsWorld ビルダー
// PMX の剛体/Joint を物理エンジンのインスタンスへ変換する。
// 剛体<->ボーンのオフセット (バインドポーズ) も生成する。
// ===========================================================================

using System.Collections.Generic;

namespace BulletPhysics.Pmx
{
    /// <summary>剛体とボーンの紐付け。ボーン追従 / 物理フィードバックに使う。</summary>
    public struct BoneLink
    {
        public RigidBody Body;
        public int BoneIndex;
        // バインド時の「ボーン→剛体」相対変換 (bone^-1 * body)。
        public RigidTransform BodyOffsetFromBone;
        public PhysicsMode Mode;
    }

    public sealed class PmxPhysicsBuilder
    {
        public PhysicsWorld World { get; } = new();
        public readonly List<BoneLink> BoneLinks = new();
        public readonly List<RigidBody> Bodies = new();

        public static PmxPhysicsBuilder Build(PmxPhysicsModel model)
        {
            var b = new PmxPhysicsBuilder();
            b.BuildBodies(model);
            b.BuildJoints(model);
            return b;
        }

        private void BuildBodies(PmxPhysicsModel model)
        {
            foreach (var rb in model.RigidBodies)
            {
                var shape = CreateShape(rb);
                var body = new RigidBody(shape)
                {
                    Name = rb.Name,
                    BoneIndex = rb.BoneIndex,
                    Group = rb.Group,
                    // PMX の 16bit フィールドは bit=1 が「そのグループと衝突する」を意味するので
                    // そのまま衝突マスクとして渡す (Bullet の collision mask 相当)。
                    CollisionMask = rb.NonCollisionGroup,
                    Mode = (PhysicsMode)rb.PhysicsMode,
                    LinearDamping = rb.LinearDamping,
                    AngularDamping = rb.AngularDamping,
                    Restitution = rb.Restitution,
                    Friction = rb.Friction,
                    WorldTransform = RigidTransform.FromEuler(rb.Position, rb.Rotation),
                };
                body.KinematicTarget = body.WorldTransform;
                // ボーン追従は質量 0 (kinematic)、それ以外は PMX 質量。
                body.SetMassProps(body.Mode == PhysicsMode.BoneFollow ? 0f : rb.Mass);

                World.AddBody(body);
                Bodies.Add(body);

                // ボーンオフセット。
                var link = new BoneLink
                {
                    Body = body,
                    BoneIndex = rb.BoneIndex,
                    Mode = body.Mode,
                    BodyOffsetFromBone = ComputeOffset(model, rb),
                };
                BoneLinks.Add(link);
            }
        }

        /// <summary>
        /// 物理開始/リセット時に、動的剛体も含む全剛体を現在のボーン姿勢へ整合させる
        /// (MMD の物理演算リセット相当)。剛体を boneWorld * BodyOffsetFromBone に置き、
        /// 速度を 0、慣性ワールドを更新、接触/蓄積インパルスをクリアする。
        /// これをしないと、フレーム0で脚が曲がっている場合に kinematic な脚コライダーだけが
        /// フレーム0へ動き、動的スカートがバインド位置に取り残されて逃げられない貫入平衡に落ちる。
        ///
        /// getBoneWorld: ボーンindex → そのボーンのワールド姿勢 (無ければ null)。
        ///   null を返したボーン (BoneIndex&lt;0 や、姿勢が得られないボーン) はバインド位置のままとする。
        /// </summary>
        public void ResetBodiesToBonePose(System.Func<int, RigidTransform?> getBoneWorld)
        {
            foreach (var link in BoneLinks)
            {
                var body = link.Body;
                if (link.BoneIndex >= 0)
                {
                    var bw = getBoneWorld(link.BoneIndex);
                    if (bw.HasValue)
                    {
                        body.WorldTransform = bw.Value * link.BodyOffsetFromBone;
                        body.KinematicTarget = body.WorldTransform;
                        body.KinematicStepTarget = body.WorldTransform;
                    }
                }
                body.LinearVelocity = Vec3.Zero;
                body.AngularVelocity = Vec3.Zero;
                body.UpdateInertiaWorld();
            }
            World.ClearContacts();
        }

        private static RigidTransform ComputeOffset(PmxPhysicsModel model, PmxRigidBody rb)
        {
            var bodyWorld = RigidTransform.FromEuler(rb.Position, rb.Rotation);
            if (rb.BoneIndex < 0 || rb.BoneIndex >= model.BonePositions.Count)
                return bodyWorld; // ボーン無し
            // バインド時ボーンは回転恒等・位置のみ。
            var boneWorld = new RigidTransform(Quat.Identity, model.BonePositions[rb.BoneIndex]);
            return boneWorld.InverseTimes(bodyWorld);
        }

        private static CollisionShape CreateShape(PmxRigidBody rb)
        {
            return rb.ShapeType switch
            {
                0 => new SphereShape(rb.Size.x),
                1 => new BoxShape(rb.Size),
                2 => new CapsuleShape(rb.Size.x, rb.Size.y),
                _ => new SphereShape(rb.Size.x),
            };
        }

        private void BuildJoints(PmxPhysicsModel model)
        {
            foreach (var pj in model.Joints)
            {
                RigidBody a = ValidBody(pj.RigidBodyAIndex);
                RigidBody b = ValidBody(pj.RigidBodyBIndex);
                if (a == null || b == null) continue; // 両端必須

                var worldFrame = RigidTransform.FromEuler(pj.Position, pj.Rotation);
                var joint = Joint.FromPmx(
                    (JointType)pj.JointType, a, b, worldFrame,
                    pj.LinearLowerLimit, pj.LinearUpperLimit,
                    pj.AngularLowerLimit, pj.AngularUpperLimit,
                    pj.SpringLinear, pj.SpringAngular);
                joint.Name = pj.Name;
                World.AddJoint(joint);
            }
        }

        private RigidBody ValidBody(int index) =>
            (index >= 0 && index < Bodies.Count) ? Bodies[index] : null;
    }
}
