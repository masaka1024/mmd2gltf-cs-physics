// タスク1: バネ/リミットの符号を実測で確認する。
// 2剛体をジョイントで繋ぎ、片方(A)固定、Bを平衡から既知角ずらして1ステップ。
// 「ずれが減る方向へトルクが働くか」を角速度と euler の変化で判定する。
// 3軸×正負でテスト。本体は不変(内部 ToEulerXYZ を同一アセンブリから呼ぶだけ)。
using System;
using BulletPhysics;

static class SignTest
{
    static int _fail = 0;
    const float DT = 1f / 30f;

    static Vec3 Axis(int i) => i == 0 ? Vec3.XAxis : i == 1 ? Vec3.YAxis : Vec3.ZAxis;

    // A固定・B可動、指定タイプのジョイント。angular limit と spring を指定。
    static (PhysicsWorld w, RigidBody b, Joint j) Build(JointType type, Vec3 angLo, Vec3 angHi, Vec3 springAng)
    {
        var w = new PhysicsWorld { Gravity = Vec3.Zero, SubSteps = 1, FixedTimeStep = DT, SolverIterations = 10 };
        var a = new RigidBody(new BoxShape(new Vec3(0.3f, 0.3f, 0.3f))) { Mode = PhysicsMode.BoneFollow, Group = 0, CollisionMask = 0 };
        a.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 2, 0)); a.SetMassProps(0f);
        var b = new RigidBody(new BoxShape(new Vec3(0.3f, 0.3f, 0.3f))) { Mode = PhysicsMode.Dynamic, Group = 0, CollisionMask = 0 };
        b.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 0, 0)); b.SetMassProps(1f);
        w.AddBody(a); w.AddBody(b);
        // 並進は free (lo>hi) にして角度だけ見る。worldFrame は中点 identity。
        var frame = new RigidTransform(Quat.Identity, new Vec3(0, 1, 0));
        var j = Joint.FromPmx(type, a, b, frame,
            new Vec3(1, 1, 1), new Vec3(-1, -1, -1),  // linear free
            angLo, angHi, Vec3.Zero, springAng);
        w.AddJoint(j);
        return (w, b, j);
    }

    // B を平衡から axis まわり s*θ 回転させ、euler[axis] の符号を確認して 1step。
    static void Case(string tag, JointType type, Vec3 angLo, Vec3 angHi, Vec3 springAng, int axis, float s, float theta)
    {
        var (w, b, j) = Build(type, angLo, angHi, springAng);
        b.WorldTransform = new RigidTransform(Quat.FromAxisAngle(Axis(axis), s * theta), new Vec3(0, 0, 0));
        b.UpdateInertiaWorld();

        // ずらし後の euler[axis] (復元の基準)
        var wA = ((RigidBody)GetA(w)).WorldTransform; // A
        // qRel = worldA^-1 * worldB (frame identity なので剛体回転の相対)
        float eBefore = EulerAxis(j, axis);
        w.StepSimulation(DT);
        float eAfter = EulerAxis(j, axis);
        float wAxis = b.AngularVelocity.Dot(Axis(axis)); // B の角速度の axis 成分

        // 復元 = |euler| が減る かつ 角速度が -s 方向
        bool eulerRestores = Math.Abs(eAfter) < Math.Abs(eBefore) - 1e-5f;
        bool velRestores = Math.Sign(wAxis) == -Math.Sign(s);
        bool ok = eulerRestores && velRestores;
        Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {tag} axis{axis} s{(s > 0 ? "+" : "-")}: " +
            $"euler {eBefore * 57.3f:F1}°→{eAfter * 57.3f:F1}° wAxis={wAxis:F3} (復元={eulerRestores}/{velRestores})");
        if (!ok) _fail++;
    }

    static object GetA(PhysicsWorld w) => w.Bodies[0];
    // ジョイントの worldA^-1*worldB の euler[axis]
    static float EulerAxis(Joint j, int axis)
    {
        var wA = j.BodyA.WorldTransform * j.FrameInA;
        var wB = j.BodyB.WorldTransform * j.FrameInB;
        var qRel = (wA.Rotation.Conjugated() * wB.Rotation).Normalized;
        var e = Joint.ToEulerXYZ(qRel);
        return axis == 0 ? e.x : axis == 1 ? e.y : e.z;
    }

    public static void Run()
    {
        Console.WriteLine("== 角度バネ 符号テスト (Spring6Dof, 平衡0からずらして復元するか) ==");
        for (int ax = 0; ax < 3; ax++)
            foreach (float s in new[] { +1f, -1f })
                Case("spring", JointType.Spring6Dof,
                    new Vec3(-3, -3, -3), new Vec3(3, 3, 3), new Vec3(50, 50, 50), ax, s, 0.3f);

        Console.WriteLine("== 角度リミット 符号テスト (Generic6Dof, 範囲±0.1を超えて復元するか) ==");
        for (int ax = 0; ax < 3; ax++)
            foreach (float s in new[] { +1f, -1f })
            {
                var lo = new Vec3(-0.1f, -0.1f, -0.1f); var hi = new Vec3(0.1f, 0.1f, 0.1f);
                Case("limit", JointType.Generic6Dof, lo, hi, Vec3.Zero, ax, s, 0.35f);
            }

        Console.WriteLine(_fail == 0 ? "== 符号テスト 全PASS ==" : $"== 符号テスト {_fail}件 FAIL ==");
    }

    static int Main() { Run(); return _fail == 0 ? 0 : 1; }
}
