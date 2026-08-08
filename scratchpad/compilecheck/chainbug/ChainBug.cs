// ===========================================================================
// タスクA: PMX を外した最小再現。
// キネマティックな固定点に、剛体を N 個、全DOFロックの 6DOF ジョイントで直列に吊るす。
//   - 並進ロック(linLo=linHi=0)・角度ロック(angLo=angHi=0)・バネ0
//   - 重力 -98 / dt=1/30 / SubSteps=2 / SolverIterations=可変 / 300 step
// 理論期待値: 剛体は 1mm も動かない (maxDrift=0, maxSpeed=0)。
// N と反復回数を掃引し、ソルバ単体の保持能力を定量化する。
// (スカート=深さ3 は通り、髪=深さ5〜6 で壊れる仮説の定量確認)
// ===========================================================================
using System;
using System.Text;
using System.Collections.Generic;
using BulletPhysics;

static class ChainBug
{
    const float Seg = 0.5f;     // セグメント間隔(髪相当)
    const float BoxHalf = 0.1f; // 箱の半サイズ

    // kinematic anchor + N dynamic を全DOFロックで直列に吊るしたワールドを作る。
    static (PhysicsWorld world, List<RigidBody> dyn) BuildChain(int n, float mass, bool tipLighter)
    {
        var world = new PhysicsWorld();

        var anchor = new RigidBody(new BoxShape(new Vec3(BoxHalf, BoxHalf, BoxHalf))) { Mode = PhysicsMode.BoneFollow, Name = "anchor" };
        anchor.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 0, 0));
        anchor.SetMassProps(0f);
        world.AddBody(anchor);

        var dyn = new List<RigidBody>();
        RigidBody prev = anchor;
        for (int i = 1; i <= n; i++)
        {
            // 先端ほど軽い場合: i=n(先端) が最軽量。均一なら mass 固定。
            float m = tipLighter ? mass * (float)(n - i + 1) / n : mass;
            var b = new RigidBody(new BoxShape(new Vec3(BoxHalf, BoxHalf, BoxHalf))) { Mode = PhysicsMode.Dynamic, Name = $"link{i}" };
            b.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, -Seg * i, 0));
            b.CollisionMask = 0; // 接触無効(ジョイントのみを検証)
            b.SetMassProps(m);
            world.AddBody(b);
            dyn.Add(b);

            // prev と b を結ぶ全DOFロック 6DOF ジョイント。フレームは両剛体の中点、回転恒等。
            var mid = new Vec3(0, -Seg * i + Seg * 0.5f, 0);
            var frame = new RigidTransform(Quat.Identity, mid);
            var j = Joint.FromPmx(JointType.Generic6Dof, prev, b, frame,
                Vec3.Zero, Vec3.Zero,   // linLo=linHi=0  → 並進ロック
                Vec3.Zero, Vec3.Zero,   // angLo=angHi=0  → 角度ロック
                Vec3.Zero, Vec3.Zero);  // バネ0
            world.AddJoint(j);
            prev = b;
        }
        return (world, dyn);
    }

    static (float maxDrift, float maxSpeed) Run(int n, int iters, int subs, float beta, float mass, bool tipLighter)
    {
        var (world, dyn) = BuildChain(n, mass, tipLighter);
        world.Gravity = new Vec3(0, -98f, 0);
        world.FixedTimeStep = 1f / 30f;
        world.SubSteps = subs;
        world.SolverIterations = iters;
        if (beta >= 0f) foreach (var j in world.Joints) j.Beta = beta;

        var init = new Vec3[dyn.Count];
        for (int i = 0; i < dyn.Count; i++) init[i] = dyn[i].WorldTransform.Origin;

        float maxDrift = 0, maxSpeed = 0;
        for (int s = 0; s < 300; s++)
        {
            world.StepSimulation(world.FixedTimeStep);
            for (int i = 0; i < dyn.Count; i++)
            {
                float d = (dyn[i].WorldTransform.Origin - init[i]).Length; if (d > maxDrift) maxDrift = d;
                float v = dyn[i].LinearVelocity.Length; if (v > maxSpeed) maxSpeed = v;
            }
        }
        return (maxDrift, maxSpeed);
    }

    // タスクB: サブステップ悪化の再検証。beta(既定0.2 vs 0)× subs(1,2,4,8) を掃引。
    static void TaskB(StringBuilder sb)
    {
        float mass = 0.02f;
        int[] subsSet = { 1, 2, 4, 8 };
        int[] Ns = { 6, 10 };
        sb.AppendLine("==================== タスクB: サブステップ悪化の検証(合成チェーン) ====================");
        sb.AppendLine("iters=10固定。Baumgarte補正速度は Beta*err/dt で刻みが細かいほど強い。");
        sb.AppendLine("Beta=0(位置補正なし)で subs 悪化が消えれば注入源=Baumgarte と確定。");
        foreach (var n in Ns)
        {
            sb.AppendLine();
            sb.AppendLine($"[N={n}] maxDrift / maxSpeed :");
            sb.Append("   beta\\subs |"); foreach (var s in subsSet) sb.Append($" {s,14} |"); sb.AppendLine();
            foreach (var beta in new[] { 0.2f, 0.0f })
            {
                sb.Append($"   {beta,8:F2} |");
                foreach (var s in subsSet) { var (d, v) = Run(n, 10, s, beta, mass, false); sb.Append($" {d,6:F4}/{v,6:F3} |"); }
                sb.AppendLine();
            }
        }
    }

    static int Main()
    {
        var task = Environment.GetEnvironmentVariable("TASK") ?? "A";
        if (task == "B") { var s2 = new StringBuilder(); TaskB(s2); Console.Write(s2.ToString()); System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_B_out.txt"), s2.ToString()); return 0; }

        int[] Ns = { 1, 2, 3, 4, 5, 6, 8, 10 };
        int[] iterSet = { 10, 20, 40, 100 };
        float mass = 0.02f;
        var sb = new StringBuilder();
        sb.AppendLine("==================== タスクA: 全DOFロック直列チェーン ====================");
        sb.AppendLine("kinematic固定点に N 剛体を全DOFロック6DOFで直列。重力-98 dt=1/30 subs=2 300step。");
        sb.AppendLine($"質量={mass}(均一) segment={Seg} box半径={BoxHalf}。★理論期待値=全条件で maxDrift=0, maxSpeed=0");
        sb.AppendLine();

        sb.AppendLine("[maxDrift] 初期位置からの最大変位 (0 であるべき):");
        sb.Append("   N\\iters |"); foreach (var it in iterSet) sb.Append($" {it,9} |"); sb.AppendLine();
        foreach (var n in Ns) { sb.Append($"   {n,7} |"); foreach (var it in iterSet) { var (d, _) = Run(n, it, 2, -1f, mass, false); sb.Append($" {d,9:F4} |"); } sb.AppendLine(); }
        sb.AppendLine();

        sb.AppendLine("[maxSpeed] 期間中の最大速度 (0 であるべき):");
        sb.Append("   N\\iters |"); foreach (var it in iterSet) sb.Append($" {it,9} |"); sb.AppendLine();
        foreach (var n in Ns) { sb.Append($"   {n,7} |"); foreach (var it in iterSet) { var (_, v) = Run(n, it, 2, -1f, mass, false); sb.Append($" {v,9:F3} |"); } sb.AppendLine(); }
        sb.AppendLine();

        sb.AppendLine("[質量分布] 均一 vs 先端ほど軽い (iters=10, subs=2, maxDrift):");
        sb.Append("   質量\\N  |"); foreach (var n in Ns) sb.Append($" {n,6} |"); sb.AppendLine();
        sb.Append("   uniform |"); foreach (var n in Ns) { var (d, _) = Run(n, 10, 2, -1f, mass, false); sb.Append($" {d,6:F3} |"); } sb.AppendLine();
        sb.Append("   tipLight|"); foreach (var n in Ns) { var (d, _) = Run(n, 10, 2, -1f, mass, true); sb.Append($" {d,6:F3} |"); } sb.AppendLine();

        Console.Write(sb.ToString());
        System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_out.txt"), sb.ToString());
        return 0;
    }
}
