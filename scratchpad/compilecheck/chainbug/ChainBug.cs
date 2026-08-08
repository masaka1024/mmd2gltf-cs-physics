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

    // ★落とし穴(2026-08-09): 真下向きチェーンは重力がチェーン軸と平行のため角度DOFが
    //   励起されない退化配置。質量/慣性/レバー/長さを変えても maxDrift は不変になる。
    //   最小再現で桁を合わせるには斜め/水平の荷重条件(重力トルクが立つ向き)が必要。詳細=Step1。
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

    static (float maxDrift, float maxSpeed) Run(int n, int iters, int subs, float beta, float mass, bool tipLighter, string order = "root2leaf")
    {
        var (world, dyn) = BuildChain(n, mass, tipLighter);
        world.Gravity = new Vec3(0, -98f, 0);
        world.FixedTimeStep = 1f / 30f;
        world.SubSteps = subs;
        world.SolverIterations = iters;
        if (System.Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
        if (beta >= 0f) foreach (var j in world.Joints) j.Beta = beta;

        // ジョイント求解順序の切替 (既定=construction=root→leaf)。エンジンは無改変で List を並べ替えるだけ。
        if (order == "leaf2root") world.Joints.Reverse();
        else if (order == "shuffle")
        {
            var rng = new Random(12345);
            for (int i = world.Joints.Count - 1; i > 0; i--) { int k = rng.Next(i + 1); (world.Joints[i], world.Joints[k]) = (world.Joints[k], world.Joints[i]); }
        }

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

    // タスクC: ジョイント求解順序の影響。root→leaf / leaf→root / shuffle を比較。
    static void TaskC(StringBuilder sb)
    {
        float mass = 0.02f;
        int[] Ns = { 4, 6, 8, 10 };
        sb.AppendLine("==================== タスクC: ジョイント求解順序の影響(合成チェーン) ====================");
        sb.AppendLine("iters=10 subs=2。root2leaf=根→葉(as-built) / leaf2root=葉→根(reverse) / shuffle=任意順(seed固定)");
        sb.AppendLine("root→leaf で大きく改善するなら『インパルスの伝播不足』が確定。");
        sb.Append("   order\\N  |"); foreach (var n in Ns) sb.Append($" {n,8} |"); sb.AppendLine();
        foreach (var ord in new[] { "root2leaf", "leaf2root", "shuffle" })
        {
            sb.Append($"   {ord,-9}|"); foreach (var n in Ns) { var (d, _) = Run(n, 10, 2, -1f, mass, false, ord); sb.Append($" {d,8:F4} |"); } sb.AppendLine();
        }
    }

    // ステップ1: 合成チェーンを実際の髪剛体パラメータへ1つずつ近づける。
    // shapeMode 0=box(0.1) / 1=capsule(capR,capH)。leverFromChild=joint原点→子剛体中心の距離。
    static (float drift, float speed) RunEx(int n, int iters, float mass, float seg,
        int shapeMode, float capR, float capH, float leverFromChild, bool tipLighter, Vec3 dir = default)
    {
        if (dir.x == 0 && dir.y == 0 && dir.z == 0) dir = new Vec3(0, -1, 0); // 既定=真下
        float dl = dir.Length; dir = new Vec3(dir.x / dl, dir.y / dl, dir.z / dl);
        var world = new PhysicsWorld();
        CollisionShape MakeShape() => shapeMode == 1 ? new CapsuleShape(capR, capH) : new BoxShape(new Vec3(BoxHalf, BoxHalf, BoxHalf));

        var anchor = new RigidBody(MakeShape()) { Mode = PhysicsMode.BoneFollow, Name = "anchor" };
        anchor.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 0, 0));
        anchor.SetMassProps(0f);
        world.AddBody(anchor);

        var dyn = new List<RigidBody>();
        RigidBody prev = anchor;
        for (int i = 1; i <= n; i++)
        {
            float m = tipLighter ? mass * (float)(n - i + 1) / n : mass;
            var b = new RigidBody(MakeShape()) { Mode = PhysicsMode.Dynamic, Name = $"link{i}" };
            b.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(dir.x * seg * i, dir.y * seg * i, dir.z * seg * i));
            b.CollisionMask = 0;
            b.SetMassProps(m);
            world.AddBody(b);
            dyn.Add(b);

            // joint 原点を子剛体中心から親方向へ leverFromChild だけずらす(=レバーアーム)。全DOFロック。
            var c = b.WorldTransform.Origin;
            var jp = new Vec3(c.x - dir.x * leverFromChild, c.y - dir.y * leverFromChild, c.z - dir.z * leverFromChild);
            var frame = new RigidTransform(Quat.Identity, jp);
            world.AddJoint(Joint.FromPmx(JointType.Generic6Dof, prev, b, frame,
                Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero));
            prev = b;
        }

        world.Gravity = new Vec3(0, -98f, 0);
        world.FixedTimeStep = 1f / 30f;
        world.SubSteps = 2;
        world.SolverIterations = iters;
        if (System.Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
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

    static void Step1(StringBuilder sb)
    {
        int N = 6; int it = 10;
        sb.AppendLine("==================== ステップ1: 合成→実髪パラメータ 桁の答え合わせ ====================");
        sb.AppendLine($"N={N} iters={it} subs=2 全DOFロック。理論値=0。1つずつ実髪(髪FR)値へ寄せて maxDrift を見る。");
        sb.AppendLine("実髪FR実測: 質量1.0 / カプセル(半径~0.4 高さ~2.0) / リンク長~2.0 / レバー~1.0");
        sb.AppendLine();
        (string name, float mass, float seg, int sh, float lever)[] cases = {
            ("baseline(box0.1,m0.02,seg0.5,lever0.25)", 0.02f, 0.5f, 0, 0.25f),
            ("+質量 1.0",                               1.0f,  0.5f, 0, 0.25f),
            ("+リンク長 2.0",                           0.02f, 2.0f, 0, 1.0f),
            ("+形状=カプセル(0.4,2.0)",                 0.02f, 0.5f, 1, 0.25f),
            ("+レバー 1.0 (jointを端へ)",               0.02f, 0.5f, 0, 1.0f),
            ("全部=実髪(m1,seg2,cap,lever1)",           1.0f,  2.0f, 1, 1.0f),
        };
        sb.AppendLine("  条件                                    | maxDrift | maxSpeed");
        foreach (var c in cases)
        {
            var (d, v) = RunEx(N, it, c.mass, c.seg, c.sh, 0.4f, 2.0f, c.lever, false);
            sb.AppendLine($"  {c.name,-40}| {d,8:F4} | {v,8:F3}");
        }
        sb.AppendLine("  → 縦チェーンでは質量/慣性/レバー/長さに maxDrift が不変(重力軸と平行でトルクが立たない)。");
        sb.AppendLine();

        // ★向きを変える: 重力に対して斜め/水平にすると角度DOF・レバーアームが励起される。
        sb.AppendLine("[チェーンの向き] N=6 iters=10, box0.1(lever0.25) と 実髪カプセル(lever1.0) で比較:");
        sb.AppendLine("  向き          | box0.1 lever0.25 | capsule(0.4,2.0) lever1.0");
        (string nm, Vec3 dir)[] dirs = {
            ("真下(0,-1,0)",   new Vec3(0,-1,0)),
            ("斜め45(1,-1,0)", new Vec3(1,-1,0)),
            ("斜め(実髪比率)",  new Vec3(0.5f,-1f,0.6f)),
            ("水平(1,0,0)",    new Vec3(1,0,0)),
        };
        foreach (var (nm, dir) in dirs)
        {
            var (db, _) = RunEx(6, 10, 0.02f, 0.5f, 0, 0.4f, 2.0f, 0.25f, false, dir);
            var (dc, _) = RunEx(6, 10, 1.0f, 2.0f, 1, 0.4f, 2.0f, 1.0f, false, dir);
            sb.AppendLine($"  {nm,-13}| {db,16:F4} | {dc,16:F4}");
        }
        sb.AppendLine();

        // 水平カントリーバー(最悪ケース)を iters 掃引 → 収束(未収束の範疇)か非収束(第2バグ)か。
        sb.AppendLine("水平カプセル(実髪param) を iters 掃引:");
        sb.Append("   iters |"); foreach (var it2 in new[] { 10, 20, 40, 100, 400 }) sb.Append($" {it2,8} |"); sb.AppendLine();
        sb.Append("  drift  |"); foreach (var it2 in new[] { 10, 20, 40, 100, 400 }) { var (d, _) = RunEx(6, it2, 1.0f, 2.0f, 1, 0.4f, 2.0f, 1.0f, false, new Vec3(1, 0, 0)); sb.Append($" {d,8:F4} |"); } sb.AppendLine();
    }

    static int Main()
    {
        var task = Environment.GetEnvironmentVariable("TASK") ?? "A";
        if (task == "1") { var s1 = new StringBuilder(); Step1(s1); Console.Write(s1.ToString()); System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_step1_out.txt"), s1.ToString()); return 0; }
        if (task == "B") { var s2 = new StringBuilder(); TaskB(s2); Console.Write(s2.ToString()); System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_B_out.txt"), s2.ToString()); return 0; }
        if (task == "C") { var s3 = new StringBuilder(); TaskC(s3); Console.Write(s3.ToString()); System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_C_out.txt"), s3.ToString()); return 0; }

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
