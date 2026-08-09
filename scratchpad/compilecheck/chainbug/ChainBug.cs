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
        if (System.Environment.GetEnvironmentVariable("WARMSTART_ANG") == "1") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
        if (System.Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
        if (float.TryParse(System.Environment.GetEnvironmentVariable("WARMFAC"), out var _wf)) BulletPhysics.Joint.WarmStartFactor = _wf;
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
        if (System.Environment.GetEnvironmentVariable("WARMSTART_ANG") == "1") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        if (System.Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
        if (System.Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
        if (float.TryParse(System.Environment.GetEnvironmentVariable("WARMFAC"), out var _wf)) BulletPhysics.Joint.WarmStartFactor = _wf;
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
        if (int.TryParse(Environment.GetEnvironmentVariable("LEVER"), out var _lv)) Joint.LinearLeverMode = _lv; // 線形レバーアーム 0=従来/1=Bullet非offset/2=Bullet offset
        var task = Environment.GetEnvironmentVariable("TASK") ?? "A";
        if (task == "EULER")
        {
            // Joint.ToEulerXYZ と Qx*Qy*Qz 再構成が逆変換か検算 (合成角で規約ズレを検出)。
            Quat QA(int ax, float a)
            {
                float s2 = (float)Math.Sin(a / 2), c2 = (float)Math.Cos(a / 2);
                return ax == 0 ? new Quat(s2, 0, 0, c2) : ax == 1 ? new Quat(0, s2, 0, c2) : new Quat(0, 0, s2, c2);
            }
            float D2R = 0.0174533f;
            var tests = new (float x, float y, float z)[] { (40, 30, 0), (60, 0, 20), (-80, 25, 10), (100, -40, 30) };
            foreach (var (x, y, z) in tests)
            {
                var q = QA(0, x * D2R) * QA(1, y * D2R) * QA(2, z * D2R);
                var e = Joint.ToEulerXYZ(q.Normalized);
                var qr = QA(0, e.x) * QA(1, e.y) * QA(2, e.z);
                float dot = Math.Abs(q.Normalized.x * qr.x + q.Normalized.y * qr.y + q.Normalized.z * qr.z + q.Normalized.w * qr.w);
                float ang = 2f * (float)Math.Acos(Math.Min(1f, dot)) * 57.2958f;
                Console.WriteLine($"  in=({x},{y},{z})deg  ToEulerXYZ=({e.x * 57.2958f:F1},{e.y * 57.2958f:F1},{e.z * 57.2958f:F1})  再構成誤差={ang:F2}deg");
            }
            return 0;
        }
        if (task == "SLIDE2")
        {
            // 本命の最小再現: キネマティック親から垂直に吊るした全ロック鎖を、親を横に往復駆動。
            // 理想(完全剛体)は鎖全体が親に追従=分解指標すべて0。
            // 実スカートの署名: 方向変化角 >> 枠傾き (枠は立ったまま位置が横滑り) が出るか、LEVERで減るか。
            int n = 10; float seg = 1.0f;
            var world = new PhysicsWorld { Gravity = new Vec3(0, -98f, 0), FixedTimeStep = 1f / 30f, SubSteps = 2, SolverIterations = 10 };
            if (Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
            var anc = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.BoneFollow, Name = "anchor" };
            anc.WorldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); anc.SetMassProps(0f); world.AddBody(anc);
            var ch = new List<RigidBody> { anc };
            RigidBody pb = anc;
            for (int i = 1; i <= n; i++)
            {
                var b = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.Dynamic, Name = $"link{i}" };
                b.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, -seg * i, 0)); // 垂直吊り
                b.CollisionMask = 0; b.SetMassProps(1f); world.AddBody(b); ch.Add(b);
                // ジョイント原点=子位置 (PMX標準。中点だと pA+pB=0 の対称性で mode2 と mode0 の差が消える)
                world.AddJoint(Joint.FromPmx(JointType.Generic6Dof, pb, b, new RigidTransform(Quat.Identity, new Vec3(0, -seg * i, 0)),
                    Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero)); // 全DOFロック
                pb = b;
            }
            var bnd = new Vec3[ch.Count];
            for (int i = 0; i < ch.Count; i++) bnd[i] = ch[i].WorldTransform.Origin;
            // 親を X に振幅2, 1Hz で往復駆動 (VMDのターン相当の横加速)。
            var dl = new List<float>(); var sl = new List<float>(); var tl = new List<float>();
            for (int s = 0; s < 300; s++)
            {
                float t = (s + 1) / 30f;
                float x = 2f * (float)Math.Sin(2 * Math.PI * 1.0 * t);
                var tgt = new RigidTransform(Quat.Identity, new Vec3(x, 0, 0));
                anc.KinematicTarget = tgt; anc.KinematicStepTarget = tgt;
                world.StepSimulation(1f / 30f);
                if (s < 30) continue; // 初期過渡は捨てる
                for (int i = 1; i < ch.Count; i++)
                {
                    var r = ch[i].WorldTransform.Origin - ch[i - 1].WorldTransform.Origin;
                    var rb2 = bnd[i] - bnd[i - 1];
                    float den = r.Length * rb2.Length;
                    dl.Add(den > 1e-9f ? (float)(Math.Acos(Math.Max(-1.0, Math.Min(1.0, r.Dot(rb2) / den))) * 180 / Math.PI) : 0);
                    sl.Add(Math.Abs(r.Length - rb2.Length));
                    var q = ch[i].WorldTransform.Rotation;
                    tl.Add((float)(2 * Math.Acos(Math.Min(1.0, Math.Abs(q.w))) * 180 / Math.PI));
                }
            }
            float Md(List<float> v) { var s2 = new List<float>(v); s2.Sort(); return s2[s2.Count / 2]; }
            float Mx2(List<float> v) { float m2 = 0; foreach (var x2 in v) if (x2 > m2) m2 = x2; return m2; }
            bool nan = false; foreach (var b in ch) { var o = b.WorldTransform.Origin; if (float.IsNaN(o.x + o.y + o.z)) nan = true; }
            Console.WriteLine($"[SLIDE2] LEVER={Joint.LinearLeverMode} 垂直吊りN={n} 全ロック 親X往復(±2,1Hz) iters=10 300step NaN={nan} (理想=全て0)");
            Console.WriteLine($"  方向変化角 中央={Md(dl):F2}° 最大={Mx2(dl):F2}°   伸び 中央={Md(sl):F4} 最大={Mx2(sl):F4}   枠傾き 中央={Md(tl):F2}° 最大={Mx2(tl):F2}°");
            Console.WriteLine($"  横滑り度=方向/枠={(Md(tl) > 0.01f ? (Md(dl) / Md(tl)).ToString("F1") : "inf")}  (実スカート自前=6.0 / 純Bullet=1.3)");
            return 0;
        }
        if (task == "SLIDE")
        {
            // 線形行監査#1の判定: 横チェーン(全DOFロック=剛体棒)を重力で垂らし、
            // 「横滑り(方向変化角)」vs「枠傾き(tilt)」vs「伸び」を分解計測。理想は完全剛体=全て0。
            // 実スカートの署名(方向66°>>枠11°)が合成でも出るか、LEVERで消えるかを見る。
            int n = 10; float seg = 1.0f; float lmass = 1f;
            var world = new PhysicsWorld { Gravity = new Vec3(0, -98f, 0), FixedTimeStep = 1f / 30f, SubSteps = 2, SolverIterations = 10 };
            if (Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
            var anchor0 = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.BoneFollow, Name = "anchor" };
            anchor0.WorldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); anchor0.SetMassProps(0f); world.AddBody(anchor0);
            var chain = new List<RigidBody> { anchor0 };
            RigidBody pv = anchor0;
            for (int i = 1; i <= n; i++)
            {
                var b = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.Dynamic, Name = $"link{i}" };
                b.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(seg * i, 0, 0)); // 横(+X)
                b.CollisionMask = 0; b.SetMassProps(lmass); world.AddBody(b); chain.Add(b);
                var jp = new Vec3(seg * i - seg * 0.5f, 0, 0); // ジョイント=中点
                world.AddJoint(Joint.FromPmx(JointType.Generic6Dof, pv, b, new RigidTransform(Quat.Identity, jp),
                    Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero)); // 全DOFロック
                pv = b;
            }
            var bind = new Vec3[chain.Count];
            for (int i = 0; i < chain.Count; i++) bind[i] = chain[i].WorldTransform.Origin;
            for (int s = 0; s < 300; s++) world.StepSimulation(1f / 30f);
            var dirA = new List<float>(); var strA = new List<float>(); var tiltA = new List<float>();
            for (int i = 1; i < chain.Count; i++)
            {
                var r = chain[i].WorldTransform.Origin - chain[i - 1].WorldTransform.Origin;
                var rb = bind[i] - bind[i - 1];
                float den = r.Length * rb.Length;
                dirA.Add(den > 1e-9f ? (float)(Math.Acos(Math.Max(-1.0, Math.Min(1.0, r.Dot(rb) / den))) * 180 / Math.PI) : 0);
                strA.Add(Math.Abs(r.Length - rb.Length));
                var q = chain[i].WorldTransform.Rotation;
                tiltA.Add((float)(2 * Math.Acos(Math.Min(1.0, Math.Abs(q.w))) * 180 / Math.PI));
            }
            float Med(List<float> v) { var s2 = new List<float>(v); s2.Sort(); return s2[s2.Count / 2]; }
            float Mx(List<float> v) { float m2 = 0; foreach (var x in v) if (x > m2) m2 = x; return m2; }
            var tip = chain[n].WorldTransform.Origin - bind[n];
            Console.WriteLine($"[SLIDE] LEVER={Joint.LinearLeverMode} 横鎖N={n} 全ロック iters=10 300step (理想=全て0)");
            Console.WriteLine($"  方向変化角 中央={Med(dirA):F2}° 最大={Mx(dirA):F2}°   伸び 中央={Med(strA):F4} 最大={Mx(strA):F4}   枠傾き 中央={Med(tiltA):F2}° 最大={Mx(tiltA):F2}°");
            Console.WriteLine($"  先端drift=({tip.x:F3},{tip.y:F3},{tip.z:F3}) |{tip.Length:F3}|   横滑り度=方向変化角/枠傾き={(Med(tiltA) > 0.01f ? (Med(dirA) / Med(tiltA)).ToString("F1") : "inf")}");
            return 0;
        }
        if (task == "1") { var s1 = new StringBuilder(); Step1(s1); Console.Write(s1.ToString()); System.IO.File.WriteAllText(System.IO.Path.Combine(AppContext.BaseDirectory, "chainbug_step1_out.txt"), s1.ToString()); return 0; }
        if (task == "MIRROR")
        {
            // ユーザ提案の最小再現: キネマ親の「背面(+Z)」に子を吊るし、6DOF角度リミット±60°で重力。
            // 子が背面(+Z)に留まれば正常、正面(-Z)へ回り込めば鏡像バグ。PMX/Unity不要。
            var s = new StringBuilder();
            s.AppendLine("== 鏡像 最小再現: 親(0,0,0)kinematic, 子(0,0,1)=背面(+Z), joint frame(0,0,0), 角度±60°ロック並進, 重力-9.8 ==");
            foreach (float capH in new[] { 0f }) // 単純点
            {
                var world = new PhysicsWorld { Gravity = new Vec3(0, -9.8f, 0), SubSteps = 2, FixedTimeStep = 1f / 60f, SolverIterations = 20 };
                if (System.Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
                var parent = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.BoneFollow, Name = "parent" };
                parent.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 0, 0)); parent.SetMassProps(0f); world.AddBody(parent);
                var child = new RigidBody(new BoxShape(new Vec3(0.1f, 0.1f, 0.1f))) { Mode = PhysicsMode.Dynamic, Name = "child" };
                child.WorldTransform = new RigidTransform(Quat.Identity, new Vec3(0, 0, 1)); child.CollisionMask = 0; child.SetMassProps(1f); world.AddBody(child);
                float lim = 1.0472f; // ±60°
                world.AddJoint(Joint.FromPmx(JointType.Generic6Dof, parent, child, new RigidTransform(Quat.Identity, new Vec3(0, 0, 0)),
                    Vec3.Zero, Vec3.Zero, new Vec3(-lim, -lim, -lim), new Vec3(lim, lim, lim), Vec3.Zero, Vec3.Zero));
                var init = child.WorldTransform.Origin;
                for (int st = 0; st < 600; st++) world.StepSimulation(1f / 60f);
                var fin = child.WorldTransform.Origin; var d = fin - init;
                s.AppendLine($"  子 初期=(0,0,1)背面 → 最終=({fin.x:F3},{fin.y:F3},{fin.z:F3})  変位=({d.x:F3},{d.y:F3},{d.z:F3})");
                s.AppendLine($"  ★z>0(背面維持)=正常 / z<0(正面へ回り込み)=鏡像バグ。 最終z={fin.z:F3} → {(fin.z > 0 ? "背面維持(正常)" : "正面へ回り込み(鏡像!)")}");
            }
            Console.Write(s.ToString()); return 0;
        }
        if (task == "3")
        {
            // タスク3: 向き × iters, 実髪形状capsule(m1,seg2,lever1)。warm等は env で切替(既定OFF)。
            var s3 = new StringBuilder();
            s3.AppendLine("== タスク3: 向き×iters (実髪capsule 0.4/2.0, mass1, lever1) maxDrift ==");
            (string nm, Vec3 dir)[] dirs = { ("斜め45", new Vec3(1, -1, 0)), ("実髪比率", new Vec3(0.5f, -1f, 0.6f)), ("水平", new Vec3(1, 0, 0)), ("真下(参考)", new Vec3(0, -1, 0)) };
            int[] its = { 10, 20, 40, 100 };
            s3.Append("  向き\\iters |"); foreach (var it in its) s3.Append($" {it,8} |"); s3.AppendLine();
            foreach (var (nm, dir) in dirs)
            {
                s3.Append($"  {nm,-9}|");
                foreach (var it in its) { var (d, _) = RunEx(6, it, 1.0f, 2.0f, 1, 0.4f, 2.0f, 1.0f, false, dir); s3.Append($" {d,8:F4} |"); }
                s3.AppendLine();
            }
            Console.Write(s3.ToString()); return 0;
        }
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
