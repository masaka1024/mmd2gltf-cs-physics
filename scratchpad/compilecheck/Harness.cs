// ===========================================================================
// Unity 非依存の検証ハーネス (UnityEngine 最小シムでビルドして実行)。
//   - 合成4シナリオ: 自由落下 / 接地 / Joint保持 / バネ振動
//   - IA.pmx スモークテスト (実PMXの回帰検出)
//   - 静止押し出しの回帰テスト ((a)->(b)是正の退行番人)
// いずれか失敗で終了コード非0。IA.pmx が無い環境では該当項目をスキップし pass 扱い。
//
// 実行例:
//   dotnet run -c Release
//   IA.pmx のパスは環境変数 MMD_TEST_PMX で指定 (未指定なら既定パスを試し、無ければスキップ)。
// ===========================================================================
using System;
using System.IO;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

static class Harness
{
    static int _fails = 0;
    // 計測用トグル (既定OFF=挙動不変)。env JSPLIT=1 でジョイントの split-impulse を全ワールドに適用。
    static readonly bool JSplit = Environment.GetEnvironmentVariable("JSPLIT") == "1";
    static readonly bool WarmS = Environment.GetEnvironmentVariable("WARMSTART") == "1";
    static readonly bool WarmA = Environment.GetEnvironmentVariable("WARMSTART_ANG") == "1";
    static readonly bool WarmOff = Environment.GetEnvironmentVariable("WARM_OFF") == "1";
    static readonly bool JointsFirst = Environment.GetEnvironmentVariable("JOINTS_FIRST") == "1";
    static PhysicsWorld Cfg(PhysicsWorld w) { if (float.TryParse(Environment.GetEnvironmentVariable("WARMFAC"), out var wf)) Joint.WarmStartFactor = wf; w.UseJointSplitImpulse = JSplit; if (WarmOff) { w.UseJointWarmStart = false; w.UseJointWarmStartAngular = false; } if (JointsFirst) w.SolveJointsFirst = true; return w; }

    static int Main()
    {
        Console.WriteLine("=== Unity Bullet 物理 検証ハーネス ===");
        // 合成シナリオは 60Hz・2サブを明示 (エンジン既定=30Hz・1サブとは別に、
        // 既存シナリオの軌道を従来基準のまま検証するため)。
        FreeFall();
        GroundRest();
        JointHold();
        SpringPendulum();

        SmokeTestIaPmx();
        RegressionStaticPush();
        KinematicInterpRegression();

        Console.WriteLine(_fails == 0 ? "\n=== 全 PASS ===" : $"\n=== {_fails} 件 FAIL ===");
        return _fails == 0 ? 0 : 1;
    }

    // ---- 合成4シナリオ (60Hz・2サブ明示) ----
    static PhysicsWorld World60() =>
        Cfg(new PhysicsWorld { Gravity = new Vec3(0, -9.8f, 0), SubSteps = 2, FixedTimeStep = 1f / 60f, SolverIterations = 10 });

    static RigidBody Dyn(CollisionShape s, float m, Vec3 p)
    {
        var b = new RigidBody(s) { Mode = PhysicsMode.Dynamic, Group = 0, CollisionMask = 0xFFFF };
        b.SetMassProps(m); b.WorldTransform = new RigidTransform(Quat.Identity, p); return b;
    }
    static RigidBody Kin(CollisionShape s, Vec3 p)
    {
        var b = new RigidBody(s) { Mode = PhysicsMode.BoneFollow, Group = 0, CollisionMask = 0xFFFF };
        b.WorldTransform = new RigidTransform(Quat.Identity, p); b.SetMassProps(0f); return b;
    }

    static void FreeFall()
    {
        var w = World60();
        var b = Dyn(new SphereShape(0.5f), 1, new Vec3(0, 10, 0)); w.AddBody(b);
        for (int i = 0; i < 60; i++) w.StepSimulation(1f / 60f);
        Check("FreeFall (1s後 y≈5.1)", InRange(b.WorldTransform.Origin.y, 4f, 6f), $"y={b.WorldTransform.Origin.y:F3}");
    }

    static void GroundRest()
    {
        var w = World60();
        var f = Kin(new BoxShape(new Vec3(20, 1, 20)), new Vec3(0, -1, 0)); w.AddBody(f);
        var b = Dyn(new SphereShape(0.5f), 1, new Vec3(0, 3, 0)); b.Friction = 0.5f; w.AddBody(b);
        for (int i = 0; i < 300; i++) w.StepSimulation(1f / 60f);
        float y = b.WorldTransform.Origin.y;
        // (b)方式では真の接触面 y≈0.50 に静止する。(a)方式はマージン分浮いて ~0.513 になるため、
        // 0.50 近傍の狭い許容で (a) への退行も検出する。
        Check("GroundRest (真接触 y≈0.50)", InRange(y, 0.49f, 0.51f), $"y={y:F4}");
    }

    static void JointHold()
    {
        var w = World60();
        var an = Kin(new BoxShape(new Vec3(0.3f, 0.3f, 0.3f)), new Vec3(0, 5, 0)); w.AddBody(an);
        var h = Dyn(new BoxShape(new Vec3(0.3f, 0.3f, 0.3f)), 1, new Vec3(0, 4, 0)); w.AddBody(h);
        w.AddJoint(Joint.FromPmx(JointType.Generic6Dof, an, h, new RigidTransform(Quat.Identity, new Vec3(0, 4.5f, 0)),
            Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero));
        for (int i = 0; i < 300; i++) w.StepSimulation(1f / 60f);
        Check("JointHold (ロック保持 y≈4.0)", InRange(h.WorldTransform.Origin.y, 3.5f, 4.5f), $"y={h.WorldTransform.Origin.y:F3}");
    }

    static void SpringPendulum()
    {
        var w = World60();
        var an = Kin(new SphereShape(0.2f), new Vec3(0, 5, 0)); w.AddBody(an);
        var bob = Dyn(new SphereShape(0.2f), 1, new Vec3(0, 4, 0)); w.AddBody(bob);
        w.AddJoint(Joint.FromPmx(JointType.Spring6Dof, an, bob, new RigidTransform(Quat.Identity, new Vec3(0, 4.5f, 0)),
            new Vec3(-2, -2, -2), new Vec3(2, 2, 2), new Vec3(1), new Vec3(-1), new Vec3(50, 50, 50), Vec3.Zero));
        float mn = 999f;
        for (int i = 0; i < 600; i++) { w.StepSimulation(1f / 60f); float y = bob.WorldTransform.Origin.y; if (i > 200) mn = Math.Min(mn, y); }
        Check("SpringPendulum (発散せず有限)", !float.IsNaN(mn) && mn > 0f && mn < 5f, $"minY={mn:F3}");
    }

    // ---- IA.pmx スモークテスト ----
    // 閾値は現在の実測に余裕を持たせた値:
    //   maxSpeed: 実測 ~19 (爆発時は数百〜NaN) → 100 (約5×)
    //   stepMax : 実測 ~3-10ms (ハング時は数千〜57000ms) → 50ms (JIT分を除いた定常。約5-15×)
    //   EPA上限hit: 実測 0 (暴走の番人) → 0
    static void SmokeTestIaPmx()
    {
        string pmx = FindPmx();
        if (pmx == null) { Skip("IA.pmx スモークテスト (PMX 未検出)"); return; }

        GjkEpa.EpaIterCapHits = 0; GjkEpa.EpaFaceCapHits = 0;
        var model = PmxReader.LoadFile(pmx);
        var b = PmxPhysicsBuilder.Build(model);
        var w = Cfg(b.World); w.Gravity = new Vec3(0, -98f, 0); // 既定 (30Hz・1サブ) のまま
        var dyn = w.Bodies.Where(r => !r.IsStaticOrKinematic).ToList();

        // 全動的剛体の初期位置を控え、静止(重力のみ)でのドリフト(初期位置からの最大変位)を追う。
        // ※既存スモークは maxSpeed<=100 のみで、髪が bind から大きく垂れる/暴れる不具合
        //   (maxDrift~8) を見逃していた。以後は maxDrift も常設で計測する。
        var initPos = dyn.Select(r => r.WorldTransform.Origin).ToArray();

        bool badNum = false; float maxSpeed = 0f; double stepMaxSteady = 0; float maxDrift = 0f; string driftName = "";
        for (int i = 0; i < 300; i++)
        {
            var sw = Stopwatch.StartNew();
            w.StepSimulation(w.FixedTimeStep);
            sw.Stop();
            if (i >= 5) stepMaxSteady = Math.Max(stepMaxSteady, sw.Elapsed.TotalMilliseconds); // 先頭は JIT 暖機
            for (int k = 0; k < dyn.Count; k++)
            {
                var r = dyn[k];
                var o = r.WorldTransform.Origin; var v = r.LinearVelocity;
                if (IsBad(o.x) || IsBad(o.y) || IsBad(o.z) || IsBad(v.x) || IsBad(v.y) || IsBad(v.z)) badNum = true;
                float sp = v.Length; if (sp > maxSpeed) maxSpeed = sp;
                float dr = (o - initPos[k]).Length; if (dr > maxDrift) { maxDrift = dr; driftName = r.Name; }
            }
        }
        long epaHits = GjkEpa.EpaIterCapHits + GjkEpa.EpaFaceCapHits;
        bool ok = !badNum && maxSpeed <= 100f && epaHits == 0 && stepMaxSteady <= 50.0;
        Check("IA.pmx スモーク (NaN/爆発/EPA暴走/遅延なし)", ok,
            $"NaN/Inf={badNum} maxSpeed={maxSpeed:F1}(<=100) EPAhit={epaHits}(=0) stepMax={stepMaxSteady:F2}ms(<=50)");
        // 静止ドリフトの2段判定 (2026-08-09, 本家突合で 7.95=仕様と確定後に格上げ)。
        // 本家の髪は静区間でも最大12.89動く(FK-rest基準)。正当な自由スイングは通し、真の爆散だけ赤にする。
        //   FAIL: maxDrift>=15.0 (本家静区間最大12.89+余裕)。真の爆散(旧20+〜NaN)を捕捉。
        //   WARN: maxDrift>=10.0 で値を明示 (8→12 のじわ悪化の兆しを拾う帯。合否には非影響)。
        // 現状 warm(0.85)=~7.95 は両方クリア (warm無効=WARM_OFF で~8.17, これもクリア)。
        Check("IA.pmx 静止ドリフト (爆散番人, <15)", maxDrift < 15.0f,
            $"maxDrift={maxDrift:F2} (最大: {driftName}) / FAIL>=15 WARN>=10 / 本家静区間最大12.89=仕様");
        if (maxDrift >= 10.0f && maxDrift < 15.0f)
            Note("IA.pmx 静止ドリフト WARN", $"maxDrift={maxDrift:F2} >=10 じわ悪化の兆し (合否には非影響)");
    }

    // ---- 静止押し出しの回帰テスト ----
    // 重力0・1ステップで「Distance>0 (非貫入) かつ 法線インパルス≠0」の接触が 0 件であること。
    // (a)方式だと非貫入接触に押し出しインパルスが乗る。(b)是正の退行番人。
    static void RegressionStaticPush()
    {
        string pmx = FindPmx();
        if (pmx == null) { Skip("静止押し出し回帰 (PMX 未検出)"); return; }

        var model = PmxReader.LoadFile(pmx);
        var b = PmxPhysicsBuilder.Build(model);
        var w = Cfg(b.World); w.Gravity = Vec3.Zero; w.SubSteps = 1; w.FixedTimeStep = 1f / 30f;
        var dump = new List<(string a, string b, float dist, float ni)>();
        w.DebugContacts = dump;
        w.StepSimulation(w.FixedTimeStep);

        int pushed = dump.Count(x => x.dist > 0f && Math.Abs(x.ni) > 1e-8f);
        var worst = dump.Where(x => x.dist > 0f && Math.Abs(x.ni) > 1e-8f)
                        .OrderByDescending(x => Math.Abs(x.ni)).Take(3).ToList();
        string detail = worst.Count == 0 ? "" : "  例:" + string.Join(", ", worst.Select(x => $"{x.a}×{x.b}(d={x.dist:F4},ni={x.ni:F4})"));
        Check("静止押し出し回帰 (非貫入インパルス0件)", pushed == 0, $"件数={pushed}{detail}");
    }

    // ---- キネマティック補間の経路回帰 ----
    // 1体を1フレーム(1/30)で x:0→1 へ動かすと、実効刻みの分割方法に依らず
    // フレーム内を等速移動し、残存速度は 30.0 になるべき (30fps入力の正しい細分)。
    // 修正前は FTS=1/60/1/120 の accumulator 経路で 0.0 になっていた (即ジャンプ→停止)。
    static void KinematicInterpRegression()
    {
        var cfgs = new (float fts, int sub)[]
        {
            (1f / 30f, 1), (1f / 30f, 2), (1f / 30f, 4), (1f / 60f, 1), (1f / 120f, 1),
        };
        foreach (var (fts, sub) in cfgs)
        {
            var w = Cfg(new PhysicsWorld { Gravity = Vec3.Zero, SolverIterations = 10, FixedTimeStep = fts, SubSteps = sub });
            var k = new RigidBody(new BoxShape(new Vec3(0.5f, 0.5f, 0.5f))) { Mode = PhysicsMode.BoneFollow };
            k.SetMassProps(0f); k.WorldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); w.AddBody(k);
            k.KinematicTarget = new RigidTransform(Quat.Identity, new Vec3(1, 0, 0));
            w.StepSimulation(1f / 30f);
            float vx = k.LinearVelocity.x;
            Check($"Kinematic補間 (FTS=1/{1f / fts:F0} Sub{sub}) 残存velx≈30", Math.Abs(vx - 30f) < 1e-2f, $"velx={vx:F3} x={k.WorldTransform.Origin.x:F3}");
        }
    }

    // ---- PMX 検出 ----
    static string FindPmx()
    {
        var env = Environment.GetEnvironmentVariable("MMD_TEST_PMX");
        if (!string.IsNullOrEmpty(env) && File.Exists(env)) return env;
        // 既定候補 (存在すれば使用)。
        string def = TestData.PmxPath();
        return File.Exists(def) ? def : null;
    }

    // ---- ユーティリティ ----
    static bool IsBad(float f) => float.IsNaN(f) || float.IsInfinity(f);
    static bool InRange(float v, float lo, float hi) => !float.IsNaN(v) && v > lo && v < hi;

    static void Check(string name, bool ok, string detail)
    {
        Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {name}  ({detail})");
        if (!ok) _fails++;
    }
    static void Skip(string name) => Console.WriteLine($"  [SKIP] {name}");
    // 既知の問題を常設で計測・追跡するための情報項目 (合否には影響しない)。
    static void Note(string name, string detail) => Console.WriteLine($"  [NOTE] {name}  ({detail})");
}
