// ===========================================================================
// NetDump: タスク20 (Bullet 2.75 行レベル突き合わせ) のための最小網エクスポータ。
//
//  タスク15 で「症状が出る最小の網 = 1房の鎖4本 + 横渡し1本 (Joint 5本 / 剛体5個+アンカー)」
//  が確定した。ここではその網を **数値のまま** 取り出し、本物の Bullet 2.75 で
//  同じ網を組み直せるようにする (tools/diagnostics/bulletref/bulletref.cpp)。
//
//  ★PMX を経由しない。剛体の質量・慣性テンソル・ワールド姿勢、ジョイントのフレームと
//    リミットを、当エンジンが実際に使っている値そのままで書き出す。
//    こうしないと「PMX の読み方の差」が「ソルバの差」に化けてしまう。
//
//  出力 (既定は実行ディレクトリ):
//    net.txt                  網の仕様 (bulletref がこれを読む)
//    net_engine_state.csv     毎サブステップの剛体状態 (pos/quat/linvel/angvel)
//    net_engine_rows.csv      毎サブステップのジョイント行 (err/目標速度/構築時の相対速度)
//
//  3段階の突き合わせ (タスク15 の方針) に対応する:
//    段階1 初期一致    … net.txt と、両者の frame 0 の state 行
//    段階2 1サブステップ目の行一致 … net_engine_rows.csv の sub=0
//    段階3 定常 err の分岐点 … 両者の rows/state の時系列
//
//  env:
//    STRAND    房の接頭辞 (既定 髪BR)。この接頭辞で始まる Joint だけ残す
//    FRAMES    トレースするフレーム数 (既定 600 = 静止ダミーVMD と同じ 20秒 @30fps)
//    SUBSTEPS  既定 2      ITERS 既定 10
//    ROWFRAMES 行CSVを出す先頭フレーム数 (既定 20)。以降は末尾窓だけ出す
//    OUTDIR    出力先ディレクトリ (既定 カレント)
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;

static class NetDump
{
    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static float EnvF(string k, float d)
    {
        float v;
        return float.TryParse(Env(k), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : d;
    }

    // float を 9 有効桁で出す。float32 は 9 桁で往復するので、これで値は無損失。
    static string F(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
    static string F3(Vec3 v) => F(v.x) + " " + F(v.y) + " " + F(v.z);
    static string F4(Quat q) => F(q.x) + " " + F(q.y) + " " + F(q.z) + " " + F(q.w);

    public static int Run(PmxPhysicsModel model, string strandDefault)
    {
        string pre = Env("STRAND") ?? strandDefault ?? "髪BR";
        int frames = EnvI("FRAMES", 600);
        int subs = EnvI("SUBSTEPS", 2);
        int iters = EnvI("ITERS", 10);
        int rowFrames = EnvI("ROWFRAMES", 20);
        string outDir = Env("OUTDIR") ?? Directory.GetCurrentDirectory();
        Directory.CreateDirectory(outDir);

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        world.FixedTimeStep = 1f / 60f;
        world.SubSteps = subs;
        world.SolverIterations = iters;
        // ★モードを足したら env 配線とエコーを同時に足すこと (§46 の配線漏れ5件目の教訓)。
        //   AXES / LEVER / ANGBETA 等は RestOsc.ApplyGlobalEnv が既に立てている。ここは残りの3つ。
        // ★タスク46: 求解順序。Bullet は 1反復内で ジョイント→接触 (接触が後勝ち)、
        //   当エンジンの既定は逆。**接触ありで行レベル照合するときは揃えないと原理的に一致しない**
        //   (JOINTORDER と同じ計装上の整合条件)。
        if (Env("JOINTS_FIRST") == "1") world.SolveJointsFirst = true;
        // ★タスク48: 接触 rhs を Bullet 2.75 の一本式へ。
        if (Env("CRHS") == "1") world.ContactRhsBullet = true;
        if (Env("CMAN") == "1") PersistentManifold.BulletManifoldPoints = true;   // タスク51
        // タスク63: 接触の位置補正を切って機序を切り分ける **診断専用** のつまみ。
        //   採用候補ではない (BAUM=0 は駆動で深貫入を 1992 件出して却下済み)。
        {
            float bv;
            if (float.TryParse(Env("BAUM"), System.Globalization.NumberStyles.Float,
                               System.Globalization.CultureInfo.InvariantCulture, out bv) && bv >= 0f)
                world.BaumgarteFactor = bv;
        }
        if (Env("LIMGATE") == "1") Joint.BulletLimitRowGating = true;   // タスク59
        if (Env("SYMDIST") == "1") PersistentManifold.SymmetricBreakingDistance = true;   // タスク67
        // タスク54: マニフォールドの維持統計。warm-start が切れる箇所を分ける。
        PersistentManifold.CollectManifoldStats = Env("MANSTATS") == "1";
        PersistentManifold.ResetManifoldStats();
        // ★★タスク49で判明した配線漏れ: 接触ソルバの4点セットが NetDump に無かった。
        //   無いと摩擦が「法線より先」に解かれ、反復0は上限 μ×Pn が 0 なので **摩擦が必ず0**になる。
        //   タスク46〜48 の行レベル照合はこれ抜きで走っていた (最大整合構成のつもりが違った)。
        if (Env("CPOOL") == "1") world.ContactPoolOrder = true;
        if (Env("NORMFIRST") == "1") world.ContactNormalBeforeFriction = true;
        if (Env("FRICALIGN") == "1") world.FrictionVelocityAligned = true;
        if (Env("FRICMUL") != null) world.FrictionCombineMultiply = Env("FRICMUL") == "1";
        if (Env("CSET") == "1")
        { world.ContactPoolOrder = true; world.FrictionVelocityAligned = true; world.FrictionCombineMultiply = true; }
        { float _sl; if (float.TryParse(Env("SLOP"), NumberStyles.Float, CultureInfo.InvariantCulture, out _sl) && _sl >= 0f) world.PenetrationSlop = _sl; }
        if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (Env("JWARM") == "lin") world.UseJointWarmStart = true;
        else if (Env("JWARM") == "both") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        float jbeta = EnvF("JBETA", -1f);
        if (jbeta >= 0f) foreach (var j in world.Joints) j.Beta = jbeta;

        // ─ 網の間引き ─
        //  既定: MinNet の「1房 + 横渡し」と同じく Joint 名の接頭辞で選ぶ
        //  KEEPBODY=<部分一致>: **剛体名**で選ぶ (タスク28 の最小再現用。特定の1体を保持する
        //                        ジョイントだけを残したい)
        string keepBody = Env("KEEPBODY");
        var kept = new List<Joint>();
        foreach (var j in world.Joints)
        {
            if (j.BodyA == null || j.BodyB == null) continue;
            bool ok = string.IsNullOrEmpty(keepBody)
                    ? j.Name.StartsWith(pre)
                    : (j.BodyA.Name.Contains(keepBody) || j.BodyB.Name.Contains(keepBody));
            if (ok) kept.Add(j);
        }
        world.Joints.Clear();
        world.Joints.AddRange(kept);

        // ★JOINTORDER=<file> (タスク22 段(b)): ジョイントを解く順序を外から与える。
        //   Bullet は btDiscreteDynamicsWorld::solveConstraints で
        //   m_constraints.quickSort(btSortConstraintOnIslandPredicate()) を掛けるため、
        //   拘束配列の順序が定義順から**島ソート順へ入れ替わる**。Gauss-Seidel は順序依存なので、
        //   これを揃えないと段(b) (反復ごとのインパルス系列) は原理的に一致しない。
        //   ファイルは1行1ジョイント名 (bulletref の rowtrace から生成)。**計装上の整合条件であって
        //   出荷の話ではない** (実機では島構成が変われば Bullet 側の順序も変わる)。
        string orderFile = Env("JOINTORDER");
        if (!string.IsNullOrEmpty(orderFile) && File.Exists(orderFile))
        {
            var want = new List<string>();
            foreach (var l in File.ReadAllLines(orderFile))
            { var t = l.Trim(); if (t.Length > 0 && t[0] != '#') want.Add(t); }
            var byName = new Dictionary<string, Joint>();
            foreach (var j in kept) byName[j.Name] = j;
            var reordered = new List<Joint>();
            foreach (var n in want) if (byName.TryGetValue(n, out var j) && !reordered.Contains(j)) reordered.Add(j);
            foreach (var j in kept) if (!reordered.Contains(j)) reordered.Add(j);   // 指定漏れは末尾へ
            kept.Clear(); kept.AddRange(reordered);
            world.Joints.Clear(); world.Joints.AddRange(kept);
            Console.WriteLine("  jointorder: " + string.Join(" -> ", want) + " (" + orderFile + ")");
        }

        var live = new List<RigidBody>();
        var liveSet = new HashSet<RigidBody>();
        foreach (var j in kept)
        {
            if (liveSet.Add(j.BodyA)) live.Add(j.BodyA);
            if (liveSet.Add(j.BodyB)) live.Add(j.BodyB);
        }
        foreach (var b in world.Bodies)
            if (b.Mode == PhysicsMode.Dynamic && !liveSet.Contains(b)) { b.Mode = PhysicsMode.BoneFollow; b.CollisionMask = 0; }
        // ★★ここで必ず張り直すこと (2026-08-22 に踏んだ計測バグ)。
        //   ブロードフェーズの候補ペアは**マスク変更前**に作られているので、
        //   Invalidate しないと **刈ったはずの剛体が接触し続ける**。
        //   実測: スカート網の照合で、net.txt に居ない 右髪２/左髪２ ↔ 頭_1 の接触が
        //   自前側にだけ 2 点出ていた (bulletref には当該剛体が無いので原理的に出ない)。
        //   NOCONTACT の側では既に直していたのに、こちらに残っていた同型のバグ。
        world.InvalidateCollisionPairs();

        bool noContact = Env("NOCONTACT") == "1";

        builder.ApplyKinematicTargets(i => (RigidTransform?)null);
        builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

        // ★CONTACTBODIES=1 (タスク28): live 剛体に接触してくる **静的/キネマティック剛体** を
        //   短いプローブで見つけ出し、書き出し対象に加える。接触編の突き合わせは接触相手が
        //   net.txt に居ないと成立しない (ジョイント編は接触ゼロで良かったが、今回は逆)。
        if (Env("CONTACTBODIES") == "1")
        {
            var probe = new List<(string a, string b, float dist, float ni)>();
            world.DebugContacts = probe;
            var partners = new HashSet<string>();
            var liveNames = new HashSet<string>();
            foreach (var b in live) liveNames.Add(b.Name);
            int pf = EnvI("PROBEFRAMES", 600);
            for (int f = 0; f < pf; f++)
            {
                probe.Clear();
                world.StepSimulation(1f / 60f);
                foreach (var c in probe)
                {
                    if (liveNames.Contains(c.a) && !liveNames.Contains(c.b)) partners.Add(c.b);
                    if (liveNames.Contains(c.b) && !liveNames.Contains(c.a)) partners.Add(c.a);
                }
            }
            world.DebugContacts = null;
            foreach (var b in world.Bodies)
                if (partners.Contains(b.Name) && !liveSet.Contains(b)) { liveSet.Add(b); live.Add(b); }
            Console.WriteLine("  contactbodies: 接触相手 " + partners.Count + " 体を追加 (" +
                              string.Join(" / ", partners) + ")  プローブ " + pf + "F");
            // プローブで動いてしまったので初期姿勢へ戻す
            builder.ApplyKinematicTargets(i => (RigidTransform?)null);
            builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);
            foreach (var b in world.Bodies) { b.LinearVelocity = Vec3.Zero; b.AngularVelocity = Vec3.Zero; }
        }

        // ★EXTRABODIES=<カンマ区切りの剛体名> (タスク40): 接触相手を **名前で固定** する。
        //   CONTACTBODIES のプローブは物理を回して相手を探すので、A/B のフラグを変えると
        //   **見つかる相手が変わり、条件ごとに網の剛体構成がずれる** (実測: モデルA で 63 体 vs 65 体)。
        //   条件間で網が違えば比較そのものが成立しない。全条件の和集合を名前で固定して渡すこと。
        //   余分なキネマティック相手が入っても、当たらなければ挙動には効かない。
        {
            string extra = Env("EXTRABODIES");
            if (!string.IsNullOrEmpty(extra))
            {
                var want = new HashSet<string>(extra.Split(','));
                int added = 0;
                foreach (var b2 in world.Bodies)
                    if (want.Contains(b2.Name) && !liveSet.Contains(b2)) { liveSet.Add(b2); live.Add(b2); added++; }
                Console.WriteLine("  extrabodies: 名前指定で " + added + " 体を追加 (要求 " + want.Count + " 体)");
                world.InvalidateCollisionPairs();
            }
        }

        // ★NOCONTACT=1: 全剛体の衝突マスクを 0 にして接触を完全に消す。
        //   ★CONTACTBODIES のプローブより**後**に適用すること。先に消すとプローブが接触を
        //     1件も見つけられず、網の剛体構成が接触ありの場合と変わってしまう。
        //     構成が変わると INITSTATE の索引 (live の並び) がずれて別の剛体へ状態を移植する。
        //     (2026-08-22 に実際に踏んだ。接触ゼロの速度差が 7.55 と出たのはこの取り違え)
        if (noContact)
        {
            foreach (var b in world.Bodies) b.CollisionMask = 0;
            // ★衝突ペアはキャッシュされている (PhysicsWorld の注記)。プローブで既に構築済みなので
            //   マスクを変えたら必ず無効化する。忘れると NOCONTACT が黙って効かない。
            world.InvalidateCollisionPairs();
        }

        // live の並び順を安定させる (剛体 Index 昇順)。bulletref 側の索引と一致させるため。
        live.Sort((x, y) => x.Index.CompareTo(y.Index));
        var idxOf = new Dictionary<RigidBody, int>();
        for (int i = 0; i < live.Count; i++) idxOf[live[i]] = i;

        // ★INITSTATE=<file> (タスク21): bulletref が --dumpstate で吐いた状態を移植する。
        //   バインド姿勢は err が厳密に 0 なので Bullet 側が行を作らない縮退状態になる。
        //   段(a) を「同じ状態から同じ行を作るか」で問うには、非縮退な姿勢から両側を
        //   **同一の初期状態**で始める必要がある。索引は net.txt の body 番号 (= live の順)。
        string initState = Env("INITSTATE");
        if (!string.IsNullOrEmpty(initState))
        {
            if (!File.Exists(initState)) { Console.WriteLine("★INITSTATE が無い: " + initState); return 1; }
            int nst = 0;
            foreach (var raw in File.ReadAllLines(initState))
            {
                var line = raw.Trim();
                if (line.Length == 0 || line[0] == '#') continue;
                var tok = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (tok.Length < 2 || tok[0] != "state") continue;
                int bi = int.Parse(tok[1], CultureInfo.InvariantCulture);
                if (bi < 0 || bi >= live.Count) continue;
                // key=v [v v ...] を拾う
                var kv = new Dictionary<string, List<float>>();
                string cur = null;
                for (int t = 2; t < tok.Length; t++)
                {
                    int eq = tok[t].IndexOf('=');
                    if (eq >= 0)
                    {
                        cur = tok[t].Substring(0, eq);
                        kv[cur] = new List<float>();
                        var rest = tok[t].Substring(eq + 1);
                        if (rest.Length > 0 && float.TryParse(rest, NumberStyles.Float, CultureInfo.InvariantCulture, out var f0))
                            kv[cur].Add(f0);
                    }
                    else if (cur != null && float.TryParse(tok[t], NumberStyles.Float, CultureInfo.InvariantCulture, out var fv))
                        kv[cur].Add(fv);
                }
                Vec3 V3(string k, Vec3 d) => kv.TryGetValue(k, out var l) && l.Count >= 3 ? new Vec3(l[0], l[1], l[2]) : d;
                Quat Q4(string k, Quat d) => kv.TryGetValue(k, out var l) && l.Count >= 4 ? new Quat(l[0], l[1], l[2], l[3]) : d;
                var b = live[bi];
                b.WorldTransform = new RigidTransform(Q4("quat", b.WorldTransform.Rotation), V3("pos", b.WorldTransform.Origin));
                b.UpdateInertiaWorld();
                b.LinearVelocity = V3("linvel", Vec3.Zero);
                b.AngularVelocity = V3("angvel", Vec3.Zero);
                if (b.IsKinematic) b.KinematicTarget = b.KinematicStepTarget = b.WorldTransform;
                nst++;
            }
            Console.WriteLine("  initstate: " + nst + " 体を " + initState + " から移植");
        }

        Console.WriteLine("netdump : 接頭辞 '" + pre + "' の Joint " + kept.Count + " 本 / 剛体 " + live.Count + " 個");
        // ★実効フラグのエコー (env の値ではなくエンジンから読み戻した値)。タスク11 の構造対策。
        {
            float bmin = float.MaxValue, bmax = float.MinValue;
            foreach (var j in kept) { if (j.Beta < bmin) bmin = j.Beta; if (j.Beta > bmax) bmax = j.Beta; }
            int cross = 0; foreach (var j in kept) if (j.IsCrossTypeJoint) cross++;
            Console.WriteLine("[実効] netdump  FixedTimeStep=1/" + (1f / world.FixedTimeStep).ToString("F2") +
                              "  SubSteps=" + world.SubSteps + "  Iters=" + world.SolverIterations +
                              "  Joint.Beta=" + (bmin == bmax ? F(bmin) : F(bmin) + "~" + F(bmax)) +
                              "  ContactBaumgarte=" + F(world.BaumgarteFactor));
            Console.WriteLine("[実効] netdump  JSplit=" + world.UseJointSplitImpulse +
                              "  JWarm=" + world.UseJointWarmStart + "/" + world.UseJointWarmStartAngular +
                              "  LeverMode=" + Joint.LinearLeverMode +
                              "  MixedAxes=" + Joint.AngularMixedAxes +
                              "  AngConv=" + Joint.BulletAngleConvention +
                              "  SpringMotor=" + Joint.SpringAsMotorRow +
                              "  RotExp=" + PhysicsWorld.BulletRotationIntegration +
                              "  CThresh=" + GjkEpa.BulletContactThreshold +
                              "  CMargin=" + CollisionShape.BulletShapeMargin +
                              "  AngBetaScale=" + F(Joint.AngularBetaScale) +
                              "  MaxCorrVel=" + F(Joint.MaxCorrectionVel) +
                              "  JointsFirst=" + world.SolveJointsFirst +
                              "  CRhsBullet=" + world.ContactRhsBullet +
                              "  CMan=" + PersistentManifold.BulletManifoldPoints +
                              "  ContactBaumgarte=" + world.BaumgarteFactor +
                              "  LimGate=" + Joint.BulletLimitRowGating +
                              "  SymDist=" + PersistentManifold.SymmetricBreakingDistance +
                              "  PoolOrder=" + world.ContactPoolOrder +
                              "  NormalFirst=" + world.ContactNormalBeforeFriction +
                              "  FricAligned=" + world.FrictionVelocityAligned +
                              "  FricMul=" + world.FrictionCombineMultiply +
                              "  Slop=" + world.PenetrationSlop.ToString("G6") +
                              "  NoContact=" + noContact +
                              "  横渡し型=" + cross + "本/鎖型=" + (kept.Count - cross) + "本");
        }

        // ─────────────────────────────────────────────────────────
        //  net.txt : 網の仕様
        // ─────────────────────────────────────────────────────────
// ★剛体 -> BodyOffsetFromBone の索引 (タスク40)。BoneLinks[i] は Bodies[i] に対応する。
        var offMap = new Dictionary<RigidBody, RigidTransform>();
        for (int i = 0; i < builder.Bodies.Count && i < builder.BoneLinks.Count; i++)
            offMap[builder.Bodies[i]] = builder.BoneLinks[i].BodyOffsetFromBone;
        Func<RigidBody, RigidTransform> offOf = rb =>
            offMap.TryGetValue(rb, out var t) ? t : RigidTransform.Identity;

        var s = new StringBuilder();
        s.Append("# bulletref net spec v1  (restosc NETDUMP=1)\n");
        s.Append("# 生成 " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "  PMX=" + TestData.PmxPath() + "\n");
        s.Append("# 数値は G9 (float32 が往復する桁数)。角度リミットはラジアン。\n");
        s.Append("world gravity " + F3(world.Gravity) + "\n");
        s.Append("world dt " + F(world.FixedTimeStep) + " substeps " + world.SubSteps + " iters " + world.SolverIterations + "\n");
        s.Append("world contactBaumgarte " + F(world.BaumgarteFactor) +
                 " slop " + F(world.PenetrationSlop) +
                 " restitutionThreshold " + F(world.RestitutionThreshold) + "\n");
        s.Append("world jsplit " + (world.UseJointSplitImpulse ? 1 : 0) +
                 " split " + (world.UseSplitImpulse ? 1 : 0) +
                 " jwarm " + (world.UseJointWarmStart ? 1 : 0) +
                 " jwarmang " + (world.UseJointWarmStartAngular ? 1 : 0) +
                 " leverMode " + Joint.LinearLeverMode +
                 " mixedAxes " + (Joint.AngularMixedAxes ? 1 : 0) +
                 " maxCorrVel " + F(Joint.MaxCorrectionVel) + "\n");

        foreach (var b in live)
        {
            int shape = (int)b.Shape.Type;
            Vec3 size;
            if (b.Shape is SphereShape sp) size = new Vec3(sp.Radius, 0f, 0f);
            else if (b.Shape is BoxShape bx) size = bx.HalfExtents;
            else if (b.Shape is CapsuleShape cp) size = new Vec3(cp.Radius, cp.Height, 0f);
            else size = Vec3.Zero;

            s.Append("body " + idxOf[b] +
                     " name=" + b.Name.Replace(' ', '_') +
                     " mode=" + (int)b.Mode +
                     " mass=" + F(b.Mass) +
                     " invMass=" + F(b.InverseMass) +
                     " shape=" + shape +
                     " size=" + F3(size) +
                     " margin=" + F(b.Shape.Margin) +
                     " inertia=" + F3(b.LocalInertiaDiag) +
                     " pos=" + F3(b.WorldTransform.Origin) +
                     " quat=" + F4(b.WorldTransform.Rotation) +
                     " linvel=" + F3(b.LinearVelocity) +
                     " angvel=" + F3(b.AngularVelocity) +
                     " lindamp=" + F(b.LinearDamping) +
                     " angdamp=" + F(b.AngularDamping) +
                     " friction=" + F(b.Friction) +
                     " restitution=" + F(b.Restitution) +
                     // ★タスク40: ボーン姿勢の復元に要る (bonePose = bodyWorld * boneoff⁻¹)。
                     //   bulletref の剛体状態から BoneDp と同一定義の |Δp| を出すため。
                     " boneoff=" + F3(offOf(b).Origin) + " boneoffq=" + F4(offOf(b).Rotation) +
                     " group=" + b.Group +
                     " mask=" + b.CollisionMask + "\n");
        }

        // タスク62: ロック軸の変位が厳密に 0 になる件の切り分け。
        //   Prepare と同じ式 (BodyX.WorldTransform * FrameInX) でアンカーを出し、
        //   bulletref の stage1 が出す anchorA/anchorB/gap と直接比べる。
        if (Env("ANCHORDUMP") == "1")
        {
            int zero = 0;
            var sb2 = new StringBuilder();
            foreach (var jj in kept)
            {
                var wa = jj.BodyA.WorldTransform * jj.FrameInA;
                var wb = jj.BodyB.WorldTransform * jj.FrameInB;
                var d = wb.Origin - wa.Origin;
                if (d.x == 0f && d.y == 0f && d.z == 0f) zero++;
                sb2.Append("joint " + jj.Name + "  anchorA " + F3(wa.Origin) + "  anchorB " + F3(wb.Origin) +
                           "  gap " + F(d.Length) + "  d " + F3(d) + Environment.NewLine);
            }
            Console.WriteLine("  [anchordump] 隙間が厳密に 0 のジョイント " + zero + " / " + kept.Count);
            File.WriteAllText(Path.Combine(outDir, "anchors_engine.txt"), sb2.ToString(), new UTF8Encoding(false));
        }

        for (int k = 0; k < kept.Count; k++)
        {
            var j = kept[k];
            s.Append("joint " + k +
                     " name=" + j.Name.Replace(' ', '_') +
                     " type=" + (int)j.Type +
                     " a=" + idxOf[j.BodyA] +
                     " b=" + idxOf[j.BodyB] +
                     " faPos=" + F3(j.FrameInA.Origin) +
                     " faQuat=" + F4(j.FrameInA.Rotation) +
                     " fbPos=" + F3(j.FrameInB.Origin) +
                     " fbQuat=" + F4(j.FrameInB.Rotation) +
                     " linLo=" + F3(j.LinearLowerLimit) +
                     " linHi=" + F3(j.LinearUpperLimit) +
                     " angLo=" + F3(j.AngularLowerLimit) +
                     " angHi=" + F3(j.AngularUpperLimit) +
                     " spLin=" + F3(j.SpringLinear) +
                     " spAng=" + F3(j.SpringAngular) +
                     " spDamp=" + F(j.SpringDamping) +
                     " beta=" + F(j.Beta) +
                     " cross=" + (j.IsCrossTypeJoint ? 1 : 0) + "\n");
        }
        string netPath = Path.Combine(outDir, "net.txt");
        File.WriteAllText(netPath, s.ToString(), new UTF8Encoding(false));
        Console.WriteLine("  -> " + netPath);

        // ─────────────────────────────────────────────────────────
        //  トレース
        // ─────────────────────────────────────────────────────────
        var st = new StringBuilder();
        st.Append("frame,sub,body,name,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n");
        var rw = new StringBuilder();
        rw.Append("frame,sub,joint,dof,angular,err,targetVel,relVel\n");

        var log = new List<(string joint, int dof, bool angular, float err, float targetVel, float relVel)>();

        // ═══ タスク21: 行レベル突き合わせ用の共通スキーマ CSV ═══
        //  ROWTRACE=1 で有効。**SUBSTEPS=1 必須** (dt=1/60 × 1サブステップ = 60サブステップ/秒 =
        //  PMXエディタの 1/30×2 と同じ刻み)。1回の StepSimulation = 1サブステップになるので、
        //  ステップ前に読んだ InverseInertiaWorld が「そのサブステップで使われた値」と一致する
        //  (姿勢は IntegratePositions の最後でしか変わらないため)。実効質量を外から厳密に再現できる。
        //  エンジンは無改変: 既存の読み取り専用フック DebugRows / DebugRowsSolved だけを使う。
        bool rowTrace = Env("ROWTRACE") == "1";
        var solved = new List<(string joint, int dof, bool angular,
            Vec3 axis, Vec3 relA, Vec3 relB, string bodyA, string bodyB,
            float accumulated, float targetVel, float relVelAfter)>();
        var tr = new StringBuilder();
        // ★角度3軸の生の値。行が立たない (リミット内) 軸も含めて全部出す。
        //   Bullet 側の getAngle(k) と直接比べるため。角度行の食い違いはここが源になりうる。
        var angLog = new List<(string joint, int dof, int state, float cur, float err)>();
        var ang = new StringBuilder();
        if (rowTrace)
        {
            ang.Append("frame,joint,dof,state,cur,err\n");
            if (subs != 1)
            {
                Console.WriteLine("★ROWTRACE=1 は SUBSTEPS=1 が必須 (実効質量の再現に必要)。中止する。");
                return 1;
            }
            tr.Append("frame,substep,iter,joint,dof,angular,axisX,axisY,axisZ,err,bias,targetVel,"
                    + "lower,upper,effMass,appliedImpulse,dImpulse,clamped,relVelBefore,relVelAfter\n");
        }
        var jointByName = new Dictionary<string, Joint>();
        foreach (var j in kept) jointByName[j.Name] = j;
        var invIA = new Dictionary<string, Matrix3x3>();
        var invMa = new Dictionary<string, float>();
        // ★接触が1つでもあると「ジョイントのソルバ構造だけの差」という主張が崩れるので、必ず数える。
        //   bulletref 側も manifold 数を出す。両方 0 であることが突き合わせの前提。
        var dbgContacts = new List<(string a, string b, float dist, float ni)>();
        world.DebugContacts = dbgContacts;
        // ★CONTACTTRACE=1 (タスク28 段(a)): 接触点の生成を bulletref と同じスキーマで出す。
        var conRows = new List<(int sub, string a, string b, Vec3 pA, Vec3 pB, Vec3 n,
            float dist, float normalBias, float pushBias, float friction,
            float normalMass, float warmNormal, float warmT1, float warmT2)>();
        bool conTrace = Env("CONTACTCSV") == "1";
        var con = new StringBuilder();
        // ★タスク47: 接触行の反復単位トレース。CONTACTCSV=1 のとき一緒に出す。
        var conIter = new List<(int iter, string a, string b, int pt,
            float ni, float t1, float t2, bool nClamp, bool tClamp, float relN,
            float fric, float maxT, float relT, float tanMass,
            Vec3 t1dir, Vec3 nDir)>();
        var conI = new StringBuilder();
        if (conTrace)
        {
            world.DebugContactRows = conRows;
            world.DebugContactIterRows = conIter;
            conI.Append("frame,iter,bodyA,bodyB,pt,ni,t1,t2,nClamp,tClamp,relN,fric,maxT,relT,tanMass,"
                      + "t1x,t1y,t1z,nx,ny,nz\n");
            con.Append("frame,substep,pt,bodyA,bodyB,pAx,pAy,pAz,pBx,pBy,pBz,nx,ny,nz,dist,"
                     + "normalBias,pushBias,friction,normalMass,warmNormal,warmT1,warmT2\n");
        }
        long contactTotal = 0;
        var contactPairs = new Dictionary<string, int>();
        int lateFrom = frames - frames / 3;
        var jordIdx = new Dictionary<string, int>();
        for (int k = 0; k < kept.Count; k++) jordIdx[kept[k].Name] = k;

        // world.StepSimulation はサブステップ境界へのフックが無いので、
        // 「1フレーム = SubSteps 個の Step」とは分けて、フレーム単位で状態を採り、
        // 行は DebugRows がフレーム内の全サブステップぶん積むのでそれを sub 番号付きで割る。
        // ★タスク69: 同一軌跡リプレイ。REPLAY=<net_*_state.csv> を指定すると、
        //   毎フレームの頭で全剛体の姿勢と速度を CSV の値で**上書き**してから 1 ステップ回す。
        //   ソルバの出した姿勢は次のフレームの頭で捨てられるので、
        //   マニフォールドの営み (Refresh / Detect / AddPoint) だけが与えられた軌跡の上で進む。
        //   「一致率が低いのは判定の差か、それとも自分の残留運動の症状か」を
        //   軌跡を揃えて切り分けるための診断専用モード。
        Dictionary<int, Dictionary<string, float[]>> replay = null;
        if (!string.IsNullOrEmpty(Env("REPLAY")))
        {
            replay = new Dictionary<int, Dictionary<string, float[]>>();
            string[] hdr = null;
            foreach (var line in File.ReadLines(Env("REPLAY")))
            {
                var c = line.Split(',');
                if (hdr == null) { hdr = c; continue; }
                int ix(string k) { for (int i = 0; i < hdr.Length; i++) if (hdr[i] == k) return i; return -1; }
                if (c[ix("sub")] != "-1") continue;
                int fr = int.Parse(c[ix("frame")], CultureInfo.InvariantCulture);
                if (!replay.TryGetValue(fr, out var d)) { d = new Dictionary<string, float[]>(); replay[fr] = d; }
                float g(string k) => float.Parse(c[ix(k)], NumberStyles.Float, CultureInfo.InvariantCulture);
                d[c[ix("name")]] = new[] { g("px"), g("py"), g("pz"), g("qx"), g("qy"), g("qz"), g("qw"),
                                           g("vx"), g("vy"), g("vz"), g("wx"), g("wy"), g("wz") };
            }
            Console.WriteLine("  [replay] " + replay.Count + " フレームの姿勢列を読み込んだ (" + Env("REPLAY") + ")");
        }

        for (int f = 0; f < frames; f++)
        {
            if (replay != null && replay.TryGetValue(f, out var pose))
            {
                foreach (var b2 in world.Bodies)
                {
                    if (!pose.TryGetValue(b2.Name, out var v)) continue;
                    b2.WorldTransform = new RigidTransform(
                        new Quat(v[3], v[4], v[5], v[6]), new Vec3(v[0], v[1], v[2]));
                    b2.LinearVelocity = new Vec3(v[7], v[8], v[9]);
                    b2.AngularVelocity = new Vec3(v[10], v[11], v[12]);
                    b2.UpdateInertiaWorld();   // 姿勢を差し替えたら派生量も更新する
                }
            }
            bool wantRows = f < rowFrames || f >= lateFrom;
            if (wantRows) { log.Clear(); Joint.DebugRows = log; }
            if (rowTrace && wantRows)
            {
                // ★ステップ前に採る。IntegratePositions の最後でしか姿勢は変わらないので、
                //   ここで読んだ逆慣性 = このサブステップで行の実効質量に使われた値。
                invIA.Clear(); invMa.Clear();
                foreach (var b in world.Bodies) { invIA[b.Name] = b.InverseInertiaWorld; invMa[b.Name] = b.InverseMass; }
                solved.Clear(); Joint.DebugRowsSolved = solved;
                angLog.Clear(); Joint.DebugAngularRows = angLog;
            }
            dbgContacts.Clear();
            world.StepSimulation(1f / 60f);
            Joint.DebugRows = null;
            Joint.DebugRowsSolved = null;
            Joint.DebugAngularRows = null;
            contactTotal += dbgContacts.Count;
            if (conTrace)
            {
                foreach (var r in conIter)
                    conI.Append(f).Append(',').Append(r.iter).Append(',')
                        .Append(r.a.Replace(',', '_')).Append(',').Append(r.b.Replace(',', '_')).Append(',')
                        .Append(r.pt).Append(',')
                        .Append(F(r.ni)).Append(',').Append(F(r.t1)).Append(',').Append(F(r.t2)).Append(',')
                        .Append(r.nClamp ? 1 : 0).Append(',').Append(r.tClamp ? 1 : 0).Append(',')
                        .Append(F(r.relN)).Append(',')
                        .Append(F(r.fric)).Append(',').Append(F(r.maxT)).Append(',')
                        .Append(F(r.relT)).Append(',').Append(F(r.tanMass)).Append(',')
                        .Append(F(r.t1dir.x)).Append(',').Append(F(r.t1dir.y)).Append(',').Append(F(r.t1dir.z)).Append(',')
                        .Append(F(r.nDir.x)).Append(',').Append(F(r.nDir.y)).Append(',').Append(F(r.nDir.z)).Append('\n');
                conIter.Clear();
                int ci = 0;
                foreach (var r in conRows)
                {
                    // ★substep 列は **実際のサブステップ**。以前ここへ点index を書いており、
                    //   自前だけ全サブステップぶんが混ざって点数が約2倍に見えていた (タスク50)。
                    con.Append(f).Append(',').Append(r.sub).Append(',').Append(ci).Append(',')
                       .Append(r.a.Replace(',', '_')).Append(',').Append(r.b.Replace(',', '_')).Append(',')
                       .Append(F(r.pA.x)).Append(',').Append(F(r.pA.y)).Append(',').Append(F(r.pA.z)).Append(',')
                       .Append(F(r.pB.x)).Append(',').Append(F(r.pB.y)).Append(',').Append(F(r.pB.z)).Append(',')
                       .Append(F(r.n.x)).Append(',').Append(F(r.n.y)).Append(',').Append(F(r.n.z)).Append(',')
                       .Append(F(r.dist)).Append(',').Append(F(r.normalBias)).Append(',').Append(F(r.pushBias)).Append(',')
                       .Append(F(r.friction)).Append(',').Append(F(r.normalMass)).Append(',')
                       .Append(F(r.warmNormal)).Append(',').Append(F(r.warmT1)).Append(',').Append(F(r.warmT2)).Append('\n');
                    ci++;
                }
                conRows.Clear();
            }
            foreach (var c in dbgContacts)
            {
                string key = string.CompareOrdinal(c.a, c.b) <= 0 ? c.a + " / " + c.b : c.b + " / " + c.a;
                contactPairs.TryGetValue(key, out int n); contactPairs[key] = n + 1;
            }

            // 状態はフレーム末 (= 最終サブステップ後) の値。sub=-1 で「フレーム末」を表す。
            for (int i = 0; i < live.Count; i++)
            {
                var b = live[i];
                st.Append(f).Append(",-1,").Append(i).Append(',').Append(b.Name.Replace(',', '_')).Append(',')
                  .Append(F3(b.WorldTransform.Origin).Replace(' ', ',')).Append(',')
                  .Append(F4(b.WorldTransform.Rotation).Replace(' ', ',')).Append(',')
                  .Append(F3(b.LinearVelocity).Replace(' ', ',')).Append(',')
                  .Append(F3(b.AngularVelocity).Replace(' ', ',')).Append('\n');
            }

            if (!wantRows) continue;
            // DebugRows はサブステップ順に積まれる。1サブステップぶんの行数は不等式拘束の
            // 出入りで可変なので、行数では区切れない。Prepare は world.Joints の順に呼ばれるので、
            // **ジョイント番号が減った所**が次のサブステップの先頭になる (同一ジョイントが
            // 複数行を出すので「先頭ジョイントの再登場」では区切れない)。
            int sub = 0, lastJ = -1;
            foreach (var r in log)
            {
                int ji;
                if (!jordIdx.TryGetValue(r.joint, out ji)) ji = lastJ;
                if (ji < lastJ) sub++;
                lastJ = ji;
                rw.Append(f).Append(',').Append(sub).Append(',').Append(r.joint.Replace(',', '_')).Append(',')
                  .Append(r.dof).Append(',').Append(r.angular ? 1 : 0).Append(',')
                  .Append(F(r.err)).Append(',').Append(F(r.targetVel)).Append(',').Append(F(r.relVel)).Append('\n');
            }

            if (!rowTrace) continue;
            foreach (var a in angLog)
                ang.Append(f).Append(',').Append(a.joint.Replace(',', '_')).Append(',')
                   .Append(a.dof).Append(',').Append(a.state).Append(',')
                   .Append(F(a.cur)).Append(',').Append(F(a.err)).Append('\n');
            // ─ 共通スキーマの行トレース ─
            //  SUBSTEPS=1 なので log = このサブステップの全行 (Prepare 順)。
            //  solved は同じ順序が iters 回ぶん積まれている (SolveVelocity が _rows 順に解くため)。
            int nRows = log.Count;
            if (nRows > 0 && solved.Count != nRows * iters)
            {
                Console.WriteLine("★F" + f + ": 行数の辻褄が合わない (prepare=" + nRows +
                                  " solved=" + solved.Count + " iters=" + iters + ")。トレースを中止。");
                return 1;
            }
            for (int k = 0; k < nRows; k++)
            {
                var pr = log[k];
                var s0 = solved[k];                 // 反復0 の記録 (軸・レバーはサブステップ中不変)
                var jj = jointByName[pr.joint];
                float lo = pr.angular ? jj.AngularLowerLimit[pr.dof] : jj.LinearLowerLimit[pr.dof];
                float hi = pr.angular ? jj.AngularUpperLimit[pr.dof] : jj.LinearUpperLimit[pr.dof];
                // 力積の上下限は Constraints.cs と同じ規則: ロック=±1e18 / 下限側=[0,+1e18] / 上限側=[-1e18,0]
                float rl, rh;
                if (lo == hi) { rl = -1e18f; rh = 1e18f; }
                else if (pr.err > 0f) { rl = 0f; rh = 1e18f; }     // err = lo-cur > 0 → 下限を割っている
                else { rl = -1e18f; rh = 0f; }
                // 実効質量: AddLinearRow / AddAngularRow と同じ式を、ステップ前の逆慣性で再現する。
                float kk;
                if (pr.angular)
                {
                    kk = s0.axis.Dot(invIA[s0.bodyA] * s0.axis) + s0.axis.Dot(invIA[s0.bodyB] * s0.axis);
                }
                else
                {
                    var rAxn = Vec3.Cross(s0.relA, s0.axis);
                    var rBxn = Vec3.Cross(s0.relB, s0.axis);
                    kk = invMa[s0.bodyA] + invMa[s0.bodyB]
                       + rAxn.Dot(invIA[s0.bodyA] * rAxn) + rBxn.Dot(invIA[s0.bodyB] * rBxn);
                }
                float eff = kk > 0f ? 1f / kk : 0f;
                string head = f + "," + f + ",";   // SUBSTEPS=1 なので substep == frame
                string tail = "," + pr.joint.Replace(',', '_') + "," + pr.dof + "," + (pr.angular ? 1 : 0) + ","
                            + F(s0.axis.x) + "," + F(s0.axis.y) + "," + F(s0.axis.z) + ","
                            + F(pr.err) + "," + F(pr.targetVel) + "," + F(pr.targetVel) + ","
                            + F(rl) + "," + F(rh) + "," + F(eff) + ",";
                // iter = -1: 構築時の行 (相対速度は Prepare 時点の値)
                tr.Append(head).Append("-1").Append(tail)
                  .Append("0,0,0,").Append(F(pr.relVel)).Append(",\n");
                float prev = 0f;
                for (int it = 0; it < iters; it++)
                {
                    var sv = solved[it * nRows + k];
                    float acc = sv.accumulated;
                    float di = acc - prev;
                    // クランプ判定: 累積がちょうど上下限に張り付いている
                    int clamped = (acc == rl || acc == rh) ? 1 : 0;
                    // 反復前の相対速度は逆算 (relVelAfter = before + dI/effMass)。クランプ時も成立する。
                    float before = eff > 0f ? sv.relVelAfter - di / eff : float.NaN;
                    tr.Append(head).Append(it).Append(tail)
                      .Append(F(acc)).Append(',').Append(F(di)).Append(',').Append(clamped).Append(',')
                      .Append(F(before)).Append(',').Append(F(sv.relVelAfter)).Append('\n');
                    prev = acc;
                }
            }
        }

        string stPath = Path.Combine(outDir, "net_engine_state.csv");
        string rwPath = Path.Combine(outDir, "net_engine_rows.csv");
        File.WriteAllText(stPath, st.ToString(), new UTF8Encoding(false));
        File.WriteAllText(rwPath, rw.ToString(), new UTF8Encoding(false));
        Console.WriteLine("  -> " + stPath);
        Console.WriteLine("  -> " + rwPath);
        if (rowTrace)
        {
            string trPath = Path.Combine(outDir, "rowtrace_engine.csv");
            File.WriteAllText(trPath, tr.ToString(), new UTF8Encoding(false));
            Console.WriteLine("  -> " + trPath + "  (共通スキーマ・iter=-1 が構築時)");
            string agPath = Path.Combine(outDir, "angles_engine.csv");
            File.WriteAllText(agPath, ang.ToString(), new UTF8Encoding(false));
            Console.WriteLine("  -> " + agPath + "  (角度3軸の生値。state 0=free/1=範囲内/2=locked/3=下限外/4=上限外)");
        }
        Console.WriteLine("  frames=" + frames + " substeps=" + subs + " iters=" + iters +
                          "  行CSVは先頭 " + rowFrames + "F と F" + lateFrom + " 以降");
        if (PersistentManifold.CollectManifoldStats)
        {
            long rk = PersistentManifold.StatRefreshKept;
            long rn = PersistentManifold.StatRefreshDropNormal;
            long rl = PersistentManifold.StatRefreshDropLateral;
            long am = PersistentManifold.StatAddMatched;
            long an = PersistentManifold.StatAddNewSlot;
            long ar = PersistentManifold.StatAddReplaced;
            Console.WriteLine("  [manstats] Refresh 生存 " + rk + " / 破棄(法線) " + rn +
                " / 破棄(横ずれ) " + rl + " / 破棄(深い幻) " + PersistentManifold.StatRefreshDropDeep +
                "   => 破棄率 " + (100.0 * (rn + rl) / Math.Max(1, rk + rn + rl)).ToString("F1") + "%");
            Console.WriteLine("  [manstats] AddPoint 既存に一致 " + am + " / 空きに新規 " + an +
                " / 4点超で置換 " + ar + "   => 一致率 " + (100.0 * am / Math.Max(1, am + an + ar)).ToString("F1") + "%");
        }
        Console.WriteLine("  接触点の総数 (全サブステップ合計) = " + contactTotal +
                          (contactTotal == 0 ? "  ← 接触ゼロ。差はジョイントだけに由来する"
                                             : "  ★接触がある。突き合わせの前提が崩れているので要確認"));
        foreach (var kv in contactPairs)
            Console.WriteLine("      " + kv.Key + "  " + kv.Value + " 点");
        world.DebugContacts = null;
        world.DebugContactRows = null;
        world.DebugContactIterRows = null;
        if (conTrace)
        {
            string cPath = Path.Combine(outDir, "contacts_engine.csv");
            File.WriteAllText(cPath, con.ToString(), new UTF8Encoding(false));
            Console.WriteLine("  -> " + cPath + "  (接触点の生成。bulletref の contacts_bullet.csv と同スキーマ)");
            string ciPath = Path.Combine(outDir, "contactiter_engine.csv");
            File.WriteAllText(ciPath, conI.ToString(), new UTF8Encoding(false));
            Console.WriteLine("  -> " + ciPath + "  (接触行の反復単位。第一不一致点の中身を見る用)");
        }
        return 0;
    }
}
