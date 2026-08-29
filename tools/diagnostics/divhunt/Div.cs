// ===========================================================================
// divhunt : 実機で起きた発散を、記録したボーン姿勢CSVでヘッドレスに再現し、
//   「何フレーム目のどの剛体から壊れたか」を特定する。
//
//   入力:
//     MMD_TEST_GLB / MMD_TEST_PMX  … モデル (glb は extras.mmd 経由)
//     BONECSV                      … MmdPhysicsBehaviour の DumpBonePoseCsv が出したCSV
//                                    (frame,boneName,posX..,quatX.. / PMX ネイティブ)
//   構成 (Unity の出荷既定に合わせる):
//     FixedTimeStep=1/60  SubSteps=2  ITERS=10   ※ env で上書き可
//
//   出力: 速度・角速度・原点からの距離のフレーム推移と、**最初に閾値を超えた剛体**。
//         NaN/Inf も検出する。閾値は env で変更可 (VMAX/WMAX/PMAX)。
// ===========================================================================
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class DivHunt
{
    static StringBuilder O = new StringBuilder();
    static void L(string s = "") { O.Append(s); O.Append('\n'); Console.WriteLine(s); }
    static string Env(string k) { var v = Environment.GetEnvironmentVariable(k); return string.IsNullOrEmpty(v) ? null : v; }
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static float EnvF(string k, float d)
    {
        float v;
        return float.TryParse(Env(k), System.Globalization.NumberStyles.Float,
                              System.Globalization.CultureInfo.InvariantCulture, out v) ? v : d;
    }
    static bool Fin(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    static bool Fin(Vec3 v) => Fin(v.x) && Fin(v.y) && Fin(v.z);

    static int Main()
    {
        SyncGuard.RequireInSync();   // ★エンジン3複製の同期を先に確かめる (不一致なら実行しない)
        try { Console.OutputEncoding = Encoding.UTF8; } catch { }

        string glb = Env("MMD_TEST_GLB");
        string pmx = Env("MMD_TEST_PMX");
        string csvPath = Env("BONECSV");
        if (csvPath == null || !File.Exists(csvPath)) { L("[SKIP] BONECSV が無い"); return 1; }
        PmxPhysicsModel model;
        if (glb != null && File.Exists(glb)) model = GlbPhysicsReader.LoadFile(glb);
        else if (pmx != null && File.Exists(pmx)) model = PmxReader.LoadFile(pmx);
        else { L("[SKIP] モデルが無い (MMD_TEST_GLB / MMD_TEST_PMX)"); return 1; }

        var csv = BoneCsv.Load(csvPath);
        if (Env("DUALSCALE") != null) return DualScale(model, csv);
        int frames = EnvI("FRAMES", csv.FrameCount);
        if (frames > csv.FrameCount) frames = csv.FrameCount;

        int subs = EnvI("SUBSTEPS", 2), iters = EnvI("ITERS", 10);
        float fts = 1f / EnvI("FPS", 60);
        float vmax = EnvF("VMAX", 200f), wmax = EnvF("WMAX", 5000f), pmax = EnvF("PMAX", 500f);

        // ★A/B つまみ (エンジンの static。既定は出荷値のまま)
        if (Env("MAXMASS") != null) PmxPhysicsBuilder.MaxDynamicMass = EnvF("MAXMASS", 1e3f);
        if (Env("LEVER") != null) Joint.LinearLeverMode = EnvI("LEVER", 1);
        if (Env("LIMGATE") != null) Joint.BulletLimitRowGating = Env("LIMGATE") == "1";
        if (Env("IMPBND") != null) Joint.LockedRowImpulseBound = EnvF("IMPBND", 1e18f);
        // ★タスク82①: 減衰クランプ上限の掃引 (既定 0.999 = ビット不変)。
        if (Env("DAMPCLAMP") != null) PhysicsWorld.DampingClampMax = EnvF("DAMPCLAMP", 0.999f);
        if (Env("LEVERGATE") != null) Joint.LeverArmGate = EnvF("LEVERGATE", 5f);
        if (Env("MAXCORR") != null) Joint.MaxCorrectionVel = EnvF("MAXCORR", 10f);
        float jbeta = EnvF("JBETA", -1f);   // ★Bullet 2.75 の 6DOF は線形・回転とも ERP 0.5 (当既定 0.2)

        var b = PmxPhysicsBuilder.Build(model);
        var world = b.World;
        if (jbeta >= 0f) foreach (var j in world.Joints) j.Beta = jbeta;
        // ★JOINTORDER=<file> (2026-08-29 タスク108): ジョイントを解く順序を外から与える **計装専用** env。
        //   restosc/NetDump.cs の同名 env と同じ仕組み・同じファイル形式 (1行1ジョイント名、# はコメント)。
        //   Bullet は solveConstraints で m_constraints を島ソート (quickSort) するため拘束配列の順が
        //   定義順から入れ替わる。Gauss-Seidel は順序依存なので、行レベルで揃えるにはこれが要る。
        //   ★env 未設定なら world.Joints に一切触れない = 既存出力はビット不変。
        {
            string ordFile = Env("JOINTORDER");
            if (!string.IsNullOrEmpty(ordFile))
            {
                if (!File.Exists(ordFile)) { L("★JOINTORDER が無い: " + ordFile); return 1; }
                var want = new List<string>();
                foreach (var ln in File.ReadAllLines(ordFile))
                { var t = ln.Trim(); if (t.Length > 0 && t[0] != '#') want.Add(t); }
                var byName = new Dictionary<string, Joint>();
                foreach (var j in world.Joints) if (j.Name != null) byName[j.Name] = j;
                var reordered = new List<Joint>();
                foreach (var n in want) if (byName.TryGetValue(n, out var jj) && !reordered.Contains(jj)) reordered.Add(jj);
                int given = reordered.Count;
                foreach (var j in world.Joints) if (!reordered.Contains(j)) reordered.Add(j);   // 指定漏れは末尾へ
                world.Joints.Clear(); world.Joints.AddRange(reordered);
                L("  [jointorder] " + string.Join(" -> ", want) + "  (指定 " + given + " / 全 " + world.Joints.Count + " 本, " + ordFile + ")");
            }
        }
        world.Gravity = new Vec3(0f, -EnvF("GRAVITY", 98f), 0f);
        world.SolverIterations = iters;
        world.SubSteps = subs;
        world.FixedTimeStep = fts;

        // ボーンindex -> CSV 列 (名前で照合)。CSV に無いボーンは「前回維持」= null を返す。
        int nb = model.BoneNames.Count;
        var have = new bool[nb];
        int matched = 0;
        for (int i = 0; i < nb; i++)
        {
            RigidTransform tmp;
            if (csv.TryGet(0, model.BoneNames[i], out tmp)) { have[i] = true; matched++; }
        }

        int frame = 0;
        Func<int, RigidTransform?> getBone = bi =>
        {
            if (bi < 0 || bi >= nb || !have[bi]) return null;
            RigidTransform xf;
            return csv.TryGet(frame, model.BoneNames[bi], out xf) ? (RigidTransform?)xf : null;
        };

        // ★検証用: 動的剛体の質量に上限をかける (PMX の異常値が原因かを切り分ける)。
        //   エンジンは変えず、構築後の剛体へ SetMassProps し直すだけ。
        // ★一律スケール: 数学的には物理不変 (重力は加速度、拘束は質量比のみ)。
        //   差が出たら float32 の精度が原因である証拠になる。
        float massScale = EnvF("MASSSCALE", 0f);
        if (massScale > 0f)
        {
            int n = 0;
            foreach (var body in world.Bodies)
            {
                if (body.IsStaticOrKinematic || body.InverseMass <= 0f) continue;
                body.SetMassProps((1f / body.InverseMass) * massScale); n++;
            }
            L("  [質量一律スケール] x" + massScale.ToString("G4") + " を " + n + " 体へ");
        }

        // ★検証用: 指定質量を超える動的剛体を「キネマティック」にする。
        //   invMass≈1.8e-15 は実質不動なので、素直に不動として扱ったらどうなるかを見る。
        //
        // ★★2026-08-26 タスク80⑥: **この診断は結論に使えない。**
        //   KinematicTarget を「起動時の姿勢」で固定するだけなので、対象剛体は
        //   アニメを追わず **バインド姿勢に釘付け**になる。鎖の関節はロックなので
        //   下流も丸ごと止まり、実測で末端 3 本以上が w=0.00 deg/frame になった。
        //   その結果 |v| 閾値ゲートが鳴らなくなるだけで、**参照と比べると 2倍悪化**する
        //   (スパイク posRMS 5.293 → 11.288)。「キネマ化すれば完治」は
        //   **閾値ゲートの見かけ**であって治癒ではない。判定は必ず参照 CSV (cmpref) で行うこと。
        float massKin = EnvF("MASSKIN", 0f);
        if (massKin > 0f)
        {
            int n = 0;
            foreach (var body in world.Bodies)
            {
                if (body.IsStaticOrKinematic || body.InverseMass <= 0f) continue;
                if (1f / body.InverseMass <= massKin) continue;
                body.Mode = PhysicsMode.BoneFollow;
                body.SetMassProps(0f);
                body.KinematicTarget = body.WorldTransform;
                body.KinematicStepTarget = body.WorldTransform;
                n++;
            }
            L("  [巨大質量をキネマティック化] " + massKin.ToString("G4") + " 超の " + n + " 体");
        }

        float massClamp = EnvF("MASSCLAMP", 0f);
        if (massClamp > 0f)
        {
            int n = 0;
            foreach (var body in world.Bodies)
            {
                if (body.IsStaticOrKinematic || body.InverseMass <= 0f) continue;
                float m = 1f / body.InverseMass;
                if (m > massClamp) { body.SetMassProps(massClamp); n++; }
            }
            L("  [質量クランプ] " + massClamp.ToString("G4") + " を超える動的剛体 " + n + " 体を丸めた");
        }

        // ★検証用: 接触を止める (全剛体の当たり判定マスクを 0 にする)。
        //   ジョイント由来か接触由来かの二分に使う。
        if (Env("NOCONTACT") == "1")
        {
            foreach (var body in world.Bodies) body.CollisionMask = 0;
            L("  [接触OFF] 全剛体の CollisionMask=0");
        }

        // ★検証用: 指定名を含む剛体だけ動的に残し、他の動的剛体は kinematic 化する (minnet 方式)。
        //   ジョイントは両端とも「残った動的 or キネマティック」なので、そのままで成立する。
        string only = Env("ONLYCHAIN");
        if (only != null)
        {
            int parked = 0, kept = 0;
            foreach (var body in world.Bodies)
            {
                if (body.IsStaticOrKinematic) continue;
                if (body.Name != null && body.Name.Contains(only)) { kept++; continue; }
                body.Mode = PhysicsMode.BoneFollow;
                body.KinematicTarget = body.WorldTransform;
                body.KinematicStepTarget = body.WorldTransform;
                parked++;
            }
            L("  [鎖のみ] '" + only + "' を動的に残し " + kept + " 体、他 " + parked + " 体を kinematic 化");
        }

        // ★検証用: 減衰を一律に上書きする (PMX の linD=angD=1 が効いているかの切り分け)。
        float dampSet = EnvF("DAMPSET", -1f);
        if (dampSet >= 0f)
        {
            int n = 0;
            foreach (var body in world.Bodies)
            {
                if (body.IsStaticOrKinematic) continue;
                body.LinearDamping = dampSet; body.AngularDamping = dampSet; n++;
            }
            L("  [減衰上書き] " + dampSet.ToString("G4") + " を " + n + " 体へ");
        }

        // 起動時の整合 (MmdPhysicsBehaviour.ResetPhysicsToBones 相当)。
        b.ApplyKinematicTargets(getBone);
        b.ResetBodiesToBonePoseFk(getBone);

        L("=".PadRight(110, '='));
        L("divhunt : 記録したボーン姿勢で実機の発散を再現する");
        L("  モデル 剛体" + model.RigidBodies.Count + " Joint" + model.Joints.Count + " ボーン" + nb +
          "   CSV " + csv.FrameCount + "F / ボーン" + csv.BoneCount + " (照合 " + matched + "本)");
        L("  刻み 1/" + EnvI("FPS", 60) + " x SubSteps" + subs + " / Iters" + iters +
          "   閾値 |v|>" + vmax + " |w|>" + wmax + "deg/s 原点距離>" + pmax);
        L("=".PadRight(110, '='));
        L(string.Format("  {0,6} {1,12} {2,12} {3,12} {4,8} {5,8}", "frame", "|v|max", "|w|max", "|p|max", "接触", "違反max"));

        const float R2D = 57.29578f;
        var CI = System.Globalization.CultureInfo.InvariantCulture;
        StringBuilder refDump = null; string refFilter = Env("REFFILTER");
        if (Env("REFDUMP") != null)
        {
            refDump = new StringBuilder(1 << 22);
            refDump.Append("frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW").Append((char)10);
        }
        // ===================================================================
        // ★NETDUMP=<dir> (タスク81⓪): 素の Bullet 2.75 (bulletref) と三者比較するための
        //   網の書き出し。ONLYCHAIN で残した鎖 + その関節相手 (駆動アンカー) だけを出す。
        //   形式は restosc/NetDump.cs の net.txt v1 と同一 (bulletref がそのまま読める)。
        //   併せて drive.csv (毎フレームのキネマ目標) を出し、bulletref 側で同じ駆動を再現する。
        // ===================================================================
        string netDir = Env("NETDUMP");
        StringBuilder driveCsv = null;
        var netBodies = new List<RigidBody>();
        var netIdx = new Dictionary<RigidBody, int>();
        if (netDir != null)
        {
            Directory.CreateDirectory(netDir);
            string pref = Env("ONLYCHAIN") ?? "";
            var keptJoints = new List<Joint>();
            foreach (var j in world.Joints)
            {
                bool hit = (j.BodyA != null && j.BodyA.Name != null && j.BodyA.Name.StartsWith(pref))
                        || (j.BodyB != null && j.BodyB.Name != null && j.BodyB.Name.StartsWith(pref));
                if (!hit) continue;
                keptJoints.Add(j);
                foreach (var bb in new[] { j.BodyA, j.BodyB })
                    if (bb != null && !netIdx.ContainsKey(bb)) { netIdx[bb] = netBodies.Count; netBodies.Add(bb); }
            }
            Func<float, string> F = v => v.ToString("G9", CI);
            Func<Vec3, string> F3 = v => F(v.x) + " " + F(v.y) + " " + F(v.z);
            Func<Quat, string> F4 = q => F(q.x) + " " + F(q.y) + " " + F(q.z) + " " + F(q.w);
            var s0 = new StringBuilder();
            s0.Append("# bulletref net spec v1  (divhunt NETDUMP)").Append((char)10);
            s0.Append("world gravity " + F3(world.Gravity)).Append((char)10);
            s0.Append("world dt " + F(fts) + " substeps " + world.SubSteps + " iters " + world.SolverIterations).Append((char)10);
            s0.Append("world contactBaumgarte " + F(world.BaumgarteFactor) + " slop " + F(world.PenetrationSlop) + " restitutionThreshold 1").Append((char)10);
            s0.Append("world jsplit " + (world.UseJointSplitImpulse ? 1 : 0) + " split " + (world.UseSplitImpulse ? 1 : 0) +
                      " jwarm " + (world.UseJointWarmStart ? 1 : 0) + " jwarmang " + (world.UseJointWarmStartAngular ? 1 : 0) +
                      " leverMode " + Joint.LinearLeverMode + " mixedAxes " + (Joint.AngularMixedAxes ? 1 : 0) +
                      " maxCorrVel " + F(Joint.MaxCorrectionVel)).Append((char)10);
            foreach (var bb in netBodies)
            {
                int shape = (int)bb.Shape.Type;
                Vec3 size;
                if (bb.Shape is SphereShape sp2) size = new Vec3(sp2.Radius, 0f, 0f);
                else if (bb.Shape is BoxShape bx2) size = bx2.HalfExtents;
                else if (bb.Shape is CapsuleShape cp2) size = new Vec3(cp2.Radius, cp2.Height, 0f);
                else size = Vec3.Zero;
                s0.Append("body " + netIdx[bb] + " name=" + (bb.Name ?? "?").Replace(' ', '_') +
                          " mode=" + (int)bb.Mode + " mass=" + F(bb.Mass) + " invMass=" + F(bb.InverseMass) +
                          " shape=" + shape + " size=" + F3(size) + " margin=" + F(bb.Shape.Margin) +
                          " inertia=" + F3(bb.LocalInertiaDiag) +
                          " pos=" + F3(bb.WorldTransform.Origin) + " quat=" + F4(bb.WorldTransform.Rotation) +
                          " linvel=" + F3(bb.LinearVelocity) + " angvel=" + F3(bb.AngularVelocity) +
                          " lindamp=" + F(bb.LinearDamping) + " angdamp=" + F(bb.AngularDamping) +
                          " friction=" + F(bb.Friction) + " restitution=" + F(bb.Restitution) +
                          " boneoff=0 0 0 boneoffq=0 0 0 1" +
                          " group=" + bb.Group + " mask=" + bb.CollisionMask).Append((char)10);
            }
            for (int k = 0; k < keptJoints.Count; k++)
            {
                var j = keptJoints[k];
                s0.Append("joint " + k + " name=" + (j.Name ?? "?").Replace(' ', '_') + " type=" + (int)j.Type +
                          " a=" + netIdx[j.BodyA] + " b=" + netIdx[j.BodyB] +
                          " faPos=" + F3(j.FrameInA.Origin) + " faQuat=" + F4(j.FrameInA.Rotation) +
                          " fbPos=" + F3(j.FrameInB.Origin) + " fbQuat=" + F4(j.FrameInB.Rotation) +
                          " linLo=" + F3(j.LinearLowerLimit) + " linHi=" + F3(j.LinearUpperLimit) +
                          " angLo=" + F3(j.AngularLowerLimit) + " angHi=" + F3(j.AngularUpperLimit) +
                          " spLin=" + F3(j.SpringLinear) + " spAng=" + F3(j.SpringAngular) +
                          " spDamp=" + F(j.SpringDamping) + " beta=" + F(j.Beta) + " cross=0").Append((char)10);
            }
            File.WriteAllText(Path.Combine(netDir, "net.txt"), s0.ToString(), new UTF8Encoding(false));
            L("  [NETDUMP] 剛体 " + netBodies.Count + " / ジョイント " + keptJoints.Count + " → " + netDir);
            driveCsv = new StringBuilder(1 << 20);
            driveCsv.Append("frame,bodyIndex,posX,posY,posZ,quatX,quatY,quatZ,quatW").Append((char)10);
        }

        int firstBad = -1; string firstBadName = null, firstBadWhy = null;
        var prev = new Vec3[world.Bodies.Count];

        // ★検証: テレポート検出を「Push の前」に置き、検出したら Step の前に再整合する。
        //   TELEPORT=0 で無効 (現状の Unity と同じ = 検出が働かない状態)。
        float teleTh = EnvF("TELEPORT", 3f);
        float teleFrac = EnvF("TELEFRAC", 0.25f);
        int teleFired = 0;

        for (frame = 0; frame < frames; frame++)
        {
            bool teleported = false;
            if (teleTh > 0f)
            {
                float th2 = teleTh * teleTh;
                int over = 0, total = 0;
                foreach (var link in b.BoneLinks)
                {
                    if (link.Mode != PhysicsMode.BoneFollow || link.BoneIndex < 0) continue;
                    var bw = getBone(link.BoneIndex);
                    if (!bw.HasValue) continue;
                    total++;
                    // ★KinematicTarget が **まだ前フレームの値** のうちに比べるのが要点。
                    var d = (bw.Value * link.BodyOffsetFromBone).Origin - link.Body.KinematicTarget.Origin;
                    if (d.LengthSquared > th2) over++;
                }
                int need = Math.Max(1, (int)Math.Ceiling(total * teleFrac));
                teleported = over >= need && over > 0;
            }

            b.ApplyKinematicTargets(getBone);
            if (teleported)
            {
                b.ResetBodiesToBonePoseFk(getBone);   // 殴られる前に置き直す
                teleFired++;
                L(string.Format("  [テレポート検出] frame={0} → 再整合", frame));
            }
            if (driveCsv != null)
            {
                // ★ApplyKinematicTargets の直後 = ソルバが見るのと同じ目標姿勢。
                foreach (var bb in netBodies)
                {
                    if (!bb.IsStaticOrKinematic) continue;
                    var t = bb.KinematicTarget;
                    driveCsv.Append(frame).Append(',').Append(netIdx[bb]).Append(',')
                            .Append(t.Origin.x.ToString("G9", CI)).Append(',')
                            .Append(t.Origin.y.ToString("G9", CI)).Append(',')
                            .Append(t.Origin.z.ToString("G9", CI)).Append(',')
                            .Append(t.Rotation.x.ToString("G9", CI)).Append(',')
                            .Append(t.Rotation.y.ToString("G9", CI)).Append(',')
                            .Append(t.Rotation.z.ToString("G9", CI)).Append(',')
                            .Append(t.Rotation.w.ToString("G9", CI)).Append((char)10);
                }
            }
            world.StepSimulation(fts);

            float vmaxF = 0f, wmaxF = 0f, pmaxF = 0f;
            string vn = null, wn = null, pn = null;
            bool nan = false; string nanName = null;
            for (int i = 0; i < world.Bodies.Count; i++)
            {
                var body = world.Bodies[i];
                if (body.IsStaticOrKinematic) continue;
                var v = body.LinearVelocity; var w = body.AngularVelocity; var p = body.WorldTransform.Origin;
                if (!Fin(v) || !Fin(w) || !Fin(p)) { nan = true; nanName = body.Name; }
                float vl = (float)Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                float wl = (float)Math.Sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * R2D;
                float pl = (float)Math.Sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                if (vl > vmaxF) { vmaxF = vl; vn = body.Name; }
                if (wl > wmaxF) { wmaxF = wl; wn = body.Name; }
                if (pl > pmaxF) { pmaxF = pl; pn = body.Name; }
            }

            if (firstBad < 0)
            {
                if (nan) { firstBad = frame; firstBadName = nanName; firstBadWhy = "NaN/Inf"; }
                else if (pmaxF > pmax) { firstBad = frame; firstBadName = pn; firstBadWhy = "原点距離 " + pmaxF.ToString("G5"); }
                else if (vmaxF > vmax) { firstBad = frame; firstBadName = vn; firstBadWhy = "|v| " + vmaxF.ToString("G5"); }
                else if (wmaxF > wmax) { firstBad = frame; firstBadName = wn; firstBadWhy = "|w| " + wmaxF.ToString("G5") + "deg/s"; }
                if (firstBad >= 0)
                    L(string.Format("  ★初回逸脱 frame={0} 剛体='{1}' 理由={2}", firstBad, firstBadName, firstBadWhy));
            }

            // ★鎖に沿った時系列トレース: TRACE=<名前の一部> TRACEFROM=<frame> TRACETO=<frame>
            string tr = Env("TRACE");
            if (tr != null && frame >= EnvI("TRACEFROM", 0) && frame <= EnvI("TRACETO", 1 << 30))
            {
                var sb = new StringBuilder();
                sb.Append("  f").Append(frame).Append(" |v|:");
                foreach (var body in world.Bodies)
                {
                    if (body.Name == null || !body.Name.Contains(tr) || body.IsStaticOrKinematic) continue;
                    var v = body.LinearVelocity;
                    float vl = (float)Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                    sb.Append(' ').Append(vl < 0.05f ? "." : vl.ToString(vl < 10f ? "F1" : "F0"));
                }
                L(sb.ToString());
            }

            // ★REFDUMP: 物理ボーンのワールド姿勢を CSV へ出す (参照との突き合わせ用)。
            //   ボーン空間 = body.WorldTransform * BodyOffsetFromBone.Inverse()  (hairfid と同じ定義)
            if (refDump != null)
            {
                foreach (var link in b.BoneLinks)
                {
                    if (link.Mode == PhysicsMode.BoneFollow || link.BoneIndex < 0) continue;
                    string bn = model.BoneNames[link.BoneIndex];
                    if (refFilter != null && !bn.Contains(refFilter)) continue;
                    var xf = link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
                    var p2 = xf.Origin; var q2 = xf.Rotation;
                    refDump.Append(frame).Append(',').Append(bn).Append(',')
                        .Append(p2.x.ToString("R", CI)).Append(',').Append(p2.y.ToString("R", CI)).Append(',').Append(p2.z.ToString("R", CI)).Append(',')
                        .Append(q2.x.ToString("R", CI)).Append(',').Append(q2.y.ToString("R", CI)).Append(',').Append(q2.z.ToString("R", CI)).Append(',').Append(q2.w.ToString("R", CI))
                        .Append((char)10);
                }
            }

            int every = EnvI("EVERY", 30);
            if (frame % every == 0 || (firstBad >= 0 && frame <= firstBad + 5))
                L(string.Format("  {0,6} {1,12:G5} {2,12:G5} {3,12:G5} {4,8} {5,8:G4}",
                                frame, vmaxF, wmaxF, pmaxF, world.DebugContactCount, MaxViolation(world)));
        }

        L();
        if (refDump != null)
        {
            File.WriteAllText(Env("REFDUMP"), refDump.ToString(), new UTF8Encoding(false));
            L("  [REFDUMP] " + Env("REFDUMP") + " へ書き出し");
        }
        if (driveCsv != null)
        {
            File.WriteAllText(Path.Combine(netDir, "drive.csv"), driveCsv.ToString(), new UTF8Encoding(false));
            L("  [NETDUMP] drive.csv へ書き出し");
        }
        L("  テレポート再整合の発動: " + teleFired + " 回");
        if (firstBad < 0) L("  => 発散なし (全" + frames + "フレームで閾値内)");
        else L("  => ★frame " + firstBad + " の '" + firstBadName + "' から壊れた (" + firstBadWhy + ")");
        File.WriteAllText(Env("OUT") ?? "divhunt.txt", O.ToString());
        return firstBad < 0 ? 0 : 2;
    }

    // ★スケール不変性の検査。質量を一律に定数倍しても物理は数学的に不変
    //   (重力は加速度、拘束は質量比のみ)。等倍と S倍を同じ入力で並走させ、
    //   **最初に速度がズレるフレームと剛体**を出す。ズレたらそこにバグがある。
    static int DualScale(PmxPhysicsModel model, BoneCsv csv)
    {
        float S = EnvF("DUALSCALE", 1e4f);
        float tol = EnvF("DUALTOL", 1e-3f);
        int frames = EnvI("FRAMES", csv.FrameCount);
        if (frames > csv.FrameCount) frames = csv.FrameCount;
        int subs = EnvI("SUBSTEPS", 2), iters = EnvI("ITERS", 10);
        float fts = 1f / EnvI("FPS", 60);
        int nb = model.BoneNames.Count;
        var have = new bool[nb];
        for (int i = 0; i < nb; i++) { RigidTransform t; if (csv.TryGet(0, model.BoneNames[i], out t)) have[i] = true; }
        int frame = 0;
        Func<int, RigidTransform?> getBone = bi =>
        {
            if (bi < 0 || bi >= nb || !have[bi]) return null;
            RigidTransform xf;
            return csv.TryGet(frame, model.BoneNames[bi], out xf) ? (RigidTransform?)xf : null;
        };
        PmxPhysicsBuilder Make(float scale)
        {
            var bb = PmxPhysicsBuilder.Build(model);
            var w = bb.World;
            w.Gravity = new Vec3(0f, -EnvF("GRAVITY", 98f), 0f);
            w.SolverIterations = iters; w.SubSteps = subs; w.FixedTimeStep = fts;
            string only = Env("ONLYCHAIN");
            if (only != null)
                foreach (var body in w.Bodies)
                {
                    if (body.IsStaticOrKinematic) continue;
                    if (body.Name != null && body.Name.Contains(only)) continue;
                    body.Mode = PhysicsMode.BoneFollow;
                    body.KinematicTarget = body.WorldTransform; body.KinematicStepTarget = body.WorldTransform;
                }
            if (Env("NOCONTACT") == "1") foreach (var body in w.Bodies) body.CollisionMask = 0;
            if (scale != 1f)
                foreach (var body in w.Bodies)
                    if (!body.IsStaticOrKinematic && body.InverseMass > 0f)
                        body.SetMassProps((1f / body.InverseMass) * scale);
            bb.ApplyKinematicTargets(getBone);
            bb.ResetBodiesToBonePoseFk(getBone);
            return bb;
        }
        var A = Make(1f); var B = Make(S);
        L("=".PadRight(100, '='));
        L("dualscale : 質量 x1 と x" + S.ToString("G4") + " を並走。数学的には完全一致するはず。許容 " + tol.ToString("G3"));
        L("=".PadRight(100, '='));
        for (frame = 0; frame < frames; frame++)
        {
            A.ApplyKinematicTargets(getBone); A.World.StepSimulation(fts);
            B.ApplyKinematicTargets(getBone); B.World.StepSimulation(fts);
            float worst = 0f; string wn = null; float va = 0f, vb = 0f;
            for (int i = 0; i < A.World.Bodies.Count; i++)
            {
                var ba = A.World.Bodies[i]; var b2 = B.World.Bodies[i];
                if (ba.IsStaticOrKinematic) continue;
                var d = ba.LinearVelocity - b2.LinearVelocity;
                float e = (float)Math.Sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
                if (e > worst) { worst = e; wn = ba.Name; va = ba.LinearVelocity.Length; vb = b2.LinearVelocity.Length; }
            }
            if (frame % EnvI("EVERY", 60) == 0 || worst > tol)
                L(string.Format("  f{0,-5} 速度差max={1,10:G4}  剛体='{2}'  |v|(x1)={3:G4} |v|(xS)={4:G4}", frame, worst, wn, va, vb));
            if (worst > tol)
            {
                L("  ★スケール不変性が破れた: frame=" + frame + " 剛体='" + wn + "'");
                File.WriteAllText(Env("OUT") ?? "dualscale.txt", O.ToString());
                return 2;
            }
        }
        L("  => 全" + frames + "フレームで不変 (許容内)");
        File.WriteAllText(Env("OUT") ?? "dualscale.txt", O.ToString());
        return 0;
    }

    // 拘束されている並進行のアンカー誤差の最大 (restosc/SynDrive と同じ定義)。
    static float MaxViolation(PhysicsWorld w)
    {
        float m = 0f;
        foreach (var j in w.Joints)
        {
            if (j.BodyA == null || j.BodyB == null) continue;
            var wA = j.BodyA.WorldTransform * j.FrameInA;
            var wB = j.BodyB.WorldTransform * j.FrameInB;
            var bA = Matrix3x3.FromQuat(wA.Rotation);
            var d = wB.Origin - wA.Origin;
            float sq = 0f;
            for (int i = 0; i < 3; i++)
            {
                float lo = j.LinearLowerLimit[i], hi = j.LinearUpperLimit[i];
                if (lo > hi) continue;
                float cur = d.Dot(bA.Column(i));
                float e = lo == hi ? cur - lo : (cur < lo ? lo - cur : (cur > hi ? cur - hi : 0f));
                sq += e * e;
            }
            float len = (float)Math.Sqrt(sq);
            if (len > m) m = len;
        }
        return m;
    }
}
