// ===========================================================================
// 髪のジッタ計測ハーネス (MMD参照CSV不要)。
//   駆動ボーンCSVで BoneFollow 剛体を動かし、髪(dynamic)の毎フレーム姿勢をダンプする。
//   ジッタ J(f) = 前フレームからの自己変化量。MMDとの「差」ではなく「揺れ」そのものを測る。
//   A/B は env KINANG=atan2 (既定OFF=従来acos)。他フラグは既存ハーネスと同じ綴り。
//   出力: frame,bodyName,posX,posY,posZ,qx,qy,qz,qw  (ボーン空間に復元した姿勢)
//        + 駆動キネマティック剛体の角速度ノルム (kinang.csv)
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class Jitter
{
    static bool IsHair(string n) => n != null && (n.Contains("髪") || n.Contains("ツインテ") || n.Contains("もみあげ") || n.Contains("前髪") || n.Contains("モミアゲ"));
    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static bool _freeze = false;

    static int Main()
    {
        string pmx = Env("MMD_TEST_PMX"), csvp = Env("MMD_TEST_BONECSV");
        string outp = Env("OUT") ?? "hairpose.csv";
        int F = int.TryParse(Env("FRAMES"), out var f0) ? f0 : 200;
        int warm = int.TryParse(Env("WARMUP"), out var w0) ? w0 : 60;
        if (pmx == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP] MMD_TEST_PMX 未設定"); return 1; }
        if (csvp == null || !File.Exists(csvp)) { Console.WriteLine("[SKIP] MMD_TEST_BONECSV 未設定"); return 1; }

        // ※ KINANG=atan2 (キネマティック角速度の atan2 化) は削除した。Bullet 2.75 も
        //    btQuaternion::getAngle() = 2*acos(w) を使っており、atan2 化は Bullet からの乖離になるため
        //    採用しない、と 2026-08-12 に判断済み。

        var model = PmxReader.LoadFile(pmx);
        var csv = BoneCsv.Load(csvp);
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        if (float.TryParse(Env("CWFAC"), NumberStyles.Float, CultureInfo.InvariantCulture, out var cwf)) world.ContactWarmStartFactor = cwf;
        if (Env("SLEEP") == "1") world.EnableSleeping = true;
        if (Env("SPLIT") == "1") world.UseSplitImpulse = true;
        if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (Env("JOINTS_FIRST") == "1") world.SolveJointsFirst = true;
        if (Env("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
        if (float.TryParse(Env("WARMFAC"), NumberStyles.Float, CultureInfo.InvariantCulture, out var jwf)) Joint.WarmStartFactor = jwf;
        if (float.TryParse(Env("BAUM"), NumberStyles.Float, CultureInfo.InvariantCulture, out var bg)) world.BaumgarteFactor = bg;
        if (float.TryParse(Env("BETA"), NumberStyles.Float, CultureInfo.InvariantCulture, out var bt)) foreach (var j in world.Joints) j.Beta = bt;
        if (float.TryParse(Env("SPTH"), NumberStyles.Float, CultureInfo.InvariantCulture, out var sp)) world.SplitImpulsePenetrationThreshold = sp;
        if (Env("NOGRAV") == "1") world.Gravity = Vec3.Zero;
        if (Env("FREEZE") == "1") _freeze = true;
        if (int.TryParse(Env("ITERS"), out var it) && it > 0) world.SolverIterations = it;
        if (int.TryParse(Env("SUBSTEPS"), out var ss) && ss > 0) world.SubSteps = ss;
        if (float.TryParse(Env("SPECMARGIN"), NumberStyles.Float, CultureInfo.InvariantCulture, out var spm) && spm > 0f) GjkEpa.SpeculativeMargin = spm; // 接触検出帯(既定0.02)

        Console.WriteLine($"[cfg2] SPLIT={world.UseSplitImpulse} JSPLIT={world.UseJointSplitImpulse} baum={world.BaumgarteFactor} spth={world.SplitImpulsePenetrationThreshold} grav={world.Gravity.y} freeze={_freeze}");
        Console.WriteLine($"[cfg] warm={world.UseJointWarmStart}/{world.UseJointWarmStartAngular} jfac={Joint.WarmStartFactor} CWFAC={world.ContactWarmStartFactor} SLEEP={world.EnableSleeping} iters={world.SolverIterations} sub={world.SubSteps} frames={F} warmup={warm}");

        Action<int> ApplyPose = f0_ => builder.ApplyKinematicTargets(bi =>
            (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(_freeze ? 0 : f0_, model.BoneNames[bi], out var bw)) ? (RigidTransform?)bw : null);

        ApplyPose(0);
        builder.ResetBodiesToBonePoseFk(i =>
            (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw)) ? (RigidTransform?)bw : null);
        for (int s = 0; s < warm; s++) world.StepSimulation(1f / 30f);

        // 計測対象: 髪 dynamic 剛体 / 参考: 駆動キネマティック剛体
        var hair = new List<int>(); var kin = new List<int>();
        for (int i = 0; i < builder.BoneLinks.Count; i++)
        {
            var l = builder.BoneLinks[i];
            string bn = (l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count) ? model.BoneNames[l.BoneIndex] : null;
            if (l.Mode == PhysicsMode.BoneFollow) { kin.Add(i); continue; }
            if (IsHair(bn)) hair.Add(i);
        }
        Console.WriteLine($"[構成] 髪dynamic剛体={hair.Count}  駆動キネマティック剛体={kin.Count}  全剛体={builder.Bodies.Count}");
        if (hair.Count == 0) { Console.WriteLine("[FAIL] 髪剛体が0。ボーン名の判定条件を確認すること。"); return 1; }

        // 貫入計測: 髪 dynamic × 体コライダー(BoneFollow) の最大貫入を毎フレーム測る。
        var buf = new List<ContactPoint>();
        var penLog = new List<float>();
        var pairPen = new Dictionary<string,float>(); var pairCnt = new Dictionary<string,int>();

        // --- 貫入オンセット計測 (2026-08-13) ---
        // 仮説: 貫入の多くは「前フレームは接触すら生成されていない → 次フレームでいきなり深く刺さる」で、
        //       検出帯 (Collision.cs の SpeculativeMargin=0.02) を 1ステップで飛び越えている。
        // ここでは各ペアの状態遷移を追い、オンセットの瞬間に
        //   (a) 直前が「接触なし」だったか  (b) 駆動剛体が接触点でどれだけ動いたか  (c) 抜けるまでの長さ
        // を記録する。エンジンは無改変。
        float penTh = float.TryParse(Env("PEN_TH"), NumberStyles.Float, CultureInfo.InvariantCulture, out var pth) ? pth : 0.01f;
        var pairState = new Dictionary<string,int>();   // 0=接触なし 1=接触(浅い) 2=貫入(>penTh)
        var penRun = new Dictionary<string,int>();
        var onsets = new List<(int frame, string pair, bool jumped, float pen, float kinMove, float kinNormal, float hairMove)>();
        var runLens = new List<int>();
        var prevKinX = new Dictionary<int,RigidTransform>();
        var prevHairX = new Dictionary<int,RigidTransform>();
        bool havePrev = false;

        Func<int,float> ScanPairs = (frame) => {
            float mx = 0f;
            foreach (int i in hair) foreach (int k in kin) {
                var A = builder.BoneLinks[i].Body; var B = builder.BoneLinks[k].Body;
                if ((B.CollisionMask & (1 << A.Group)) == 0 || (A.CollisionMask & (1 << B.Group)) == 0) continue;
                string key = A.Name + " × " + B.Name;
                var aa = A.ComputeAabb(); var bb = B.ComputeAabb();
                int st = 0; float deepest = 0f; ContactPoint deep = default;
                if (aa.Intersects(ref bb)) {
                    buf.Clear(); GjkEpa.Detect(A, B, buf);
                    foreach (var cp in buf) {
                        float pen = -cp.Distance;
                        if (pen > mx) mx = pen;
                        if (st == 0) st = 1;                    // 接触点が出た=帯の中
                        if (pen > deepest) { deepest = pen; deep = cp; }
                        if (pen > 0f) {
                            pairPen[key] = pairPen.TryGetValue(key, out var pv) ? Math.Max(pv, pen) : pen;
                            pairCnt[key] = pairCnt.TryGetValue(key, out var cv) ? cv + 1 : 1;
                        }
                    }
                    if (deepest > penTh) st = 2;
                }
                int prev = pairState.TryGetValue(key, out var ps) ? ps : 0;
                if (st == 2 && prev != 2) {
                    float kinMove = 0f, kinNormal = 0f, hairMove = 0f;
                    if (havePrev && prevKinX.TryGetValue(k, out var pxB) && prevHairX.TryGetValue(i, out var pxA)) {
                        // 接触点を各剛体のローカルへ落とし、前フレーム姿勢で戻して「素材点の移動量」を測る。
                        var w = deep.PositionWorldB;
                        var wPrev = pxB.TransformPoint(B.WorldTransform.Inverse().TransformPoint(w));
                        kinMove = (w - wPrev).Length;
                        kinNormal = (w - wPrev).Dot(deep.Normal);
                        var wa = deep.PositionWorldA;
                        hairMove = (wa - pxA.TransformPoint(A.WorldTransform.Inverse().TransformPoint(wa))).Length;
                    }
                    onsets.Add((frame, key, prev == 0, deepest, kinMove, kinNormal, hairMove));
                    penRun[key] = 0;
                }
                if (st == 2) penRun[key] = (penRun.TryGetValue(key, out var rl) ? rl : 0) + 1;
                else if (prev == 2) { runLens.Add(penRun.TryGetValue(key, out var rl2) ? rl2 : 0); penRun[key] = 0; }
                pairState[key] = st;
            }
            foreach (int k in kin) prevKinX[k] = builder.BoneLinks[k].Body.WorldTransform;
            foreach (int i in hair) prevHairX[i] = builder.BoneLinks[i].Body.WorldTransform;
            havePrev = true;
            return mx;
        };

        var sb = new StringBuilder(); sb.Append("frame,bodyName,posX,posY,posZ,qx,qy,qz,qw\n");
        var sk = new StringBuilder(); sk.Append("frame,bodyName,angvel,linvel\n");
        var ci = CultureInfo.InvariantCulture;

        for (int f = 0; f < F; f++)
        {
            ApplyPose(f);
            world.StepSimulation(1f / 30f);
            foreach (int i in hair)
            {
                var l = builder.BoneLinks[i];
                var bw = l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse(); // ボーン空間へ復元
                var p = bw.Origin; var q = bw.Rotation;
                sb.Append(f.ToString(ci)).Append(',').Append(l.Body.Name).Append(',')
                  .Append(p.x.ToString("R", ci)).Append(',').Append(p.y.ToString("R", ci)).Append(',').Append(p.z.ToString("R", ci)).Append(',')
                  .Append(q.x.ToString("R", ci)).Append(',').Append(q.y.ToString("R", ci)).Append(',').Append(q.z.ToString("R", ci)).Append(',')
                  .Append(q.w.ToString("R", ci)).Append('\n');
            }
            penLog.Add(ScanPairs(f));
            foreach (int i in kin)
            {
                var b = builder.BoneLinks[i].Body;
                sk.Append(f.ToString(ci)).Append(',').Append(b.Name).Append(',')
                  .Append(b.AngularVelocity.Length.ToString("R", ci)).Append(',')
                  .Append(b.LinearVelocity.Length.ToString("R", ci)).Append('\n');
            }
        }
        File.WriteAllText(outp, sb.ToString());
        File.WriteAllText(Path.ChangeExtension(outp, null) + "_kinang.csv", sk.ToString());
        Console.WriteLine("[常時接触ペア] 待機区間で貫入が続いた 髪×体 ペア (最大貫入順)");
        var pl = new List<KeyValuePair<string,float>>(pairPen); pl.Sort((x,y)=>y.Value.CompareTo(x.Value));
        for (int i2 = 0; i2 < pl.Count && i2 < 12; i2++)
            Console.WriteLine($"    {pl[i2].Key,-34} 最大貫入={pl[i2].Value:F5}  接触フレーム数={pairCnt[pl[i2].Key]}");
        var ps = new List<float>(penLog); ps.Sort();
        Console.WriteLine($"[貫入] 髪×体 最大貫入 中央={ps[ps.Count/2]:F5} p90={ps[(int)(ps.Count*0.9)]:F5} 最大={ps[ps.Count-1]:F5}  (>0.5のフレーム={penLog.FindAll(x=>x>0.5f).Count})");
        // --- 貫入オンセットの集計 ---
        Console.WriteLine();
        Console.WriteLine($"[貫入オンセット] pen>{penTh:F3} を「貫入」として、非貫入→貫入の瞬間を数える");
        if (onsets.Count == 0) Console.WriteLine("    オンセット 0件");
        else
        {
            int jumped = 0; foreach (var o in onsets) if (o.jumped) jumped++;
            Console.WriteLine($"    総オンセット={onsets.Count}  うち直前フレームは接触なし(帯を飛び越え)={jumped} ({jumped * 100.0 / onsets.Count:F1}%)");
            Console.WriteLine($"    {"指標",-30}{"中央",10}{"p90",10}{"最大",10}");
            void Q(string label, Func<(int, string, bool, float, float, float, float), float> sel, bool jumpedOnly)
            {
                var v = new List<float>();
                foreach (var o in onsets) if (!jumpedOnly || o.jumped) v.Add(sel(o));
                if (v.Count == 0) { Console.WriteLine($"    {label,-30}{"-",10}"); return; }
                v.Sort();
                Console.WriteLine($"    {label,-30}{v[v.Count / 2],10:F5}{v[(int)(v.Count * 0.9)],10:F5}{v[v.Count - 1],10:F5}");
            }
            Q("オンセット時の貫入深さ", o => o.Item4, false);
            Q("駆動剛体の1F移動量(接触点)", o => o.Item5, false);
            Q("  同 法線成分", o => Math.Abs(o.Item6), false);
            Q("髪側の1F移動量(接触点)", o => o.Item7, false);
            Q("[飛び越え分のみ] 貫入深さ", o => o.Item4, true);
            Q("[飛び越え分のみ] 駆動1F移動量", o => o.Item5, true);
            if (runLens.Count > 0)
            {
                runLens.Sort();
                Console.WriteLine($"    {"貫入が続いたフレーム数",-28}{runLens[runLens.Count / 2],10}{runLens[(int)(runLens.Count * 0.9)],10}{runLens[runLens.Count - 1],10}  (n={runLens.Count})");
            }
            Console.WriteLine($"    ★検出帯 SpeculativeMargin=0.02 / 実効刻み {1.0 / 30.0 / world.SubSteps:F5}s");
            var op = Path.ChangeExtension(outp, null) + "_onset.csv";
            var ob = new StringBuilder("frame,pair,jumped,pen,kinMove,kinNormal,hairMove\n");
            foreach (var o in onsets)
                ob.Append(o.frame.ToString(ci)).Append(',').Append(o.pair.Replace(",", " ")).Append(',')
                  .Append(o.jumped ? 1 : 0).Append(',').Append(o.pen.ToString("R", ci)).Append(',')
                  .Append(o.kinMove.ToString("R", ci)).Append(',').Append(o.kinNormal.ToString("R", ci)).Append(',')
                  .Append(o.hairMove.ToString("R", ci)).Append('\n');
            File.WriteAllText(op, ob.ToString());
            Console.WriteLine($"    [出力] {op}");
        }
        Console.WriteLine($"[出力] {outp} / {Path.ChangeExtension(outp, null)}_kinang.csv");
        return 0;
    }
}
