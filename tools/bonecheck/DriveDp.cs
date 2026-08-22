// ===========================================================================
// DriveDp (タスク35): **駆動あり**での per-frame |Δp| を参照(PMXエディタのFixベイク)と
//   部位別に突き合わせる常設ゲート。
//
//  なぜ要るか:
//    既存ゲートの穴が2つあった。
//      - bonecheck/hairfid は モデルA 基準で、モデルA は **ばね定数が全ゼロ** の唯一のモデル。
//        つまり「ばねの実装差」を1ビットも評価できない。
//      - BoneDp は駆動なし静止なので、揺れ物が「動くべきときに動くか」を見ていない。
//    ばね持ちモデル (モデルB) の焼き込み済み参照を使い、
//    **動いている最中の揺れの量** を部位別に比べる。
//
//  ★|Δp| の定義は BoneDp / analyze_static_bake.py / タスク6・9 と完全に同一:
//    - ボーン姿勢は剛体から復元する (bodyWorld * BodyOffsetFromBone の逆)
//    - 30fps 標本 (駆動ループが 1/30 刻みなので毎フレームがそのまま標本)
//    - ボーン別に中央値/p90 を取り、部位ごとにボーン間中央値を代表値にする
//    参照側は同じ式を CSV のボーン姿勢に当てるだけ (剛体復元は不要=既にボーン姿勢)。
//
//  env:
//    DRIVEDP=1        このモードの起動スイッチ (Program.Main の先頭で分岐)
//    MMD_TEST_PMX     対象PMX      MMD_TEST_BONECSV  焼き込み済み参照CSV
//    FRAMES           既定 = CSV の全フレーム        WARMUP  既定 60
//    AB               A/B の軸。既定 "springmotor"。"rotexp" / "none" も可。
//    ANGCONV/AXES/LEVER/SPRINGMOTOR/ROTEXP/SUBSTEPS/ITERS
//                     … 土台として **両条件に同じだけ** 掛ける (被験項ではない)
//    OUT              出力をファイルにも書く
//
//  ★この計測は「軌跡の一致」を求めない (揺れ物はカオス的鋭敏性がある)。
//    見るのは **統計量の比** だけ。1.0 に近いほど参照と同じ揺れ量。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace BoneCheck
{
    static class DriveDp
    {
        /// <summary>出荷既定の控え (タスク37)。エンジンに触る前の静的初期化時点で読む。</summary>
        static string G(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        static readonly bool ShippedSpringMotor = Joint.SpringAsMotorRow;
    // ★2026-08-23: 出荷既定を静的初期化時に控え、env 明示時だけ上書きする
        //   (SpringAsMotorRow で確立した方式を 9 フラグへ広げたもの)。
        static readonly bool ShipAngConv = Joint.BulletAngleConvention;
        static readonly bool ShipAxes    = Joint.AngularMixedAxes;
        static readonly int  ShipLever   = Joint.LinearLeverMode;
        static readonly bool ShipCThresh = GjkEpa.BulletContactThreshold;
        static readonly bool ShipRotExp  = PhysicsWorld.BulletRotationIntegration;
        static readonly bool ShipCMan    = PersistentManifold.BulletManifoldPoints;
        static readonly bool ShipLimGate = Joint.BulletLimitRowGating;
        static readonly bool ShipSymDist = PersistentManifold.SymmetricBreakingDistance;


        static string Env(string k) { return Environment.GetEnvironmentVariable(k); }
        static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }

        static float Med(List<float> v)
        {
            if (v.Count == 0) return float.NaN;
            var c = new List<float>(v); c.Sort(); return c[c.Count / 2];
        }
        static float Pct(List<float> v, float q)
        {
            if (v.Count == 0) return float.NaN;
            var c = new List<float>(v); c.Sort(); return c[Math.Min(c.Count - 1, (int)(c.Count * q))];
        }

        // 部位キー: 左右を落とし、最初の数字/アンダースコアの手前まで。
        //   スカート_0_0 -> スカート / 右髪1 -> 髪 / ﾈｸﾀｲ1 -> ﾈｸﾀｲ
        static readonly Regex LR = new Regex("^[右左]");
        static readonly Regex Head = new Regex("^([^0-9_０-９]+)");
        static string PartOf(string bone)
        {
            string s = LR.Replace(bone, "");
            var m = Head.Match(s);
            return (m.Success && m.Groups[1].Value.Length > 0) ? m.Groups[1].Value : s;
        }

        sealed class Series
        {
            public List<string> Bones = new List<string>();
            public List<List<float>> Dp = new List<List<float>>();   // ボーン別の |Δp| 列
            public bool Bad;
        }

        public static int Run()
        {
            string pmxPath = TestData.PmxPath();
            string csvPath = BoneCsv.FindPath();
            var O = new StringBuilder();
            Action<string> L = delegate (string s) { Console.WriteLine(s); O.Append(s).Append('\n'); };

            if (pmxPath == null || !File.Exists(pmxPath)) { Console.WriteLine("[SKIP] PMX 未検出。"); return 0; }
            if (csvPath == null || !File.Exists(csvPath)) { Console.WriteLine("[SKIP] 参照CSV 未検出 (MMD_TEST_BONECSV)。"); return 0; }

            var model = PmxReader.LoadFile(pmxPath);
            var csv = BoneCsv.Load(csvPath);

            L("".PadRight(112, '='));
            L("drivedp : 駆動ありの per-frame |Δp| を参照と部位別に突き合わせる (タスク35)");
            L("  PMX = " + Path.GetFileName(pmxPath));
            L("  参照 = " + Path.GetFileName(csvPath) + "  (" + csv.BoneCount + " ボーン / " + csv.FrameCount + " フレーム)");
            L("  ★軌跡の一致は求めない。部位ごとの 揺れ量の比 だけを見る (1.00 = 参照と同じ量)。");
            L("".PadRight(112, '='));

            // ─ 参照が焼き込み済みかの検証 ─
            //   素のモーションVMDには揺れ物のキーが無い。dynamic ボーンが CSV に無ければゲートは成立しない。
            var builder0 = PmxPhysicsBuilder.Build(model);
            var dynBones = new List<string>();
            foreach (var l in builder0.BoneLinks)
            {
                if (l.Mode == PhysicsMode.BoneFollow) continue;
                if (l.BoneIndex < 0 || l.BoneIndex >= model.BoneNames.Count) continue;
                string bn = model.BoneNames[l.BoneIndex];
                if (!dynBones.Contains(bn)) dynBones.Add(bn);
            }
            int missing = 0;
            foreach (var b in dynBones) if (!csv.HasBone(b)) missing++;
            L("  物理ボーン " + dynBones.Count + " 本中、参照にキーがあるもの " + (dynBones.Count - missing) + " 本");
            if (missing > 0)
            {
                L("  ★FAIL: 参照CSVに物理ボーンのキーが " + missing + " 本ぶん無い。");
                L("         = この参照は物理を焼き込んでいない。ゲートとして成立しないので中止する。");
                return 1;
            }

            int frames = Math.Min(csv.FrameCount, EnvI("FRAMES", csv.FrameCount));
            int warm = EnvI("WARMUP", 60);
            L("  区間 = " + frames + " フレーム (ウォームアップ " + warm + " ステップ)  刻み 1/30 × SubSteps");
            L("");

            // ─ 参照側の |Δp| ─
            var refS = new Series();
            foreach (var b in dynBones)
            {
                var list = new List<float>();
                RigidTransform prev = default(RigidTransform); bool have = false;
                for (int f = 0; f < frames; f++)
                {
                    RigidTransform bw;
                    if (!csv.TryGet(f, b, out bw)) { have = false; continue; }
                    if (have) list.Add((bw.Origin - prev.Origin).Length);
                    prev = bw; have = true;
                }
                refS.Bones.Add(b); refS.Dp.Add(list);
            }

            // ─ 自前側 ─
            Func<Action, Series> Once = delegate (Action apply)
            {
                // 土台 (両条件に同じだけ掛ける)。A/B 軸だけを apply で切り替える。
                Joint.BulletAngleConvention = Env("ANGCONV") != null ? Env("ANGCONV") == "1" : ShipAngConv;
                Joint.AngularMixedAxes = Env("AXES") != null ? Env("AXES") == "1" : ShipAxes;
                Joint.LinearLeverMode = EnvI("LEVER", ShipLever);
                // ★2026-08-22 (タスク37) 既定が ON になったので、env 未設定のときに false へ
                //   上書きしてはいけない。出荷既定は ShippedSpringMotor に静的初期化時点で控えてある。
                Joint.SpringAsMotorRow = Env("SPRINGMOTOR") != null ? Env("SPRINGMOTOR") == "1" : ShippedSpringMotor;
                PhysicsWorld.BulletRotationIntegration = Env("ROTEXP") != null ? Env("ROTEXP") == "1" : ShipRotExp;
                // タスク38: 接触側の逸脱2件。★CMARGIN は形状の構築時に読むので Build より前に。
                GjkEpa.BulletContactThreshold = Env("CTHRESH") != null ? Env("CTHRESH") == "1" : ShipCThresh;
                CollisionShape.BulletShapeMargin = Env("CMARGIN") == "1";
                string crhsEnv = Env("CRHS");   // ★タスク48 (world 生成後に適用・未設定=出荷既定)
                PersistentManifold.BulletManifoldPoints = Env("CMAN") != null ? Env("CMAN") == "1" : ShipCMan;   // ★タスク51
                Joint.BulletLimitRowGating = Env("LIMGATE") != null ? Env("LIMGATE") == "1" : ShipLimGate;   // ★タスク59
                PersistentManifold.SymmetricBreakingDistance = Env("SYMDIST") != null ? Env("SYMDIST") == "1" : ShipSymDist;   // ★タスク67
                if (apply != null) apply();

                var builder = PmxPhysicsBuilder.Build(model);
                var world = builder.World;
                world.SubSteps = EnvI("SUBSTEPS", world.SubSteps);
                world.SolverIterations = EnvI("ITERS", world.SolverIterations);
                if (crhsEnv != null) world.ContactRhsBullet = crhsEnv == "1";
                // ★タスク71: 配線漏れの修正。drivedp も摩擦セットと求解順を受け付けていなかった。
                if (Env("CSET") == "1")
                { world.ContactPoolOrder = true; world.FrictionVelocityAligned = true; world.FrictionCombineMultiply = true; }
                if (Env("CPOOL") == "1") world.ContactPoolOrder = true;
                if (Env("NORMFIRST") == "1") world.ContactNormalBeforeFriction = true;
                if (Env("FRICALIGN") == "1") world.FrictionVelocityAligned = true;
                if (Env("FRICMUL") == "1") world.FrictionCombineMultiply = true;
                if (Env("JOINTS_FIRST") == "1") world.SolveJointsFirst = true;
                {
                    float bv;
                    if (float.TryParse(Env("BAUM"), NumberStyles.Float, CultureInfo.InvariantCulture, out bv) && bv >= 0f)
                        world.BaumgarteFactor = bv;
                }
                Console.WriteLine("  [実効] drivedp  AngConv=" + Joint.BulletAngleConvention + "  MixedAxes=" + Joint.AngularMixedAxes
                    + "  Lever=" + Joint.LinearLeverMode + "  CThresh=" + GjkEpa.BulletContactThreshold
                    + "  CRhs=" + world.ContactRhsBullet + "  CMan=" + PersistentManifold.BulletManifoldPoints
                    + "  LimGate=" + Joint.BulletLimitRowGating + "  SymDist=" + PersistentManifold.SymmetricBreakingDistance
                    + "  PoolOrder=" + world.ContactPoolOrder + "  NormalFirst=" + world.ContactNormalBeforeFriction
                    + "  FricAligned=" + world.FrictionVelocityAligned + "  FricMul=" + world.FrictionCombineMultiply
                    + "  JointsFirst=" + world.SolveJointsFirst + "  ContactBaumgarte=" + world.BaumgarteFactor);

                var links = new List<BoneLink>();
                var names = new List<string>();
                foreach (var l in builder.BoneLinks)
                {
                    if (l.Mode == PhysicsMode.BoneFollow) continue;
                    if (l.BoneIndex < 0 || l.BoneIndex >= model.BoneNames.Count) continue;
                    links.Add(l); names.Add(model.BoneNames[l.BoneIndex]);
                }

                int fcur = 0;
                Func<int, RigidTransform?> at = delegate (int bi)
                {
                    RigidTransform bw;
                    if (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(fcur, model.BoneNames[bi], out bw))
                        return (RigidTransform?)bw;
                    return null;
                };
                builder.ApplyKinematicTargets(at);
                builder.ResetBodiesToBonePoseFk(at);
                for (int s = 0; s < warm; s++) world.StepSimulation(1f / 30f);

                var r = new Series();
                var series = new List<float>[links.Count];
                for (int i = 0; i < links.Count; i++) series[i] = new List<float>();
                var prev = new Vec3[links.Count];
                bool have = false;
                // ★タスク73: 症状窓を探すための毎フレーム出力と、その窓を移植するための
                //   全剛体スナップショット。どちらも env 未設定なら何もしない。
                int snapAt = EnvI("STATEDUMP_AT", -1);
                string snapOut = Env("STATEDUMP_OUT");
                for (int f = 0; f < frames; f++)
                {
                    fcur = f;
                    builder.ApplyKinematicTargets(at);
                    if (f == snapAt && !string.IsNullOrEmpty(snapOut))
                    {
                        // bulletref の --initstate と同じ書式 (ステップ**前**の状態)
                        var sbq = new StringBuilder();
                        sbq.Append("# drivedp snapshot at frame ").Append(f)
                           .Append('\n');
                        for (int bi = 0; bi < world.Bodies.Count; bi++)
                        {
                            var bd = world.Bodies[bi];
                            var tr = bd.WorldTransform; var q = tr.Rotation; var o = tr.Origin;
                            var lv = bd.LinearVelocity; var av = bd.AngularVelocity;
                            sbq.Append("state ").Append(bi).Append(" name=").Append(bd.Name.Replace(' ', '_'))
                               .Append(" pos=").Append(G(o.x)).Append(' ').Append(G(o.y)).Append(' ').Append(G(o.z))
                               .Append(" quat=").Append(G(q.x)).Append(' ').Append(G(q.y)).Append(' ').Append(G(q.z)).Append(' ').Append(G(q.w))
                               .Append(" linvel=").Append(G(lv.x)).Append(' ').Append(G(lv.y)).Append(' ').Append(G(lv.z))
                               .Append(" angvel=").Append(G(av.x)).Append(' ').Append(G(av.y)).Append(' ').Append(G(av.z))
                               .Append('\n');
                        }
                        File.WriteAllText(snapOut, sbq.ToString(), new UTF8Encoding(false));
                        Console.WriteLine("  -> " + snapOut + " (frame " + f + " / " + world.Bodies.Count + " 体)");
                    }
                    world.StepSimulation(1f / 30f);
                    for (int i = 0; i < links.Count; i++)
                    {
                        var bone = links[i].Body.WorldTransform * links[i].BodyOffsetFromBone.Inverse();
                        if (have) series[i].Add((bone.Origin - prev[i]).Length);
                        prev[i] = bone.Origin;
                    }
                    have = true;
                }
                for (int i = 0; i < links.Count; i++) { r.Bones.Add(names[i]); r.Dp.Add(series[i]); }
                foreach (var l in links)
                {
                    var o = l.Body.WorldTransform.Origin;
                    if (float.IsNaN(o.x + o.y + o.z) || Math.Abs(o.x) > 1e6f) r.Bad = true;
                }
                return r;
            };

            string ab = (Env("AB") ?? "springmotor").ToLowerInvariant();
            var labels = new List<string>();
            var applies = new List<Action>();
            if (ab == "none") { labels.Add("自前"); applies.Add(null); }
            else if (ab == "rotexp")
            {
                labels.Add("RotExp OFF"); applies.Add(delegate { PhysicsWorld.BulletRotationIntegration = false; });
                labels.Add("RotExp ON"); applies.Add(delegate { PhysicsWorld.BulletRotationIntegration = true; });
            }
            else
            {
                labels.Add("モータ行 OFF"); applies.Add(delegate { Joint.SpringAsMotorRow = false; });
                labels.Add("モータ行 ON"); applies.Add(delegate { Joint.SpringAsMotorRow = true; });
            }

            var results = new List<Series>();
            for (int i = 0; i < labels.Count; i++)
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var s = Once(applies[i]);
                sw.Stop();
                results.Add(s);
                L("  [" + labels[i] + "] 実行 " + sw.Elapsed.TotalSeconds.ToString("F1") + " 秒"
                  + (s.Bad ? "   ★NaN/発散を検出" : ""));
            }
            Joint.BulletAngleConvention = false; Joint.AngularMixedAxes = false;
            Joint.LinearLeverMode = 0; Joint.SpringAsMotorRow = ShippedSpringMotor;
            PhysicsWorld.BulletRotationIntegration = false;
            L("");

            // ─ 部位別集計 ─
            var parts = new List<string>();
            var partOf = new Dictionary<string, string>();
            foreach (var b in dynBones)
            {
                string p = PartOf(b); partOf[b] = p;
                if (!parts.Contains(p)) parts.Add(p);
            }
            parts.Sort(StringComparer.Ordinal);

            Func<Series, string, bool, float> Agg = delegate (Series s, string part, bool p90)
            {
                var per = new List<float>();
                for (int i = 0; i < s.Bones.Count; i++)
                {
                    string pp;
                    if (!partOf.TryGetValue(s.Bones[i], out pp) || pp != part) continue;
                    per.Add(p90 ? Pct(s.Dp[i], 0.9f) : Med(s.Dp[i]));
                }
                return Med(per);
            };
            Func<string, int> Count = delegate (string part)
            {
                int n = 0;
                foreach (var kv in partOf) if (kv.Value == part) n++;
                return n;
            };

            var hdr = new StringBuilder();
            hdr.Append(string.Format("  {0,-8} {1,4} | {2,9} {3,9} |", "部位", "本数", "参照 中央", "参照 p90"));
            for (int i = 0; i < labels.Count; i++)
                hdr.Append(string.Format(" {0,9} {1,6} {2,9} {3,6} |", "中央", "比", "p90", "比"));
            var sub = new StringBuilder();
            sub.Append(string.Format("  {0,-8} {1,4} | {2,9} {3,9} |", "", "", "", ""));
            for (int i = 0; i < labels.Count; i++)
                sub.Append(string.Format(" {0,-33} |", labels[i]));
            L(sub.ToString());
            L(hdr.ToString());
            L("  " + "".PadRight(hdr.Length - 2, '-'));
            foreach (var part in parts)
            {
                float rm = Agg(refS, part, false), rp = Agg(refS, part, true);
                var line = new StringBuilder();
                line.Append(string.Format("  {0,-8} {1,4} | {2,9:F4} {3,9:F4} |", part, Count(part), rm, rp));
                for (int i = 0; i < results.Count; i++)
                {
                    float em = Agg(results[i], part, false), ep = Agg(results[i], part, true);
                    line.Append(string.Format(" {0,9:F4} {1,6:F2} {2,9:F4} {3,6:F2} |",
                        em, rm > 0 ? em / rm : float.NaN, ep, rp > 0 ? ep / rp : float.NaN));
                }
                L(line.ToString());
            }
            L("");
            L("  読み方: 比 > 1 = 参照より動きすぎ / < 1 = 動かなさすぎ。中央は平時の揺れ量、p90 は大振れ側。");
            L("  ★ばね系の変更はこのゲートを通すこと (モデルA は spring 全ゼロで、ばねの差を評価できない)。");

            // ★ボーン別の生値を出す (部位統計の標本不確かさをボーン・ブートストラップで見積もるため。
            //   もみあげ4本/アホ毛2本のような少数部位は中央値の精度自体が低く、
            //   カオス感度だけでは許容を見積もれない = タスク36 の判定で 0.001 差を退行と誤判定した)。
            string bcsv = Env("BONECSV_OUT");
            if (!string.IsNullOrEmpty(bcsv))
            {
                var sb = new StringBuilder();
                sb.Append("bone,part,cond,med,p90\n");
                for (int i = 0; i < refS.Bones.Count; i++)
                    sb.Append(refS.Bones[i]).Append(',').Append(partOf[refS.Bones[i]]).Append(",参照,")
                      .Append(Med(refS.Dp[i]).ToString("G9")).Append(',')
                      .Append(Pct(refS.Dp[i], 0.9f).ToString("G9")).Append('\n');
                for (int c = 0; c < results.Count; c++)
                    for (int i = 0; i < results[c].Bones.Count; i++)
                        sb.Append(results[c].Bones[i]).Append(',').Append(partOf[results[c].Bones[i]]).Append(',')
                          .Append(labels[c].Replace(',', '_')).Append(',')
                          .Append(Med(results[c].Dp[i]).ToString("G9")).Append(',')
                          .Append(Pct(results[c].Dp[i], 0.9f).ToString("G9")).Append('\n');
                File.WriteAllText(bcsv, sb.ToString(), new UTF8Encoding(false));
                L("  -> " + bcsv + " (ボーン別の生値)");
            }

            // ★タスク73: 毎フレームの |Δp| (部位で絞れる)。症状窓の特定用。
            string dpf = Env("DPFRAMES_OUT");
            if (!string.IsNullOrEmpty(dpf))
            {
                string want = Env("DPPART");
                var sb = new StringBuilder();
                sb.Append("frame,bone,part,cond,dp\n");
                void Emit(string cond, Series ss)
                {
                    for (int i = 0; i < ss.Bones.Count; i++)
                    {
                        string pt = partOf[ss.Bones[i]];
                        if (!string.IsNullOrEmpty(want) && pt != want) continue;
                        var seq = ss.Dp[i];
                        for (int f = 0; f < seq.Count; f++)
                            sb.Append(f).Append(',').Append(ss.Bones[i]).Append(',').Append(pt).Append(',')
                              .Append(cond.Replace(',', '_')).Append(',').Append(seq[f].ToString("G9")).Append('\n');
                    }
                }
                Emit("参照", refS);
                for (int c = 0; c < results.Count; c++) Emit(labels[c], results[c]);
                File.WriteAllText(dpf, sb.ToString(), new UTF8Encoding(false));
                L("  -> " + dpf + " (毎フレームの |Δp|" + (string.IsNullOrEmpty(want) ? "" : " / 部位=" + want) + ")");
            }

            string outp = Env("OUT");
            if (!string.IsNullOrEmpty(outp)) File.WriteAllText(outp, O.ToString(), new UTF8Encoding(false));
            return 0;
        }
    }
}
