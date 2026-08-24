// ===========================================================================
// ContactTrace: タスク28 段(a) — 接触の「生成」を Bullet 2.75 と突き合わせるための当エンジン側ダンプ。
//
//  背景: タスク26 で モデルB のスカートの残留振動が **100% 接触由来** と確定した
//        (接触ゼロで |Δp| が 0.0499 → 4.0e-07 と5桁落ちる)。しかも **深貫入 1.0228 が解けない**。
//        接触側の既知逸脱 (slop / 求解順) を Bullet 値へ戻すと**逆に悪化**したので、
//        残る差は「接触行の作り方」ではなく **接触点そのものの生成** の可能性が高い。
//
//  出すもの:
//    CONTACTTRACE=1
//      1) 後半窓で最も深く貫入しているペアの一覧 (深さ順)。最小再現の対象を選ぶため
//      2) 指定した1ペア (PAIR="A|B") の毎サブステップの接触点ダンプ (共通スキーマ)
//
//  共通スキーマ (bulletref 側と同じ列):
//    frame,substep,pt,bodyA,bodyB,pAx..pAz,pBx..pBz,nx,ny,nz,dist,
//    normalBias,pushBias,friction,normalMass,warmNormal,warmT1,warmT2
//
//  env: MMD_TEST_PMX / BODIES / FRAMES(既定1800) / SUBSTEPS(既定2) / ITERS
//       PAIR="剛体A|剛体B"  … 指定すると そのペアだけ毎サブステップ出す
//       TOP=n             … 深いペアの上位n件 (既定 12)
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;

static class ContactTrace
{
    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static string F(float v) => v.ToString("G9", CultureInfo.InvariantCulture);

    public static int Run(PmxPhysicsModel model, string filter)
    {
        int frames = EnvI("FRAMES", 1800);
        int top = EnvI("TOP", 12);
        string pair = Env("PAIR");
        float dt = 1f / 60f;
        var O = new StringBuilder();
        void L(string s = "") { Console.WriteLine(s); O.Append(s); O.Append('\n'); }

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        world.FixedTimeStep = dt;
        world.SubSteps = EnvI("SUBSTEPS", 2);
        world.SolverIterations = EnvI("ITERS", 10);
        builder.ApplyKinematicTargets(i => (RigidTransform?)null);
        builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

        L("=".PadRight(112, '='));
        L("contacttrace : 接触の生成を出す (タスク28 段(a))  " + frames + "F  SubSteps=" + world.SubSteps +
          "  フィルタ='" + filter + "'" + (pair != null ? "  PAIR=" + pair : ""));
        L("=".PadRight(112, '='));

        var rows = new List<(int sub, string a, string b, Vec3 pA, Vec3 pB, Vec3 n,
            float dist, float normalBias, float pushBias, float friction,
            float normalMass, float warmNormal, float warmT1, float warmT2)>();
        world.DebugContactRows = rows;

        // ペアごとの統計 (後半窓)
        var deepest = new Dictionary<string, (float minDist, int pts, int frames, float sumBias)>();
        int lateFrom = frames - frames / 3;
        var csv = new StringBuilder();
        csv.Append("frame,substep,pt,bodyA,bodyB,pAx,pAy,pAz,pBx,pBy,pBz,nx,ny,nz,dist,"
                 + "normalBias,pushBias,friction,normalMass,warmNormal,warmT1,warmT2\n");

        bool Match(string a, string b)
        {
            if (filter != "*" && !a.Contains(filter) && !b.Contains(filter)) return false;
            return true;
        }

        for (int f = 0; f < frames; f++)
        {
            rows.Clear();
            world.StepSimulation(dt);

            // rows は 1フレーム内の全サブステップぶんが順に積まれている。
            // サブステップの区切りは分からないので、ここでは「フレーム内の通し番号」を substep 列に入れる。
            int idx = 0;
            foreach (var r in rows)
            {
                if (!Match(r.a, r.b)) { idx++; continue; }
                string key = string.CompareOrdinal(r.a, r.b) <= 0 ? r.a + "|" + r.b : r.b + "|" + r.a;
                if (f >= lateFrom)
                {
                    deepest.TryGetValue(key, out var st);
                    if (st.pts == 0) st = (float.MaxValue, 0, 0, 0f);
                    st.pts++;
                    if (r.dist < st.minDist) st.minDist = r.dist;
                    st.sumBias += r.normalBias;
                    deepest[key] = st;
                }
                if (pair != null && key == pair)
                {
                    csv.Append(f).Append(',').Append(idx).Append(',').Append(idx).Append(',')
                       .Append(r.a.Replace(',', '_')).Append(',').Append(r.b.Replace(',', '_')).Append(',')
                       .Append(F(r.pA.x)).Append(',').Append(F(r.pA.y)).Append(',').Append(F(r.pA.z)).Append(',')
                       .Append(F(r.pB.x)).Append(',').Append(F(r.pB.y)).Append(',').Append(F(r.pB.z)).Append(',')
                       .Append(F(r.n.x)).Append(',').Append(F(r.n.y)).Append(',').Append(F(r.n.z)).Append(',')
                       .Append(F(r.dist)).Append(',')
                       .Append(F(r.normalBias)).Append(',').Append(F(r.pushBias)).Append(',')
                       .Append(F(r.friction)).Append(',').Append(F(r.normalMass)).Append(',')
                       .Append(F(r.warmNormal)).Append(',').Append(F(r.warmT1)).Append(',').Append(F(r.warmT2)).Append('\n');
                }
                idx++;
            }
        }
        world.DebugContactRows = null;

        var list = new List<(string k, float minDist, int pts, float meanBias)>();
        foreach (var kv in deepest)
            list.Add((kv.Key, kv.Value.minDist, kv.Value.pts, kv.Value.pts > 0 ? kv.Value.sumBias / kv.Value.pts : 0f));
        list.Sort((x, y) => x.minDist.CompareTo(y.minDist));

        L("  後半窓 (F" + lateFrom + "〜) で最も深く貫入しているペア (深さ順, 上位 " + top + "):");
        L(string.Format("  {0,-34} {1,12} {2,10} {3,14}", "ペア", "最深 dist", "点×回数", "平均 normalBias"));
        for (int i = 0; i < Math.Min(top, list.Count); i++)
            L(string.Format("  {0,-34} {1,12:G6} {2,10} {3,14:G6}",
                            list[i].k, list[i].minDist, list[i].pts, list[i].meanBias));
        L();
        L("  接触ペア総数 (後半窓) = " + list.Count);

        if (pair != null)
        {
            string cp = Env("CSV") ?? "contacttrace.csv";
            File.WriteAllText(cp, csv.ToString(), new UTF8Encoding(false));
            L("  ペア '" + pair + "' の接触点ダンプ -> " + cp);
        }
        else
        {
            L("  ★PAIR=\"剛体A|剛体B\" を指定すると、そのペアの毎サブステップの接触点を CSV に出す。");
        }
        L("=".PadRight(112, '='));
        File.WriteAllText(Env("OUT") ?? "contacttrace.txt", O.ToString(), new UTF8Encoding(false));
        return 0;
    }
}
