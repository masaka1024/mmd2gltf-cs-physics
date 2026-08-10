// タスクC(調査のみ・修正なし): スカート×太もも の持続的な食い込みを切り分ける。
//   1) MMDでも食い込むか (CSVボーンから剛体を復元し、自前と同じ narrowphase で貫入算出)
//   2) バインドポーズで既に食い込んでいるか (物理0ステップ)
//   3) 常習ペアのリング/列別一覧
//   4) 接触が解けない理由 (ShouldCollide / 接触検出 / 法線インパルス / NormalBias / 実効質量)
// 貫入は GjkEpa.Detect(a,b) の最小 Distance から算出 (自前もMMDも同一計算)。本体無改変。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class SkirtThigh
{
    static readonly string Pmx1 = TestData.PmxPath();
    static readonly string Pmx2 = TestData.PmxPath();
    const float FRAME = 1f / 30f;
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    static readonly string[] LegNames = { "下半身", "下半身1", "下半身3", "下半身4", "右太もも", "左太もも", "左足", "右足", "右ひざ", "左ひざ" };

    static BoneCsv csv; static PmxPhysicsModel model; static int F;
    static PmxPhysicsBuilder builder; static List<RigidBody> skirtRb = new(); static List<RigidBody> legRb = new();
    static Dictionary<RigidBody, (string bone, RigidTransform off)> link = new();
    static List<ContactPoint> buf = new();

    // GjkEpa で a,b の貫入深さ(>0)を返す。接触なしは0。
    static float PenDepth(RigidBody a, RigidBody b)
    {
        buf.Clear(); GjkEpa.Detect(a, b, buf);
        float d = 0; foreach (var cp in buf) { float pen = -cp.Distance; if (pen > d) d = pen; } return d;
    }

    static RigidTransform RefWorld(RigidBody rb, int f)
    {
        var (bone, off) = link[rb];
        if (csv.TryGet(f, bone, out var bw)) return bw * off;
        return rb.WorldTransform;
    }

    static int Main()
    {
        string pmx = File.Exists(Pmx1) ? Pmx1 : Pmx2;
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP]"); return 0; }
        csv = BoneCsv.Load(csvPath); model = PmxReader.LoadFile(pmx); F = csv.FrameCount;
        builder = PmxPhysicsBuilder.Build(model);
        for (int i = 0; i < builder.Bodies.Count; i++)
        {
            var b = builder.Bodies[i]; var ln = builder.BoneLinks[i];
            string bone = (ln.BoneIndex >= 0 && ln.BoneIndex < model.BoneNames.Count) ? model.BoneNames[ln.BoneIndex] : "";
            link[b] = (bone, ln.BodyOffsetFromBone);
            if (b.Name.StartsWith("スカート") && !b.IsStaticOrKinematic) skirtRb.Add(b);
            if (LegNames.Contains(b.Name)) legRb.Add(b);
        }
        L("==================== スカート×太もも 持続食い込み 診断 ====================");
        L($"スカート動的剛体={skirtRb.Count} 脚コライダー={legRb.Count} ({string.Join(",", legRb.Select(r => r.Name))})");

        // ---- 2) バインドポーズ (物理0ステップ) の貫入 ----
        L("\n[2] バインドポーズ (物理を回さない初期状態) のスカート×脚 貫入 (>0.05)");
        var bindHits = new List<(string s, string l, float d)>();
        foreach (var s in skirtRb) foreach (var l in legRb) { float d = PenDepth(s, l); if (d > 0.05f) bindHits.Add((s.Name, l.Name, d)); }
        foreach (var h in bindHits.OrderByDescending(x => x.d).Take(20)) L($"   {h.s} × {h.l} : 深さ={h.d:F4}");
        L($"   バインドで貫入(>0.05)するペア数={bindHits.Count}  (スカート_1_5×左太もも 含む: {bindHits.Any(h => h.s == "スカート_1_5" && h.l == "左太もも")})");

        // ---- 1)&3) MMD vs 自前 の貫入 (全フレーム) ----
        // MMD: CSVボーン*offset に剛体を置いて Detect。自前: 物理を回して Detect。脚は両方CSV駆動で同一。
        var pairSelfMean = new Dictionary<string, double>(); var pairSelfMax = new Dictionary<string, float>();
        var pairRefMean = new Dictionary<string, double>(); var pairRefMax = new Dictionary<string, float>();
        var pairN = new Dictionary<string, int>();
        void Acc(Dictionary<string, double> mean, Dictionary<string, float> max, string k, float v) { mean[k] = mean.GetValueOrDefault(k) + v; if (v > max.GetValueOrDefault(k)) max[k] = v; }

        // MMDパス: 全剛体をCSV姿勢に置く
        var saved = builder.Bodies.Select(b => b.WorldTransform).ToArray();
        for (int f = 0; f < F; f++)
        {
            foreach (var s in skirtRb) s.WorldTransform = RefWorld(s, f);
            foreach (var l in legRb) l.WorldTransform = RefWorld(l, f);
            foreach (var s in skirtRb) foreach (var l in legRb)
            { if (!AabbNear(s, l)) continue; float d = PenDepth(s, l); if (d > 0.02f) { string k = s.Name + "×" + l.Name; Acc(pairRefMean, pairRefMax, k, d); } }
        }
        for (int i = 0; i < builder.Bodies.Count; i++) builder.Bodies[i].WorldTransform = saved[i];

        // 自前パス: 物理(実効1/60=Sub2)で駆動
        builder = PmxPhysicsBuilder.Build(model); // 作り直し(MMDパスでTransformを弄ったため)
        skirtRb.Clear(); legRb.Clear(); link.Clear();
        for (int i = 0; i < builder.Bodies.Count; i++)
        {
            var b = builder.Bodies[i]; var ln = builder.BoneLinks[i];
            string bone = (ln.BoneIndex >= 0 && ln.BoneIndex < model.BoneNames.Count) ? model.BoneNames[ln.BoneIndex] : "";
            link[b] = (bone, ln.BodyOffsetFromBone);
            if (b.Name.StartsWith("スカート") && !b.IsStaticOrKinematic) skirtRb.Add(b);
            if (LegNames.Contains(b.Name)) legRb.Add(b);
        }
        var world = builder.World; world.SolverIterations = 10; world.FixedTimeStep = FRAME; world.SubSteps = 2;
        var dbg = new List<(string a, string b, float dist, float ni)>(); world.DebugContacts = dbg;
        var driven = new List<(BoneLink l, string bone)>();
        foreach (var l in builder.BoneLinks) if (l.Mode == PhysicsMode.BoneFollow && l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[l.BoneIndex])) driven.Add((l, model.BoneNames[l.BoneIndex]));
        void Apply(int f) { foreach (var (l, bn) in driven) if (csv.TryGet(f, bn, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(FRAME);

        // #4 用ダンプ対象
        var d4 = new List<string>();
        RigidBody sk15 = skirtRb.FirstOrDefault(r => r.Name == "スカート_1_5"); RigidBody th = legRb.FirstOrDefault(r => r.Name == "左太もも");
        bool sc = (sk15 != null && th != null) && PhysicsWorld.ShouldCollide(sk15, th);

        for (int f = 0; f < F; f++)
        {
            Apply(f); dbg.Clear(); world.StepSimulation(FRAME);
            foreach (var s in skirtRb) foreach (var l in legRb)
            { if (!AabbNear(s, l)) continue; float d = PenDepth(s, l); if (d > 0.02f) { string k = s.Name + "×" + l.Name; Acc(pairSelfMean, pairSelfMax, k, d); pairN[k] = pairN.GetValueOrDefault(k) + 1; } }
            // #4: スカート_1_5×左太もも の接触状態を F4..12 でダンプ
            if (f >= 4 && f <= 12 && sk15 != null && th != null)
            {
                float dep = PenDepth(sk15, th);
                float ni = 0; bool found = false;
                foreach (var c in dbg) if ((c.a == "スカート_1_5" && c.b == "左太もも") || (c.a == "左太もも" && c.b == "スカート_1_5")) { ni += c.ni; found = true; }
                float bias = dep > 0.005f ? 0.2f * (dep - 0.005f) / (FRAME / 2) : 0f; // Baumgarte: 0.2*(pen-slop)/dt, dt=1/60
                float em = EffMassNormal(sk15, th);
                d4.Add($"   F{f}: 貫入={dep:F4} 接触検出={(found ? "有" : "無")} 法線ni={ni:F4} NormalBias={bias:F3} 実効質量={em:F4}");
            }
        }

        // ---- 3) 常習ペア ランキング (自前) ----
        L("\n[3] スカート×脚 の持続食い込み 上位 (自前, 平均深さ, 出現フレーム数)");
        foreach (var kv in pairSelfMean.OrderByDescending(k => k.Value / Math.Max(1, pairN.GetValueOrDefault(k.Key))).Take(16))
        { int n = pairN.GetValueOrDefault(kv.Key); L($"   {kv.Key,-22} 平均={kv.Value / Math.Max(1, n):F4} 最大={pairSelfMax[kv.Key]:F4} 出現={n}F"); }

        // ---- 1) MMD vs 自前 の並記 (主要ペア) ----
        L("\n[1] MMD vs 自前 の貫入 (同一計算, 平均/最大 深さ)  ★食い込みがMMD由来か自前固有かの判定");
        var keys = pairSelfMean.Keys.Union(pairRefMean.Keys).OrderByDescending(k => Math.Max(pairSelfMax.GetValueOrDefault(k), pairRefMax.GetValueOrDefault(k))).Take(14);
        L("   ペア                     | 自前 平均/最大   | MMD 平均/最大");
        foreach (var k in keys)
        {
            int nS = pairN.GetValueOrDefault(k);
            double sm = pairSelfMean.GetValueOrDefault(k) / Math.Max(1, nS);
            L($"   {k,-22} | {sm,6:F4}/{pairSelfMax.GetValueOrDefault(k),6:F4} | {(pairRefMean.ContainsKey(k) ? (pairRefMean[k] / F).ToString("F4") : "  -   ")}/{pairRefMax.GetValueOrDefault(k),6:F4}");
        }
        L("   (MMD平均は全Fで割った値。剛体位置はMMDベイクCSVボーン*offsetで復元し自前と同一のGjkEpaで測定)");

        // ---- 4) 接触が解けない理由 ----
        L("\n[4] スカート_1_5×左太もも の接触状態 (F4..12)");
        L($"   ShouldCollide(衝突フィルタ) = {sc} {(sc ? "(衝突対象=接触は解かれるべき)" : "(衝突対象外=貫入は未解決の幾何重なり)")}");
        foreach (var s in d4) L(s);
        L("   判定材料: 接触検出=有 かつ ni>0 なら「押し戻す力は出ているが戻らない」(定常釣り合い)。");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "skirtthigh_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    static bool AabbNear(RigidBody a, RigidBody b)
    {
        var aa = a.ComputeAabb(); var bb = b.ComputeAabb(); return aa.Intersects(ref bb);
    }

    // 法線方向の実効質量 (接触点は Detect の最深点)。thigh は kinematic(invMass=0)。
    static float EffMassNormal(RigidBody a, RigidBody b)
    {
        buf.Clear(); GjkEpa.Detect(a, b, buf); if (buf.Count == 0) return 0;
        ContactPoint cp = buf[0]; float best = float.MaxValue; foreach (var c in buf) if (c.Distance < best) { best = c.Distance; cp = c; }
        var n = cp.Normal; var rA = cp.PositionWorldA - a.CenterOfMass; var rB = cp.PositionWorldB - b.CenterOfMass;
        var rAxn = Vec3.Cross(rA, n); var rBxn = Vec3.Cross(rB, n);
        float k = a.InverseMass + b.InverseMass + rAxn.Dot(a.InverseInertiaWorld * rAxn) + rBxn.Dot(b.InverseInertiaWorld * rBxn);
        return k > 0 ? 1f / k : 0f;
    }
}
