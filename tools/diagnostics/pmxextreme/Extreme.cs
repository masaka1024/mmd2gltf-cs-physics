// ===========================================================================
// pmxextreme : PMX の物理パラメータのうち「Bullet 2.75 の受け口が受け付けない値」を
//   数え上げる。**読み取りのみ**。物理は一切回さない。
//
//   耐性スレッド(2026-08-25)の事実確認用。異常値の有無をモデル単位で登録し、
//   参照スイートのどのモデルが異常値を持つかを一覧にするために使う。
//
//   検査項目 (Bullet 2.75 実ソースの受け口が課す制約):
//     - damping    : btRigidBody::setDamping が [0,1] へ GEN_clamped する (btRigidBody.cpp:139)
//     - mass       : 負値・非有限。float32 の慣性テンソル/インパルス分母が飽和する規模も報告
//     - friction   : 負値・非有限
//     - restitution: 負値・非有限
//     - size       : 非正・非有限 (形状の半径/半幅。使う成分だけを見る)
//     - limit      : lower > upper の逆転、非有限
//     - spring     : 負値・非有限
//
//   使い方: MMD_TEST_PMX=<pmx>  もしくは  MODELS=<一覧ファイル> で一括。OUT=<txt> で出力先。
// ===========================================================================
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

static class PmxExtreme
{
    static StringBuilder O = new StringBuilder();
    static void L(string s = "") { O.Append(s); O.Append('\n'); Console.WriteLine(s); }
    static string Env(string k) { var v = Environment.GetEnvironmentVariable(k); return string.IsNullOrEmpty(v) ? null : v; }

    // 1モデル分の集計。
    sealed class Rep
    {
        public string Name;
        public int Bodies, Dynamic, Joints;
        public int LinDampOver, AngDampOver;      // >1 の剛体数
        public float LinDampMax = float.MinValue, AngDampMax = float.MinValue;
        public int DampNeg;                        // <0
        public int DampEq1;                        // ちょうど 1.0 (Bullet は完全停止、当エンジンは 0.999 へ丸める)
        public int DampIn999;                      // (0.999, 1.0) の開区間 = 当エンジンのクランプだけが効く帯
        public float DampMaxAll = 0f;              // 全剛体を通じた減衰の最大 (>1 に限らない)
        public int MassNonPos, MassNonFinite;
        public float MassMin = float.MaxValue, MassMax = float.MinValue;
        public int FricBad, RestBad, SizeBad, LimitInv, SpringNeg, NonFinite;
        public List<string> Samples = new List<string>();
        public bool Any => LinDampOver + AngDampOver + DampNeg + MassNonPos + MassNonFinite
                         + FricBad + RestBad + SizeBad + LimitInv + SpringNeg + NonFinite > 0;
    }

    static bool Fin(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    static bool Fin(Vec3 v) => Fin(v.x) && Fin(v.y) && Fin(v.z);

    static Rep Scan(string path)
    {
        var m = PmxReader.LoadFile(path);
        var r = new Rep { Name = Path.GetFileNameWithoutExtension(path), Bodies = m.RigidBodies.Count, Joints = m.Joints.Count };
        for (int i = 0; i < m.RigidBodies.Count; i++)
        {
            var rb = m.RigidBodies[i];
            if (rb.PhysicsMode != 0) r.Dynamic++;

            if (!Fin(rb.LinearDamping) || !Fin(rb.AngularDamping)) r.NonFinite++;
            if (rb.LinearDamping > 1f) { r.LinDampOver++; if (rb.LinearDamping > r.LinDampMax) r.LinDampMax = rb.LinearDamping; }
            if (rb.AngularDamping > 1f) { r.AngDampOver++; if (rb.AngularDamping > r.AngDampMax) r.AngDampMax = rb.AngularDamping; }
            if (rb.LinearDamping < 0f || rb.AngularDamping < 0f) r.DampNeg++;
            if (rb.LinearDamping == 1f) r.DampEq1++;
            if (rb.AngularDamping == 1f) r.DampEq1++;
            if (rb.LinearDamping > 0.999f && rb.LinearDamping < 1f) r.DampIn999++;
            if (rb.AngularDamping > 0.999f && rb.AngularDamping < 1f) r.DampIn999++;
            if (Fin(rb.LinearDamping) && rb.LinearDamping > r.DampMaxAll) r.DampMaxAll = rb.LinearDamping;
            if (Fin(rb.AngularDamping) && rb.AngularDamping > r.DampMaxAll) r.DampMaxAll = rb.AngularDamping;

            if (!Fin(rb.Mass)) r.MassNonFinite++;
            else if (rb.PhysicsMode != 0)
            {
                if (rb.Mass <= 0f) r.MassNonPos++;
                if (rb.Mass < r.MassMin) r.MassMin = rb.Mass;
                if (rb.Mass > r.MassMax) r.MassMax = rb.Mass;
            }

            if (!Fin(rb.Friction) || rb.Friction < 0f) r.FricBad++;
            if (!Fin(rb.Restitution) || rb.Restitution < 0f) r.RestBad++;
            if (!Fin(rb.Size)) r.SizeBad++;
            else
            {
                // 球は X のみ、カプセルは X,Y、箱は X,Y,Z を使う。使う成分だけを見る。
                int n = rb.ShapeType == 0 ? 1 : rb.ShapeType == 2 ? 2 : 3;
                float[] s = { rb.Size.x, rb.Size.y, rb.Size.z };
                for (int k = 0; k < n; k++) if (s[k] <= 0f) { r.SizeBad++; break; }
            }

            bool odd = rb.LinearDamping > 1f || rb.AngularDamping > 1f
                    || (rb.PhysicsMode != 0 && (rb.Mass > 1e6f || rb.Mass <= 0.001f));
            int cap = Env("LIST") == "1" ? int.MaxValue : 6;
            if (odd && r.Samples.Count < cap)
                r.Samples.Add("    #" + i + " '" + rb.Name + "' mode=" + rb.PhysicsMode +
                              " mass=" + rb.Mass.ToString("G6") +
                              " linD=" + rb.LinearDamping.ToString("G6") +
                              " angD=" + rb.AngularDamping.ToString("G6"));
        }
        foreach (var j in m.Joints)
        {
            if (!Fin(j.LinearLowerLimit) || !Fin(j.LinearUpperLimit) ||
                !Fin(j.AngularLowerLimit) || !Fin(j.AngularUpperLimit) ||
                !Fin(j.SpringLinear) || !Fin(j.SpringAngular)) { r.NonFinite++; continue; }
            if (j.LinearLowerLimit.x > j.LinearUpperLimit.x || j.LinearLowerLimit.y > j.LinearUpperLimit.y ||
                j.LinearLowerLimit.z > j.LinearUpperLimit.z || j.AngularLowerLimit.x > j.AngularUpperLimit.x ||
                j.AngularLowerLimit.y > j.AngularUpperLimit.y || j.AngularLowerLimit.z > j.AngularUpperLimit.z) r.LimitInv++;
            if (j.SpringLinear.x < 0 || j.SpringLinear.y < 0 || j.SpringLinear.z < 0 ||
                j.SpringAngular.x < 0 || j.SpringAngular.y < 0 || j.SpringAngular.z < 0) r.SpringNeg++;
        }
        return r;
    }

    static void Head()
    {
        L(string.Format("  {0,-34} {1,5} {2,5} {3,5} | {4,6} {5,6} {6,8} | {7,10} {8,10} | {9,3} {10,3} {11,3} {12,3} {13,3}",
                        "モデル", "剛体", "動的", "Joint", "linD>1", "angD>1", "減衰最大", "質量min", "質量max",
                        "μ", "e", "形", "限", "非") + string.Format("  {0,5} {1,6}", "d=1", "d>.999"));
    }

    static void Row(Rep r)
    {
        float dm = Math.Max(r.LinDampMax == float.MinValue ? 0f : r.LinDampMax,
                            r.AngDampMax == float.MinValue ? 0f : r.AngDampMax);
        string dmax = (r.LinDampOver + r.AngDampOver) == 0 ? "-" : dm.ToString("G5");
        L(string.Format("  {0,-34} {1,5} {2,5} {3,5} | {4,6} {5,6} {6,8} | {7,10} {8,10} | {9,3} {10,3} {11,3} {12,3} {13,3}{14}",
                        Trim(r.Name, 34), r.Bodies, r.Dynamic, r.Joints,
                        r.LinDampOver == 0 ? "-" : r.LinDampOver.ToString(),
                        r.AngDampOver == 0 ? "-" : r.AngDampOver.ToString(), dmax,
                        r.Dynamic == 0 ? "-" : r.MassMin.ToString("G4"),
                        r.Dynamic == 0 ? "-" : r.MassMax.ToString("G4"),
                        r.FricBad, r.RestBad, r.SizeBad, r.LimitInv, r.NonFinite,
                        r.Any ? "  ★" : "") + string.Format("  {0,5} {1,6}", r.DampEq1, r.DampIn999));
        foreach (var s in r.Samples) L(s);
    }

    static string Trim(string s, int n) => s.Length <= n ? s : s.Substring(0, n - 1) + "…";

    static int Main()
    {
        SyncGuard.RequireInSync();   // ★エンジン3複製の同期を先に確かめる (不一致なら実行しない)
        try { Console.OutputEncoding = Encoding.UTF8; } catch { }
        L("=".PadRight(130, '='));
        L("pmxextreme : PMX の物理パラメータのうち Bullet 2.75 の受け口が受け付けない値を数える (読み取りのみ)");
        L("  linD>1 / angD>1 = btRigidBody::setDamping が [0,1] へクランプする範囲の外 (btRigidBody.cpp:139)");
        L("  質量min/max は **動的剛体のみ**。μ/e/形/限/非 は 摩擦・反発・形状サイズ・リミット逆転・非有限 の件数。");
        L("=".PadRight(130, '='));

        var paths = new List<string>();
        string ml = Env("MODELS");
        if (ml != null && File.Exists(ml))
            foreach (var ln in File.ReadAllLines(ml)) { var t = ln.Trim(); if (t.Length > 0 && File.Exists(t)) paths.Add(t); }
        else
        {
            string p = Env("MMD_TEST_PMX") ?? TestData.PmxPath();
            if (p == null || !File.Exists(p)) { L("[SKIP] PMX が無い (MMD_TEST_PMX / MODELS)"); return 0; }
            paths.Add(p);
        }

        Head();
        int flagged = 0;
        foreach (var p in paths)
        {
            try { var r = Scan(p); Row(r); if (r.Any) flagged++; }
            catch (Exception e)
            {
                L(string.Format("  {0,-34} ★読取失敗 {1}: {2}",
                                Trim(Path.GetFileNameWithoutExtension(p), 34), e.GetType().Name, e.Message));
            }
        }
        L();
        L("  => " + paths.Count + "モデル中 " + flagged + "モデルが異常値を持つ");
        File.WriteAllText(Env("OUT") ?? "pmxextreme.txt", O.ToString());
        return 0;
    }
}
