// ===========================================================================
// SynDrive: タスク25 — 参照VMDが無いモデルで「駆動下の揺れ幅」を A/B する。
//
//  背景: タスク23 の31モデルスイープで モデルS が **駆動なし静止で p90 201.7→4.4 (98%減)**
//        になった。同時に拘束違反も 24分の1 なので「暴れが収まって正しく収束した」とも読めるが、
//        「揺れが死んだ」可能性も残る。**駆動なし静止では動きの豊かさは測れない。**
//        参照VMDが無いので、合成駆動で 既定 vs 3点構成 の**相対比較**を行う。
//
//  ★これは相対比較専用。合成駆動は参照の動きではないので、**忠実度の絶対判定には使えない**。
//    見るのは「同じ駆動を与えたとき、新構成で揺れ幅が保たれるか死ぬか」だけ。
//
//  駆動のかけ方: BoneFollow (ボーン追従) の剛体すべてに **同一の剛体変換 M(t)** を掛ける。
//    KinematicTarget = M(t) * (バインド時の剛体ワールド姿勢)
//    体全体が M(t) で揺れる = センターボーンを振ったのと同じ。FK を通さないので実装が薄く、
//    両条件で厳密に同一の駆動になることが自明。
//
//  ★指標は「駆動系から見た相対姿勢」で測る。M(t) をそのまま含めると体の回転が支配して
//    揺れ物の差が埋もれる。local = M(t)⁻¹ * bodyWorld として、その per-30fps 角速度と
//    per-frame |Δp| を見る。
//
//  env:
//    SYNDRIVE   yaw | sway | both  (必須。これがモード起動のスイッチ)
//    AMPDEG     ヨー振幅 [deg] 既定 40
//    AMPPOS     横移動振幅 [PMX単位] 既定 3
//    PERIOD     周期 [秒] 既定 2.0
//    FRAMES     既定 1800 (30秒 @60)   SUBSTEPS 既定 2   ITERS 既定 10
//    BODIES     対象剛体フィルタ ('*' で全部)
//    WARMUP     計測前に捨てるフレーム 既定 120
//    HOLD       ★駆動目標を N フレームに1回しか更新しない (間は保持)。既定 1 = 毎フレーム更新。
//               Unity で描画が重くなったときの再現: Animator はレンダフレームに1回しかボーンを
//               書かないが FixedUpdate は 60Hz で回るので、ボーンは「N-1 フレーム静止 → 1 フレームで
//               N フレーム分ジャンプ」になる。キネマティック速度は (目標-現在)/dt なので
//               **その1フレームだけ実速度の N 倍のキックが入る**。60fps で N=1、10fps で N=6。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;

static class SynDrive
{
    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static float EnvF(string k, float d)
    {
        float v;
        return float.TryParse(Env(k), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : d;
    }

    const float R2D = 57.29578f;

    sealed class Stat
    {
        public string Label;
        public float WMed, WP90, WMean, WMax;
        public float DpMed, DpP90;
        public float ViolMed, ViolP90, ViolMax;
        public int Bodies;
    }

    static float Med(List<float> v) { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[c.Count / 2]; }
    static float Pct(List<float> v, float q) { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[Math.Min(c.Count - 1, (int)(c.Count * q))]; }

    // RestOsc.JointViolation と同一定義 (拘束されている移動軸方向の誤差だけ。自由DOFは除外)。
    static float Violation(Joint j)
    {
        if (j.BodyA == null || j.BodyB == null) return 0f;
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
        return (float)Math.Sqrt(sq);
    }

    public static int Run(PmxPhysicsModel model, string filter)
    {
        string mode = Env("SYNDRIVE");
        int frames = EnvI("FRAMES", 1800);
        int warmup = EnvI("WARMUP", 120);
        float ampDeg = EnvF("AMPDEG", 40f);
        float ampPos = EnvF("AMPPOS", 3f);
        float period = EnvF("PERIOD", 2f);
        int hold = Math.Max(1, EnvI("HOLD", 1));
        float dt = 1f / 60f;
        var O = new StringBuilder();
        void L(string s = "") { Console.WriteLine(s); O.Append(s); O.Append('\n'); }

        L("=".PadRight(104, '='));
        L("syndrive : 合成駆動での揺れ幅 A/B (既定 vs 3点構成)  " + frames + "F  駆動=" + mode +
          "  振幅 " + ampDeg.ToString("G4") + "deg / " + ampPos.ToString("G4") + "単位  周期 " + period.ToString("G4") + "s" +
          (hold > 1 ? "  ★HOLD=" + hold + " (駆動更新 " + (60f / hold).ToString("G3") + "Hz 相当 = 描画コマ落ちの再現)" : ""));
        L("  ★相対比較専用。合成駆動は参照の動きではないので忠実度の絶対判定には使わない。");
        L("  ★指標は駆動系から見た相対姿勢 (local = M(t)⁻¹ * bodyWorld)。体の回転そのものは含まない。");
        L("=".PadRight(104, '='));

        // 駆動 M(t)
        RigidTransform M(float t)
        {
            float ph = 2f * (float)Math.PI * t / Math.Max(1e-6f, period);
            float s = (float)Math.Sin(ph);
            var q = Quat.Identity;
            var p = Vec3.Zero;
            if (mode == "yaw" || mode == "both")
                q = Quat.FromAxisAngle(new Vec3(0, 1, 0), ampDeg * s / R2D);
            if (mode == "sway" || mode == "both")
                p = new Vec3(ampPos * s, 0, 0);
            return new RigidTransform(q, p);
        }

        Stat Once(string label, Action apply)
        {
            // static つまみは毎回リセットしてから条件を適用する (A/B の取り違え防止)
            Joint.BulletAngleConvention = false;
            Joint.AngularMixedAxes = false;
            Joint.LinearLeverMode = 0;
            Joint.MaxCorrectionVel = 10f;
            apply?.Invoke();

            var builder = PmxPhysicsBuilder.Build(model);
            var world = builder.World;
            world.FixedTimeStep = dt;
            world.SubSteps = EnvI("SUBSTEPS", 2);
            world.SolverIterations = EnvI("ITERS", 10);
            builder.ApplyKinematicTargets(i => (RigidTransform?)null);
            builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

            // バインド時の kinematic 剛体姿勢を控える (これに M(t) を掛けたものが目標)
            var kin = new List<(RigidBody b, RigidTransform bind)>();
            foreach (var b in world.Bodies)
                if (b.Mode == PhysicsMode.BoneFollow) kin.Add((b, b.WorldTransform));

            var target = new List<RigidBody>();
            foreach (var b in world.Bodies)
                if (b.Mode != PhysicsMode.BoneFollow && b.Name != null && (filter == "*" || b.Name.Contains(filter)))
                    target.Add(b);
            if (target.Count == 0) throw new Exception("対象剛体が0: フィルタ '" + filter + "'");
            var names = new HashSet<string>();
            foreach (var t in target) names.Add(t.Name);
            var jw = new List<Joint>();
            foreach (var j in world.Joints)
                if (j.BodyA != null && j.BodyB != null && (names.Contains(j.BodyA.Name) || names.Contains(j.BodyB.Name)))
                    jw.Add(j);

            var ws = new List<float>(); var dps = new List<float>(); var vio = new List<float>();
            var prevQ = new Quat[target.Count];
            var prevP = new Vec3[target.Count];
            bool have = false;
            float wmax = 0f, wsum = 0f; int wn = 0;

            for (int f = 0; f < frames; f++)
            {
                // HOLD>1 のときは駆動時刻を保持境界へ量子化する (レンダフレームでしか更新されない状況)。
                float t = (f / hold) * hold * dt;
                var m = M(t);
                foreach (var (b, bind) in kin) b.KinematicTarget = m * bind;
                world.StepSimulation(dt);

                if (f < warmup) continue;
                var mi = m.Inverse();
                // 30fps 標本 (偶数フレーム) で、駆動系から見た相対姿勢の差を角速度にする
                if ((f & 1) == 0)
                {
                    for (int i = 0; i < target.Count; i++)
                    {
                        var loc = mi * target[i].WorldTransform;
                        var q = loc.Rotation.Normalized;
                        if (have)
                        {
                            var dq = (q * prevQ[i].Conjugated()).Normalized;
                            float w2 = dq.w < 0f ? -dq.w : dq.w;
                            if (w2 > 1f) w2 = 1f;
                            float w = (float)(2.0 * Math.Acos(w2)) * R2D * 30f;
                            ws.Add(w); wsum += w; wn++; if (w > wmax) wmax = w;
                            dps.Add((loc.Origin - prevP[i]).Length);
                        }
                        prevQ[i] = q; prevP[i] = loc.Origin;
                    }
                    have = true;
                }
                foreach (var j in jw) vio.Add(Violation(j));
            }

            return new Stat
            {
                Label = label, Bodies = target.Count,
                WMed = Med(ws), WP90 = Pct(ws, 0.9f), WMean = wn > 0 ? wsum / wn : 0f, WMax = wmax,
                DpMed = Med(dps), DpP90 = Pct(dps, 0.9f),
                ViolMed = Med(vio), ViolP90 = Pct(vio, 0.9f), ViolMax = Pct(vio, 0.999f),
            };
        }

        var oldS = Once("既定", null);
        var newS = Once("3点構成", () =>
        {
            Joint.BulletAngleConvention = true;
            Joint.AngularMixedAxes = true;
            Joint.LinearLeverMode = 1;
        });
        // 後始末 (プロセス内の後続に漏らさない)
        Joint.BulletAngleConvention = false; Joint.AngularMixedAxes = false; Joint.LinearLeverMode = 0;

        L("  対象剛体 = " + oldS.Bodies + " 体  (フィルタ '" + filter + "')");
        L();
        L(string.Format("  {0,-12} {1,10} {2,10} {3,10} {4,10} | {5,12} {6,12} | {7,10} {8,10}",
                        "条件", "|w|中央", "|w|p90", "|w|平均", "|w|最大", "|Δp|中央", "|Δp|p90", "違反中央", "違反p90"));
        void Row(Stat s) => L(string.Format("  {0,-12} {1,10:F2} {2,10:F2} {3,10:F2} {4,10:F1} | {5,12:G5} {6,12:G5} | {7,10:G5} {8,10:G5}",
                                            s.Label, s.WMed, s.WP90, s.WMean, s.WMax, s.DpMed, s.DpP90, s.ViolMed, s.ViolP90));
        Row(oldS); Row(newS);
        L();
        float rW = oldS.WP90 > 1e-9f ? newS.WP90 / oldS.WP90 : float.NaN;
        float rM = oldS.WMed > 1e-9f ? newS.WMed / oldS.WMed : float.NaN;
        float rD = oldS.DpP90 > 1e-12f ? newS.DpP90 / oldS.DpP90 : float.NaN;
        L(string.Format("  比 (3点構成 / 既定):  |w|中央 {0:F3}   |w|p90 {1:F3}   |Δp|p90 {2:F3}   違反p90 {3:F3}",
                        rM, rW, rD, oldS.ViolP90 > 1e-12f ? newS.ViolP90 / oldS.ViolP90 : float.NaN));
        L();
        L("  判定の読み方 (タスク25):");
        L("    ・駆動下でも p90 が保たれる (比 0.8 以上) → 静止時の p90 減は **暴れの正常収束**");
        L("    ・駆動下でも p90 が大きく落ちる (比 0.5 未満) → **痩せの実害**");
        L("    ※ この合成駆動は参照の動きではない。**絶対値ではなく比だけを見ること。**");
        string verdict = float.IsNaN(rW) ? "判定不能 (既定の p90 が 0)"
                       : rW >= 0.8f ? "★駆動下では揺れ幅が保たれている = 静止時の減少は暴れの正常収束と読める"
                       : rW < 0.5f ? "★駆動下でも揺れ幅が落ちている = 痩せの実害の疑い"
                       : "中間 (0.5〜0.8)。要追加判断";
        L("  判定: " + verdict);
        L("=".PadRight(104, '='));
        File.WriteAllText(Env("OUT") ?? "syndrive.txt", O.ToString(), new UTF8Encoding(false));
        return 0;
    }
}
