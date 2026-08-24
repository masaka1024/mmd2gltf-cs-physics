// ===========================================================================
// restosc: 「静止しているはずの揺れ物が止まらない」を数値で捕まえる診断ツール。
//
// 前提: 駆動なし (モーション無し・バインド姿勢固定)。重力+ジョイント+接触だけの自律系。
//       この条件で振動が持続するなら、必ずどこかが毎ステップ仕事をしている。
//       (減衰があるので自由振動=共振は必ず死ぬ)
//
// 出す数字:
//   - 対象剛体の |角速度| の時系列 -> 減衰 / 一定振幅(リミットサイクル) / 発散 の判別
//   - 後半窓の平均|w| = 「収まったか」の一意な指標
//   - 自己相関でリミットサイクルの周期を推定 -> ステップ数の整数倍なら毎ステップ注入
//   - 接触点数・最大貫入・法線インパルスの時系列 -> 接触が振動と同期しているか
//   - 全ジョイント/全接触を止めたときの比較 (エネルギー源の切り分け)
//
// 本体は無改変。接触は読み取り専用フック DebugContacts から取る。
//
// env (既定は Unity の MmdPhysicsBehaviour と同じ = 現行の見た目を再現):
//   MMD_TEST_PMX  対象PMX
//   BODIES        対象剛体の名前に含まれる文字列 (既定 ﾈｸﾀｲ)
//   FRAMES        フレーム数 (既定 900 = 15秒 @60)
//   SUBSTEPS      既定 2      ITERS 既定 10     FPS 既定 60
//   AB            切り分けスイープを回す (1=する)
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class RestOsc
{
    static StringBuilder O = new StringBuilder();
    static void L(string s = "") { Console.WriteLine(s); O.Append(s); O.Append('\n'); }

    // ★2026-08-23 完全セットv1 の既定化にともなう統一:
    //   出荷既定を **静的初期化時に控え**、env が明示されたときだけ上書きする。
    //   無条件に `Env("F") == "1"` を代入していると、env 未設定で新既定を打ち消してしまう。
    //   (SpringAsMotorRow で先に確立した方式をそのまま 9 フラグへ広げる)
    static readonly bool ShipAngConv  = Joint.BulletAngleConvention;
    static readonly bool ShipAxes     = Joint.AngularMixedAxes;
    static readonly int  ShipLever    = Joint.LinearLeverMode;
    static readonly bool ShipCThresh  = GjkEpa.BulletContactThreshold;
    static readonly bool ShipRotExp   = PhysicsWorld.BulletRotationIntegration;
    static readonly bool ShipCMan     = PersistentManifold.BulletManifoldPoints;
    static readonly bool ShipLimGate  = Joint.BulletLimitRowGating;
    static readonly bool ShipSymDist  = PersistentManifold.SymmetricBreakingDistance;
    // ★タスク78: 腕長ゲートと摩擦合成も出荷既定を控える (env 明示時だけ上書き。0 も効く)。
    static readonly float ShipLeverGate = Joint.LeverArmGate;
    static readonly bool ShipFricMul = new PhysicsWorld().FrictionCombineMultiply;

    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static float EnvF(string k, float d)
    {
        float v;
        return float.TryParse(Env(k), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : d;
    }

    sealed class Result
    {
        public string Label;
        public float[] MeanW;        // フレームごとの 対象剛体 平均|角速度| (deg/s)
        public float[] MaxW;
        public float[] ContactN;     // 対象剛体に付いた接触点数
        public float[] MaxPen;       // 最大貫入 (正=めり込み)
        public float[] SumNi;        // 法線インパルス合計
        public float LateMeanW;      // 後半1/3の平均|w| = 収束指標
        public float EarlyMeanW;
        public int Period;           // 自己相関で推定した周期 (フレーム)
        public float PenMax;         // 全フレーム最大の貫入
        public string PenPair;       // そのときの相手
        public float PenLate;        // 後半1/3の最大貫入
        // ★2026-08-21 差し替え: 旧「伸び」= 重心間距離/バインド距離 は廃止した。
        //   あの指標は移動DOFが自由なジョイント (房をまたぐ横渡し等) の正当な滑りを
        //   拘束違反として数えてしまう。IA の最悪 髪BR3縦_J は移動X/Zが範囲[-0.851,0.851]で
        //   自由なのに比1.80と出ていた (アンカー誤差は許容範囲の内側、残差ゼロで完全収束)。
        //   新指標 = アンカー誤差: 拘束されている移動軸方向の成分だけを取る (自由DOFは除外)。
        //   単位は PMX 単位の絶対量。0 が正しい。過去のスイープの「伸び」列は無効。
        public float ViolMax;        // 拘束違反(アンカー誤差)の最大
        public float ViolLate;       // 同 後半1/3の最大
        public string ViolWorst;     // 最大時のジョイント名
        public float ViolMed;        // ★ジョイント別「後半窓の最大違反」の中央値
        public float ViolP90;
        public ulong PoseHash;       // 全剛体の最終姿勢のハッシュ (スモークのビット比較用)
        public float LateMedW, LateP90W, LateMaxW;  // ★後半窓の |w| 分布 (剛体×フレームの全サンプル)
        public float LateQuietFrac;                 // 同 |w| < 5 deg/s のサンプル割合 = 「止まっている毛の割合」
        // ★30fpsサンプル版: 2フレームおきの姿勢差から出した |w|。VMDが記録できるのはこれ。
        //   当エンジンの振動は周期2フレーム(=30Hz)なので、30fpsサンプルでは原理的にエイリアスして消える。
        //   リファレンス(PMXエディタのベイクVMD)は30fps記録なので、比較はこちらで行うのが正しい。
        public float S30MeanW, S30MedW, S30P90W, S30MaxW, S30QuietFrac;
        // ★相対角速度: ジョイントが繋ぐ2剛体の |wB - wA|。ジョイントが実際に制御している量。
        //   絶対|w|が大きくても相対が小さければ「鎖全体が一緒に揺れている」= ジョイントは無罪。
        public float RelMeanW, RelMedW, RelP90W;
    }

    static PmxPhysicsModel _model;
    static string _filter;
    static float _baum = -1f;   // >=0 でBaumgarteFactorを上書き
    static float _jbeta = -1f;  // >=0 でJoint.Betaを上書き

    static Result Run(string label, Action<PhysicsWorld, PmxPhysicsBuilder> tweak)
    {
        int frames = EnvI("FRAMES", 900);
        int fps = EnvI("FPS", 60);
        float dt = 1f / fps;

        var builder = PmxPhysicsBuilder.Build(_model);
        var world = builder.World;
        // Unity 既定 (MmdPhysicsBehaviour) に合わせる
        world.FixedTimeStep = 1f / 60f;
        world.SubSteps = EnvI("SUBSTEPS", 2);
        world.SolverIterations = EnvI("ITERS", 10);
        // static つまみは実行ごとにリセット (A/B の取り違え防止)
        Joint.MaxCorrectionVel = 10f;
        if (_baum >= 0f) world.BaumgarteFactor = _baum;
        if (Env("CRHS") != null) world.ContactRhsBullet = Env("CRHS") == "1";   // ★タスク48 (未設定=出荷既定)
        world.FrictionCombineMultiply = Env("FRICMUL") != null ? Env("FRICMUL") == "1" : ShipFricMul;   // ★タスク78 (未設定=出荷既定)
        // ★2026-08-21: 単体モードで JSPLIT が黙って無視されていたのを修正 (JBETA と同じ取りこぼし)。
        //   AB モードの各条件は tweak で上書きするので影響しない。
        if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        // ジョイント warm-start の A/B (JWARM=lin | both)。係数は Joint.WarmStartFactor(0.85)。
        {
            string wm = Env("JWARM");
            if (wm == "lin") world.UseJointWarmStart = true;
            else if (wm == "both") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        }
        // ★計測系の健全性確認 (DUMMY=1): 計測に無関係な遠方剛体を1体足す。
        //   指標が1ビットも動かなければ、ハーネスが「無関係な剛体を拾っていない」ことの証拠になる。
        if (Env("DUMMY") == "1")
        {
            var d = new RigidBody(new SphereShape(0.5f))
            {
                Name = "__fardummy__",
                WorldTransform = new RigidTransform(Quat.Identity, new Vec3(10000f, 10000f, 10000f)),
            };
            d.SetMassProps(1f);
            world.AddBody(d);
        }
        if (_jbeta >= 0f) foreach (var j in world.Joints) j.Beta = _jbeta;
        if (tweak != null) tweak(world, builder);

        var dbg = new List<(string a, string b, float dist, float ni)>();
        world.DebugContacts = dbg;

        // 対象剛体 (動的で名前が一致するもの)
        var target = new List<RigidBody>();
        foreach (var b in world.Bodies)
            if (b.Mode != PhysicsMode.BoneFollow && b.Name != null && (_filter == "*" || b.Name.Contains(_filter)))
                target.Add(b);
        if (target.Count == 0) throw new Exception("対象剛体が0: フィルタ '" + _filter + "'");
        var names = new HashSet<string>(target.Select(t => t.Name));

        // 起動時の再整合 (MmdPhysicsBehaviour.Start 相当。駆動が無いのでバインド姿勢)
        builder.ApplyKinematicTargets(i => (RigidTransform?)null);
        builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

        var r = new Result
        {
            Label = label,
            MeanW = new float[frames], MaxW = new float[frames],
            ContactN = new float[frames], MaxPen = new float[frames], SumNi = new float[frames],
        };
        // --- 鎖の健全性: 対象剛体に繋がるジョイントの「拘束方向のアンカー誤差」---
        //     自由な移動DOFは除外する (旧「重心間距離比」はそこを拘束違反として数えていた)。0 が正しい。
        var jw = new List<(Joint j, float bind)>();
        foreach (var j in world.Joints)
        {
            if (j.BodyA == null || j.BodyB == null) continue;
            if (!names.Contains(j.BodyA.Name) && !names.Contains(j.BodyB.Name)) continue;
            jw.Add((j, 0f));
        }
        var perJointLate = new float[jw.Count];   // ジョイント別 後半窓の最大違反

        const float R2D = 57.29578f;
        var lateSamples = new List<float>();
        var s30Samples = new List<float>();
        var relSamples = new List<float>();
        var prev30 = new Quat[target.Count];      // 2フレーム前の姿勢
        bool have30 = false;
        for (int f = 0; f < frames; f++)
        {
            dbg.Clear();
            world.StepSimulation(dt);
            float sum = 0f, mx = 0f;
            bool lateW = f >= frames - frames / 3;
            foreach (var b in target)
            {
                float w = b.AngularVelocity.Length * R2D;
                sum += w; if (w > mx) mx = w;
                if (lateW) lateSamples.Add(w);
            }
            r.MeanW[f] = sum / target.Count;
            r.MaxW[f] = mx;
            // 30fpsサンプル: 偶数フレームだけ姿勢を拾い、1つ前(=2フレーム前)との差を角速度にする
            if ((f & 1) == 0)
            {
                for (int t = 0; t < target.Count; t++)
                {
                    var q = target[t].WorldTransform.Rotation.Normalized;
                    if (have30 && lateW)
                    {
                        var dq = (q * prev30[t].Conjugated()).Normalized;
                        float w2 = dq.w < 0f ? -dq.w : dq.w;
                        if (w2 > 1f) w2 = 1f;
                        s30Samples.Add((float)(2.0 * Math.Acos(w2)) * R2D * 30f); // /(1/30)s
                    }
                    prev30[t] = q;
                }
                have30 = true;
            }
            int cn = 0; float pen = 0f, ni = 0f;
            foreach (var c in dbg)
            {
                if (!names.Contains(c.a) && !names.Contains(c.b)) continue;
                cn++;
                if (-c.dist > pen) pen = -c.dist;
                ni += c.ni;
            }
            r.ContactN[f] = cn; r.MaxPen[f] = pen; r.SumNi[f] = ni;
            float smax = 0f;
            bool late = f >= frames - frames / 3;
            if (late)
                foreach (var (j, _b) in jw)
                    relSamples.Add((j.BodyB.AngularVelocity - j.BodyA.AngularVelocity).Length * R2D);
            for (int q = 0; q < jw.Count; q++)
            {
                var (j, _bind) = jw[q];
                float cur = JointViolation(j);
                if (cur > smax) smax = cur;
                if (cur > r.ViolMax) { r.ViolMax = cur; r.ViolWorst = j.Name; }
                if (late && cur > perJointLate[q]) perJointLate[q] = cur;
            }
            if (f >= frames - frames / 3 && smax > r.ViolLate) r.ViolLate = smax;
            if (pen > r.PenMax)
            {
                r.PenMax = pen;
                foreach (var c in dbg)
                    if ((names.Contains(c.a) || names.Contains(c.b)) && Math.Abs(-c.dist - pen) < 1e-6f)
                    { r.PenPair = c.a + " <-> " + c.b; break; }
            }
        }
        r.PenLate = 0f;
        for (int f = frames - frames / 3; f < frames; f++) if (r.MaxPen[f] > r.PenLate) r.PenLate = r.MaxPen[f];
        if (perJointLate.Length > 0)
        {
            var pj = (float[])perJointLate.Clone(); Array.Sort(pj);
            r.ViolMed = pj[pj.Length / 2];
            r.ViolP90 = pj[Math.Min(pj.Length - 1, (int)(pj.Length * 0.9))];
        }
        if (lateSamples.Count > 0)
        {
            lateSamples.Sort();
            r.LateMedW = lateSamples[lateSamples.Count / 2];
            r.LateP90W = lateSamples[(int)(lateSamples.Count * 0.9)];
            r.LateMaxW = lateSamples[lateSamples.Count - 1];
            int q = 0; foreach (var v in lateSamples) if (v < 5f) q++;
            r.LateQuietFrac = (float)q / lateSamples.Count;
        }
        if (s30Samples.Count > 0)
        {
            s30Samples.Sort();
            float sm = 0f; foreach (var v in s30Samples) sm += v;
            r.S30MeanW = sm / s30Samples.Count;
            r.S30MedW = s30Samples[s30Samples.Count / 2];
            r.S30P90W = s30Samples[(int)(s30Samples.Count * 0.9)];
            r.S30MaxW = s30Samples[s30Samples.Count - 1];
            int q2 = 0; foreach (var v in s30Samples) if (v < 5f) q2++;
            r.S30QuietFrac = (float)q2 / s30Samples.Count;
        }
        if (relSamples.Count > 0)
        {
            relSamples.Sort();
            float sm2 = 0f; foreach (var v in relSamples) sm2 += v;
            r.RelMeanW = sm2 / relSamples.Count;
            r.RelMedW = relSamples[relSamples.Count / 2];
            r.RelP90W = relSamples[(int)(relSamples.Count * 0.9)];
        }
        // 姿勢ハッシュ (FNV-1a over float bits)。フラグが効いているかのビット比較に使う。
        {
            ulong h = 14695981039346656037UL;
            void Mix(float v) { uint b = (uint)BitConverter.SingleToInt32Bits(v); for (int k = 0; k < 4; k++) { h ^= (byte)(b >> (k * 8)); h *= 1099511628211UL; } }
            foreach (var b2 in world.Bodies)
            {
                if (b2.Name == "__fardummy__") continue;   // 診断用の遠方ダミーはハッシュ対象外
                var t = b2.WorldTransform; Mix(t.Origin.x); Mix(t.Origin.y); Mix(t.Origin.z); Mix(t.Rotation.x); Mix(t.Rotation.y); Mix(t.Rotation.z); Mix(t.Rotation.w);
            }
            r.PoseHash = h;
        }
        int third = frames / 3;
        r.EarlyMeanW = r.MeanW.Take(third).Average();
        r.LateMeanW = r.MeanW.Skip(frames - third).Average();
        r.Period = EstimatePeriod(r.MeanW, frames - third, frames);
        return r;
    }

    // 後半の系列から自己相関のピークで周期を出す。0=周期性が見えない。
    static int EstimatePeriod(float[] x, int lo, int hi)
    {
        int n = hi - lo;
        if (n < 40) return 0;
        double mean = 0; for (int i = lo; i < hi; i++) mean += x[i]; mean /= n;
        double var0 = 0; for (int i = lo; i < hi; i++) { double d = x[i] - mean; var0 += d * d; }
        if (var0 < 1e-12) return 0;
        int best = 0; double bestv = 0.30;   // 相関0.30未満は「周期性なし」とする
        for (int lag = 2; lag < n / 3; lag++)
        {
            double s = 0;
            for (int i = lo; i < hi - lag; i++) s += (x[i] - mean) * (x[i + lag] - mean);
            double c = s / var0;
            if (c > bestv) { bestv = c; best = lag; }
        }
        return best;
    }

    static void Sparkline(string tag, float[] v)
    {
        int cols = 60, n = v.Length;
        float mx = 0; foreach (var x in v) if (x > mx) mx = x;
        if (mx <= 0) { L("  " + tag.PadRight(12) + " (全て0)"); return;}
        var sb = new StringBuilder();
        for (int c = 0; c < cols; c++)
        {
            int a = (int)((long)c * n / cols), b = (int)((long)(c + 1) * n / cols);
            float m = 0; for (int i = a; i < b && i < n; i++) if (v[i] > m) m = v[i];
            sb.Append(" .:-=+*#%@"[Math.Min(9, (int)(m / mx * 9.999f))]);
        }
        L("  " + tag.PadRight(12) + " |" + sb + "| 最大=" + mx.ToString("G4"));
    }

    // ---- 一括モード: MODELS=<一覧ファイル> で全モデルを回し、収束しないものを挙げる ----
    static int Batch(string listFile)
    {
        var paths = new List<string>();
        foreach (var ln in File.ReadAllLines(listFile))
        { var t = ln.Trim(); if (t.Length > 0 && File.Exists(t)) paths.Add(t); }
        _filter = Env("BODIES") ?? "*";
        int frames = EnvI("FRAMES", 600);
        float jb = EnvF("JBETA", 0.02f);
        bool useJsplit = Env("JSPLIT") == "1";
        // LEVERAB=1 : 既定(LinearLeverMode=0) vs Bullet2.75式(=1) の A/B。
        int leverAB = EnvI("LEVERAB", -1);
        // ★タスク23: 3点構成 (ANGCONV + AXES + LEVER1) の A/B。
        //   ANGCONV/AXES/LEVER を env で直接立てると ApplyGlobalEnv が **両側** に適用してしまうので、
        //   一括では専用の env を使い、new 側だけで立てる。
        bool angconvAB = Env("ANGCONVAB") == "1";
        // ★タスク32: ばねのモーター行化 (SpringAsMotorRow) の一括A/B。
        //   SPRINGMOTORAB=1 で new 側だけ ON、=2 で **候補構成 (モーター行 + ROTEXP)**、=3 で 3点構成も併用。
        int springAB = EnvI("SPRINGMOTORAB", 0);
        // ★タスク38: 接触の受理閾値 (GjkEpa.BulletContactThreshold) の一括A/B。
        //   CTHRESH を env で直接立てると ApplyGlobalEnv が **両側** に適用してしまうので専用 env。
        bool cthreshAB = Env("CTHRESHAB") == "1";
        L("=".PadRight(120, '='));
        L("restosc 一括: 駆動なし(バインド姿勢)で全動的剛体が収束するか  " + paths.Count + "モデル / " +
          frames + "F / フィルタ='" + _filter + "'");
        L("  比(後半平均|w| / 前半平均|w|) が小さいほど収束。1付近 = 止まっていない。");
        L("=".PadRight(120, '='));
        L(leverAB >= 0
            ? "  比較: 既定(Lever0/Beta0.2)  vs  Lever" + leverAB + (Env("JBETA") != null ? "/Beta" + jb.ToString("0.00") : "/Beta0.2") + "   (違反=拘束方向のアンカー誤差、0が正しい)"
            : springAB > 0
            ? "  比較: 既定  vs  ばねモーター行" + (springAB == 2 ? " + ROTEXP (タスク36 候補構成)" : springAB >= 3 ? " + 3点構成" : "") + " (Bullet 2.75 internalUpdateSprings 方式)"
            : cthreshAB
            ? "  比較: 既定  vs  接触の受理閾値を Bullet 2.75 方式へ (形状サイズ比例)   (違反=拘束方向のアンカー誤差、0が正しい)"
            : angconvAB
            ? "  比較: 既定  vs  3点構成 (BulletAngleConvention + AngularMixedAxes + LinearLeverMode=1)   (違反=拘束方向のアンカー誤差、0が正しい)"
            : useJsplit
            ? "  比較: 既定  vs  UseJointSplitImpulse=ON   (伸び=ジョイント2剛体間距離/バインド、1.0が正しい)"
            : "  比較: Joint.Beta 既定0.2  vs  " + jb.ToString("0.000") + "   (違反=同上)");
        L("  判定は **30fpsサンプルの中央値**(=大半の剛体が止まっているか)。p90 が半減すると動き自体が痩せる。");
        L(string.Format("  {0,-26} {1,5} {2,7} {3,8} {4,7} {5,6} | {6,7} {7,8} {8,7} {9,6}",
                        "モデル", "剛体", "既定中", "既定p90", "既定相対", "既定違反", "新中", "新p90", "新相対", "新違反"));
        var bad = new List<string>();
        foreach (var p in paths)
        {
            string nm = Path.GetFileNameWithoutExtension(p);
            try
            {
                _model = PmxReader.LoadFile(p);
                int dyn = 0;
                foreach (var rb in _model.RigidBodies) if (rb.PhysicsMode != 0) dyn++;
                if (dyn == 0) { L(string.Format("  {0,-40} {1,5}  (動的剛体なし)", Trim(nm, 40), dyn)); continue; }
                _jbeta = -1f;
                if (leverAB >= 0) Joint.LinearLeverMode = 0;
                if (angconvAB) { Joint.BulletAngleConvention = false; Joint.AngularMixedAxes = false; Joint.LinearLeverMode = 0; }
                // ★springAB の「old 側」は **陽的経路** を指す (既定は 2026-08-22 から モーター行)。
                //   採用後は「出荷既定 vs 旧経路」の回帰対照として読むこと。
                if (cthreshAB) GjkEpa.BulletContactThreshold = false;
                if (springAB > 0) { Joint.SpringAsMotorRow = false; Joint.BulletAngleConvention = false; Joint.AngularMixedAxes = false; Joint.LinearLeverMode = 0; PhysicsWorld.BulletRotationIntegration = false; }
                var oldR = Run("old", null);   // 既定
                Result newR;
                if (leverAB >= 0)
                {
                    // JBETA を明示した場合だけ new 側の Beta も変える (未指定なら既定 0.2 のまま)。
                    Joint.LinearLeverMode = leverAB;
                    _jbeta = Env("JBETA") != null ? jb : -1f;
                    newR = Run("new", null);
                    _jbeta = -1f; Joint.LinearLeverMode = 0;
                }
                else if (cthreshAB)
                {
                    GjkEpa.BulletContactThreshold = true;
                    newR = Run("new", null);
                    GjkEpa.BulletContactThreshold = false;
                }
                else if (springAB > 0)
                {
                    Joint.SpringAsMotorRow = true;
                    if (springAB == 2) PhysicsWorld.BulletRotationIntegration = true;   // タスク36 候補構成
                    if (springAB >= 3) { Joint.BulletAngleConvention = true; Joint.AngularMixedAxes = true; Joint.LinearLeverMode = 1; }
                    newR = Run("new", null);
                    Joint.SpringAsMotorRow = ShippedSpringMotor; Joint.BulletAngleConvention = false;
                    Joint.AngularMixedAxes = false; Joint.LinearLeverMode = 0;
                    PhysicsWorld.BulletRotationIntegration = false;
                }
                else if (angconvAB)
                {
                    Joint.BulletAngleConvention = true; Joint.AngularMixedAxes = true; Joint.LinearLeverMode = 1;
                    newR = Run("new", null);
                    Joint.BulletAngleConvention = false; Joint.AngularMixedAxes = false; Joint.LinearLeverMode = 0;
                }
                else if (useJsplit) { newR = Run("new", (w, bb) => { w.UseJointSplitImpulse = true; }); }
                else { _jbeta = jb; newR = Run("new", null); _jbeta = -1f; }
                float ro = oldR.EarlyMeanW > 1e-9f ? oldR.LateMeanW / oldR.EarlyMeanW : 0f;
                float rn = newR.EarlyMeanW > 1e-9f ? newR.LateMeanW / newR.EarlyMeanW : 0f;
                string mark = "";
                // ★NaN / 発散の検出 (タスク23)。中央値の比較より先に見る。
                bool BadNum(Result r) => float.IsNaN(r.S30MedW) || float.IsInfinity(r.S30MedW)
                                      || float.IsNaN(r.ViolP90) || float.IsInfinity(r.ViolP90)
                                      || r.S30MedW > 1e5f || r.ViolP90 > 1e3f;
                if (BadNum(newR) && !BadNum(oldR))
                { mark = "  ★★NaN/発散"; bad.Add(nm + " ★NaN/発散 中央 " + oldR.S30MedW.ToString("G4") + " -> " + newR.S30MedW.ToString("G4") + " / 違反p90 " + oldR.ViolP90.ToString("G4") + " -> " + newR.ViolP90.ToString("G4")); }
                else if (BadNum(oldR))
                { mark = "  (既定側が既に NaN/発散)"; }
                else if (newR.ViolP90 > oldR.ViolP90 * 1.10f + 1e-4f)
                { mark = "  ★拘束違反悪化"; bad.Add(nm + " 違反p90 " + oldR.ViolP90.ToString("G4") + " -> " + newR.ViolP90.ToString("G4")); }
                else if (newR.S30MedW > oldR.S30MedW * 1.05f + 0.2f)
                { mark = "  ★中央悪化"; bad.Add(nm + " 中央 " + oldR.S30MedW.ToString("F2") + " -> " + newR.S30MedW.ToString("F2")); }
                else if (oldR.S30P90W > 1f && newR.S30P90W < oldR.S30P90W * 0.5f)
                { mark = "  △動きが痩せた"; }
                L(string.Format("  {0,-26} {1,5} {2,7:F1} {3,8:F1} {4,7:F1} {5,8:G4} | {6,7:F1} {7,8:F1} {8,7:F1} {9,8:G4}{10}",
                                Trim(nm, 26), dyn, oldR.S30MedW, oldR.S30P90W, oldR.RelMedW, oldR.ViolP90,
                                newR.S30MedW, newR.S30P90W, newR.RelMedW, newR.ViolP90, mark));
            }
            catch (Exception e) { L(string.Format("  {0,-40}  [読めない] {1}", Trim(nm, 40), e.Message)); }
        }
        L();
        L("---------- 変更で悪化したモデル (中央値が上がった or 拘束違反が増えた) ----------");
        if (bad.Count == 0) L("  なし (全モデルで 中央値・拘束違反 とも悪化なし)");
        else foreach (var b in bad) L("  ★ " + b);
        File.WriteAllText(Env("OUT") ?? "restosc_batch.txt", O.ToString());
        return 0;
    }
    // 静止させたあと、角度3軸の「行が出る/出ない」が毎ステップ入れ替わっていないかを数える。
    static void Chatter()
    {
        int frames = EnvI("FRAMES", 900);
        int fps = EnvI("FPS", 60);
        float dt = 1f / fps;
        var builder = PmxPhysicsBuilder.Build(_model);
        var world = builder.World;
        builder.ApplyKinematicTargets(i => (RigidTransform?)null);
        builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);
        // 先に落ち着かせる
        for (int f = 0; f < frames * 2 / 3; f++) world.StepSimulation(dt);

        var log = new List<(string joint, int dof, int state, float cur, float err)>();
        Joint.DebugAngularRows = log;
        var prev = new Dictionary<(string, int), int>();
        var toggles = new Dictionary<(string, int), int>();
        var seen = new Dictionary<(string, int), int>();
        var errAbs = new List<float>();
        int steps = 0, nAct = 0, nTot = 0;
        int measure = frames / 3;
        for (int f = 0; f < measure; f++)
        {
            log.Clear();
            world.StepSimulation(dt);
            steps++;
            // このフレームの最後のサブステップ分だけを見る (同じ(joint,dof)が複数回出るので後勝ち)
            var cur = new Dictionary<(string, int), int>();
            var errNow = new Dictionary<(string, int), float>();
            foreach (var e in log)
            {
                if (_filter != "*" && !e.joint.Contains(_filter)) continue;
                cur[(e.joint, e.dof)] = e.state;
                errNow[(e.joint, e.dof)] = Math.Abs(e.err);
            }
            foreach (var kv in cur)
            {
                nTot++;
                bool act = kv.Value >= 2;
                if (act) { nAct++; errAbs.Add(errNow[kv.Key]); }
                if (!seen.ContainsKey(kv.Key)) { seen[kv.Key] = 0; toggles[kv.Key] = 0; }
                seen[kv.Key]++;
                if (prev.TryGetValue(kv.Key, out var pv))
                {
                    bool pact = pv >= 2;
                    if (pact != act) toggles[kv.Key]++;
                }
                prev[kv.Key] = kv.Value;
            }
        }
        Joint.DebugAngularRows = null;
        L();
        L("  ★★角度リミット行のチャタリング診断 (静止させた後の " + steps + " ステップ)");
        L("     対象 (joint,軸) 組 = " + seen.Count + " / 行が立っていた割合 = " +
          (nTot > 0 ? (100.0 * nAct / nTot).ToString("F1") : "-") + "%");
        var rates = new List<double>();
        foreach (var kv in seen) rates.Add(kv.Value > 1 ? (double)toggles[kv.Key] / (kv.Value - 1) : 0);
        rates.Sort();
        if (rates.Count > 0)
            L("     ★ON/OFF切替率 (1.0=毎ステップ反転): 中央=" + rates[rates.Count / 2].ToString("F3") +
              " p90=" + rates[(int)(rates.Count * 0.9)].ToString("F3") +
              " 最大=" + rates[rates.Count - 1].ToString("F3") +
              " / 切替率>0.4 の組=" + rates.FindAll(x => x > 0.4).Count);
        errAbs.Sort();
        if (errAbs.Count > 0)
            L("     行が立ったときの |err| (rad): 中央=" + errAbs[errAbs.Count / 2].ToString("G4") +
              " p90=" + errAbs[(int)(errAbs.Count * 0.9)].ToString("G4") +
              " 最大=" + errAbs[errAbs.Count - 1].ToString("G4"));
    }

    // ★リファレンス(ベイク済みVMD由来のボーンCSV)の姿勢を剛体へ流し込み、
    //   エンジン自身のコードで euler のリミット超過量を出す。規約の取り違えを避けるため
    //   Python で再実装せず、DebugAngularRows フックをそのまま使う。
    //   各ステップの **最初のサブステップ** の記録だけ採る (それ以降は自前物理が動かしてしまうため)。
    static int RefCheck(string csvPath)
    {
        var csv = BoneCsv.Load(csvPath);
        var builder = PmxPhysicsBuilder.Build(_model);
        var world = builder.World;
        world.FixedTimeStep = 1f / 60f; world.SubSteps = 1; world.SolverIterations = 1;
        var links = new List<(BoneLink l, string bone)>();
        foreach (var l in builder.BoneLinks)
            if (l.BoneIndex >= 0 && l.BoneIndex < _model.BoneNames.Count && csv.HasBone(_model.BoneNames[l.BoneIndex]))
                links.Add((l, _model.BoneNames[l.BoneIndex]));
        L("=".PadRight(96, '='));
        L("refcheck : リファレンス姿勢での角度リミット超過量 (エンジンの euler をそのまま使用)");
        L("  CSV=" + Path.GetFileName(csvPath) + "  流し込むリンク=" + links.Count + " / 全" + builder.BoneLinks.Count);
        L("=".PadRight(96, '='));
        var log = new List<(string joint, int dof, int state, float cur, float err)>();
        var errAbs = new List<float>();
        int nAct = 0, nTot = 0, frames = 0;
        int lo = EnvI("REFFROM", 0), hi = EnvI("REFTO", csv.FrameCount - 1);
        for (int f = lo; f <= hi; f++)
        {
            foreach (var (l, b) in links)
                if (csv.TryGet(f, b, out var bw)) l.Body.WorldTransform = bw * l.BodyOffsetFromBone;
            log.Clear();
            Joint.DebugAngularRows = log;
            world.StepSimulation(1f / 60f);
            Joint.DebugAngularRows = null;
            var first = new Dictionary<(string, int), (int st, float err)>();
            foreach (var e in log)
            {
                if (_filter != "*" && !e.joint.Contains(_filter)) continue;
                if (!first.ContainsKey((e.joint, e.dof))) first[(e.joint, e.dof)] = (e.state, Math.Abs(e.err));
            }
            foreach (var kv in first)
            {
                nTot++;
                if (kv.Value.st >= 2) { nAct++; errAbs.Add(kv.Value.err); }
            }
            frames++;
        }
        errAbs.Sort();
        L("  対象フレーム=" + frames + " / (joint,軸)サンプル=" + nTot);
        L("  ★行が立っていた割合 = " + (nTot > 0 ? (100.0 * nAct / nTot).ToString("F1") : "-") + "%");
        if (errAbs.Count > 0)
        {
            const float R2D = 57.29578f;
            L("  ★リミット超過量 |err|: 中央=" + (errAbs[errAbs.Count / 2] * R2D).ToString("F4") + "°" +
              "  p90=" + (errAbs[(int)(errAbs.Count * 0.9)] * R2D).ToString("F4") + "°" +
              "  最大=" + (errAbs[errAbs.Count - 1] * R2D).ToString("F4") + "°");
        }
        File.WriteAllText(Env("OUT") ?? "refcheck.txt", O.ToString());
        return 0;
    }

    // ★同一駆動での比較: 参照CSVでボーン追従剛体を動かし、揺れ物は自前物理で回す。
    //   同じフレーム・同じ量で「自前」と「リファレンス(CSVそのもの)」の |w| を比べる。
    //   これまでの比較は「駆動なしのバインド姿勢」vs「ダンス中の静区間」でシナリオが違っていた。
    static int Drive(string csvPath)
    {
        var csv = BoneCsv.Load(csvPath);
        var builder = PmxPhysicsBuilder.Build(_model);
        var world = builder.World;
        // ★2026-08-21 修正: 参照CSVは30fps。1フレーム進めるたびに 1/30 秒ぶん回さなければ
        //   ボーン駆動が2倍速になり、かつ自前だけ 1/60 刻みで標本化されて比較が成立しない
        //   (bonecheck/hairfid はどちらも 1/30 で回している)。刻み自体は従来と同じ 1/120。
        //   LEGACY60=1 で旧挙動 (FixedTimeStep=1/60, 1/60 ずつ進める) を再現できる。
        bool legacy60 = Env("LEGACY60") == "1";
        float dt = legacy60 ? 1f / 60f : 1f / 30f;
        float sampleHz = legacy60 ? 60f : 30f;
        world.FixedTimeStep = dt;
        world.SubSteps = EnvI("SUBSTEPS", legacy60 ? 2 : 4);
        world.SolverIterations = EnvI("ITERS", 10);
        if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        // ジョイント warm-start の A/B (JWARM=lin | both)。係数は Joint.WarmStartFactor(0.85)。
        {
            string wm = Env("JWARM");
            if (wm == "lin") world.UseJointWarmStart = true;
            else if (wm == "both") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        }
        float jb = EnvF("JBETA", -1f);
        if (jb >= 0f) foreach (var j in world.Joints) j.Beta = jb;

        var driven = new List<(BoneLink l, string bone)>();
        var mine = new List<(RigidBody b, string bone)>();
        var mineLink = new List<BoneLink>();
        foreach (var l in builder.BoneLinks)
        {
            if (l.BoneIndex < 0 || l.BoneIndex >= _model.BoneNames.Count) continue;
            string bn = _model.BoneNames[l.BoneIndex];
            if (!csv.HasBone(bn)) continue;
            if (l.Mode == PhysicsMode.BoneFollow) driven.Add((l, bn));
            else if (_filter == "*" || l.Body.Name.Contains(_filter) || bn.Contains(_filter)) { mine.Add((l.Body, bn)); mineLink.Add(l); }
        }
        L("=".PadRight(96, '='));
        L("drive : 同一駆動での |w| 比較 (自前物理 vs リファレンスCSV)");
        L("  CSV=" + Path.GetFileName(csvPath) + " " + csv.FrameCount + "F  駆動=" + driven.Count +
          "本  比較対象(動的)=" + mine.Count + "本  dt=1/" + (int)(1f / dt) + " SubSteps=" + world.SubSteps +
          (Env("JSPLIT") == "1" ? "  JSPLIT=ON" : "") + (jb >= 0f ? "  Beta=" + jb : "") +
          (BulletPhysics.Joint.AngularMixedAxes ? "  AXES=mixed" : "  AXES=orthoA") +
          "  Lever=" + BulletPhysics.Joint.LinearLeverMode);
        EchoFlags(world, "drive");
        L("=".PadRight(96, '='));

        int F = Math.Min(csv.FrameCount, EnvI("FRAMES", csv.FrameCount));
        int warm = EnvI("WARMUP", 60);
        void Apply(int f) { foreach (var (l, b) in driven) if (csv.TryGet(f, b, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Apply(0);
        builder.ResetBodiesToBonePoseFk(i => { var n = _model.BoneNames[i]; return csv.TryGet(0, n, out var x) ? (RigidTransform?)x : null; });
        for (int w2 = 0; w2 < warm; w2++) { world.StepSimulation(dt); }

        const float R2D = 57.29578f;
        float Ang(Quat a2, Quat b2) { var d = (b2 * a2.Conjugated()).Normalized; float w3 = d.w < 0 ? -d.w : d.w; if (w3 > 1f) w3 = 1f; return (float)(2.0 * Math.Acos(w3)) * R2D; }
        var prevMine = new Quat[mine.Count];
        var selfW = new List<float>(); var refW = new List<float>();
        int quietF = 0;
        // ★タスク9: 静止忠実度。自前と参照を同一定義 (per-frame |Δp|, PMX単位/フレーム@30fps) で比べる。
        //   タスク6 と同じ計算。ボーン姿勢は BodyOffsetFromBone の逆で剛体から復元する。
        var dpSelf = new List<float>[mine.Count]; var dpRef = new List<float>[mine.Count];
        for (int i = 0; i < mine.Count; i++) { dpSelf[i] = new List<float>(); dpRef[i] = new List<float>(); }
        var prevSelfP = new Vec3[mine.Count];
        Vec3 BonePos(int i) => (mineLink[i].Body.WorldTransform * mineLink[i].BodyOffsetFromBone.Inverse()).Origin;

        // ★ERRSTAT: 静区間で「ジョイントがどれだけ制限を越えたまま居座っているか」を測る。
        //   残留角速度 ≒ Beta*err/dt なので、err が定常なのか毎ステップ符号反転しているのかで
        //   機構が分かれる (定常=何かが押し戻し続けている / 反転=補正の行き過ぎによる限界振動)。
        bool errstat = Env("ERRSTAT") == "1";
        // ★INJECT: dtスケーリング監査。1フレーム(=全サブステップ)あたりの補正注入量を積算する。
        //   注入量 = Σ|targetVel| (= Σ|Clamp(err*Beta*invDt)|)。設計上 dt 不変であるべき量。
        bool inject = Env("INJECT") == "1";
        var rowLog = inject ? new List<(string joint, int dof, bool angular, float err, float targetVel, float relVel)>() : null;
        if (inject) Joint.DebugRows = rowLog;
        string dumpJoint = Env("DUMPJOINT");
        var dump = new StringBuilder();
        if (dumpJoint != null) dump.AppendLine("frame,substep,joint,dof,angular,err,targetVel,relVelAtBuild");
        double injLinSum = 0, injAngSum = 0; int injFrames = 0;
        var injLinPerFrame = new List<float>(); var injAngPerFrame = new List<float>();
        var errLinPerFrame = new List<float>(); var rowsPerFrame = new List<float>();
        // 行構築時点の相対速度 (=前サブステップの求解結果)。目標速度に対してどれだけ届いていないか。
        var tvLinAll = new List<float>(); var rvLinAll = new List<float>();
        var tvAngAll = new List<float>(); var rvAngAll = new List<float>();
        var jList = new List<Joint>();
        if (errstat)
            foreach (var j in world.Joints)
                if (j.BodyA != null && j.BodyB != null &&
                    (_filter == "*" || j.Name.Contains(_filter) || j.BodyB.Name.Contains(_filter))) jList.Add(j);
        var linErr = new List<float>(); var angErr = new List<float>();
        int linRows = 0, angRows = 0, linSlots = 0, angSlots = 0, linFlip = 0, angFlip = 0, linSeq = 0, angSeq = 0;
        var prevLin = new float[jList.Count * 3]; var prevAng = new float[jList.Count * 3];
        var hasPrev = new bool[jList.Count * 6];
        var violMax = new float[jList.Count];   // ★タスク8の統一指標 (拘束方向のアンカー誤差)
        for (int f = 0; f < F; f++)
        {
            Apply(f);
            if (inject) rowLog.Clear();
            world.StepSimulation(dt);
            if (f == 0) { for (int i = 0; i < mine.Count; i++) prevMine[i] = mine[i].b.WorldTransform.Rotation.Normalized; continue; }
            // 静区間判定: 参照CSVの下半身の回転変化
            bool quiet = false;
            if (csv.TryGet(f - 1, "下半身", out var l0) && csv.TryGet(f, "下半身", out var l1))
                quiet = Ang(l0.Rotation, l1.Rotation) * 30f < 5f;
            for (int i = 0; i < mine.Count; i++)
            {
                var bp = BonePos(i);
                if (quiet && f > 0)
                {
                    dpSelf[i].Add((bp - prevSelfP[i]).Length);
                    if (csv.TryGet(f - 1, mine[i].bone, out var rp0) && csv.TryGet(f, mine[i].bone, out var rp1))
                        dpRef[i].Add((rp1.Origin - rp0.Origin).Length);
                }
                prevSelfP[i] = bp;
                var q = mine[i].b.WorldTransform.Rotation.Normalized;
                if (quiet)
                {
                    selfW.Add(Ang(prevMine[i], q) * sampleHz);   // 参照と同じ標本化間隔で測る
                    if (csv.TryGet(f - 1, mine[i].bone, out var r0) && csv.TryGet(f, mine[i].bone, out var r1))
                        refW.Add(Ang(r0.Rotation, r1.Rotation) * 30f);  // 参照は30fps
                }
                prevMine[i] = q;
            }
            if (quiet) quietF++;

            if (inject && quiet)
            {
                // 1フレーム分 = SubSteps 回ぶんの行が rowLog に溜まっている。
                float sl = 0f, sa = 0f, el = 0f; int nl = 0, nrow = 0;
                var seen = new Dictionary<string, int>();
                foreach (var r in rowLog)
                {
                    if (_filter != "*" && !r.joint.Contains(_filter)) continue;
                    nrow++;
                    if (r.angular) { sa += Math.Abs(r.targetVel); tvAngAll.Add(Math.Abs(r.targetVel)); rvAngAll.Add(Math.Abs(r.relVel)); }
                    else { sl += Math.Abs(r.targetVel); el += Math.Abs(r.err); nl++; tvLinAll.Add(Math.Abs(r.targetVel)); rvLinAll.Add(Math.Abs(r.relVel)); }
                    if (dumpJoint != null && r.joint == dumpJoint)
                    {
                        string key = r.dof + "|" + r.angular;
                        seen.TryGetValue(key, out int k); seen[key] = k + 1;
                        dump.AppendLine(string.Format(CultureInfo.InvariantCulture,
                            "{0},{1},{2},{3},{4},{5:G9},{6:G9},{7:G9}", f, k, r.joint, r.dof, r.angular ? 1 : 0, r.err, r.targetVel, r.relVel));
                    }
                }
                injLinPerFrame.Add(sl); injAngPerFrame.Add(sa);
                errLinPerFrame.Add(nl > 0 ? el / nl : 0f); rowsPerFrame.Add(nrow);
                injLinSum += sl; injAngSum += sa; injFrames++;
            }

            if (errstat && quiet)
            {
                for (int ji = 0; ji < jList.Count; ji++)
                {
                    var j = jList[ji];
                    float vv = JointViolation(j); if (vv > violMax[ji]) violMax[ji] = vv;
                    var wA = j.BodyA.WorldTransform * j.FrameInA;
                    var wB = j.BodyB.WorldTransform * j.FrameInB;
                    var bA = Matrix3x3.FromQuat(wA.Rotation);
                    var linDelta = wB.Origin - wA.Origin;
                    var eul = EulerXYZ(wA.Rotation.Conjugated() * wB.Rotation);
                    for (int i = 0; i < 3; i++)
                    {
                        // 並進
                        float lo = j.LinearLowerLimit[i], hi = j.LinearUpperLimit[i];
                        if (!(lo > hi))
                        {
                            linSlots++;
                            float cur = linDelta.Dot(bA.Column(i));
                            float e = lo == hi ? lo - cur : (cur < lo ? lo - cur : (cur > hi ? hi - cur : 0f));
                            if (e != 0f) { linRows++; linErr.Add(Math.Abs(e)); }
                            int k2 = ji * 3 + i;
                            if (hasPrev[k2]) { linSeq++; if (prevLin[k2] * e < 0f) linFlip++; }
                            prevLin[k2] = e; hasPrev[k2] = true;
                        }
                        // 回転
                        float alo = j.AngularLowerLimit[i], ahi = j.AngularUpperLimit[i];
                        if (!(alo > ahi))
                        {
                            angSlots++;
                            float cur = eul[i];
                            float e = alo == ahi ? alo - cur : (cur < alo ? alo - cur : (cur > ahi ? ahi - cur : 0f));
                            if (e != 0f) { angRows++; angErr.Add(Math.Abs(e) * 57.29578f); }
                            int k2 = jList.Count * 3 + ji * 3 + i;
                            if (hasPrev[k2]) { angSeq++; if (prevAng[ji * 3 + i] * e < 0f) angFlip++; }
                            prevAng[ji * 3 + i] = e; hasPrev[k2] = true;
                        }
                    }
                }
            }
        }
        void Rep(string tag, List<float> v)
        {
            if (v.Count == 0) { L("  " + tag + ": サンプルなし"); return; }
            v.Sort(); float sm = 0f; foreach (var x in v) sm += x;
            int q = 0; foreach (var x in v) if (x < 5f) q++;
            L(string.Format("  {0,-12} n={1,-8} 平均={2,8:F2} 中央={3,8:F2} p90={4,9:F2} 最大={5,10:F1} 静止率(<5)={6,6:P1}",
                            tag, v.Count, sm / v.Count, v[v.Count / 2], v[(int)(v.Count * 0.9)], v[v.Count - 1], (float)q / v.Count));
            // 低速側の形: 0付近にスパイクがあれば「眠っている(非活性化)」、なだらかなら「収束している」。
            int Below(float t) { int c = 0; foreach (var x in v) if (x < t) c++; return c; }
            L(string.Format("     └ 分位 p1={0,7:F3} p5={1,7:F3} p10={2,7:F3} p25={3,7:F3} p75={4,8:F2}   割合 <0.01={5,5:P1} <0.1={6,5:P1} <1={7,5:P1}",
                            v[(int)(v.Count * 0.01)], v[(int)(v.Count * 0.05)], v[(int)(v.Count * 0.10)], v[(int)(v.Count * 0.25)], v[(int)(v.Count * 0.75)],
                            (float)Below(0.01f) / v.Count, (float)Below(0.1f) / v.Count, (float)Below(1f) / v.Count));
        }
        L("  静区間フレーム=" + quietF + " / " + F);
        // ★静止忠実度: ボーン別の |Δp| 分布を自前/参照で比べ、比の中央を出す。1.0 が理想。
        {
            var rMed = new List<float>(); var rP90 = new List<float>();
            var sMedAll = new List<float>(); var refMedAll = new List<float>();
            var sP90All = new List<float>(); var refP90All = new List<float>();
            for (int i = 0; i < mine.Count; i++)
            {
                if (dpSelf[i].Count < 30 || dpRef[i].Count < 30) continue;
                float sm = MedOf(dpSelf[i]), rm = MedOf(dpRef[i]);
                float sp = PctOf(dpSelf[i], 0.9f), rp = PctOf(dpRef[i], 0.9f);
                sMedAll.Add(sm); refMedAll.Add(rm); sP90All.Add(sp); refP90All.Add(rp);
                if (rm > 1e-9f) rMed.Add(sm / rm);
                if (rp > 1e-9f) rP90.Add(sp / rp);
            }
            L("  --- 静止忠実度 (per-frame |Δp|, PMX単位/フレーム@30fps。タスク6と同一定義) ---");
            L(string.Format("    ボーン別 中央|Δp|: 自前 中央={0:G4} / 参照 中央={1:G4}      ★中央比の中央 = {2:F3}",
                            MedOf(sMedAll), MedOf(refMedAll), MedOf(rMed)));
            L(string.Format("    ボーン別 p90|Δp| : 自前 中央={0:G4} / 参照 中央={1:G4}      ★p90比の中央  = {2:F3}",
                            MedOf(sP90All), MedOf(refP90All), MedOf(rP90)));
            L("    (1.000 が理想。>1=動きすぎ / <1=痩せている。対象ボーン=" + rMed.Count + "本)");
        }
        if (inject)
        {
            Joint.DebugRows = null;
            float Med(List<float> v) { if (v.Count == 0) return 0f; var c = new List<float>(v); c.Sort(); return c[c.Count / 2]; }
            L("  --- 補正注入量 (静区間、1フレーム=" + world.SubSteps + "サブステップ分を積算) ---");
            L(string.Format("    SubSteps={0}  invDt={1:F1}(=SubSteps/FixedTimeStep)  対象行/フレーム 中央={2:F0}",
                            world.SubSteps, world.SubSteps / world.FixedTimeStep, Med(rowsPerFrame)));
            L(string.Format("    Σ|targetVel| /フレーム  並進 平均={0:F2} 中央={1:F2}   回転 平均={2:F2} 中央={3:F2}",
                            injFrames > 0 ? injLinSum / injFrames : 0, Med(injLinPerFrame),
                            injFrames > 0 ? injAngSum / injFrames : 0, Med(injAngPerFrame)));
            L(string.Format("    並進 |err| 平均(行あたり) 中央={0:G6}", Med(errLinPerFrame)));
            L(string.Format("    行あたり |目標速度| 中央  並進={0:G6} 回転={1:G6}", Med(tvLinAll), Med(tvAngAll)));
            L(string.Format("    行構築時 |相対速度| 中央  並進={0:G6} 回転={1:G6}   ← 目標に届いていれば目標速度と同程度になる",
                            Med(rvLinAll), Med(rvAngAll)));
            L(string.Format("    未達比 (相対速度/目標速度)  並進={0:F2}倍 回転={1:F2}倍",
                            Med(tvLinAll) > 1e-9f ? Med(rvLinAll) / Med(tvLinAll) : 0f,
                            Med(tvAngAll) > 1e-9f ? Med(rvAngAll) / Med(tvAngAll) : 0f));
            if (dumpJoint != null)
            {
                File.WriteAllText(Env("DUMPOUT") ?? "inject_dump.csv", dump.ToString());
                L("    [CSV] " + (Env("DUMPOUT") ?? "inject_dump.csv") + " へ出力");
            }
        }
        if (errstat)
        {
            void RepE(string tag, List<float> v, int rows, int slots, int flip, int seq, string unit)
            {
                if (v.Count == 0) { L("  " + tag + ": 違反行なし (slots=" + slots + ")"); return; }
                v.Sort();
                L(string.Format("  {0}: 行あり率={1,6:P1} ({2}/{3})  |err| 中央={4,9:F4}{8} p90={5,9:F4}{8} 最大={6,9:F3}{8}  符号反転率={7,6:P1}",
                                tag, (float)rows / Math.Max(1, slots), rows, slots,
                                v[v.Count / 2], v[(int)(v.Count * 0.9)], v[v.Count - 1],
                                (float)flip / Math.Max(1, seq), unit));
            }
            L("  --- 静区間のジョイント制限違反 (対象Joint=" + jList.Count + ") ---");
            if (jList.Count > 0)
            {
                var vm = (float[])violMax.Clone(); Array.Sort(vm);
                L(string.Format("  拘束違反(アンカー誤差, 自由DOF除外, 0が正しい): 中央={0:G4} p90={1:G4} 最大={2:G4}",
                                vm[vm.Length / 2], vm[Math.Min(vm.Length - 1, (int)(vm.Length * 0.9))], vm[vm.Length - 1]));
            }
            RepE("並進", linErr, linRows, linSlots, linFlip, linSeq, "");
            RepE("回転", angErr, angRows, angSlots, angFlip, angSeq, "°");
        }
        Rep("自前", selfW);
        Rep("リファレンス", refW);
        File.WriteAllText(Env("OUT") ?? "drive.txt", O.ToString());
        return 0;
    }

    // engine の Joint.ToEulerXYZ (internal) と同一式のローカル複製。診断専用。
    static Vec3 EulerXYZ(Quat q)
    {
        var m = Matrix3x3.FromQuat(q.Normalized);
        float m02 = m.Row0.z; float x, y, z;
        if (m02 < 1f - 1e-6f)
        {
            if (m02 > -1f + 1e-6f)
            {
                y = (float)Math.Asin(Math.Clamp(m02, -1f, 1f));
                x = (float)Math.Atan2(-m.Row1.z, m.Row2.z);
                z = (float)Math.Atan2(-m.Row0.y, m.Row0.x);
            }
            else { y = -(float)(Math.PI / 2); x = -(float)Math.Atan2(m.Row1.x, m.Row1.y); z = 0f; }
        }
        else { y = (float)(Math.PI / 2); x = (float)Math.Atan2(m.Row1.x, m.Row1.y); z = 0f; }
        return new Vec3(x, y, z);
    }

    // ★新しい鎖の健全性指標: 拘束されている移動軸方向のアンカー誤差 (自由DOFは除外)。
    //   Joint.Prepare の err と同じ式。単位は PMX 単位、0 が正しい。
    static float JointViolation(Joint j)
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
            if (lo > hi) continue;                       // 自由DOF: 除外
            float cur = d.Dot(bA.Column(i));
            float e = lo == hi ? cur - lo : (cur < lo ? lo - cur : (cur > hi ? cur - hi : 0f));
            sq += e * e;
        }
        return (float)Math.Sqrt(sq);
    }

    // ★タスク11-1: 実効フラグのエコー。env から読んだ値ではなく、**実際にエンジンへ適用された値**を出す。
    //   これまで JBETA / JSPLIT が単体モードで黙殺されていた事故が2件あったため、常時表示する。
    static void EchoFlags(PhysicsWorld w, string where)
    {
        float jbMin = float.MaxValue, jbMax = float.MinValue;
        foreach (var j in w.Joints) { if (j.Beta < jbMin) jbMin = j.Beta; if (j.Beta > jbMax) jbMax = j.Beta; }
        string jb = w.Joints.Count == 0 ? "-" : (jbMin == jbMax ? jbMin.ToString("G6") : jbMin.ToString("G6") + "~" + jbMax.ToString("G6"));
        L("[実効] " + where +
          "  FixedTimeStep=1/" + (w.FixedTimeStep > 0 ? (1f / w.FixedTimeStep).ToString("F2") : "-") +
          "  SubSteps=" + w.SubSteps + "  Iters=" + w.SolverIterations +
          "  Joint.Beta=" + jb +
          "  ContactBaumgarte=" + w.BaumgarteFactor.ToString("G6"));
        L("[実効] " + where +
          "  JSplit=" + w.UseJointSplitImpulse + "  Split=" + w.UseSplitImpulse +
          "  JWarm=" + w.UseJointWarmStart + "/" + w.UseJointWarmStartAngular +
          "  LeverMode=" + Joint.LinearLeverMode + "  MixedAxes=" + Joint.AngularMixedAxes +
          "  AngConv=" + Joint.BulletAngleConvention +
          "  SpringMotor=" + Joint.SpringAsMotorRow +
          "  AngBetaScale=" + Joint.AngularBetaScale.ToString("G6") +
          "  ErrDeadband=" + Joint.ErrDeadband.ToString("G6") +
          "  BiasDeadband=" + Joint.BiasDeadband.ToString("G6") +
          "  MaxCorrVel=" + Joint.MaxCorrectionVel.ToString("G6"));
        int cross = 0, chain = 0;
        foreach (var j in w.Joints) { if (j.BodyA == null || j.BodyB == null) continue; if (j.IsCrossTypeJoint) cross++; else chain++; }
        L("[実効] " + where + "  構造分類: 横渡し型(並進に自由/範囲あり)=" + cross + "本 / 鎖型(全軸ロック)=" + chain + "本" +
          "  XSplit=" + Joint.SplitCrossOnly + " (対象=" + (Joint.SplitCrossOnly ? cross + "本" : "0本") + ")" +
          "  FreezeAx=" + Joint.FreezeCrossAxes + "  ChainOnlySplit=" + Joint.SplitChainOnly);
    }

    static float MedOf(List<float> v)
    { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[c.Count / 2]; }
    static float PctOf(List<float> v, float q)
    { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[Math.Min(c.Count - 1, (int)(c.Count * q))]; }

    /// <summary>出荷既定の控え。**エンジンに触る前**の静的初期化時点で読む。
    /// 既定が変わってもツール側を書き換えずに済むようにするため (タスク37)。</summary>
    static readonly bool ShippedSpringMotor = Joint.SpringAsMotorRow;

    // env から読む「エンジン全体に効く static つまみ」。Main とスモークの両方から呼ぶ。
    // 毎回既定へ戻してから適用する (スモークが条件を跨いで汚染しないため)。
    static void ApplyGlobalEnv()
    {
        Joint.AngularMixedAxes = Env("AXES") != null ? Env("AXES") == "1" : ShipAxes;
        // タスク22: 角度抽出を Bullet 2.75 実挙動 (R_B⁻¹R_A のオイラー + 角度行の軸反転) へ。
        Joint.BulletAngleConvention = Env("ANGCONV") != null ? Env("ANGCONV") == "1" : ShipAngConv;
        // タスク59: ロック軸の行生成を Bullet の testLimitValue / needApplyForce へ。
        Joint.BulletLimitRowGating = Env("LIMGATE") != null ? Env("LIMGATE") == "1" : ShipLimGate;
        // タスク32: ばねを Bullet 2.75 のモーター行として解く。
        // ★2026-08-22 (タスク37) 既定が ON になったので、env 未設定のときに false へ
        //   上書きしてはいけない。出荷既定は ShippedSpringMotor に静的初期化時点で控えてある。
        // タスク51: マニフォールドの点管理を Bullet 実ソースへ。形状の構築より後でよい (毎ステップ読む)。
        PersistentManifold.BulletManifoldPoints = Env("CMAN") != null ? Env("CMAN") == "1" : ShipCMan;
        // タスク67: validContactDistance の対称化。
        PersistentManifold.SymmetricBreakingDistance = Env("SYMDIST") != null ? Env("SYMDIST") == "1" : ShipSymDist;
        Joint.LeverArmGate = Env("LEVERGATE") != null ? EnvF("LEVERGATE", ShipLeverGate) : ShipLeverGate;   // ★タスク78
        Joint.SpringAsMotorRow = Env("SPRINGMOTOR") != null ? Env("SPRINGMOTOR") == "1" : ShippedSpringMotor;
        // タスク34: 姿勢積分を Bullet 2.75 の実経路 (指数写像) へ。
        PhysicsWorld.BulletRotationIntegration = Env("ROTEXP") != null ? Env("ROTEXP") == "1" : ShipRotExp;
        // タスク38: 接触側の逸脱2件。どちらも形状/接触の構築時に読むので Build より前に。
        GjkEpa.BulletContactThreshold = Env("CTHRESH") != null ? Env("CTHRESH") == "1" : ShipCThresh;
        CollisionShape.BulletShapeMargin = Env("CMARGIN") == "1";
        Joint.LinearLeverMode = Env("LEVER") != null ? EnvI("LEVER", ShipLever) : ShipLever;
        Joint.AngularBetaScale = Env("ANGBETA") != null ? EnvF("ANGBETA", 1f) : 1f;
        Joint.ErrDeadband = Env("ERRDB") != null ? EnvF("ERRDB", 0f) : 0f;
        Joint.BiasDeadband = Env("BIASDB") != null ? EnvF("BIASDB", 0f) : 0f;
        Joint.SplitCrossOnly = Env("XSPLIT") == "1";      // 横渡し型のみ split
        Joint.SplitChainOnly = Env("XSPLIT") == "2";     // 対照: 鎖型のみ split
        Joint.FreezeCrossAxes = Env("FREEZEAX") == "1";   // 診断: 横渡しのロック軸をバインド方向へ凍結
    }

    // ★タスク11-2: 「ONにしたのに出力が既定と1ビットも変わらないフラグ」を機械検出する。
    //   env を実際に立ててから通常の実行経路を通すので、env の配線漏れ (JBETA/JSPLIT で2回踏んだ) も捕まる。
    static int Smoke()
    {
        int frames = EnvI("FRAMES", 240);
        L("=".PadRight(96, '='));
        L("smoke : 各フラグを env 経由で切り替え、出力が既定とビット一致しないことを確認する");
        L("  一致してしまう = そのフラグは配線が切れている (黙殺されている)。" + frames + "フレーム。");
        L("  ★2026-08-23: 完全セットv1 が既定 ON になったので、9フラグは **OFF 側** で検査する");
        L("=".PadRight(96, '='));
        ulong Sig()
        {
            ApplyGlobalEnv();
            var r = Run("smoke", null);
            return r.PoseHash;
        }
        foreach (var k in new[] { "AXES", "ANGCONV", "SPRINGMOTOR", "ROTEXP", "CTHRESH", "CMARGIN", "CRHS", "CMAN", "LIMGATE", "SYMDIST", "LEVER", "ANGBETA", "ERRDB", "JSPLIT", "JBETA", "SUBSTEPS", "ITERS", "SPLIT", "WARMSTART", "BAUM", "DUMMY" })
            Environment.SetEnvironmentVariable(k, null);
        _jbeta = -1f; _baum = -1f;
        ulong baseSig = Sig();
        L(string.Format("  {0,-16} {1,-14} {2,20} {3}", "フラグ", "値", "姿勢ハッシュ", "判定"));
        L(string.Format("  {0,-16} {1,-14} {2,20:X16} {3}", "(既定)", "-", baseSig, ""));
        int fail = 0;
        void Check(string key, string val, Action pre = null)
        {
            Environment.SetEnvironmentVariable(key, val);
            _jbeta = key == "JBETA" ? EnvF("JBETA", -1f) : -1f;
            _baum = key == "BAUM" ? EnvF("BAUM", -1f) : -1f;
            pre?.Invoke();
            ulong sig = Sig();
            Environment.SetEnvironmentVariable(key, null);
            _jbeta = -1f; _baum = -1f;
            bool ok = sig != baseSig;
            if (!ok) fail++;
            L(string.Format("  {0,-16} {1,-14} {2,20:X16} {3}", key, val, sig, ok ? "OK (効いている)" : "★NG 既定と一致=黙殺"));
        }
        Check("JSPLIT", "1");
        Check("ANGCONV", "0");   // ★既定ON → OFF 側で検査
        Check("SPRINGMOTOR", "0");   // ★既定が ON になったので、効いていることは OFF 側で確かめる
        Check("ROTEXP", "0");
        Check("CTHRESH", "0");
        Check("CRHS", "0");
        Check("CMAN", "0");
        Check("LIMGATE", "0");
        Check("SYMDIST", "0");
        // ★CMARGIN はここでは検査しない。**無反応が正常**と確定しているため (タスク38)。
        //   PMX のスカート(箱)×脚(カプセル)は解析解で解くので Margin を読まず、
        //   箱×箱 は同一グループが当たり判定マスクで弾かれてペアが立たない。
        //   ここに置くと毎回★NGが出て、スモークの他の行まで信用されなくなる。
        Check("JBETA", "0.05");
        Check("LEVER", "0");
        Check("AXES", "0");
        Check("ANGBETA", "0");
        Check("ERRDB", "0.01");
        Check("BIASDB", "0.01");
        Check("JWARM", "both");
        Check("XSPLIT", "1");
        Check("FREEZEAX", "1");
        Check("SUBSTEPS", "4");
        Check("ITERS", "30");
        Check("BAUM", "0");
        L();
        L(fail == 0 ? "  => 全フラグ OK (配線漏れなし)" : "  => ★" + fail + "件のフラグが黙殺されている");
        // DUMMY だけは逆の期待: 無関係な遠方剛体なので **一致しなければならない**
        Environment.SetEnvironmentVariable("DUMMY", "1");
        ulong dsig = Sig();
        Environment.SetEnvironmentVariable("DUMMY", null);
        L(string.Format("  {0,-16} {1,-14} {2,20:X16} {3}", "DUMMY(遠方剛体)", "1", dsig,
                        dsig == baseSig ? "OK (期待どおり既定と一致)" : "★NG 無関係な剛体が結果を変えている"));
        File.WriteAllText(Env("OUT") ?? "smoke.txt", O.ToString());
        return fail == 0 && dsig == baseSig ? 0 : 1;
    }

    // ═══ タスク15 事前確認: 「症状が出る最小の網」を探す ═══
    //  1房の鎖だけ残す → 横渡しを足す → 房数を増やす、と段階的に網を広げ、
    //  定常 err (並進ロック行) が 0.013 規模へ育つのはどの段階かを見る。
    //  網の縮小は PMX を作り変えず、ワールド構築後に Joint を間引き+他の動的剛体を kinematic 化して行う。
    static int MinNet()
    {
        int frames = EnvI("FRAMES", 1800);
        int subs = EnvI("SUBSTEPS", 2);
        bool IsCross(string n) => n.Contains("縦_J") || n.Contains("横_J");
        L("=".PadRight(104, '='));
        L("minnet : 症状 (定常err) が出る最小の網を探す  SubSteps=" + subs + "  " + frames + "F  駆動なし");
        L("  err は「拘束されている並進行の |err|」。全ての残存ジョイントの後半窓サンプルの中央値。");
        L("=".PadRight(104, '='));
        {
            var wtmp = PmxPhysicsBuilder.Build(_model).World;
            if (Env("JSPLIT") == "1") wtmp.UseJointSplitImpulse = true;
            EchoFlags(wtmp, "minnet");
        }
        L(string.Format("  {0,-30} {1,7} {2,7} {3,12} {4,12} {5,10} {6,12}", "網", "Joint", "動的", "err中央", "err p90", "|w|中央", "|Δp|中央"));

        void Case(string label, Func<Joint, bool> keep)
        {
            var builder = PmxPhysicsBuilder.Build(_model);
            var world = builder.World;
            world.FixedTimeStep = 1f / 60f; world.SubSteps = subs; world.SolverIterations = EnvI("ITERS", 10);
            if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;   // ★配線漏れ修正 (5件目) 2026-08-21
            if (Env("JWARM") == "lin") world.UseJointWarmStart = true;
            else if (Env("JWARM") == "both") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
            var kept = new List<Joint>();
            foreach (var j in world.Joints) if (j.BodyA != null && j.BodyB != null && keep(j)) kept.Add(j);
            world.Joints.Clear(); world.Joints.AddRange(kept);
            // 残ったジョイントが触る剛体だけ動的のまま。他の動的剛体は kinematic 化+非衝突で無力化。
            var live = new HashSet<RigidBody>();
            foreach (var j in kept) { live.Add(j.BodyA); live.Add(j.BodyB); }
            int dyn = 0;
            foreach (var b in world.Bodies)
            {
                if (b.Mode == PhysicsMode.Dynamic && !live.Contains(b)) { b.Mode = PhysicsMode.BoneFollow; b.CollisionMask = 0; }
                else if (b.Mode == PhysicsMode.Dynamic) dyn++;
            }
            builder.ApplyKinematicTargets(i => (RigidTransform?)null);
            builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

            var log = new List<(string joint, int dof, bool angular, float err, float targetVel, float relVel)>();
            var errs = new List<float>(); var ws = new List<float>();
            var dps = new List<float>(); var prevP = new Dictionary<RigidBody, Vec3>();
            const float R2D = 57.29578f;
            for (int f = 0; f < frames; f++)
            {
                bool late = f >= frames - frames / 3;
                if (late) { Joint.DebugRows = log; log.Clear(); }
                world.StepSimulation(1f / 60f);
                if (!late) continue;
                Joint.DebugRows = null;
                foreach (var r in log) if (!r.angular) errs.Add(Math.Abs(r.err));
                foreach (var b in live) if (b.Mode == PhysicsMode.Dynamic)
                {
                    ws.Add(b.AngularVelocity.Length * R2D);
                    // 参照CSVと同じ土俵の per-frame |Δp| (30fps相当なので 2ステップ分をまとめる)
                    var pnow = b.WorldTransform.Origin;
                    if (prevP.TryGetValue(b, out var pp) && (f % 2) == 0) dps.Add((pnow - pp).Length);
                    if ((f % 2) == 0) prevP[b] = pnow;
                }
            }
            Joint.DebugRows = null;
            L(string.Format("  {0,-30} {1,7} {2,7} {3,12:G6} {4,12:G6} {5,10:F2} {6,12:G4}",
                            label, kept.Count, dyn, MedOf(errs), PctOf(errs, 0.9f), MedOf(ws), MedOf(dps)));
            if (Env("NAMES") == "1")
            {
                var jn = new List<string>(); foreach (var j in kept) jn.Add(j.Name);
                var bn = new List<string>(); foreach (var b in live) if (b.Mode == PhysicsMode.Dynamic) bn.Add(b.Name);
                jn.Sort(); bn.Sort();
                L("      Joint: " + string.Join(" / ", jn));
                L("      剛体  : " + string.Join(" / ", bn));
            }
        }

        string pre = Env("STRAND") ?? "髪BR";
        Case("1房の鎖のみ (" + pre + ", 横渡しなし)", j => j.Name.StartsWith(pre) && !IsCross(j.Name));
        Case("1房 + 横渡し", j => j.Name.StartsWith(pre));
        Case("2房の鎖のみ", j => (j.Name.StartsWith("髪BR") || j.Name.StartsWith("髪BL")) && !IsCross(j.Name));
        Case("2房 + 横渡し", j => j.Name.StartsWith("髪BR") || j.Name.StartsWith("髪BL"));
        Case("髪すべての鎖のみ", j => j.Name.Contains("髪") && !IsCross(j.Name));
        Case("髪すべて (横渡し込み)", j => j.Name.Contains("髪"));
        Case("全ジョイント (現行と同じ)", j => true);
        File.WriteAllText(Env("OUT") ?? "minnet.txt", O.ToString());
        return 0;
    }

    static string Trim(string s, int n) => s.Length <= n ? s : s.Substring(0, n - 1) + "~";

    // ═══ タスク4: err 再生成源の特定 (駆動なし静止) ═══
    //  「補正しても err が毎サブステップ作り直される」犯人を拘束ファミリー別の消去法で絞る。
    //  指標は監査と同じ「フレーム内 err 減衰比」= 最終サブステップの|err| / 最初のサブステップの|err|。
    //  外乱がまったく無ければ (1-Beta)^(SubSteps-1) になるはず。1 に近いほど再生成されている。
    static int ErrSrc()
    {
        string target = Env("TARGETJOINT") ?? "髪BR3縦_J";
        int frames = EnvI("FRAMES", 1800);
        int subs = EnvI("SUBSTEPS", 2);
        float beta0 = EnvF("JBETA", 0.2f);
        L("=".PadRight(110, '='));
        L("errsrc : err 再生成源の消去法  対象=" + target + "  駆動なし静止  SubSteps=" + subs + "  " + frames + "F");
        L("  指標: フレーム内 err 減衰比 (最終サブ/最初サブ) の中央値。理論(外乱なし)=" +
          Math.Pow(1.0 - beta0, subs - 1).ToString("F3") + " / 1.0 = 完全に再生成されている");
        L("=".PadRight(110, '='));
        L(string.Format("  {0,-34} {1,10} {2,12} {3,12} {4,8}", "条件", "減衰比中央", "err中央(初サブ)", "|w|中央", "n"));

        var results = new List<string>();
        void Case(string label, Action<PhysicsWorld, PmxPhysicsBuilder> tweak)
        {
            var builder = PmxPhysicsBuilder.Build(_model);
            var world = builder.World;
            world.FixedTimeStep = 1f / 60f;
            world.SubSteps = subs;
            world.SolverIterations = EnvI("ITERS", 10);
            if (Env("JSPLIT") == "1") world.UseJointSplitImpulse = true;   // ★配線漏れ修正 2026-08-21
        // ジョイント warm-start の A/B (JWARM=lin | both)。係数は Joint.WarmStartFactor(0.85)。
        {
            string wm = Env("JWARM");
            if (wm == "lin") world.UseJointWarmStart = true;
            else if (wm == "both") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        }
            foreach (var j in world.Joints) j.Beta = beta0;
            tweak?.Invoke(world, builder);
            if (label == "既定 (現行の見た目)") EchoFlags(world, "errsrc");   // 実効値を必ず晒す
            builder.ApplyKinematicTargets(i => (RigidTransform?)null);
            builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

            var log = new List<(string joint, int dof, bool angular, float err, float targetVel, float relVel)>();
            var ratios = new List<float>(); var err0s = new List<float>(); var ws = new List<float>();
            var bodies = new List<RigidBody>();
            foreach (var b in world.Bodies) if (!b.IsStaticOrKinematic && b.Name.Contains(_filter == "*" ? "" : _filter)) bodies.Add(b);

            const float R2D = 57.29578f;
            for (int f = 0; f < frames; f++)
            {
                bool late = f >= frames - frames / 3;
                if (late) { Joint.DebugRows = log; log.Clear(); }
                world.StepSimulation(1f / 60f);
                if (!late) continue;
                Joint.DebugRows = null;
                // 同じ (dof, angular) の出現順 = サブステップ番号
                var first = new Dictionary<string, float>(); var last = new Dictionary<string, float>();
                var cnt = new Dictionary<string, int>();
                foreach (var r in log)
                {
                    if (r.joint != target) continue;
                    string k = r.dof + "|" + r.angular;
                    cnt.TryGetValue(k, out int c); cnt[k] = c + 1;
                    if (c == 0) first[k] = Math.Abs(r.err);
                    last[k] = Math.Abs(r.err);
                }
                foreach (var kv in first)
                    if (kv.Value > 1e-9f && cnt[kv.Key] >= 2) { ratios.Add(last[kv.Key] / kv.Value); err0s.Add(kv.Value); }
                foreach (var b in bodies) ws.Add(b.AngularVelocity.Length * R2D);
            }
            Joint.DebugRows = null;
            float Med(List<float> v) { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[c.Count / 2]; }
            L(string.Format("  {0,-34} {1,10:F3} {2,12:G6} {3,12:F2} {4,8}",
                            label, Med(ratios), Med(err0s), Med(ws), ratios.Count));
        }

        Case("既定 (現行の見た目)", null);
        Case("角度リミット行OFF (AngBeta=0)", (w, b) => { Joint.AngularBetaScale = 0f; });
        Joint.AngularBetaScale = 1f;
        Case("ばね全殺し", (w, b) => { foreach (var j in w.Joints) { j.SpringLinear = Vec3.Zero; j.SpringAngular = Vec3.Zero; } });
        Case("接触OFF (全剛体マスク0)", (w, b) => { foreach (var x in w.Bodies) x.CollisionMask = 0; });
        Case("隣接ジョイントのみ Beta=0", (w, b) =>
        {
            // 対象ジョイントの2剛体に繋がる「他の」ジョイントの位置補正だけ切る。
            RigidBody ta = null, tb = null;
            foreach (var j in w.Joints) if (j.Name == target) { ta = j.BodyA; tb = j.BodyB; }
            int n = 0;
            foreach (var j in w.Joints)
            {
                if (j.Name == target) continue;
                if (j.BodyA == ta || j.BodyB == ta || j.BodyA == tb || j.BodyB == tb) { j.Beta = 0f; n++; }
            }
            L("    (隣接ジョイント " + n + " 本の Beta を 0 にした)");
        });
        Case("★JSPLIT ON (位置補正を擬似速度へ)", (w, b) => { w.UseJointSplitImpulse = true; });
        Case("対象ジョイント以外すべて Beta=0", (w, b) =>
        { foreach (var j in w.Joints) if (j.Name != target) j.Beta = 0f; });
        Case("接触OFF + 対象以外 Beta=0", (w, b) =>
        {
            foreach (var x in w.Bodies) x.CollisionMask = 0;
            foreach (var j in w.Joints) if (j.Name != target) j.Beta = 0f;
        });
        File.WriteAllText(Env("OUT") ?? "errsrc.txt", O.ToString());
        return 0;
    }

    // ═══ タスク4-3 / タスク5: 行単位の収支 ═══
    //  1サブステップぶんの各行の最終インパルスを取り、対象行の相対速度への寄与へ分解する。
    //  寄与が「err を増やす向き」の行が、補正を打ち消している当人。
    //  タスク5: 対象ジョイントの線形ロック行について 10反復後の残留相対速度(=未収束量)も出す。
    static int RowBudget()
    {
        string target = Env("TARGETJOINT") ?? "髪BR3縦_J";
        string scope = Env("SCOPE") ?? "髪BR";       // 収支に載せる行の範囲 (部分一致)
        int frames = EnvI("FRAMES", 1200);
        int subs = EnvI("SUBSTEPS", 2);
        var builder = PmxPhysicsBuilder.Build(_model);
        var world = builder.World;
        world.FixedTimeStep = 1f / 60f;
        world.SubSteps = subs;
        world.SolverIterations = EnvI("ITERS", 10);
        float jb = EnvF("JBETA", -1f);
        if (jb >= 0f) foreach (var j in world.Joints) j.Beta = jb;
        builder.ApplyKinematicTargets(i => (RigidTransform?)null);
        builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

        var byName = new Dictionary<string, RigidBody>();
        foreach (var b in world.Bodies) byName[b.Name] = b;

        var solved = new List<(string joint, int dof, bool angular, Vec3 axis, Vec3 relA, Vec3 relB,
                               string bodyA, string bodyB, float accumulated, float targetVel, float relVelAfter)>();
        for (int f = 0; f < frames; f++)
        {
            bool last = f == frames - 1;
            if (last) { Joint.DebugRowsSolvedJoint = scope; Joint.DebugRowsSolved = solved; }
            world.StepSimulation(1f / 60f);
        }
        Joint.DebugRowsSolved = null; Joint.DebugRowsSolvedJoint = null;

        // 最終サブステップ・最終反復 = 各 (joint,dof,angular) の最後の出現
        var fin = new Dictionary<string, (string joint, int dof, bool angular, Vec3 axis, Vec3 relA, Vec3 relB,
                                          string bodyA, string bodyB, float accumulated, float targetVel, float relVelAfter)>();
        foreach (var r in solved) fin[r.joint + "|" + r.dof + "|" + r.angular] = r;

        L("=".PadRight(112, '='));
        L("rowbudget : 行単位の収支  対象=" + target + "  収支範囲='" + scope + "'  SubSteps=" + subs +
          "  反復=" + world.SolverIterations + (jb >= 0f ? "  Beta=" + jb : ""));
        L("=".PadRight(112, '='));

        // --- タスク5: 対象ジョイントの線形ロック行の未収束量 ---
        foreach (var j in world.Joints)
        {
            if (j.Name != target || j.BodyA == null || j.BodyB == null) continue;
            var wA = j.BodyA.WorldTransform * j.FrameInA;
            var wB = j.BodyB.WorldTransform * j.FrameInB;
            float com = (j.BodyA.WorldTransform.Origin - j.BodyB.WorldTransform.Origin).Length;
            L(string.Format("  [幾何] アンカー間距離={0:G6} (ロック軸のみ0が正しい)  重心間距離={1:G6}  剛体A='{2}' B='{3}'",
                            (wB.Origin - wA.Origin).Length, com, j.BodyA.Name, j.BodyB.Name));
            string Kind(float lo, float hi) => lo > hi ? "自由" : (lo == hi ? "ロック" : "範囲[" + lo.ToString("G4") + "," + hi.ToString("G4") + "]");
            L(string.Format("  [制限] 移動 X={0} Y={1} Z={2}", Kind(j.LinearLowerLimit.x, j.LinearUpperLimit.x),
                            Kind(j.LinearLowerLimit.y, j.LinearUpperLimit.y), Kind(j.LinearLowerLimit.z, j.LinearUpperLimit.z)));
            L(string.Format("         回転 X={0} Y={1} Z={2}", Kind(j.AngularLowerLimit.x, j.AngularUpperLimit.x),
                            Kind(j.AngularLowerLimit.y, j.AngularUpperLimit.y), Kind(j.AngularLowerLimit.z, j.AngularUpperLimit.z)));
        }
        L("  --- 10反復後の残留 (未収束量) : ジョイント '" + target + "' の全行 ---");
        L(string.Format("  {0,-6} {1,-5} {2,14} {3,14} {4,14} {5,10}", "行", "DOF", "目標速度", "反復後相対速度", "残差", "|残差|/|目標|"));
        foreach (var kv in fin)
        {
            var r = kv.Value; if (r.joint != target) continue;
            float resid = r.relVelAfter - r.targetVel;
            L(string.Format("  {0,-6} {1,-5} {2,14:G6} {3,14:G6} {4,14:G6} {5,10:F3}",
                            r.angular ? "回転" : "並進", r.dof, r.targetVel, r.relVelAfter, resid,
                            Math.Abs(r.targetVel) > 1e-9f ? Math.Abs(resid) / Math.Abs(r.targetVel) : 0f));
        }

        // --- タスク4-3: 対象行への寄与分解 ---
        // 行 r のインパルス λ が、対象行 t の相対速度へ与える変化を計算する。
        // 行 r: A へ -λ*axis (点 relA), B へ +λ*axis (点 relB) / 回転行はトルク力積。
        foreach (var kv in fin)
        {
            var t = kv.Value; if (t.joint != target) continue;
            var ba = byName[t.bodyA]; var bb = byName[t.bodyB];
            Vec3 DLin(RigidBody body, in (string joint, int dof, bool angular, Vec3 axis, Vec3 relA, Vec3 relB,
                      string bodyA, string bodyB, float accumulated, float targetVel, float relVelAfter) r, out Vec3 dAng)
            {
                dAng = Vec3.Zero; var dv = Vec3.Zero;
                float sgn = body.Name == r.bodyB ? 1f : (body.Name == r.bodyA ? -1f : 0f);
                if (sgn == 0f) return dv;
                var P = r.axis * (r.accumulated * sgn);
                if (r.angular) { dAng = body.InverseInertiaWorld * P; return dv; }
                dv = P * body.InverseMass;
                var arm = body.Name == r.bodyB ? r.relB : r.relA;
                dAng = body.InverseInertiaWorld * Vec3.Cross(arm, P);
                return dv;
            }
            var contrib = new List<(string tag, float v)>();
            float total = 0f;
            foreach (var kv2 in fin)
            {
                var r = kv2.Value;
                var dvA = DLin(ba, r, out var dwA);
                var dvB = DLin(bb, r, out var dwB);
                if (dvA.LengthSquared + dwA.LengthSquared + dvB.LengthSquared + dwB.LengthSquared < 1e-24f) continue;
                float d = t.angular
                    ? (dwB - dwA).Dot(t.axis)
                    : ((dvB + Vec3.Cross(dwB, t.relB)) - (dvA + Vec3.Cross(dwA, t.relA))).Dot(t.axis);
                if (Math.Abs(d) < 1e-7f) continue;
                contrib.Add((r.joint + "[" + (r.angular ? "回転" : "並進") + r.dof + "]" + (r.joint == t.joint ? " ←自分" : ""), d));
                total += d;
            }
            contrib.Sort((x, y) => Math.Abs(y.v).CompareTo(Math.Abs(x.v)));
            L("");
            L("  --- 収支: " + target + " の " + (t.angular ? "回転" : "並進") + t.dof +
              " 行 (目標速度=" + t.targetVel.ToString("G6") + " / 反復後=" + t.relVelAfter.ToString("G6") + ") ---");
            L("     各行の最終インパルスが、この行の相対速度に与えた寄与 (目標と同符号=補正を助ける / 逆符号=打ち消す)");
            // 分類は「目標との符号」ではなく「求解が消さねばならなかった速度との符号」で行う。
            // 反復前の相対速度 = 反復後 - 全寄与。必要な変化 = 目標 - 反復前。
            float before = t.relVelAfter - total;
            float need = t.targetVel - before;
            float sgnT = need >= 0 ? 1f : -1f;
            float help = 0f, hurt = 0f;
            foreach (var c in contrib) { if (c.v * sgnT > 0) help += c.v * sgnT; else hurt += -c.v * sgnT; }
            L(string.Format("     反復前の相対速度={0:G6} → 目標={1:G6} (必要な変化={2:G6})", before, t.targetVel, need));
            L(string.Format("     合計={0:G6}  (必要な向きの和={1:G6} / 逆向きの和={2:G6})", total, help, hurt));
            int n = 0;
            foreach (var c in contrib)
            {
                if (n++ >= 12) break;
                L(string.Format("     {0,-34} {1,12:G6}  {2}", c.tag, c.v, c.v * sgnT > 0 ? "必要な向き" : "★逆向き(打ち消す)"));
            }
        }
        File.WriteAllText(Env("OUT") ?? "rowbudget.txt", O.ToString());
        return 0;
    }

    static int Main()
    {
        ApplyGlobalEnv();

        if (Env("MINNET") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "髪";
            return MinNet();
        }
        // タスク28: 接触の生成を出す (段(a) の当エンジン側)。
        if (Env("CONTACTTRACE") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "*";
            return ContactTrace.Run(_model, _filter);
        }
        // タスク24 続き: 駆動なし静止のボーン |Δp| を 既定 vs 3点構成 で出す (ベイクと同じ物差し)。
        if (Env("BONEDP") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "*";
            return BoneDp.Run(_model, _filter);
        }
        // タスク25: 合成駆動での揺れ幅 A/B (参照VMDが無いモデル用)。
        if (!string.IsNullOrEmpty(Env("SYNDRIVE")))
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "*";
            return SynDrive.Run(_model, _filter);
        }
        // タスク20: 最小網を数値のまま書き出し、本物の Bullet 2.75 (tools/diagnostics/bulletref) で
        // 同じ網を組み直せるようにする。エンジンは読み取りのみ (DebugRows フック)。
        if (Env("NETDUMP") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "髪";
            return NetDump.Run(_model, "髪BR");
        }
        if (Env("SMOKE") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "髪";
            return Smoke();
        }
        if (Env("ROWBUDGET") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "髪";
            return RowBudget();
        }
        if (Env("ERRSRC") == "1")
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "髪";
            return ErrSrc();
        }
        string dv = Env("DRIVE");
        if (!string.IsNullOrEmpty(dv) && File.Exists(dv))
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "*";
            return Drive(dv);
        }
        string rc = Env("REFCHECK");
        if (!string.IsNullOrEmpty(rc) && File.Exists(rc))
        {
            _model = PmxReader.LoadFile(TestData.PmxPath());
            _filter = Env("BODIES") ?? "*";
            return RefCheck(rc);
        }
        string ml = Env("MODELS");
        if (!string.IsNullOrEmpty(ml) && File.Exists(ml)) return Batch(ml);
        string pmx = TestData.PmxPath();
        if (pmx == null) { L("[SKIP] PMX が無い (MMD_TEST_PMX)"); return 0; }
        _model = PmxReader.LoadFile(pmx);
        _filter = Env("BODIES") ?? "ﾈｸﾀｲ";
        // ★単体モードでも JBETA を効かせる (従来は Batch/Drive のみで、単体では黙って無視されていた)。
        if (Env("JBETA") != null) _jbeta = EnvF("JBETA", -1f);
        int frames = EnvI("FRAMES", 900);

        L("=".PadRight(96, '='));
        L("restosc : 駆動なし(バインド姿勢)で揺れ物が収束するかを測る");
        L("  PMX=" + Path.GetFileName(pmx) + "  ボーン" + _model.BoneNames.Count +
          " 剛体" + _model.RigidBodies.Count + " Joint" + _model.Joints.Count);
        L("  対象フィルタ='" + _filter + "'  " + frames + "フレーム @" + EnvI("FPS", 60) + "Hz" +
          "  SubSteps=" + EnvI("SUBSTEPS", 2) + " Iters=" + EnvI("ITERS", 10));
        L("=".PadRight(96, '='));

        var runs = new List<Result>();
        runs.Add(Run("既定 (現行の見た目)", null));

        if (Env("AB") == "1")
        {
            // --- エネルギー源の切り分け。既定値からの単項目変更のみ。 ---
            runs.Add(Run("接触なし (全剛体マスク0)", (w, b) => {
                foreach (var x in w.Bodies) x.CollisionMask = 0;
            }));
            runs.Add(Run("接触ウォームスタート0", (w, b) => { w.ContactWarmStartFactor = 0f; }));
            runs.Add(Run("Baumgarte 0.2->0", (w, b) => { w.BaumgarteFactor = 0f; }));
            runs.Add(Run("SplitImpulse ON (閾値0)", (w, b) => {
                w.UseSplitImpulse = true; w.SplitImpulsePenetrationThreshold = 0f;
            }));
            runs.Add(Run("摩擦0 (全剛体)", (w, b) => {
                foreach (var x in w.Bodies) x.Friction = 0f;
            }));
            runs.Add(Run("ばね全殺し", (w, b) => {
                foreach (var j in w.Joints) { j.SpringLinear = Vec3.Zero; j.SpringAngular = Vec3.Zero; }
            }));
            runs.Add(Run("角度リミット10倍", (w, b) => {
                foreach (var j in w.Joints) { j.AngularLowerLimit *= 10f; j.AngularUpperLimit *= 10f; }
            }));
            runs.Add(Run("★JointBeta 0.2->0", (w, b) => {
                foreach (var j in w.Joints) j.Beta = 0f;
            }));
            runs.Add(Run("★接触なし+JointBeta0", (w, b) => {
                foreach (var x in w.Bodies) x.CollisionMask = 0;
                foreach (var j in w.Joints) j.Beta = 0f;
            }));
            foreach (var bv in new[] { 2.0f, 1.0f, 0.7f, 0.5f, 0.3f, 0.15f, 0.10f, 0.05f, 0.02f })
            {
                float bb = bv;
                runs.Add(Run("JointBeta " + bb.ToString("0.00"), (w, b) => {
                    foreach (var j in w.Joints) j.Beta = bb;
                }));
            }
            runs.Add(Run("★JointSplitImpulse ON", (w, b) => { w.UseJointSplitImpulse = true; }));
            runs.Add(Run("★Joint warm ON(両方)", (w, b) => {
                w.UseJointWarmStart = true; w.UseJointWarmStartAngular = true;
            }));
            runs.Add(Run("★MaxCorrVel 10->1e9", (w, b) => { Joint.MaxCorrectionVel = 1e9f; }));
            runs.Add(Run("★MaxCorrVel 10->100", (w, b) => { Joint.MaxCorrectionVel = 100f; }));
            runs.Add(Run("★MaxCorrVel 10->2", (w, b) => { Joint.MaxCorrectionVel = 2f; }));
            runs.Add(Run("Iters 10->50", (w, b) => { w.SolverIterations = 50; }));
            runs.Add(Run("SubSteps 2->8", (w, b) => { w.SubSteps = 8; }));
        }

        L();
        L("---------- 収束の判定 (対象剛体の平均|角速度| deg/s) ----------");
        L(string.Format("  {0,-26} {1,12} {2,12} {3,9} {4,8} {5,10} {6,10} {7,10}",
                        "条件", "前半1/3平均", "後半1/3平均", "比(後/前)", "周期F", "違反中央", "違反p90", "違反max"));
        var baseR = runs[0];
        foreach (var r in runs)
        {
            float ratio = r.EarlyMeanW > 1e-9f ? r.LateMeanW / r.EarlyMeanW : 0f;
            L(string.Format("  {0,-26} {1,12:F4} {2,12:F4} {3,9:F3} {4,8} {5,10:F1} {6,10:F4} {7,10:F4}",
                            r.Label, r.EarlyMeanW, r.LateMeanW, ratio,
                            r.Period == 0 ? "-" : r.Period.ToString(),
                            r.ViolMed, r.ViolP90, r.ViolMax));
        }
        L();
        L("  読み方: 後半平均が前半より十分小さければ収束。比が 1 付近なら止まっていない(リミットサイクル)。");
        L("          既定に対して後半平均が大きく下がった条件が、エネルギー源。");

        L();
        L("  ★後半窓の |w| 分布 (剛体×フレームの全サンプル。リファレンス本編静区間: 平均61.3 中央13.1 p90 157.5):");
        L(string.Format("    {0,-26} {1,9} {2,9} {3,9} {4,10} {5,10}", "条件", "平均", "中央", "p90", "最大", "静止率(<5)"));
        foreach (var r in runs)
            L(string.Format("    {0,-26} {1,9:F2} {2,9:F2} {3,9:F2} {4,10:F1} {5,9:P1}",
                            r.Label, r.LateMeanW, r.LateMedW, r.LateP90W, r.LateMaxW, r.LateQuietFrac));
        // ★角度リミット行のチャタリング診断 (既定条件のみ)
        if (Env("CHATTER") == "1") Chatter();
        L();
        L("  ★★30fpsサンプル版 |w| (VMDが記録できる量。リファレンスと同じ土俵):");
        L("     リファレンス(本編ダンス静区間 819F): 平均61.3 中央13.1 p90 157.5");
        L(string.Format("    {0,-26} {1,9} {2,9} {3,9} {4,10} {5,10}", "条件", "平均", "中央", "p90", "最大", "静止率(<5)"));
        foreach (var r in runs)
            L(string.Format("    {0,-26} {1,9:F2} {2,9:F2} {3,9:F2} {4,10:F1} {5,9:P1}",
                            r.Label, r.S30MeanW, r.S30MedW, r.S30P90W, r.S30MaxW, r.S30QuietFrac));
        L();
        L("  ★★絶対|w| vs 相対|w| (相対=ジョイント2剛体の角速度差。ジョイントが制御している当の量)");
        L(string.Format("    {0,-26} {1,9} {2,9} | {3,9} {4,9} {5,9}",
                        "条件", "絶対中央", "絶対p90", "相対平均", "相対中央", "相対p90"));
        foreach (var r in runs)
            L(string.Format("    {0,-26} {1,9:F2} {2,9:F2} | {3,9:F2} {4,9:F2} {5,9:F2}",
                            r.Label, r.LateMedW, r.LateP90W, r.RelMeanW, r.RelMedW, r.RelP90W));
        L();
        L("  最深の接触ペア / 拘束違反(アンカー誤差, 0が正しい):");
        foreach (var r in runs)
            L("    " + r.Label.PadRight(26) + " 貫入" + r.PenMax.ToString("F4") +
              "  違反最大" + r.ViolMax.ToString("G4") + " (" + (r.ViolWorst ?? "-") + ")");
        L();
        L("---------- 時系列 (既定条件) ----------");
        Sparkline("平均|w| deg/s", baseR.MeanW);
        Sparkline("最大|w| deg/s", baseR.MaxW);
        Sparkline("接触点数", baseR.ContactN);
        Sparkline("最大貫入", baseR.MaxPen);
        Sparkline("法線力積和", baseR.SumNi);

        // CSV
        string outp = Env("OUT") ?? "restosc.csv";
        using (var sw = new StreamWriter(outp))
        {
            sw.WriteLine("frame,meanW_deg_s,maxW_deg_s,contactN,maxPen,sumNormalImpulse");
            for (int f = 0; f < baseR.MeanW.Length; f++)
                sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0},{1:G7},{2:G7},{3},{4:G7},{5:G7}",
                             f, baseR.MeanW[f], baseR.MaxW[f], (int)baseR.ContactN[f], baseR.MaxPen[f], baseR.SumNi[f]));
        }
        L();
        L("[出力] " + Path.GetFullPath(outp));
        File.WriteAllText(Path.ChangeExtension(outp, ".txt"), O.ToString());
        return 0;
    }
}
