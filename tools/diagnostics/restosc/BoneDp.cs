// ===========================================================================
// BoneDp: 駆動なし静止での **ボーンの per-frame |Δp|** を 既定 vs 3点構成 で出す。
//
//  なぜ要るか (タスク24 の続き):
//    神託で「Tda式V4X は 髪は止まる / スカートは振動」と分かった。スカートは本家でも
//    振動しているので、当エンジンの「中央値悪化 197→284」が過剰なのか妥当なのかは
//    **振幅を本家と同じ物差しで比べないと決まらない**。
//    本家側は PMXエディタのベイク VMD から `analyze_static_bake.py` で
//    per-frame |Δp| (PMX単位/フレーム@30fps) が取れる。こちらはその**当エンジン版**。
//
//  ★定義は analyze_static_bake.py / タスク6 / タスク9 と完全に同一:
//    - ボーン姿勢は剛体から復元する (`bodyWorld * BodyOffsetFromBone⁻¹`)
//    - **30fps 標本** (dt=1/60 で回して2フレームおきに採る)
//    - 後半1/3のフレームで**ボーン別の中央値**を取り、そのボーン間中央値を代表値にする
//
//  env:
//    MMD_TEST_PMX  対象PMX      BODIES  剛体名フィルタ ('*' で全部)
//    FRAMES        既定 1800    SUBSTEPS 既定 2   ITERS 既定 10
//    BONEDP=1      このモードの起動スイッチ
//    CSV           指定すると全ボーンの時系列を書き出す
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;

static class BoneDp
{
    /// <summary>出荷既定の控え (タスク37)。エンジンに触る前の静的初期化時点で読む。</summary>
    static readonly bool ShippedSpringMotor = Joint.SpringAsMotorRow;
    // ★A/B の A 条件 (旧エンジン相当) でだけ world の接触 rhs を旧方式へ落とすためのフラグ。
    //   ContactRhsBullet は静的ではなく world のインスタンス値なので、静的な apply では触れない。
    static bool _forceOldContactRhs = false;
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


    static string Env(string k) => Environment.GetEnvironmentVariable(k);
    static int EnvI(string k, int d) { int v; return int.TryParse(Env(k), out v) ? v : d; }
    static float EnvF(string k, float d) { float v; return float.TryParse(Env(k), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : d; }

    static float Med(List<float> v) { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[c.Count / 2]; }
    static float Pct(List<float> v, float q) { if (v.Count == 0) return float.NaN; var c = new List<float>(v); c.Sort(); return c[Math.Min(c.Count - 1, (int)(c.Count * q))]; }

    sealed class Res
    {
        public string Label;
        public List<string> Bones = new();
        public List<float> PerBoneMed = new();
        public List<float> PerBoneP90 = new();
        public float Med, P90;
        // ★タスク44: 静止判定の3つ組 (到達値単独での判定は禁止)。
        public List<float> WinMed = new();   // 10秒窓ごとの |Δp| 中央 (ボーン間中央)
        public bool Converged;               // 収束したか (REFFLOOR 未指定なら判定しない)
        public float TauSec = float.NaN;     // 収束時間 [秒]
        public float Floor;                  // 収束後フロア = 最終窓の中央
        public long SpringRows, SpringClamped;
        public double SpringAbs, SpringShaved;
        public bool Bad;          // NaN / 発散
        // ★実効値の控え。**env ではなくエンジンから読み戻す** (タスク46 でエコーが env を
        //   読んでいたため『SpringMotor=False』と誤表示し、土台が崩れたかに見えた)。
        public bool SpringMotorUsed, AngConvUsed, MixedAxesUsed, RotExpUsed, CThreshUsed;
        public int LeverUsed;
    }

    public static int Run(PmxPhysicsModel model, string filter)
    {
        // ★タスク44: 静止窓の標準は 3600F (60秒)。1800F では収束途中を切り取ってしまう
        //   (実測: 純Bullet は10秒で収束、当エンジンの一部構成は40〜60秒かかる)。
        int frames = EnvI("FRAMES", 3600);
        float dt = 1f / 60f;
        var O = new StringBuilder();
        void L(string s = "") { Console.WriteLine(s); O.Append(s); O.Append('\n'); }

        L("=".PadRight(104, '='));
        L("bonedp : 駆動なし静止の per-frame |Δp| (PMX単位/フレーム@30fps)  既定 vs 3点構成  " + frames + "F");
        L("  ★定義は analyze_static_bake.py と完全に同一。PMXエディタのベイクとそのまま比べられる。");
        L("    ボーン姿勢 = bodyWorld * BodyOffsetFromBone⁻¹ / 30fps標本 / 後半1/3でボーン別中央");
        L("=".PadRight(104, '='));

        // ★2026-08-23 A/B の再定義 (完全セットv1 の既定化にともなう)。
        //   旧: A=「既定」/ B=「3点構成」。3点が既定に入ったのでこの対比は意味を失った。
        //   新: **A =「旧エンジン相当」= 完全セットv1 の 9 フラグを全部 OFF**(歴史比較用)
        //       **B =「出荷既定」= v1 が効いた状態**。個別フラグの env 上書きは従来どおり
        //       両条件に効く (LIMGATE/SYMDIST/CTHRESH/CMAN/CRHS は土台として共通)。
        //   A は「切替前のエンジンで測るとどうだったか」を毎回並べて出すためのもので、
        //   採否の判定に使うのは B の側である。
        Res Once(string label, Action apply)
        {
            Joint.BulletAngleConvention = Env("ANGCONV") != null ? Env("ANGCONV") == "1" : ShipAngConv;
            Joint.AngularMixedAxes = Env("AXES") != null ? Env("AXES") == "1" : ShipAxes;
            Joint.LinearLeverMode = Env("LEVER") != null ? EnvI("LEVER", ShipLever) : ShipLever;
            PhysicsWorld.BulletRotationIntegration = Env("ROTEXP") != null ? Env("ROTEXP") == "1" : ShipRotExp;
            Joint.BulletLimitRowGating = Env("LIMGATE") != null ? Env("LIMGATE") == "1" : ShipLimGate;   // タスク59 (両条件に共通の土台)
            PersistentManifold.SymmetricBreakingDistance = Env("SYMDIST") != null ? Env("SYMDIST") == "1" : ShipSymDist;   // タスク67 (同上)
            // ★2026-08-22 (タスク37) 既定が ON になったので、env 未設定のときに false へ
            //   上書きしてはいけない。出荷既定は ShippedSpringMotor に静的初期化時点で控えてある。
            Joint.SpringAsMotorRow = Env("SPRINGMOTOR") != null ? Env("SPRINGMOTOR") == "1" : ShippedSpringMotor;
            Joint.MaxCorrectionVel = 10f;
            // ★タスク27: ばねクランプの診断。両条件に同じだけ掛ける (被験項ではなく土台)。
            Joint.DisableSpringClamp = Env("SPRINGCLAMP") == "off";
            Joint.CollectSpringClampStats = true;
            Joint.ResetSpringClampStats();
            // タスク38: 接触側の逸脱2件。両条件に同じだけ掛ける土台 (被験項ではない)。
            //   ★CMARGIN は **形状の構築時**に読まれるので、必ず Build より前に立てること。
            GjkEpa.BulletContactThreshold = Env("CTHRESH") != null ? Env("CTHRESH") == "1" : ShipCThresh;
            CollisionShape.BulletShapeMargin = Env("CMARGIN") == "1";
            apply?.Invoke();

            var builder = PmxPhysicsBuilder.Build(model);
            var world = builder.World;
            world.FixedTimeStep = dt;
            world.SubSteps = EnvI("SUBSTEPS", 2);
            world.SolverIterations = EnvI("ITERS", 10);
            // ★タスク26: 接触側の逸脱を **両条件に同じだけ** 掛ける (接触は A/B の土台であって被験項ではない)。
            //   SLOP        : PenetrationSlop。Bullet 2.75 は m_linearSlop = 0.0、当エンジン既定 0.005
            //   JOINTS_FIRST: SolveJointsFirst。Bullet はジョイント→接触の順、当エンジン既定は逆
            //   BAUM / CWFAC: 接触 Baumgarte と warm 係数 (どちらも既定が既に Bullet 値 0.2 / 0.85)
            {
                float v;
                if (float.TryParse(Env("SLOP"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f)
                    world.PenetrationSlop = v;
                if (Env("JOINTS_FIRST") == "1") world.SolveJointsFirst = true;
                if (Env("CRHS") != null) world.ContactRhsBullet = Env("CRHS") == "1";   // ★タスク48 (未設定=出荷既定)
                if (_forceOldContactRhs) world.ContactRhsBullet = false;   // A 条件 (旧エンジン相当)
                PersistentManifold.BulletManifoldPoints = Env("CMAN") != null ? Env("CMAN") == "1" : ShipCMan;   // ★タスク51
                if (float.TryParse(Env("BAUM"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f)
                    world.BaumgarteFactor = v;
                if (float.TryParse(Env("CWFAC"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f)
                    world.ContactWarmStartFactor = v;
                // ★タスク30: 接触ソルバの Bullet 整合セット (すべて既定OFF・ビット不変)
                if (Env("CPOOL") == "1") world.ContactPoolOrder = true;          // 法線プール→摩擦プール
                if (Env("NORMFIRST") == "1") world.ContactNormalBeforeFriction = true;
                if (Env("FRICALIGN") == "1") world.FrictionVelocityAligned = true; // 1方向・接線速度整列
                if (Env("FRICMUL") == "1") world.FrictionCombineMultiply = true;   // 摩擦合成=積
                // ★CSET は **3フラグ** (CPOOL + FRICALIGN + FRICMUL)。
                //   4つ目の ContactNormalBeforeFriction は NORMFIRST で別に立てること。
                //   (以前ここに「4点セット」と書いていたが数が合っていなかった)
                if (Env("CSET") == "1")
                { world.ContactPoolOrder = true; world.FrictionVelocityAligned = true; world.FrictionCombineMultiply = true; }
            }
            // NOCONTACT=1: 全剛体の衝突マスクを 0 にして接触を完全に消す (切り分け専用)。
            //   「残留振動の源が接触かジョイントか」を一発で割る。
            if (Env("NOCONTACT") == "1") foreach (var b in world.Bodies) b.CollisionMask = 0;
            builder.ApplyKinematicTargets(i => (RigidTransform?)null);
            builder.ResetBodiesToBonePoseFk(i => (RigidTransform?)null);

            var links = new List<BoneLink>();
            var names = new List<string>();
            foreach (var l in builder.BoneLinks)
            {
                if (l.Mode == PhysicsMode.BoneFollow) continue;
                if (l.Body == null || l.Body.Name == null) continue;
                if (filter != "*" && !l.Body.Name.Contains(filter)) continue;
                links.Add(l);
                names.Add(l.BoneIndex >= 0 && l.BoneIndex < model.BoneNames.Count
                          ? model.BoneNames[l.BoneIndex] : l.Body.Name);
            }
            if (links.Count == 0) throw new Exception("対象が0: フィルタ '" + filter + "'");

            var series = new List<float>[links.Count];
            var all = new List<float>[links.Count];               // ★全区間 (窓集計用)
            for (int i = 0; i < links.Count; i++) { series[i] = new List<float>(); all[i] = new List<float>(); }
            var prev = new Vec3[links.Count];
            bool have = false;
            int lateFrom = frames - frames / 3;

            for (int f = 0; f < frames; f++)
            {
                world.StepSimulation(dt);
                if ((f & 1) != 0) continue;                       // 30fps 標本
                for (int i = 0; i < links.Count; i++)
                {
                    var bone = links[i].Body.WorldTransform * links[i].BodyOffsetFromBone.Inverse();
                    if (have)
                    {
                        float d = (bone.Origin - prev[i]).Length;
                        all[i].Add(d);
                        if (f >= lateFrom) series[i].Add(d);
                    }
                    prev[i] = bone.Origin;
                }
                have = true;
            }

            var r = new Res { Label = label };
            r.SpringMotorUsed = Joint.SpringAsMotorRow; r.AngConvUsed = Joint.BulletAngleConvention;
            r.MixedAxesUsed = Joint.AngularMixedAxes; r.LeverUsed = Joint.LinearLeverMode;
            r.RotExpUsed = PhysicsWorld.BulletRotationIntegration; r.CThreshUsed = GjkEpa.BulletContactThreshold;
            // ─ ★タスク44: 収束の3つ組 ─
            //   定義 (**固定。途中で変えないこと**):
            //     窓        = WINSEC 秒 (既定10)。30fps 標本なので 1窓 = WINSEC*30 サンプル。
            //     窓の代表値 = その窓のボーン別中央値の、ボーン間中央値。
            //     収束した  = ある窓以降 **最後まで** 窓の代表値が REFFLOOR * 3 以下。
            //     τ        = その最初の窓の開始時刻 [秒]。満たす窓が無ければ「収束せず」。
            //     フロア    = 最終窓の代表値。参照比 = フロア / REFFLOOR。
            //   ★係数 3 は固定の取り決めであってつまみではない。REFFLOOR 未指定なら判定を出さない。
            {
                int perWin = Math.Max(1, EnvI("WINSEC", 10) * 30);
                int total = all[0].Count;
                // ★端数の最終窓も、半分以上サンプルがあれば採る。切り捨てると
                //   **収束が起きる終盤の10秒がまるごと落ちる** (実測: 3600F で 1799 サンプル → 5窓しか出ず、
                //   50〜60秒の窓が消えていた)。
                for (int st2 = 0; st2 < total; st2 += perWin)
                {
                    int len = Math.Min(perWin, total - st2);
                    if (len < perWin / 2) break;
                    var perBone = new List<float>();
                    for (int i = 0; i < links.Count; i++)
                        perBone.Add(Med(all[i].GetRange(st2, len)));
                    r.WinMed.Add(Med(perBone));
                }
                r.Floor = r.WinMed.Count > 0 ? r.WinMed[r.WinMed.Count - 1] : float.NaN;
                float refFloor = EnvF("REFFLOOR", -1f);
                if (refFloor > 0f && r.WinMed.Count > 0)
                {
                    float band = refFloor * 3f;
                    int first = -1;
                    for (int w = r.WinMed.Count - 1; w >= 0; w--)
                    {
                        if (r.WinMed[w] <= band) first = w; else break;
                    }
                    r.Converged = first >= 0;
                    r.TauSec = first >= 0 ? first * EnvI("WINSEC", 10) : float.NaN;
                }
            }
            for (int i = 0; i < links.Count; i++)
            {
                r.Bones.Add(names[i]);
                r.PerBoneMed.Add(Med(series[i]));
                r.PerBoneP90.Add(Pct(series[i], 0.9f));
            }
            r.Med = Med(r.PerBoneMed);
            r.P90 = Med(r.PerBoneP90);
            r.SpringRows = Joint.SpringRows; r.SpringClamped = Joint.SpringClamped;
            r.SpringAbs = Joint.SpringImpulseAbsSum; r.SpringShaved = Joint.SpringShavedSum;
            foreach (var v2 in r.PerBoneMed) if (float.IsNaN(v2) || float.IsInfinity(v2)) r.Bad = true;
            foreach (var l in links) { var o2 = l.Body.WorldTransform.Origin; if (float.IsNaN(o2.x + o2.y + o2.z) || Math.Abs(o2.x) > 1e6f) r.Bad = true; }
            return r;
        }

        // A: 旧エンジン相当 = 完全セットv1 の 9 フラグを全部 OFF (env より優先して落とす)
        var oldR = Once("旧エンジン", () =>
        {
            Joint.BulletAngleConvention = false;
            Joint.AngularMixedAxes = false;
            Joint.LinearLeverMode = 0;
            Joint.BulletLimitRowGating = false;
            PersistentManifold.SymmetricBreakingDistance = false;
            PersistentManifold.BulletManifoldPoints = false;
            GjkEpa.BulletContactThreshold = false;
            PhysicsWorld.BulletRotationIntegration = false;
            _forceOldContactRhs = true;   // world 生成後に world.ContactRhsBullet=false を当てる
        });
        _forceOldContactRhs = false;
        // B: 出荷既定 (完全セットv1)。env 上書きは Once の中で効いている。
        var newR = Once("出荷既定", null);
        Joint.SpringAsMotorRow = ShippedSpringMotor;

        L("  対象ボーン = " + oldR.Bones.Count + " 本  (剛体フィルタ '" + filter + "')");
        {
            // 実効フラグのエコー (env ではなくエンジンから読み戻した値)。接触側も出す。
            var w2 = PmxPhysicsBuilder.Build(model).World;
            float v;
            if (float.TryParse(Env("SLOP"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f) w2.PenetrationSlop = v;
            if (Env("JOINTS_FIRST") == "1") w2.SolveJointsFirst = true;
            if (Env("CRHS") != null) w2.ContactRhsBullet = Env("CRHS") == "1";
            if (float.TryParse(Env("BAUM"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f) w2.BaumgarteFactor = v;
            if (float.TryParse(Env("CWFAC"), NumberStyles.Float, CultureInfo.InvariantCulture, out v) && v >= 0f) w2.ContactWarmStartFactor = v;
            if (Env("CPOOL") == "1") w2.ContactPoolOrder = true;
            if (Env("NORMFIRST") == "1") w2.ContactNormalBeforeFriction = true;
            if (Env("FRICALIGN") == "1") w2.FrictionVelocityAligned = true;
            if (Env("FRICMUL") == "1") w2.FrictionCombineMultiply = true;
            if (Env("CSET") == "1") { w2.ContactPoolOrder = true; w2.FrictionVelocityAligned = true; w2.FrictionCombineMultiply = true; }
            void FlagEcho(Res r) => L("  [実効] " + r.Label.PadRight(10)
                + " SpringMotor=" + r.SpringMotorUsed + "  AngConv=" + r.AngConvUsed
                + "  MixedAxes=" + r.MixedAxesUsed + "  Lever=" + r.LeverUsed
                + "  RotExp=" + r.RotExpUsed + "  CThresh=" + r.CThreshUsed);
            FlagEcho(oldR); FlagEcho(newR);
            L("  [実効] 接触側2: PoolOrder=" + w2.ContactPoolOrder + "  NormalFirst=" + w2.ContactNormalBeforeFriction +
              "  FricAligned=" + w2.FrictionVelocityAligned + "  FricMul=" + w2.FrictionCombineMultiply);
            L("  [実効] 接触側 (両条件に共通): CRhsBullet=" + w2.ContactRhsBullet + "  CMan=" + PersistentManifold.BulletManifoldPoints + "  SymDist=" + PersistentManifold.SymmetricBreakingDistance + "  Slop=" + w2.PenetrationSlop.ToString("G6") +
              "  JointsFirst=" + w2.SolveJointsFirst +
              "  ContactBaumgarte=" + w2.BaumgarteFactor.ToString("G6") +
              "  ContactWarm=" + w2.ContactWarmStartFactor.ToString("G6") +
              "  RestitutionThreshold=" + w2.RestitutionThreshold.ToString("G6") +
              "  NoContact=" + (Env("NOCONTACT") == "1"));
        }
        L();
        L(string.Format("  {0,-12} {1,16} {2,16}", "条件", "|Δp| 中央", "|Δp| p90"));
        L(string.Format("  {0,-12} {1,16:G5} {2,16:G5}", oldR.Label, oldR.Med, oldR.P90));
        L(string.Format("  {0,-12} {1,16:G5} {2,16:G5}", newR.Label, newR.Med, newR.P90));
        // ★タスク44: 静止判定は3つ組 (収束の有無 / 収束時間τ / 収束後フロアの参照比)。
        //   到達値 (上の |Δp| 中央) だけで採否を決めてはいけない。
        {
            float refFloor = EnvF("REFFLOOR", -1f);
            L();
            L(string.Format("  ★静止判定の3つ組 (窓 {0} 秒 / 全 {1} 秒 / 収束帯 = 参照フロア×3){2}",
                EnvI("WINSEC", 10), frames / 60, refFloor > 0f ? "" : "   ※REFFLOOR 未指定のため収束判定は出さない"));
            L(string.Format("  {0,-12} {1,10} {2,10} {3,14} {4,10}", "条件", "収束", "τ[秒]", "フロア", "参照比"));
            void Triple(Res r) => L(string.Format("  {0,-12} {1,10} {2,10} {3,14:G5} {4,10}",
                r.Label,
                refFloor > 0f ? (r.Converged ? "した" : "★せず") : "-",
                float.IsNaN(r.TauSec) ? "-" : r.TauSec.ToString("F0"),
                r.Floor,
                refFloor > 0f ? (r.Floor / refFloor).ToString("F2") + "x" : "-"));
            Triple(oldR); Triple(newR);
            L("  窓ごとの推移 (|Δp| 中央):");
            void Win(Res r) => L("    " + r.Label.PadRight(12) + string.Join(" ", r.WinMed.ConvertAll(x => x.ToString("G4").PadLeft(9))));
            Win(oldR); Win(newR);
            L();
        }
        L();
        L("  [ばねクランプ] DisableSpringClamp=" + (Env("SPRINGCLAMP") == "off"));
        void SRow(Res r) => L(string.Format("    {0,-12} ばね行 {1,10} / クランプ発動 {2,10} ({3,6:F2}%)  力積合計 {4,12:G5} / 削り {5,12:G5} ({6,6:F2}%)  NaN/発散 {7}",
            r.Label, r.SpringRows, r.SpringClamped,
            r.SpringRows > 0 ? 100.0 * r.SpringClamped / r.SpringRows : 0.0,
            r.SpringAbs, r.SpringShaved,
            r.SpringAbs > 0 ? 100.0 * r.SpringShaved / r.SpringAbs : 0.0,
            r.Bad ? "★あり" : "なし"));
        SRow(oldR); SRow(newR);
        L();
        L(string.Format("  {0,-12} {1,16:F3} {2,16:F3}", "比(新/既定)",
                        oldR.Med > 1e-12f ? newR.Med / oldR.Med : float.NaN,
                        oldR.P90 > 1e-12f ? newR.P90 / oldR.P90 : float.NaN));
        L();
        L("  ★PMXエディタのベイクを analyze_static_bake.py に掛けて出る「ボーン別中央の中央値」と");
        L("    この2つを並べれば、既定と3点構成のどちらが本家に近いかが決まる。");
        L("    参考: 参照データ(IA髪)の静区間フロア = 0.0028 〜 0.0128");
        L("=".PadRight(104, '='));

        string csv = Env("CSV");
        if (!string.IsNullOrEmpty(csv))
        {
            var sb = new StringBuilder();
            sb.Append("bone,med_default,p90_default,med_3flag,p90_3flag\n");
            for (int i = 0; i < oldR.Bones.Count; i++)
                sb.Append(oldR.Bones[i].Replace(',', '_')).Append(',')
                  .Append(oldR.PerBoneMed[i].ToString("G9", CultureInfo.InvariantCulture)).Append(',')
                  .Append(oldR.PerBoneP90[i].ToString("G9", CultureInfo.InvariantCulture)).Append(',')
                  .Append(newR.PerBoneMed[i].ToString("G9", CultureInfo.InvariantCulture)).Append(',')
                  .Append(newR.PerBoneP90[i].ToString("G9", CultureInfo.InvariantCulture)).Append('\n');
            File.WriteAllText(csv, sb.ToString(), new UTF8Encoding(false));
            L("  ボーン別を書き出した: " + csv);
        }
        File.WriteAllText(Env("OUT") ?? "bonedp.txt", O.ToString(), new UTF8Encoding(false));
        return 0;
    }
}
