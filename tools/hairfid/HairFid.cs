// ===========================================================================
// (2) 髪のフレーム単位 MMD突合 (スカートと同じ土俵)。
//   MMDVMD由来の hair CSV(108ボーン)で BoneFollow(体)を駆動し、髪(dynamic)を自前物理で動かす。
//   髪ボーン姿勢(ボーン空間 = body.WorldTransform * BodyOffsetFromBone.Inverse())をMMDと突合。
//   位置(ワールド)と角度差(閾値3e-3rad, acos不使用)を 房別/段別/静区間・ターン窓で集計。
//   髪×体の貫入も同時計測(元症状の第一候補)。
// warm-start(0.85)は既定ON。A/Bは env WARM_OFF=1。
// ===========================================================================
using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class HairFid
{
    const float DT = 1f / 30f;
    static bool IsHair(string n) => n != null && (n.Contains("髪") || n.Contains("ツインテ") || n.Contains("もみあげ") || n.Contains("前髪") || n.Contains("モミアゲ"));
    static bool IsSkirt(string n) => n != null && n.Contains("スカート");
    static readonly string[] BodyColliderBones = { "頭", "頭2", "首", "上半身", "上半身2", "下半身", "下半身1", "下半身3", "下半身4", "右足", "左足", "右ひざ", "左ひざ", "右太もも", "左太もも" };

    // 相対回転角(rad)。dq=b*conj(a); angle=2*atan2(|xyz|,|w|)。acos不使用。
    static float RelAngle(Quat a, Quat b)
    {
        float cx = -a.x, cy = -a.y, cz = -a.z, cw = a.w; // conj(a)
        float dx = b.w * cx + b.x * cw + b.y * cz - b.z * cy;
        float dy = b.w * cy - b.x * cz + b.y * cw + b.z * cx;
        float dz = b.w * cz + b.x * cy - b.y * cx + b.z * cw;
        float dw = b.w * cw - b.x * cx - b.y * cy - b.z * cz;
        float s = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return 2f * (float)Math.Atan2(s, Math.Abs(dw));
    }
    static float Deg(float rad) => rad * 57.29578f;

    static (float med, float p90, float max) Stat(List<float> v)
    {
        if (v.Count == 0) return (0, 0, 0);
        v.Sort(); return (v[v.Count / 2], v[(int)(v.Count * 0.9)], v[v.Count - 1]);
    }
    // 房(strand)と段(segment番号)を名前から: 例 髪FR5 -> strand=髪FR, seg=5。末尾数字を分離。
    static (string strand, int seg) Parse(string n)
    {
        int i = n.Length; while (i > 0 && char.IsDigit(n[i - 1])) i--;
        int seg = i < n.Length ? int.Parse(n.Substring(i)) : 0;
        return (n.Substring(0, i), seg);
    }

    static int Main()
    {
        string pmx = TestData.PmxPath();
        string csvp = TestData.Resolve("MMD_TEST_HAIRCSV", "modelA_bone_world_pose_hair.csv");
        if (pmx == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP] no pmx"); return 0; }
        if (csvp == null || !File.Exists(csvp)) { Console.WriteLine("[SKIP] no hair csv (MMD_TEST_HAIRCSV か testdata/ に置く)"); return 0; }
        long bytes = new FileInfo(csvp).Length;
        if (bytes != 65805999L && bytes != 65617640L) { Console.WriteLine($"[FAIL] hair CSV バイト数不一致 {bytes} (期待65805999=ON or 65617640=OFF)。取り違え防止のため中止。"); return 1; }

        // ★慣性は剛体構築時に確定するので Build より前に設定する (Bullet 2.75 は 0.04, 既定0=従来)。
        if (float.TryParse(Environment.GetEnvironmentVariable("INERTIAMARGIN"), out var _im) && _im >= 0f)
            CapsuleShape.InertiaMargin = _im;

        var model = PmxReader.LoadFile(pmx);
        var csv = BoneCsv.Load(csvp);
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        if (Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
        if (Environment.GetEnvironmentVariable("SPLIT") == "1") world.UseSplitImpulse = true; // 接触の貫入回復を擬似速度側へ(綱引き回避の検証)
        // ★2026-08-20 追加: ジョイント側 split impulse。静止の髪振動に最も効いた候補の駆動系検証用。
        if (Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (Environment.GetEnvironmentVariable("JOINTS_FIRST") == "1") world.SolveJointsFirst = true; // Bullet同順(ジョイント→接触,接触が後勝ち)
        if (float.TryParse(Environment.GetEnvironmentVariable("CWFAC"), out var _cwf)) world.ContactWarmStartFactor = _cwf; // 接触warm-start係数(Bullet=0.85)
        // ジョイントwarm-start引継ぎ係数。既定OFF(UseJointWarmStart=false)なので、旧既定をA/Bで
        // 再現するときだけ WARMSTART/WARMSTART_ANG と併せて使う。bonecheck/restsim/chainbug にも同名あり。
        if (float.TryParse(Environment.GetEnvironmentVariable("WARMFAC"), out var _wf)) Joint.WarmStartFactor = _wf;
        if (int.TryParse(Environment.GetEnvironmentVariable("LEVER"), out var _lv)) Joint.LinearLeverMode = _lv; // 線形レバーアーム 0/1/2
        // ★2026-08-21 追加: ジョイント位置補正係数の掃引 (既定0.2)。未設定=無変更。
        if (float.TryParse(Environment.GetEnvironmentVariable("JBETA"),
                System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var _jb) && _jb >= 0f)
            foreach (var j in world.Joints) j.Beta = _jb;
        if (float.TryParse(Environment.GetEnvironmentVariable("MAXCORR"), out var _mc)) Joint.MaxCorrectionVel = _mc; // 位置補正速度上限(既定10, Bulletは無制限)
        if (Environment.GetEnvironmentVariable("MIXAXES") == "1") Joint.AngularMixedAxes = true; // 角度リミット行=Bullet混合軸
        if (Environment.GetEnvironmentVariable("CNBF") == "1") world.ContactNormalBeforeFriction = true; // 法線→摩擦順(Bullet)
        // 補正層再現: 1=出力のみ(計測をaligned姿勢で行い剛体は復元) 2=フィードバック(aligned姿勢を剛体へ書き戻し=次stepへ影響)
        int alignMode = int.TryParse(Environment.GetEnvironmentVariable("ALIGN"), out var _al) ? _al : 0;
        float alpha = float.TryParse(Environment.GetEnvironmentVariable("ALPHA"), out var _aa) ? _aa : 0f; // 回転clamp割合(0=無効)
        bool order2 = Environment.GetEnvironmentVariable("ORDER2") == "1"; // 順序比較: 位置=物理回転で再構成/回転のみclamp
        if (alignMode > 0) Console.WriteLine($"[cfg] ALIGN={alignMode} ALPHA={alpha} ORDER2={order2} ({(alignMode == 1 ? "出力のみ" : "フィードバック")}: 位置=親チェーン再構成/回転={(alpha > 0 ? $"リミットへ{alpha:P0}clamp" : "物理")})");
        if (int.TryParse(Environment.GetEnvironmentVariable("SUBSTEPS"), out var _ss) && _ss > 0) world.SubSteps = _ss; // 計算予算掃引
        if (int.TryParse(Environment.GetEnvironmentVariable("ITERS"), out var _it) && _it > 0) world.SolverIterations = _it;
        // ★2026-08-20 追加: 接触の位置補正係数を A/B するための env。未設定=無変更(既定のまま)。
        //   既定値そのものを 0.2 -> 0 にした変更 (静止では全31モデル改善) の、駆動系での裏取り用。
        //   BAUM=0.2 を渡すと変更前の挙動を再現できる。
        if (float.TryParse(Environment.GetEnvironmentVariable("BAUM"),
                System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var _bg) && _bg >= 0f)
        {
            world.BaumgarteFactor = _bg;
            Console.WriteLine($"[cfg] BAUM={_bg} (接触Baumgarte。既定は 0)");
        }
        // 鎖のたわみ対策 (UE5移植版からの逆輸入)。既定 0 = 従来とビット不変。
        if (int.TryParse(Environment.GetEnvironmentVariable("JOINTITERS"), out var _ji) && _ji > 0) world.JointVelocityIterations = _ji;
        if (float.TryParse(Environment.GetEnvironmentVariable("JOINTMAXCORR"), out var _jm) && _jm > 0f) world.JointMaxCorrectionVel = _jm;
        // 決定的テスト: スカートジョイントを自由化して「接触だけ」で貫入が解消するか見る(綱引き vs 接触能力の切り分け)。
        // 1=角度リミット自由(付いたまま回転自由) / 2=角度+並進自由(実質ジョイント無効=接触+重力のみ)。エンジン無改変。
        int skirtJFree = int.TryParse(Environment.GetEnvironmentVariable("SKIRT_JFREE"), out var sjf) ? sjf : 0;
        if (skirtJFree > 0)
        {
            int cnt = 0;
            foreach (var j in world.Joints)
            {
                if (!(IsSkirt(j.BodyA?.Name) || IsSkirt(j.BodyB?.Name))) continue;
                j.AngularLowerLimit = new Vec3(1, 1, 1); j.AngularUpperLimit = new Vec3(-1, -1, -1);
                if (skirtJFree >= 2) { j.LinearLowerLimit = new Vec3(1, 1, 1); j.LinearUpperLimit = new Vec3(-1, -1, -1); }
                cnt++;
            }
            Console.WriteLine($"[cfg] SKIRT_JFREE={skirtJFree} スカート絡みジョイント{cnt}本を自由化(1=角度,2=角度+並進)");
        }
        Console.WriteLine($"[cfg] warm={world.UseJointWarmStart}/{world.UseJointWarmStartAngular} fac={Joint.WarmStartFactor} split={world.UseSplitImpulse} frames={csv.FrameCount}");

        // 髪 dynamic 剛体リンクと、体コライダー(BoneFollow)リンク。
        var hairLinks = new List<(BoneLink link, string bone, RigidTransform bindBone)>();
        var bodyLinks = new List<(BoneLink link, string bone)>();
        var driven = new List<(BoneLink link, string bone)>();
        for (int i = 0; i < builder.BoneLinks.Count; i++)
        {
            var link = builder.BoneLinks[i];
            string bone = (link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count) ? model.BoneNames[link.BoneIndex] : null;
            if (link.Mode == PhysicsMode.BoneFollow)
            {
                if ((BodyColliderBones.Contains(link.Body.Name) || (bone != null && BodyColliderBones.Contains(bone))) && bone != null && csv.HasBone(bone)) bodyLinks.Add((link, bone));
                if (bone != null && csv.HasBone(bone)) driven.Add((link, bone));
            }
            else if ((IsHair(link.Body.Name) || IsSkirt(link.Body.Name)) && bone != null && csv.HasBone(bone))
            {
                var bindBone = new RigidTransform(Quat.Identity, model.BonePositions[link.BoneIndex]);
                hairLinks.Add((link, bone, bindBone));
            }
        }
        int nHair = hairLinks.Count(x => IsHair(x.link.Body.Name));
        Console.WriteLine($"[構成] 比較剛体={hairLinks.Count}(髪{nHair}+スカート{hairLinks.Count - nHair}) 体コライダー={bodyLinks.Count} 駆動BoneFollow={driven.Count}");

        // ===== 補正層の相関分析モード (CORR=1): OFF(=MMD_TEST_HAIRCSV)→ON 差分の説明変数相関 =====
        if (Environment.GetEnvironmentVariable("CORR") == "1")
        {
            string onCsvPath = TestData.Resolve("CORR_ON_CSV", "modelA_bone_world_pose_hair.csv");
            if (onCsvPath == null || new FileInfo(onCsvPath).Length != 65805999L) { Console.WriteLine($"[FAIL] ON CSV バイト数不一致: {onCsvPath}"); return 1; }
            if (bytes != 65617640L) { Console.WriteLine("[FAIL] CORR モードは MMD_TEST_HAIRCSV に OFF版(65617640B) を指定すること。"); return 1; }
            var onCsv = BoneCsv.Load(onCsvPath);
            Console.WriteLine($"[CORR] OFF(補正前)={Path.GetFileName(csvp)}  ON(補正後)={Path.GetFileName(onCsvPath)}");
            return OffOnCorr.Run(model, builder, hairLinks, bodyLinks, driven, csv, onCsv);
        }

        // ===== スカートジョイントの種別分類 (取付=体↔スカート / 縦=リング違い / 横=同リング) =====
        int RingOf(string n)
        {
            if (n == null || !n.StartsWith("スカート_")) return -1;
            var p = n.Split('_');
            return p.Length >= 3 && int.TryParse(p[1], out var r) ? r : -1;
        }
        var jClass = new List<(Joint j, int t)>(); // t: 0=取付 1=縦 2=横
        foreach (var j in world.Joints)
        {
            bool aS = IsSkirt(j.BodyA?.Name), bS = IsSkirt(j.BodyB?.Name);
            if (!aS && !bS) continue;
            int t;
            if (aS != bS) t = 0;
            else { int ra = RingOf(j.BodyA.Name), rb2 = RingOf(j.BodyB.Name); t = (ra >= 0 && rb2 >= 0 && ra != rb2) ? 1 : 2; }
            jClass.Add((j, t));
        }
        string[] tName = { "取付", "縦", "横" };
        Console.WriteLine($"[スカートJoint分類] 取付={jClass.Count(x => x.t == 0)} 縦={jClass.Count(x => x.t == 1)} 横={jClass.Count(x => x.t == 2)}");
        if (Environment.GetEnvironmentVariable("DUMPJAB") == "1")
            foreach (var (j, t) in jClass) if (t == 0) Console.WriteLine($"   取付: A={j.BodyA?.Name} B={j.BodyB?.Name}");
        // 種別別自由化: env SKIRT_JTYPE=attach|vert|horiz でその種別のみ 並進+角度 自由(=外す)。
        string jt = Environment.GetEnvironmentVariable("SKIRT_JTYPE");
        if (jt != null)
        {
            int tf = jt == "attach" ? 0 : jt == "vert" ? 1 : jt == "horiz" ? 2 : -1;
            if (tf >= 0)
            {
                int cnt = 0;
                foreach (var (j, t) in jClass)
                    if (t == tf)
                    {
                        j.LinearLowerLimit = new Vec3(1, 1, 1); j.LinearUpperLimit = new Vec3(-1, -1, -1);
                        j.AngularLowerLimit = new Vec3(1, 1, 1); j.AngularUpperLimit = new Vec3(-1, -1, -1);
                        cnt++;
                    }
                Console.WriteLine($"[cfg] SKIRT_JTYPE={jt}: {tName[tf]}ジョイント{cnt}本を完全自由化");
            }
        }
        // ジョイントのアンカー誤差 |worldA-worldB| 収集 (自前sim / MMDベイクCSV幾何)
        var jErrOurs = new List<float>[3] { new(), new(), new() };
        var jErrRef = new List<float>[3] { new(), new(), new() };
        // 角度リミット実効挙動: 種別×サンプル(joint×DOF×frame)で「リミット外に居る率」と「超過量」。
        // 同じ物差し(XYZ euler)で 自前sim と MMDベイクCSV幾何 を比較=リミットが実効的にどれだけ破られているか。
        var angTotal = new long[3]; var angOut = new long[3];
        var angOverOurs = new List<float>[3] { new(), new(), new() };
        var angTotalR = new long[3]; var angOutR = new long[3];
        var angOverRef = new List<float>[3] { new(), new(), new() };
        Vec3 EulerXYZ(Quat q) // Bullet matrixToEulerXYZ と同型 (行列経由)
        {
            float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
            float m02 = 2f * (xz + wy);
            if (m02 < 1f - 1e-6f && m02 > -1f + 1e-6f)
                return new Vec3((float)Math.Atan2(-2f * (yz - wx), 1f - 2f * (xx + yy)),
                                (float)Math.Asin(m02),
                                (float)Math.Atan2(-2f * (xy - wz), 1f - 2f * (yy + zz)));
            return new Vec3((float)Math.Atan2(2f * (yz + wx), 1f - 2f * (xx + zz)), m02 >= 1f ? 1.5707963f : -1.5707963f, 0f);
        }
        void CollectAng(bool isRef)
        {
            foreach (var (j2, t2) in jClass)
            {
                var qRel = (j2.BodyA.WorldTransform * j2.FrameInA).Rotation.Conjugated() * (j2.BodyB.WorldTransform * j2.FrameInB).Rotation;
                var eu = Joint.ToEulerXYZ(qRel.Normalized); // エンジンと同一分解 (規約差排除)
                for (int d = 0; d < 3; d++)
                {
                    float lo = j2.AngularLowerLimit[d], hi = j2.AngularUpperLimit[d];
                    if (lo > hi) continue; // free
                    float over = eu[d] < lo ? lo - eu[d] : eu[d] > hi ? eu[d] - hi : 0f;
                    if (isRef) { angTotalR[t2]++; if (over > 1e-4f) { angOutR[t2]++; angOverRef[t2].Add(over * 57.29578f); } }
                    else
                    {
                        angTotal[t2]++; if (over > 1e-4f) { angOut[t2]++; angOverOurs[t2].Add(over * 57.29578f); }

                    }
                }
            }
        }
        float JErr(Joint j)
        {
            var aA = (j.BodyA.WorldTransform * j.FrameInA).Origin;
            var aB = (j.BodyB.WorldTransform * j.FrameInB).Origin;
            return (aA - aB).Length;
        }

        // --- バインド相対0 確認: 各髪剛体を bind に置き、ボーン姿勢復元 vs PMX bind bone ---
        float bindPosMax = 0, bindAngMax = 0;
        foreach (var (link, bone, bindBone) in hairLinks)
        {
            var bw = link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse(); // 復元ボーン姿勢
            bindPosMax = Math.Max(bindPosMax, (bw.Origin - bindBone.Origin).Length);
            bindAngMax = Math.Max(bindAngMax, RelAngle(bw.Rotation, bindBone.Rotation));
        }
        Console.WriteLine($"[バインド相対0] 復元ボーン姿勢 vs PMX bind: 位置max={bindPosMax:F5} 角度max={Deg(bindAngMax):F4}° 期待0");
        if (bindPosMax > 1e-3f || bindAngMax > 3e-3f) { Console.WriteLine("[FAIL] バインド相対が0でない。測定の土俵がずれているため中止。"); return 1; }

        // --- FK-rest 初期化 + warmup ---
        // ★2026-08-09バグ修正+統一: 駆動式は共通ヘルパ ApplyKinematicTargets に集約 (手書き駆動式の再発防止)。
        // (旧: 手書きで `= bw` と offset を掛け忘れ、体コライダー誤配置のまま全simが汚染された)
        void ApplyPose(int f) => builder.ApplyKinematicTargets(bi =>
            (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(f, model.BoneNames[bi], out var bw)) ? (RigidTransform?)bw : null);
        ApplyPose(0);
        builder.ResetBodiesToBonePoseFk(i => (i >= 0 && i < model.BoneNames.Count && csv.TryGet(0, model.BoneNames[i], out var bw)) ? (RigidTransform?)bw : null);
        for (int s = 0; s < 60; s++) world.StepSimulation(DT);

        int F = csv.FrameCount;
        var buf = new List<ContactPoint>();
        // per-frame メトリクス収集
        var posDiff = new Dictionary<string, List<float>>();  // 髪ボーン -> 位置差列
        var angDiff = new Dictionary<string, List<float>>();  // -> 角度差列(deg)
        var oursDev = new Dictionary<string, List<float>>();   // 自前 bind からの角度偏差(deg)
        var refDev = new Dictionary<string, List<float>>();    // MMD bind からの角度偏差(deg)
        foreach (var (l, b, _) in hairLinks) { posDiff[b] = new(); angDiff[b] = new(); oursDev[b] = new(); refDev[b] = new(); }
        // ターン窓判定用: 下半身ヨー角速度
        var yawRate = new float[F];
        // ★符号付き位置差(自前-MMD) per-frame 累積 (髪/スカート別)。鏡像=Z符号が系統的にずれる。
        var hSx = new float[F]; var hSy = new float[F]; var hSz = new float[F]; var hCn = new int[F];
        var sSx = new float[F]; var sSy = new float[F]; var sSz = new float[F]; var sCn = new int[F];
        // 貫入: pair -> max, deep>0.5 のフレーム記録(診断込み)
        var penMax = new Dictionary<string, float>();
        var deepFrames = new List<(int f, string pair, float d, float ni, bool aabb, int mpts)>();
        var deepByPair = new Dictionary<string, SortedSet<int>>(); // 継続フレーム算出用
        var penSeries = new Dictionary<string, Dictionary<int, float>>(); // pair -> frame -> pen (>0.3のみ), per-step回復速度用
        // 移動ロック相対保持: skirt子ボーンの(子-親)相対位置の bind相対からのズレ (ON0.371/OFF0.706 に対する自前値)
        var skirtChildren = hairLinks.Where(x => IsSkirt(x.link.Body.Name) && x.link.BoneIndex >= 0).Select(x => x.link.BoneIndex).ToList();
        var relDriftVals = new List<float>();     // 自前(sim)
        var relDriftRef = new List<float>();      // MMD(CSV: ON or OFF, 同subset/同定義)
        // 伸び(回転不変): | |子-親| - |bind子-親| |。回転では変わらないので、相対保持のうち「鎖の伸び」成分だけを分離する。
        var stretchVals = new List<float>(); var stretchRef = new List<float>();
        // 方向変化角: 現(子-親) と bind(子-親) のなす角。振り子なら枠の傾きと同程度、横滑りなら枠より大きい。
        var dirVals = new List<float>(); var dirRef = new List<float>();
        var dbg = new List<(string a, string b, float dist, float ni)>();
        world.DebugContacts = dbg;

        Quat prevLow = Quat.Identity; bool havePrev = false;
        for (int f = 0; f < F; f++)
        {
            ApplyPose(f);
            dbg.Clear();
            world.StepSimulation(DT);
            // 補正層再現 (ALIGN>0): aligned姿勢へ物理剛体を配置。mode1は計測後に復元(出力のみ)、mode2は恒久(フィードバック)。
            RigidTransform[] savedPose = null;
            if (alignMode > 0)
            {
                RigidTransform? Drv(int bi) => (bi >= 0 && bi < model.BoneNames.Count && csv.TryGet(f, model.BoneNames[bi], out var dw)) ? (RigidTransform?)dw : null;
                var aligned = builder.ComputeAlignedBonePoses(Drv, order2 ? 0f : alpha, true);
                if (order2 && alpha > 0f)
                {
                    // 順序比較: 位置は α=0 (物理回転の親チェーン) のまま、回転だけ α clamp 版から合成。
                    var rotOnly = builder.ComputeAlignedBonePoses(Drv, alpha, true);
                    for (int bi = 0; bi < aligned.Length; bi++)
                        if (aligned[bi].HasValue && rotOnly[bi].HasValue)
                            aligned[bi] = new RigidTransform(rotOnly[bi].Value.Rotation, aligned[bi].Value.Origin);
                }
                if (alignMode == 1) { savedPose = new RigidTransform[hairLinks.Count]; for (int k = 0; k < hairLinks.Count; k++) savedPose[k] = hairLinks[k].link.Body.WorldTransform; }
                foreach (var (link, _, _) in hairLinks)
                    if (aligned[link.BoneIndex].HasValue)
                    { link.Body.WorldTransform = aligned[link.BoneIndex].Value * link.BodyOffsetFromBone; link.Body.UpdateInertiaWorld(); }
            }
            foreach (var (j2, t2) in jClass) jErrOurs[t2].Add(JErr(j2)); // ジョイント種別別アンカー誤差(自前)
            CollectAng(false); // 角度リミット実効挙動(自前)
            // 下半身ヨー速度
            if (csv.TryGet(f, "下半身", out var low))
            {
                if (havePrev) yawRate[f] = Deg(RelAngle(prevLow, low.Rotation)) / DT;
                prevLow = low.Rotation; havePrev = true;
            }
            // 髪ボーン突合
            foreach (var (link, bone, bindBone) in hairLinks)
            {
                var ours = link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
                if (!csv.TryGet(f, bone, out var refb)) continue;
                posDiff[bone].Add((ours.Origin - refb.Origin).Length);
                angDiff[bone].Add(Deg(RelAngle(ours.Rotation, refb.Rotation)));
                var dv = ours.Origin - refb.Origin; // 符号付き(自前-MMD)
                if (IsHair(bone)) { hSx[f] += dv.x; hSy[f] += dv.y; hSz[f] += dv.z; hCn[f]++; }
                else { sSx[f] += dv.x; sSy[f] += dv.y; sSz[f] += dv.z; sCn[f]++; }
                oursDev[bone].Add(Deg(RelAngle(bindBone.Rotation, ours.Rotation)));
                refDev[bone].Add(Deg(RelAngle(bindBone.Rotation, refb.Rotation)));
            }
            // 移動ロック相対保持: この frame の (子-親) 相対位置ズレ
            {
                var wpos = new Dictionary<int, Vec3>();
                foreach (var (link, bn, _) in hairLinks) wpos[link.BoneIndex] = (link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse()).Origin;
                foreach (var (link, bn) in driven) if (csv.TryGet(f, bn, out var bw)) wpos[link.BoneIndex] = bw.Origin;
                foreach (int ci in skirtChildren)
                {
                    int pi = (ci < model.BoneParents.Count) ? model.BoneParents[ci] : -1;
                    if (pi < 0) continue;
                    var bindRel = model.BonePositions[ci] - model.BonePositions[pi];
                    // 自前(sim): wpos に子・親があれば
                    float DirDeg(Vec3 r0)
                    {
                        float den = r0.Length * bindRel.Length;
                        if (den < 1e-9f) return 0;
                        double c0 = Math.Max(-1.0, Math.Min(1.0, r0.Dot(bindRel) / den));
                        return (float)(Math.Acos(c0) * 180.0 / Math.PI);
                    }
                    if (wpos.ContainsKey(ci) && wpos.ContainsKey(pi))
                    {
                        var r = wpos[ci] - wpos[pi];
                        relDriftVals.Add((r - bindRel).Length);
                        stretchVals.Add(Math.Abs(r.Length - bindRel.Length));
                        dirVals.Add(DirDeg(r));
                    }
                    // MMD(CSV): 同subset/同親定義で CSV 位置から
                    if (csv.TryGet(f, model.BoneNames[ci], out var cc) && csv.TryGet(f, model.BoneNames[pi], out var cp))
                    {
                        var r = cc.Origin - cp.Origin;
                        relDriftRef.Add((r - bindRel).Length);
                        stretchRef.Add(Math.Abs(r.Length - bindRel.Length));
                        dirRef.Add(DirDeg(r));
                    }
                }
            }
            // 髪×体 貫入 (自前物理の結果) + 診断
            foreach (var (hl, hb, _) in hairLinks)
                foreach (var (bl, bbn) in bodyLinks)
                {
                    if (!PhysicsWorld.ShouldCollide(hl.Body, bl.Body)) continue;
                    buf.Clear(); GjkEpa.Detect(hl.Body, bl.Body, buf);
                    float pen = 0; foreach (var cp in buf) pen = Math.Max(pen, -cp.Distance);
                    if (pen <= 0) continue;
                    string pair = hb + "×" + bl.Body.Name;
                    penMax[pair] = Math.Max(penMax.GetValueOrDefault(pair), pen);
                    if (pen > 0.3f) { if (!penSeries.TryGetValue(pair, out var ps)) { ps = new(); penSeries[pair] = ps; } ps[f] = pen; }
                    if (pen > 0.5f)
                    {
                        // 診断: 法線インパルス合計, AABB, ★永続マニフォールド点数(=このペアのDebugContacts件数)
                        float ni = 0; int mfp = 0;
                        foreach (var c in dbg) if ((c.a == hl.Body.Name && c.b == bl.Body.Name) || (c.a == bl.Body.Name && c.b == hl.Body.Name)) { ni += c.ni; mfp++; }
                        var aa = hl.Body.ComputeAabb(); var bb = bl.Body.ComputeAabb(); bool aabb = aa.Intersects(ref bb);
                        deepFrames.Add((f, pair, pen, ni, aabb, mfp));
                        if (!deepByPair.TryGetValue(pair, out var st)) { st = new(); deepByPair[pair] = st; }
                        st.Add(f);
                    }
                }
            // ALIGN=1 (出力のみ): 計測が終わったので物理剛体の姿勢を復元 (次stepへは影響させない)。
            if (savedPose != null)
                for (int k = 0; k < hairLinks.Count; k++)
                { hairLinks[k].link.Body.WorldTransform = savedPose[k]; hairLinks[k].link.Body.UpdateInertiaWorld(); }
        }

        // ===== MMDの貫入 (幾何): 髪も体もMMDベイクCSV姿勢に置いて Detect。仕様かバグかの切り分け =====
        var refPenMax = new Dictionary<string, float>(); int refDeep = 0; float refDeepMax = 0;
        for (int f = 0; f < F; f++)
        {
            foreach (var (bl, bbn) in bodyLinks) if (csv.TryGet(f, bbn, out var bw)) { bl.Body.WorldTransform = bw * bl.BodyOffsetFromBone; bl.Body.UpdateInertiaWorld(); }
            foreach (var (hl, hbn, _) in hairLinks) if (csv.TryGet(f, hbn, out var bw)) { hl.Body.WorldTransform = bw * hl.BodyOffsetFromBone; hl.Body.UpdateInertiaWorld(); }
            foreach (var (j2, t2) in jClass) jErrRef[t2].Add(JErr(j2)); // ジョイント種別別アンカー誤差(MMDベイクCSV幾何)
            CollectAng(true); // 角度リミット実効挙動(MMD幾何)
            foreach (var (hl, hb, _) in hairLinks)
                foreach (var (bl, bbn) in bodyLinks)
                {
                    if (!PhysicsWorld.ShouldCollide(hl.Body, bl.Body)) continue;
                    buf.Clear(); GjkEpa.Detect(hl.Body, bl.Body, buf);
                    float pen = 0; foreach (var cp in buf) pen = Math.Max(pen, -cp.Distance);
                    if (pen <= 0) continue;
                    string pair = hb + "×" + bl.Body.Name;
                    refPenMax[pair] = Math.Max(refPenMax.GetValueOrDefault(pair), pen);
                    if (pen > 0.5f) { refDeep++; refDeepMax = Math.Max(refDeepMax, pen); }
                }
        }

        // ===== per-step 貫入回復速度 (静区間・連続deep・同ペアでの pen 変化) =====
        // 「押し返せない」か「押し返しが遅い」かの切り分け。理論: Baumgarte 0.2 で 1step あたり pen×0.2 減るはず。
        var stepDeltas = new List<float>(); var pen0s = new List<float>();
        foreach (var kv in penSeries)
        {
            var s = kv.Value;
            foreach (var f in s.Keys)
            {
                if (f + 1 >= F || !s.ContainsKey(f + 1)) continue;
                float p0 = s[f], p1 = s[f + 1];
                if (p0 <= 0.5f) continue;                              // deep状態からの1ステップ
                if (yawRate[f] >= 15f || yawRate[f + 1] >= 15f) continue; // 静区間のみ(脚がほぼ動かない=接触回復を純粋に見る)
                stepDeltas.Add(p1 - p0); pen0s.Add(p0);
            }
        }

        // ===== 集計 =====
        var O = new StringBuilder();
        // ジョイント種別別 アンカー誤差 (格子のどこで滑っているか)
        O.AppendLine("[ジョイント種別別 アンカー誤差 |worldA-worldB| (自前sim / MMDベイクCSV幾何)]");
        for (int t = 0; t < 3; t++)
        {
            var (om, op, ox) = Stat(jErrOurs[t]);
            var (rm2, rp2, rx2) = Stat(jErrRef[t]);
            O.AppendLine($"   {tName[t],-3}: 自前 中央={om:F4}/p90={op:F4}/最大={ox:F4}   MMD 中央={rm2:F4}/p90={rp2:F4}/最大={rx2:F4}");
        }
        O.AppendLine("[角度リミット実効挙動 (リミット外率% / 超過量deg 中央/p90)]");
        for (int t = 0; t < 3; t++)
        {
            var (oM, oP, _) = Stat(angOverOurs[t]);
            var (rM, rP, _) = Stat(angOverRef[t]);
            double oPct = angTotal[t] > 0 ? 100.0 * angOut[t] / angTotal[t] : 0;
            double rPct = angTotalR[t] > 0 ? 100.0 * angOutR[t] / angTotalR[t] : 0;
            O.AppendLine($"   {tName[t],-3}: 自前 外率={oPct:F1}% 超過={oM:F2}/{oP:F2}°   MMD 外率={rPct:F1}% 超過={rM:F2}/{rP:F2}°");
        }
        if (stepDeltas.Count > 0)
        {
            var sorted = new List<float>(stepDeltas); sorted.Sort();
            float medD = sorted[sorted.Count / 2];
            float meanD = 0; foreach (var d in stepDeltas) meanD += d; meanD /= stepDeltas.Count;
            float meanP0 = 0; foreach (var p in pen0s) meanP0 += p; meanP0 /= pen0s.Count;
            int dec = stepDeltas.Count(d => d < -1e-4f), inc = stepDeltas.Count(d => d > 1e-4f);
            float theoStep = 0.2f * meanP0;         // Baumgarte 期待 1step 回復量
            O.AppendLine($"[per-step 貫入回復(静区間,deep,同ペア)] n={stepDeltas.Count} 平均pen={meanP0:F3}");
            O.AppendLine($"  1step Δpen: 中央={medD:F4} 平均={meanD:F4} (負=回復)  減少={dec}/増加={inc}");
            O.AppendLine($"  実効回復速度={-meanD / DT:F2} u/s (理論Baumgarte={0.2f * meanP0 / DT:F2} u/s, 1step理論={theoStep:F3})  実効/理論={(theoStep > 0 ? -meanD / theoStep : 0):P0}");
        }
        else O.AppendLine("[per-step 貫入回復] 静区間deepサンプル0");
        if (relDriftVals.Count > 0)
        {
            relDriftVals.Sort();
            float rm = relDriftVals[relDriftVals.Count / 2], rp = relDriftVals[(int)(relDriftVals.Count * 0.9)], rx = relDriftVals[relDriftVals.Count - 1];
            float refm = 0, refp = 0, refx = 0;
            if (relDriftRef.Count > 0) { relDriftRef.Sort(); refm = relDriftRef[relDriftRef.Count / 2]; refp = relDriftRef[(int)(relDriftRef.Count * 0.9)]; refx = relDriftRef[relDriftRef.Count - 1]; }
            O.AppendLine($"[移動ロック相対保持 skirt 同subset/同定義] 自前 中央={rm:F3}/p90={rp:F3}/最大={rx:F3}  MMD(CSV) 中央={refm:F3}/p90={refp:F3}/最大={refx:F3}  (小さい=固く保持)");
            if (stretchVals.Count > 0 && stretchRef.Count > 0)
            {
                stretchVals.Sort(); stretchRef.Sort();
                float sm = stretchVals[stretchVals.Count / 2], sp = stretchVals[(int)(stretchVals.Count * 0.9)], sx = stretchVals[stretchVals.Count - 1];
                float tm = stretchRef[stretchRef.Count / 2], tp = stretchRef[(int)(stretchRef.Count * 0.9)], tx = stretchRef[stretchRef.Count - 1];
                O.AppendLine($"[伸び(回転不変) | |子-親|-|bind| |] 自前 中央={sm:F3}/p90={sp:F3}/最大={sx:F3}  MMD(CSV) 中央={tm:F3}/p90={tp:F3}/最大={tx:F3}  (回転で変わらない=鎖の伸びのみ)");
                if (dirVals.Count > 0 && dirRef.Count > 0)
                {
                    dirVals.Sort(); dirRef.Sort();
                    float dm = dirVals[dirVals.Count / 2], dp2 = dirVals[(int)(dirVals.Count * 0.9)];
                    float em = dirRef[dirRef.Count / 2], ep = dirRef[(int)(dirRef.Count * 0.9)];
                    O.AppendLine($"[方向変化角 (現r vs bind r)] 自前 中央={dm:F1}°/p90={dp2:F1}°  MMD(CSV) 中央={em:F1}°/p90={ep:F1}°  (振り子=枠傾きと同程度 / 横滑り=枠より大)");
                }
            }
        }
        // ターン窓 = ヨー>360°/s の frame を含む ±15f 窓 (簡易)。静区間=それ以外。
        bool[] turn = new bool[F];
        for (int f = 0; f < F; f++) if (Math.Abs(yawRate[f]) > 360f) for (int k = Math.Max(0, f - 15); k < Math.Min(F, f + 15); k++) turn[k] = true;
        int nturn = turn.Count(x => x), nquiet = F - nturn;
        O.AppendLine($"\n===== (2) 髪 MMD突合 (自前 warm={world.UseJointWarmStart} fac={Joint.WarmStartFactor}) =====");
        O.AppendLine($"フレーム {F} (ターン窓={nturn}f 静区間={nquiet}f)");

        // ★符号付き位置差(自前-MMD) 静区間平均: 鏡像診断。Z(前後)が支配的に非0なら鏡像。
        double hx = 0, hy = 0, hz = 0; int hn = 0; double kx = 0, ky = 0, kz = 0; int kn = 0;
        for (int f = 0; f < F; f++)
        {
            if (turn[f]) continue;
            if (hCn[f] > 0) { hx += hSx[f]; hy += hSy[f]; hz += hSz[f]; hn += hCn[f]; }
            if (sCn[f] > 0) { kx += sSx[f]; ky += sSy[f]; kz += sSz[f]; kn += sCn[f]; }
        }
        string Dom(double x, double y, double z) => Math.Abs(z) >= Math.Abs(x) && Math.Abs(z) >= Math.Abs(y) ? "dz(前後=鏡像!)" : (Math.Abs(x) >= Math.Abs(y) ? "dx(左右)" : "dy(上下)");
        if (hn > 0) O.AppendLine($"[符号付き位置差 自前-MMD 髪 静区間平均] ({hx / hn:F3},{hy / hn:F3},{hz / hn:F3}) 支配={Dom(hx / hn, hy / hn, hz / hn)}");
        if (kn > 0) O.AppendLine($"[符号付き位置差 自前-MMD スカート 静区間平均] ({kx / kn:F3},{ky / kn:F3},{kz / kn:F3}) 支配={Dom(kx / kn, ky / kn, kz / kn)}");

        // 全体 位置/角度 差 (全frame×全髪ボーン)
        List<float> allPos = new(), allAng = new(); float signSum = 0; int signN = 0;
        foreach (var (l, b, _) in hairLinks)
        {
            allPos.AddRange(posDiff[b]); allAng.AddRange(angDiff[b]);
            for (int k = 0; k < oursDev[b].Count; k++) { signSum += oursDev[b][k] - refDev[b][k]; signN++; }
        }
        var (pm, pp, px) = Stat(allPos); var (am, ap, ax) = Stat(allAng);
        O.AppendLine($"[全体] 位置差(u) 中央={pm:F3}/p90={pp:F3}/最大={px:F3}   角度差(°) 中央={am:F2}/p90={ap:F2}/最大={ax:F2}");
        O.AppendLine($"[符号] 平均(自前bind偏差 - MMDbind偏差)={signSum / Math.Max(1, signN):F2}° (正=自前が動きすぎ / 負=動かなさすぎ)");

        // 静区間/ターン窓 別 (角度差)
        List<float> qAng = new(), tAng = new(), qPos = new(), tPos = new();
        foreach (var (l, b, _) in hairLinks)
        {
            var pl = posDiff[b]; var al = angDiff[b];
            // posDiff/angDiff は f昇順(欠損スキップ無し=全frame)。turn[] と同長。
            for (int f = 0, idx = 0; f < F; f++) { if (idx >= al.Count) break; (turn[f] ? tAng : qAng).Add(al[idx]); (turn[f] ? tPos : qPos).Add(pl[idx]); idx++; }
        }
        var (qam, qap, qax) = Stat(qAng); var (tam, tap, tax) = Stat(tAng);
        var (qpm, qpp, qpx) = Stat(qPos); var (tpm, tpp, tpx) = Stat(tPos);
        O.AppendLine($"[静区間] 角度差 中央={qam:F2}/p90={qap:F2}/最大={qax:F2}  位置差 中央={qpm:F3}/p90={qpp:F3}/最大={qpx:F3}");
        O.AppendLine($"[ターン窓] 角度差 中央={tam:F2}/p90={tap:F2}/最大={tax:F2}  位置差 中央={tpm:F3}/p90={tpp:F3}/最大={tpx:F3}");

        // 段(segment tier)別 角度差
        O.AppendLine("[段別] seg=末尾番号 の角度差(中央/p90/最大):");
        var bySeg = new Dictionary<int, List<float>>();
        foreach (var (l, b, _) in hairLinks) { int seg = Parse(b).seg; if (!bySeg.ContainsKey(seg)) bySeg[seg] = new(); bySeg[seg].AddRange(angDiff[b]); }
        foreach (var kv in bySeg.OrderBy(k => k.Key)) { var (m, p, x) = Stat(kv.Value); O.AppendLine($"   seg{kv.Key}: {m:F2}/{p:F2}/{x:F2} (°)"); }

        // 房別 上位(角度差最大が大きい房)
        O.AppendLine("[房別] 角度差最大 上位8房:");
        var byStrand = new Dictionary<string, List<float>>();
        foreach (var (l, b, _) in hairLinks) { string st = Parse(b).strand; if (!byStrand.ContainsKey(st)) byStrand[st] = new(); byStrand[st].AddRange(angDiff[b]); }
        foreach (var kv in byStrand.Select(k => (k.Key, Stat(k.Value))).OrderByDescending(x => x.Item2.max).Take(8)) O.AppendLine($"   {kv.Key}: 中央={kv.Item2.med:F2}/p90={kv.Item2.p90:F2}/最大={kv.Item2.max:F2}°");

        // 髪×体 貫入
        O.AppendLine("\n[髪×体 貫入] ペア別 最大貫入 上位10:");
        foreach (var kv in penMax.OrderByDescending(k => k.Value).Take(10)) O.AppendLine($"   {kv.Key}: {kv.Value:F3}");
        // 分母明示 (過去の 158 vs 7001 誤り防止): 自前もMMDも 全Fフレーム × 同一ペア集合 × pen>0.5。
        int selfDF = deepFrames.Select(x => x.f).Distinct().Count();
        int selfDP = deepFrames.Select(x => x.pair).Distinct().Count();
        int deepTurn = deepFrames.Count(x => turn[x.f]);
        O.AppendLine($"[分母] 全{F}f × 髪{hairLinks.Count}×体{bodyLinks.Count}ペア を同一条件で計数(自前=物理結果/MMD=幾何配置, 定義pen>0.5, GjkEpa.Detect)");
        O.AppendLine($"[自前 深貫入>0.5] イベント={deepFrames.Count} (ユニークフレーム={selfDF}, ユニークペア={selfDP}) 区間: ターン窓={deepTurn} 静={deepFrames.Count - deepTurn} 最大={(deepFrames.Count > 0 ? deepFrames.Max(x => x.d) : 0):F3}");
        O.AppendLine($"[MMD 深貫入>0.5(幾何)] イベント={refDeep} 最大={refDeepMax:F3}  ★同分母。自前>>MMD=自前固有の衝突抜け");

        // ★上位20 深貫入 診断: 検出されてないか / 検出済みで押し戻せてないか の分岐
        O.AppendLine("\n[上位20 深貫入 診断] (AABB=broadphase重なり, 点=マニフォールド接触点数, ni=法線力積, 継続=連続deepフレーム数)");
        foreach (var e in deepFrames.OrderByDescending(x => x.d).Take(20))
        {
            var st = deepByPair[e.pair]; int dur = 1;
            for (int k = e.f - 1; st.Contains(k); k--) dur++;
            for (int k = e.f + 1; st.Contains(k); k++) dur++;
            O.AppendLine($"   f{e.f,4} {e.pair,-20} pen={e.d:F3} AABB={(e.aabb ? "重" : "離")} 点={e.mpts} ni={e.ni:F4} 継続={dur}f {(turn[e.f] ? "[ターン]" : "[静]")}");
        }
        O.AppendLine("   (点=永続マニフォールドの接触点数=このペアのDebugContacts件数)");
        O.AppendLine("   ★読み: AABB離 or 点0 → 未検出。 点1でni大 → 単点支持で回り込み沈み(=4点マニフォールド化の対象)。 ni≈0 → 接触未構築。");
        // 深貫入時のマニフォールド点数分布 (単点支持が主因かの確認)
        var mfpDist = new int[6];
        foreach (var e in deepFrames) mfpDist[Math.Min(5, e.mpts)]++;
        O.AppendLine($"[深貫入時マニフォールド点数分布] 点0={mfpDist[0]} 点1={mfpDist[1]} 点2={mfpDist[2]} 点3={mfpDist[3]} 点4={mfpDist[4]} 点5+={mfpDist[5]}");

        Console.Write(O.ToString());
        return 0;
    }
}
