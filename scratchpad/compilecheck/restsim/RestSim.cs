// 静止(bindポーズ・アニメOFF・重力のみ)でスカート箱が脚カプセルを貫通しないかを実測する。
// ユーザ報告:「自作カスタムで初期姿勢のままなのにスカートが突き抜ける(アニメOFF)」の回帰テスト。
// スカート=薄い箱(dynamic)、脚=カプセル(BoneFollow/kinematic)。box×capsule の接触が
// 取りこぼされると、重力で箱が脚を貫通する。GjkEpa.Detect(=解析box×capsule込み)で貫入を測る。
using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

static class RestSim
{
    const float FRAME = 1f / 30f;
    static readonly string[] LegNames = { "下半身", "下半身1", "下半身3", "下半身4", "右太もも", "左太もも", "左足", "右足", "右ひざ", "左ひざ" };
    static List<ContactPoint> buf = new();

    static float PenDepth(RigidBody a, RigidBody b)
    {
        buf.Clear(); GjkEpa.Detect(a, b, buf);
        float d = 0; foreach (var cp in buf) { float pen = -cp.Distance; if (pen > d) d = pen; } return d;
    }
    static bool AabbNear(RigidBody a, RigidBody b)
    {
        var aa = a.ComputeAabb(); var bb = b.ComputeAabb(); return aa.Intersects(ref bb);
    }

    static Vec3 ClosestOnSeg(Vec3 p, Vec3 a, Vec3 b)
    {
        var ab = b - a; float t = ab.LengthSquared > 1e-12f ? (p - a).Dot(ab) / ab.LengthSquared : 0f;
        t = t < 0 ? 0 : (t > 1 ? 1 : t); return a + ab * t;
    }
    // 箱中心から脚カプセル軸までの符号付きクリアランス(距離-半径)。負=中心がカプセル内部(=反対側へ抜けた疑い)。
    static float SignedClearance(RigidBody box, RigidBody cap)
    {
        var c = (CapsuleShape)cap.Shape;
        var p0 = cap.WorldTransform.TransformPoint(new Vec3(0, c.HalfHeight, 0));
        var p1 = cap.WorldTransform.TransformPoint(new Vec3(0, -c.HalfHeight, 0));
        var bc = box.WorldTransform.Origin;
        var cp = ClosestOnSeg(bc, p0, p1);
        return (bc - cp).Length - c.Radius;
    }

    static int Main()
    {
        bool useGlb = Environment.GetEnvironmentVariable("USE_GLB") == "1";
        PmxPhysicsModel model;
        if (useGlb)
        {
            string glb = TestData.GlbPath();
            if (glb == null || !File.Exists(glb)) { Console.WriteLine("[SKIP] no glb"); return 0; }
            model = GlbPhysicsReader.LoadFile(glb);
            Console.WriteLine($"[loader] GLB: {glb}");
        }
        else
        {
            string pmx = TestData.PmxPath();
            if (pmx == null || !File.Exists(pmx)) { Console.WriteLine("[SKIP] no pmx"); return 0; }
            model = PmxReader.LoadFile(pmx);
            Console.WriteLine($"[loader] PMX: {pmx}");
        }
        var builder = PmxPhysicsBuilder.Build(model);
        int subs = int.TryParse(Environment.GetEnvironmentVariable("SUBSTEPS"), out var sv) ? sv : 2;
        float grav = float.TryParse(Environment.GetEnvironmentVariable("GRAV"), out var gv) ? gv : 98f;
        var world = builder.World;
        int iters = int.TryParse(Environment.GetEnvironmentVariable("ITERS"), out var iv) ? iv : 10;
        world.SolverIterations = iters; world.FixedTimeStep = FRAME; world.SubSteps = subs;
        world.Gravity = new Vec3(0f, -grav, 0f); // MMDスケール重力(エンジンネイティブ単位)
        float beta = -1;
        if (float.TryParse(Environment.GetEnvironmentVariable("BETA"), out var bt)) { beta = bt; foreach (var j in world.Joints) j.Beta = bt; }
        if (Environment.GetEnvironmentVariable("SPLIT") == "1") world.UseSplitImpulse = true;
        if (Environment.GetEnvironmentVariable("JSPLIT") == "1") world.UseJointSplitImpulse = true;
        if (Environment.GetEnvironmentVariable("WARMSTART_ANG") == "1") { world.UseJointWarmStart = true; world.UseJointWarmStartAngular = true; }
        if (Environment.GetEnvironmentVariable("WARMSTART") == "1") world.UseJointWarmStart = true;
        if (Environment.GetEnvironmentVariable("WARM_OFF") == "1") { world.UseJointWarmStart = false; world.UseJointWarmStartAngular = false; }
        BulletPhysics.Joint.WarmAngRows = 0; BulletPhysics.Joint.WarmAngToggles = 0;
        if (float.TryParse(Environment.GetEnvironmentVariable("WARMFAC"), out var _wf)) BulletPhysics.Joint.WarmStartFactor = _wf;

        // タスクC: ジョイント求解順序切替 (エンジン無改変, world.Joints を並べ替えるだけ)。
        // キネマティック根から BFS で各剛体の深さを求め、各ジョイントの深さ=max(端点深さ) で並べる。
        string order = Environment.GetEnvironmentVariable("ORDER") ?? "current";
        if (order != "current")
        {
            var depth = new Dictionary<RigidBody, int>();
            var q = new Queue<RigidBody>();
            foreach (var b in builder.Bodies) if (b.IsStaticOrKinematic) { depth[b] = 0; q.Enqueue(b); }
            while (q.Count > 0)
            {
                var b = q.Dequeue();
                foreach (var j in world.Joints)
                {
                    RigidBody o = j.BodyA == b ? j.BodyB : (j.BodyB == b ? j.BodyA : null);
                    if (o != null && !depth.ContainsKey(o)) { depth[o] = depth[b] + 1; q.Enqueue(o); }
                }
            }
            int JD(Joint j) { int da = depth.TryGetValue(j.BodyA, out var x) ? x : 9999; int db = depth.TryGetValue(j.BodyB, out var y) ? y : 9999; return Math.Max(da, db); }
            var sorted = new List<Joint>(world.Joints);
            sorted.Sort((a, b) => order == "leaf2root" ? JD(b).CompareTo(JD(a)) : JD(a).CompareTo(JD(b)));
            world.Joints.Clear(); world.Joints.AddRange(sorted);
            Console.WriteLine($"[cfg] ORDER={order} (joints reordered by chain depth)");
        }
        Console.WriteLine($"[cfg] subs={subs} grav={grav} iters={world.SolverIterations} beta={(beta < 0 ? "default" : beta.ToString())} split={world.UseSplitImpulse}");

        // アニメOFF=bindポーズ維持: 追従(kinematic)剛体の目標をbind世界姿勢で固定。
        // MIRROR_BF=1: Unityブリッジの「対を持たない3回目のZ反転」を模擬。ボーン追従目標だけを
        // MirrorZ(=pos.z反転, rot(-x,-y,z,w))して、物理定義(ジョイント枠/動的体初期位置=+z真値)との
        // Z鏡像ズレを再現する。これで髪が正面(-z)へ引かれ体を貫通すれば、Unity症状のヘッドレス再現。
        bool mirrorBF = Environment.GetEnvironmentVariable("MIRROR_BF") == "1";
        RigidTransform MirrorZ(RigidTransform t) => new RigidTransform(
            new Quat(-t.Rotation.x, -t.Rotation.y, t.Rotation.z, t.Rotation.w),
            new Vec3(t.Origin.x, t.Origin.y, -t.Origin.z));
        foreach (var b in builder.Bodies)
            if (b.Mode == PhysicsMode.BoneFollow)
            {
                var tgt = mirrorBF ? MirrorZ(b.WorldTransform) : b.WorldTransform;
                b.KinematicTarget = tgt; b.KinematicStepTarget = tgt;
                if (mirrorBF) b.WorldTransform = tgt; // 初期位置も鏡像側へ(Unityは毎フレームPushで上書き)
            }
        if (mirrorBF) Console.WriteLine("[cfg] MIRROR_BF: bone-follow targets Z-mirrored (Unityブリッジ3回目Z反転の模擬)");

        // 切り分け: ANGFREE=1 で全ジョイントの角度リミットを自由化 (角度拘束を無効化)。
        if (Environment.GetEnvironmentVariable("ANGFREE") == "1")
        {
            foreach (var j in builder.World.Joints) { j.AngularLowerLimit = new Vec3(1, 1, 1); j.AngularUpperLimit = new Vec3(-1, -1, -1); }
            Console.WriteLine("[cfg] ANGFREE: all angular limits freed");
        }
        // 切り分け: ANGSCALE=値 で角度リミット幅を倍率。0=完全ロック(回転不可)。
        if (float.TryParse(Environment.GetEnvironmentVariable("ANGSCALE"), out var asc))
        {
            foreach (var j in builder.World.Joints)
            {
                var lo = j.AngularLowerLimit; var hi = j.AngularUpperLimit;
                if (lo.x > hi.x) continue; // free はそのまま
                j.AngularLowerLimit = new Vec3(lo.x * asc, lo.y * asc, lo.z * asc);
                j.AngularUpperLimit = new Vec3(hi.x * asc, hi.y * asc, hi.z * asc);
            }
            Console.WriteLine($"[cfg] ANGSCALE={asc} (0=lock)");
        }
        // 切り分け: LINFREE=1 で全ジョイントの並進リミットを自由化。
        if (Environment.GetEnvironmentVariable("LINFREE") == "1")
        {
            foreach (var j in builder.World.Joints) { j.LinearLowerLimit = new Vec3(1, 1, 1); j.LinearUpperLimit = new Vec3(-1, -1, -1); }
            Console.WriteLine("[cfg] LINFREE: all linear limits freed");
        }
        // 髪ジョイントのパラメータを数件ダンプ。
        if (Environment.GetEnvironmentVariable("DUMPJ") == "1")
        {
            int c = 0;
            foreach (var j in builder.World.Joints)
            {
                string na = j.BodyA?.Name ?? "?", nb = j.BodyB?.Name ?? "?";
                if (!(na.Contains("髪") || nb.Contains("髪"))) continue;
                Console.WriteLine($"[J] {na}->{nb} angLo={j.AngularLowerLimit} angHi={j.AngularUpperLimit} linLo={j.LinearLowerLimit} linHi={j.LinearUpperLimit} sprAng={j.SpringAngular} sprLin={j.SpringLinear}");
                if (++c >= 6) break;
            }
        }

        // 切り分け: DAMPBOOST=値 で全動的剛体の減衰を底上げ (暴れが減衰で収まるか)。
        if (float.TryParse(Environment.GetEnvironmentVariable("DAMPBOOST"), out var db))
        {
            foreach (var b in builder.Bodies) if (!b.IsStaticOrKinematic) { b.LinearDamping = Math.Max(b.LinearDamping, db); b.AngularDamping = Math.Max(b.AngularDamping, db); }
            Console.WriteLine($"[cfg] DAMPBOOST: dyn damping >= {db}");
        }

        // 切り分け: NOCONTACT=1 で全剛体の衝突マスクを0(何とも当たらない)にし、ジョイントのみで検証。
        if (Environment.GetEnvironmentVariable("NOCONTACT") == "1")
        {
            foreach (var b in builder.Bodies) b.CollisionMask = 0;
            Console.WriteLine("[cfg] NOCONTACT: all collision masks zeroed (joints only)");
        }

        // DUMP_BIND=1: 対象ボーンのPMXバインド世界位置と、UnityToMmdPos(=Zのみ反転)予測値を並記。
        // Unityログの UnityToMmd(boneTransform.position) がこの「予測(鏡像)」と一致すれば鏡像バグ確定。
        if (Environment.GetEnvironmentVariable("DUMP_BIND") == "1")
        {
            string[] targets = { "頭", "上半身2", "上半身", "下半身", "右腕", "左腕", "髪FR5", "髪FL5", "髪FR4" };
            Console.WriteLine("[bind] bone : PMXバインド(x,y,z) | UnityToMmd予測=(x,y,-z) ← Unityログがこの鏡像に一致すればバグ");
            for (int i = 0; i < model.BoneNames.Count; i++)
            {
                if (Array.IndexOf(targets, model.BoneNames[i]) < 0) continue;
                var p = model.BonePositions[i];
                Console.WriteLine($"   {model.BoneNames[i],-8}: PMX=({p.x,7:F3},{p.y,7:F3},{p.z,7:F3}) | 鏡像予測=({p.x,7:F3},{p.y,7:F3},{-p.z,7:F3})");
            }
        }

        var skirt = builder.Bodies.Where(b => b.Name.StartsWith("スカート") && !b.IsStaticOrKinematic).ToList();
        var legs = builder.Bodies.Where(b => LegNames.Contains(b.Name)).ToList();

        var bindY = skirt.ToDictionary(s => s, s => s.WorldTransform.Origin.y);
        // 各スカート箱の bind 時の「最寄り脚カプセルへの符号付きクリアランス」。
        var bindClear = skirt.ToDictionary(s => s, s => legs.Where(l => l.Shape is CapsuleShape).Select(l => SignedClearance(s, l)).DefaultIfEmpty(999f).Min());
        // スカート箱の厚み(最小半サイズ)。これを超える貫入=箱が脚に完全に埋没≒貫通。
        float boxThick = 0.085f;

        float bindPeak = 0;
        foreach (var s in skirt) foreach (var l in legs) { if (!AabbNear(s, l)) continue; float d = PenDepth(s, l); if (d > bindPeak) bindPeak = d; }

        var O = new StringBuilder();
        O.AppendLine($"model rigidbodies={builder.Bodies.Count} skirt(dyn box)={skirt.Count} legs(capsule)={legs.Count}");
        O.AppendLine($"[bind/0step] peak skirt×leg penetration = {bindPeak:F4}");

        // ★全動的剛体(髪/スカート等)の発散(爆発)監視。bind位置を控える。
        var dyn = builder.Bodies.Where(b => !b.IsStaticOrKinematic).ToList();
        var hair = dyn.Where(b => b.Name.Contains("髪") || b.Name.Contains("ツインテ") || b.Name.Contains("もみあげ") || b.Name.Contains("前髪")).ToList();
        var dynBind = dyn.ToDictionary(b => b, b => b.WorldTransform.Origin);

        // bind(0step)時の初期貫入: 動的×動的(髪絡み) と 動的×追従。
        float bindDD = 0; string bindDDn = ""; float bindDK = 0; string bindDKn = "";
        var kin = builder.Bodies.Where(b => b.IsStaticOrKinematic).ToList();
        for (int i = 0; i < dyn.Count; i++)
            for (int j = i + 1; j < dyn.Count; j++)
            { if (!AabbNear(dyn[i], dyn[j])) continue; if (!PhysicsWorld.ShouldCollide(dyn[i], dyn[j])) continue; float d = PenDepth(dyn[i], dyn[j]); if (d > bindDD) { bindDD = d; bindDDn = $"{dyn[i].Name}×{dyn[j].Name}"; } }
        foreach (var b in dyn) foreach (var k in kin)
        { if (!AabbNear(b, k)) continue; if (!PhysicsWorld.ShouldCollide(b, k)) continue; float d = PenDepth(b, k); if (d > bindDK) { bindDK = d; bindDKn = $"{b.Name}×{k.Name}"; } }
        O.AppendLine($"[bind pen] dyn×dyn max={bindDD:F4} ({bindDDn})  dyn×kin max={bindDK:F4} ({bindDKn})");

        // 体コライダー(髪の相手になる kinematic 体) = 髪自身でない kinematic 全部。
        var bodyKin = kin;
        int N = 180; float runPeak = 0; bool nan = false;
        float maxSpeed = 0; string maxSpeedName = ""; float maxDisp = 0; string maxDispName = "";
        int explodeStep = -1;
        int deepPenRun = 0; // タスクX: 髪×体 深貫入(>0.5) の (剛体×ステップ) 延べ件数(ShouldCollideゲート有)
        int deepPenRunRaw = 0; float runRawPeakPen = 0; string runRawPeakPair = ""; // ゲート無し(生geo)
        var tsSteps = new HashSet<int> { 0, 1, 2, 4, 8, 16, 32, 64, 128, 179 };
        for (int f = 0; f < N; f++)
        {
            world.StepSimulation(FRAME);
            if (explodeStep < 0)
                foreach (var b in dyn) if (b.LinearVelocity.Length > 5f) { explodeStep = f; break; }
            if (tsSteps.Contains(f))
            {
                float ms = 0; float sumDisp = 0;
                foreach (var b in dyn) { float sp = b.LinearVelocity.Length; if (sp > ms) ms = sp; sumDisp += (b.WorldTransform.Origin - dynBind[b]).Length; }
                O.AppendLine($"   t[{f,3}] maxSpeed={ms,8:F3} sumDisp={sumDisp,9:F2}");
            }
            foreach (var s in skirt) { var o = s.WorldTransform.Origin; if (float.IsNaN(o.x) || float.IsNaN(o.y) || float.IsNaN(o.z)) nan = true; }
            foreach (var b in dyn)
            {
                var o = b.WorldTransform.Origin;
                if (float.IsNaN(o.x) || float.IsNaN(o.y) || float.IsNaN(o.z)) nan = true;
                float sp = b.LinearVelocity.Length; if (sp > maxSpeed) { maxSpeed = sp; maxSpeedName = b.Name; }
                float dp = (o - dynBind[b]).Length; if (dp > maxDisp) { maxDisp = dp; maxDispName = b.Name; }
            }
            foreach (var s in skirt) foreach (var l in legs) { if (!AabbNear(s, l)) continue; float d = PenDepth(s, l); if (d > runPeak) runPeak = d; }
            // タスクX: 髪×体 深貫入(>0.5) を延べ計上(ゲート有/無)。生geoピークも追跡。
            foreach (var b in hair) foreach (var k in bodyKin)
            {
                if (!AabbNear(b, k)) continue; float d = PenDepth(b, k);
                if (d > 0.5f) { deepPenRunRaw++; if (PhysicsWorld.ShouldCollide(b, k)) deepPenRun++; }
                if (d > runRawPeakPen) { runRawPeakPen = d; runRawPeakPair = $"{b.Name}×{k.Name}"; }
            }
        }
        // モデル寸法(全剛体bindのbboxの対角)を基準に「爆発」を判断。
        float exploded = dyn.Count(b => (b.WorldTransform.Origin - dynBind[b]).Length > 2.0f);
        O.AppendLine($"[explosion] dyn bodies={dyn.Count} (hair={hair.Count})  maxSpeed={maxSpeed:F2} ({maxSpeedName})  maxDisp={maxDisp:F3} ({maxDispName})  moved>2.0:{exploded}  firstExplodeStep={explodeStep}");

        float finalPeak = 0; string worst = "";
        int throughCount = 0; // 貫入 > 厚み ×2 (完全貫通の疑い) のスカート箱数
        foreach (var s in skirt)
        {
            float sMax = 0; string sl = "";
            foreach (var l in legs) { if (!AabbNear(s, l)) continue; float d = PenDepth(s, l); if (d > sMax) { sMax = d; sl = l.Name; } }
            if (sMax > finalPeak) { finalPeak = sMax; worst = $"{s.Name}×{sl}"; }
            if (sMax > boxThick * 2f) throughCount++;
        }
        float maxDrop = 0; string dropName = "";
        foreach (var s in skirt) { float drop = bindY[s] - s.WorldTransform.Origin.y; if (drop > maxDrop) { maxDrop = drop; dropName = s.Name; } }

        O.AppendLine($"[after {N}steps @rest] NaN={nan}  run-peak pen={runPeak:F4}  final-peak pen={finalPeak:F4} ({worst})");
        O.AppendLine($"[after] skirt boxes with pen>{boxThick * 2f:F3}(=貫通疑い) : {throughCount}/{skirt.Count}");
        O.AppendLine($"[after] max downward drop of a skirt box = {maxDrop:F4} ({dropName})");

        // 符号付きクリアランス: 箱中心が脚カプセル内部(負)へ入り込んだ=反対側へ突き抜けた疑い。
        int wrongSide = 0; float worstNeg = 0; string worstNegName = ""; int flipped = 0;
        foreach (var s in skirt)
        {
            float fin = legs.Where(l => l.Shape is CapsuleShape).Select(l => SignedClearance(s, l)).DefaultIfEmpty(999f).Min();
            if (fin < -0.02f) wrongSide++;
            if (fin < worstNeg) { worstNeg = fin; worstNegName = s.Name; }
            if (bindClear[s] >= -0.01f && fin < -0.05f) flipped++; // bindは外側だったのに内側へ抜けた
        }
        O.AppendLine($"[after] boxes with center INSIDE a leg capsule (clear<-0.02) = {wrongSide}/{skirt.Count}  worst={worstNeg:F4} ({worstNegName})");
        O.AppendLine($"[after] boxes that FLIPPED outside->inside(=突き抜け疑い) = {flipped}/{skirt.Count}");
        // ★方向診断(2026-08-09): 髪剛体の変位ベクトル(final-bind)。重力主因なら dy 支配。
        //   dz(前後)支配なら重力では説明できない=鏡像バグ疑い。
        {
            var dxs = new List<float>(); var dys = new List<float>(); var dzs = new List<float>();
            foreach (var b in hair) { var d = b.WorldTransform.Origin - dynBind[b]; dxs.Add(d.x); dys.Add(d.y); dzs.Add(d.z); }
            float Med(List<float> v) { v.Sort(); return v.Count > 0 ? v[v.Count / 2] : 0; }
            float Avg(List<float> v) { float s = 0; foreach (var x in v) s += x; return v.Count > 0 ? s / v.Count : 0; }
            float amx = Math.Abs(Avg(dxs)), amy = Math.Abs(Avg(dys)), amz = Math.Abs(Avg(dzs));
            string dom = amy >= amx && amy >= amz ? "dy(重力)" : (amz >= amx ? "dz(前後=鏡像疑い!)" : "dx(左右)");
            O.AppendLine($"[方向診断 髪変位ベクトル final-bind] 中央(dx,dy,dz)=({Med(dxs):F3},{Med(dys):F3},{Med(dzs):F3}) 平均=({Avg(dxs):F3},{Avg(dys):F3},{Avg(dzs):F3}) 支配軸={dom}");
            // タスクX計測: 髪 maxDrift(=変位の最大長)、髪×体 深貫入(>0.5) の final件数。
            float hairMaxDrift = 0; foreach (var b in hair) { float dd = (b.WorldTransform.Origin - dynBind[b]).Length; if (dd > hairMaxDrift) hairMaxDrift = dd; }
            int deepPenFinal = 0; foreach (var b in hair) foreach (var k in bodyKin) { if (!AabbNear(b, k)) continue; if (!PhysicsWorld.ShouldCollide(b, k)) continue; if (PenDepth(b, k) > 0.5f) deepPenFinal++; }
            // ゲート無し(生の幾何貫入): 衝突グループを無視した実際の埋没。ゲート版との差=「衝突しない設定で貫通」の量。
            int deepPenRawFinal = 0; float worstRaw = 0; string worstRawPair = ""; int nonCollideDeep = 0;
            foreach (var b in hair) foreach (var k in bodyKin)
            {
                if (!AabbNear(b, k)) continue; float d = PenDepth(b, k);
                if (d > 0.5f) { deepPenRawFinal++; if (!PhysicsWorld.ShouldCollide(b, k)) nonCollideDeep++; }
                if (d > worstRaw) { worstRaw = d; worstRawPair = $"{b.Name}×{k.Name}{(PhysicsWorld.ShouldCollide(b, k) ? "[衝突ON]" : "[衝突OFF]")}"; }
            }
            O.AppendLine($"[タスクX] iters={world.SolverIterations} 髪maxDrift={hairMaxDrift:F3} dz={Avg(dzs):F3}");
            O.AppendLine($"   深貫入>0.5 ゲート有: final={deepPenFinal} 延べ={deepPenRun} / 生geo: final={deepPenRawFinal}(内衝突OFF={nonCollideDeep}) 延べ={deepPenRunRaw} run中最深={runRawPeakPen:F2}({runRawPeakPair}) final最深={worstRaw:F2}({worstRawPair})");
            // 髪×体で「衝突ON」に設定されたペアは存在するか。存在しなければ衝突は前方スイングを止めようがない。
            int collideOnPairs = 0, collideOnOverlap = 0; float collideOnPeakPen = 0; string collideOnPeakPair = "";
            foreach (var b in hair) foreach (var k in bodyKin)
            {
                if (!PhysicsWorld.ShouldCollide(b, k)) continue; collideOnPairs++;
                if (!AabbNear(b, k)) continue; float d = PenDepth(b, k);
                if (d > 0.02f) collideOnOverlap++;
                if (d > collideOnPeakPen) { collideOnPeakPen = d; collideOnPeakPair = $"{b.Name}×{k.Name}"; }
            }
            O.AppendLine($"   髪×体 衝突ONペア数={collideOnPairs} (final時 重なり>0.02={collideOnOverlap} 最深={collideOnPeakPen:F3} {collideOnPeakPair})");
            // 変位が大きい上位5剛体の符号付き内訳 + bind→final の絶対Z(前後の跨ぎを見る)。
            var top = hair.Select(b => (b, d: b.WorldTransform.Origin - dynBind[b])).OrderByDescending(x => x.d.Length).Take(5);
            foreach (var (b, d) in top) O.AppendLine($"   {b.Name}: 変位({d.x:F2},{d.y:F2},{d.z:F2})|{d.Length:F2}|  Z: bind={dynBind[b].z:F2}→final={b.WorldTransform.Origin.z:F2} {(dynBind[b].z > 0 && b.WorldTransform.Origin.z < 0 ? "★背→前 跨ぎ" : "")}");
        }
        long war = BulletPhysics.Joint.WarmAngRows, wat = BulletPhysics.Joint.WarmAngToggles;
        if (war > 0) O.AppendLine($"[warm-ang] 角度warm行={war} トグル={wat} トグル率={(double)wat / war:P1}");
        Console.Write(O.ToString());
        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "restsim_out.txt"), O.ToString(), new UTF8Encoding(false));
        return 0;
    }
}
