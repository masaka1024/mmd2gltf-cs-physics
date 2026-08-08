// 検証のみ(修正禁止): PmxPhysicsBuilder が構築した物理系が、PMXエディタCSVエクスポート
// (正解データ) の記述どおりかを照合する。剛体パラメータ/衝突フィルタ/慣性/Joint/
// FrameInA,B/ボーンオフセット を、CSVから独立に再計算した期待値と突き合わせる。
//
// 正解CSV: 環境変数 MMD_TEST_PMXCSV か既定パス。UTF-8, ';'始まりがヘッダ行。
// PMXバイナリ: MMD_TEST_PMX か既定パス。CSVとバイナリは同一モデルの別表現。
//
// 許容誤差の根拠: CSVの数値は float32 由来で有効約7桁。位置は最大~18程度なので
//   桁落ち由来の差は ~2e-5 以下。回転は「バイナリ(rad, f32)」を CSV では「deg(f32)」で
//   出力するため deg→rad 往復で ~1e-6 の差が乗る。よって 1e-4~1e-3 を閾値にすれば
//   丸め誤差を誤検出せず、意味のある差(桁違い/符号/取り違え)は確実に捕捉できる。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

static class PmxVerify
{
    const string DefPmx = @"C:\Users\masa_\BA_c1\Assets\mmd-for-unity-proj-mmd-for-unity-v2.1b-6-g82ac2fe\mmd-for-unity-proj-mmd-for-unity-82ac2fe\IA1\IA.pmx";
    const string DefCsv = @"C:\Users\masa_\Downloads\IA\IA\ia.csv";

    // 許容誤差 (根拠は冒頭コメント)。
    const float TolPos = 1e-3f, TolSize = 1e-4f, TolRot = 1e-3f, TolMass = 1e-4f;
    const float TolDamp = 1e-4f, TolInertiaRel = 1e-3f, TolLimit = 1e-4f, TolSpring = 1e-3f;
    const float TolFrame = 1e-3f, TolOffset = 1e-3f;
    // クォータニオン角(acosベース)の許容: 単位クォータニオンの内積は float32 で ~1e-7 の誤差を持ち、
    // acos がこれを √ε ≈ 1e-3 rad に増幅する(near-1 で微分が発散)。よって euler 成分の直接比較
    // (TolRot=1e-3) とは別に、クォータニオン角比較は 3e-3 rad(≈0.17°) を下限ノイズの閾値とする。
    const float TolQuatAngle = 3e-3f;
    const double D2R = Math.PI / 180.0;

    static StringBuilder O = new StringBuilder();
    static void L(string s = "") { O.Append(s); O.Append('\n'); }

    // 項目ごとの合否集計。
    class Agg
    {
        public string Field; public int n, fail; public double maxDiff; public string worst = "";
        public Agg(string f) { Field = f; }
        public void Add(double expected, double actual, double tol, string who)
        {
            n++; double d = Math.Abs(expected - actual);
            if (d > maxDiff) { maxDiff = d; }
            if (d > tol) { fail++; if (fail <= 8) worst += $"\n      {who}: 期待={expected:G7} 実測={actual:G7} 差={d:G4}"; }
        }
        public string Line() => $"  {Field,-26} 照合={n,4} 不一致={fail,3} 最大差={maxDiff:G4}" + (fail > 0 ? "  ★" + worst : "  一致");
    }

    static List<string[]> rowsBody = new(), rowsJoint = new(), rowsBone = new();

    static int Main()
    {
        string pmx = Environment.GetEnvironmentVariable("MMD_TEST_PMX"); if (string.IsNullOrEmpty(pmx) || !File.Exists(pmx)) pmx = DefPmx;
        string csv = Environment.GetEnvironmentVariable("MMD_TEST_PMXCSV"); if (string.IsNullOrEmpty(csv) || !File.Exists(csv)) csv = DefCsv;
        if (!File.Exists(pmx) || !File.Exists(csv)) { Console.WriteLine($"[SKIP] pmx={File.Exists(pmx)} csv={File.Exists(csv)}"); return 0; }

        ParseCsv(csv);
        var model = PmxReader.LoadFile(pmx);
        var builder = PmxPhysicsBuilder.Build(model);

        L("==================== PMX構築系 vs CSV(正解) 照合 ====================");
        L($"CSV: {csv}");
        L($"件数: 剛体 CSV={rowsBody.Count}/エンジン={model.RigidBodies.Count}  Joint CSV={rowsJoint.Count}/エンジン={model.Joints.Count}/構築={builder.World.Joints.Count}  ボーン CSV={rowsBone.Count}/エンジン={model.BoneNames.Count}");

        // CSV ボーン位置マップ
        var boneCsvPos = new Dictionary<string, Vec3>();
        foreach (var r in rowsBone) boneCsvPos[Unq(r[1])] = new Vec3(F(r[5]), F(r[6]), F(r[7]));
        // CSV 剛体名->row, 剛体名->(pos,rot) (Joint用)
        var bodyCsvByName = new Dictionary<string, string[]>();
        foreach (var r in rowsBody) bodyCsvByName[Unq(r[1])] = r;

        CheckBodies(model, builder, boneCsvPos);
        CheckCollision(model, builder);
        CheckInertia(model, builder);
        CheckJoints(model, builder, bodyCsvByName);

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "pmxverify_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }

    // ---------- 1. 剛体基本パラメータ + 6.オフセット ----------
    static void CheckBodies(PmxPhysicsModel model, PmxPhysicsBuilder builder, Dictionary<string, Vec3> boneCsvPos)
    {
        L("\n---------- 1. 剛体の基本パラメータ ----------");
        int nameMis = 0, boneMis = 0, typeMis = 0, groupMis = 0, shapeMis = 0;
        var aSize = new Agg("サイズ(3成分)"); var aPos = new Agg("位置"); var aRot = new Agg("回転(deg→rad)");
        var aMass = new Agg("質量"); var aLd = new Agg("移動減衰"); var aAd = new Agg("回転減衰");
        var aRe = new Agg("反発力"); var aFr = new Agg("摩擦力"); var aOff = new Agg("BodyOffsetFromBone.Origin");
        var offRotMis = new List<string>();

        int n = Math.Min(rowsBody.Count, model.RigidBodies.Count);
        for (int i = 0; i < n; i++)
        {
            var r = rowsBody[i]; var rb = model.RigidBodies[i]; var body = builder.Bodies[i];
            string cname = Unq(r[1]);
            if (cname != rb.Name) { nameMis++; if (nameMis <= 8) L($"    [名前/順序不一致] idx{i}: CSV='{cname}' エンジン='{rb.Name}'"); }
            // 関連ボーン
            string cbone = Unq(r[3]);
            string ebone = (rb.BoneIndex >= 0 && rb.BoneIndex < model.BoneNames.Count) ? model.BoneNames[rb.BoneIndex] : "";
            if (cbone != ebone) { boneMis++; if (boneMis <= 8) L($"    [関連ボーン不一致] {cname}: CSV='{cbone}' エンジン='{ebone}'"); }
            if (int.Parse(r[4]) != (int)rb.PhysicsMode) typeMis++;
            if (int.Parse(r[5]) != rb.Group) groupMis++;
            if (int.Parse(r[7]) != rb.ShapeType) shapeMis++;
            aSize.Add(F(r[8]), rb.Size.x, TolSize, cname + ".x"); aSize.Add(F(r[9]), rb.Size.y, TolSize, cname + ".y"); aSize.Add(F(r[10]), rb.Size.z, TolSize, cname + ".z");
            aPos.Add(F(r[11]), rb.Position.x, TolPos, cname + ".x"); aPos.Add(F(r[12]), rb.Position.y, TolPos, cname + ".y"); aPos.Add(F(r[13]), rb.Position.z, TolPos, cname + ".z");
            aRot.Add(F(r[14]) * D2R, rb.Rotation.x, TolRot, cname + ".x"); aRot.Add(F(r[15]) * D2R, rb.Rotation.y, TolRot, cname + ".y"); aRot.Add(F(r[16]) * D2R, rb.Rotation.z, TolRot, cname + ".z");
            aMass.Add(F(r[17]), rb.Mass, TolMass, cname); aLd.Add(F(r[18]), rb.LinearDamping, TolDamp, cname); aAd.Add(F(r[19]), rb.AngularDamping, TolDamp, cname);
            aRe.Add(F(r[20]), rb.Restitution, TolDamp, cname); aFr.Add(F(r[21]), rb.Friction, TolDamp, cname);

            // 6. BodyOffsetFromBone: 期待 Origin = bodyPos - bonePos(CSVボーン), Rotation = bodyRot
            var off = builder.BoneLinks[i].BodyOffsetFromBone;
            if (cbone != "" && boneCsvPos.TryGetValue(cbone, out var bp))
            {
                var exp = new Vec3(F(r[11]) - bp.x, F(r[12]) - bp.y, F(r[13]) - bp.z);
                aOff.Add(exp.x, off.Origin.x, TolOffset, cname + ".x"); aOff.Add(exp.y, off.Origin.y, TolOffset, cname + ".y"); aOff.Add(exp.z, off.Origin.z, TolOffset, cname + ".z");
                var expRot = Quat.FromEuler((float)(F(r[14]) * D2R), (float)(F(r[15]) * D2R), (float)(F(r[16]) * D2R));
                double qa = QuatAngle(expRot, off.Rotation);
                if (qa > TolQuatAngle) { offRotMis.Add($"{cname}(euler=({r[14]},{r[15]},{r[16]})deg, quatΔ={qa:G3}rad)"); }
            }
        }
        L($"  名前(順序)一致: {(nameMis == 0 ? "全一致" : nameMis + "件不一致")}");
        L($"  関連ボーン一致: {(boneMis == 0 ? "全一致" : boneMis + "件不一致")}");
        L($"  物理タイプ不一致={typeMis}  グループ不一致={groupMis}  形状種別不一致={shapeMis}");
        foreach (var a in new[] { aSize, aPos, aRot, aMass, aLd, aAd, aRe, aFr }) L(a.Line());
        L("\n---------- 6. 剛体⇔ボーン紐付け / BodyOffsetFromBone ----------");
        L(aOff.Line());
        L($"  BodyOffsetFromBone.Rotation 不一致={offRotMis.Count}" + (offRotMis.Count > 0 ? " : " + string.Join(",", offRotMis.Take(8)) : " (全一致)"));
    }

    // ---------- 2. 衝突フィルタ ----------
    static void CheckCollision(PmxPhysicsModel model, PmxPhysicsBuilder builder)
    {
        L("\n---------- 2. 衝突フィルタ (非衝突グループ文字列 → マスク / ShouldCollide) ----------");
        int n = model.RigidBodies.Count;
        var nonColl = new HashSet<int>[n];
        int maskMatchCollide = 0, maskMatchNonColl = 0;
        for (int i = 0; i < n; i++)
        {
            var set = ParseGroups(rowsBody[i][6]); nonColl[i] = set;
            ushort nonBits = 0; foreach (var g in set) if (g >= 0 && g < 16) nonBits |= (ushort)(1 << g);
            ushort collideBits = (ushort)(~nonBits & 0xFFFF);
            ushort eng = builder.Bodies[i].CollisionMask;
            if (eng == collideBits) maskMatchCollide++;
            if (eng == nonBits) maskMatchNonColl++;
        }
        L($"  CollisionMask が「衝突ビット(=非衝突の反転, 0-based)」と一致: {maskMatchCollide}/{n}");
        L($"  CollisionMask が「非衝突ビットそのまま(0-based)」と一致: {maskMatchNonColl}/{n}");
        // 診断: 先頭数件の生値
        L("  診断(先頭6): 剛体 group nonCollStr | engMask 期待衝突Mask 非衝突Mask");
        for (int i = 0; i < Math.Min(6, n); i++)
        {
            var set = nonColl[i]; ushort nonBits = 0; foreach (var g in set) if (g >= 0 && g < 16) nonBits |= (ushort)(1 << g);
            ushort collideBits = (ushort)(~nonBits & 0xFFFF);
            L($"    {Unq(rowsBody[i][1]),-14} g={rowsBody[i][5]} \"{rowsBody[i][6]}\" | eng=0x{builder.Bodies[i].CollisionMask:X4} 衝突=0x{collideBits:X4} 非衝突=0x{nonBits:X4}");
        }
        // 全ペア ShouldCollide 照合 (0-based と 1-based 両方)。
        for (int basis = 0; basis < 2; basis++)
        {
            var nc = new HashSet<int>[n];
            for (int i = 0; i < n; i++)
            {
                nc[i] = new HashSet<int>();
                foreach (var g in nonColl[i]) nc[i].Add(basis == 0 ? g : g - 1); // 1-based解釈は-1
            }
            long pairs = 0, mis = 0;
            for (int i = 0; i < n; i++)
                for (int k = i + 1; k < n; k++)
                {
                    int gi = builder.Bodies[i].Group, gk = builder.Bodies[k].Group;
                    bool expCollide = !nc[i].Contains(gk) && !nc[k].Contains(gi);
                    bool eng = PhysicsWorld.ShouldCollide(builder.Bodies[i], builder.Bodies[k]);
                    pairs++; if (expCollide != eng) mis++;
                }
            L($"  全ペア ShouldCollide 照合 ({(basis == 0 ? "0-based" : "1-based")}解釈): 総ペア={pairs} 不一致={mis} ({100.0 * mis / pairs:F2}%)");
        }
    }

    // ---------- 3. 慣性テンソル ----------
    static void CheckInertia(PmxPhysicsModel model, PmxPhysicsBuilder builder)
    {
        L("\n---------- 3. 慣性テンソル (CSV形状/サイズ/質量から独立算出, カプセル=外接箱近似) ----------");
        var a = new Agg("LocalInertiaDiag");
        int n = Math.Min(rowsBody.Count, model.RigidBodies.Count);
        for (int i = 0; i < n; i++)
        {
            var r = rowsBody[i]; var body = builder.Bodies[i];
            if (body.IsStaticOrKinematic) continue; // 質量0/kinematic は慣性0
            float m = F(r[17]); int shape = int.Parse(r[7]);
            float sx = F(r[8]), sy = F(r[9]), sz = F(r[10]);
            Vec3 exp;
            if (shape == 0) { float I = 0.4f * m * sx * sx; exp = new Vec3(I, I, I); }
            else if (shape == 1)
            {
                float lx = sx * 2, ly = sy * 2, lz = sz * 2, c = m / 12f;
                exp = new Vec3(c * (ly * ly + lz * lz), c * (lx * lx + lz * lz), c * (lx * lx + ly * ly));
            }
            else // カプセル: 外接箱近似 (r=sx, halfHeight=sy/2), upAxis=Y
            {
                float rr = sx, hy = rr + sy * 0.5f;
                float lx = 2 * rr, ly = 2 * hy, lz = 2 * rr, c = m / 12f;
                exp = new Vec3(c * (ly * ly + lz * lz), c * (lx * lx + lz * lz), c * (lx * lx + ly * ly));
            }
            var act = body.LocalInertiaDiag; string who = Unq(r[1]);
            a.Add(exp.x, act.x, Math.Max(TolInertiaRel * Math.Abs(exp.x), 1e-5), who + ".x");
            a.Add(exp.y, act.y, Math.Max(TolInertiaRel * Math.Abs(exp.y), 1e-5), who + ".y");
            a.Add(exp.z, act.z, Math.Max(TolInertiaRel * Math.Abs(exp.z), 1e-5), who + ".z");
        }
        L(a.Line());
    }

    // ---------- 4. Joint + 5. FrameInA/B ----------
    static void CheckJoints(PmxPhysicsModel model, PmxPhysicsBuilder builder, Dictionary<string, string[]> bodyCsv)
    {
        L("\n---------- 4. Joint (接続剛体/種別/限界/バネ) ----------");
        int nameMis = 0, abMis = 0, typeMis = 0;
        var aPos = new Agg("Joint位置"); var aRot = new Agg("Joint回転(deg→rad)");
        var aLinLo = new Agg("移動下限"); var aLinHi = new Agg("移動上限");
        var aAngLo = new Agg("回転下限(deg→rad)"); var aAngHi = new Agg("回転上限(deg→rad)");
        var aSprL = new Agg("バネ定数-移動"); var aSprA = new Agg("バネ定数-回転");
        var routeMis = new List<string>();
        var aFrameAO = new Agg("FrameInA.Origin"); var aFrameBO = new Agg("FrameInB.Origin");
        int frameARotMis = 0, frameBRotMis = 0;

        int n = Math.Min(rowsJoint.Count, model.Joints.Count);
        for (int i = 0; i < n; i++)
        {
            var r = rowsJoint[i]; var pj = model.Joints[i];
            string cname = Unq(r[1]);
            if (cname != pj.Name) { nameMis++; if (nameMis <= 8) L($"    [Joint名/順序不一致] idx{i}: CSV='{cname}' エンジン='{pj.Name}'"); }
            // 接続剛体 A/B (名前)
            string caA = Unq(r[3]), caB = Unq(r[4]);
            string eaA = (pj.RigidBodyAIndex >= 0 && pj.RigidBodyAIndex < model.RigidBodies.Count) ? model.RigidBodies[pj.RigidBodyAIndex].Name : "";
            string eaB = (pj.RigidBodyBIndex >= 0 && pj.RigidBodyBIndex < model.RigidBodies.Count) ? model.RigidBodies[pj.RigidBodyBIndex].Name : "";
            if (caA != eaA || caB != eaB) { abMis++; if (abMis <= 8) L($"    [接続剛体不一致] {cname}: CSV A/B='{caA}'/'{caB}' エンジン='{eaA}'/'{eaB}'"); }
            if (int.Parse(r[5]) != pj.JointType) typeMis++;

            aPos.Add(F(r[6]), pj.Position.x, TolPos, cname); aPos.Add(F(r[7]), pj.Position.y, TolPos, cname); aPos.Add(F(r[8]), pj.Position.z, TolPos, cname);
            aRot.Add(F(r[9]) * D2R, pj.Rotation.x, TolRot, cname); aRot.Add(F(r[10]) * D2R, pj.Rotation.y, TolRot, cname); aRot.Add(F(r[11]) * D2R, pj.Rotation.z, TolRot, cname);
            aLinLo.Add(F(r[12]), pj.LinearLowerLimit.x, TolLimit, cname); aLinLo.Add(F(r[13]), pj.LinearLowerLimit.y, TolLimit, cname); aLinLo.Add(F(r[14]), pj.LinearLowerLimit.z, TolLimit, cname);
            aLinHi.Add(F(r[15]), pj.LinearUpperLimit.x, TolLimit, cname); aLinHi.Add(F(r[16]), pj.LinearUpperLimit.y, TolLimit, cname); aLinHi.Add(F(r[17]), pj.LinearUpperLimit.z, TolLimit, cname);
            aAngLo.Add(F(r[18]) * D2R, pj.AngularLowerLimit.x, TolLimit, cname); aAngLo.Add(F(r[19]) * D2R, pj.AngularLowerLimit.y, TolLimit, cname); aAngLo.Add(F(r[20]) * D2R, pj.AngularLowerLimit.z, TolLimit, cname);
            aAngHi.Add(F(r[21]) * D2R, pj.AngularUpperLimit.x, TolLimit, cname); aAngHi.Add(F(r[22]) * D2R, pj.AngularUpperLimit.y, TolLimit, cname); aAngHi.Add(F(r[23]) * D2R, pj.AngularUpperLimit.z, TolLimit, cname);
            aSprL.Add(F(r[24]), pj.SpringLinear.x, TolSpring, cname); aSprL.Add(F(r[25]), pj.SpringLinear.y, TolSpring, cname); aSprL.Add(F(r[26]), pj.SpringLinear.z, TolSpring, cname);
            aSprA.Add(F(r[27]), pj.SpringAngular.x, TolSpring, cname); aSprA.Add(F(r[28]), pj.SpringAngular.y, TolSpring, cname); aSprA.Add(F(r[29]), pj.SpringAngular.z, TolSpring, cname);
        }
        L($"  Joint名(順序)一致: {(nameMis == 0 ? "全一致" : nameMis + "件不一致")}  接続剛体A/B一致: {(abMis == 0 ? "全一致" : abMis + "件不一致")}  種別不一致={typeMis}");
        foreach (var a in new[] { aPos, aRot, aLinLo, aLinHi, aAngLo, aAngHi, aSprL, aSprA }) L(a.Line());

        // 4b. FromPmx 種別ルーティングの検算 (最終 Joint 限界が種別ごとの仕様どおりか)
        L("\n---------- 4b. FromPmx 種別ルーティング検算 ----------");
        var typeCount = new Dictionary<int, int>();
        foreach (var j in builder.World.Joints)
        {
            var pj = model.Joints.First(p => p.Name == j.Name);
            int t = pj.JointType; typeCount[t] = typeCount.GetValueOrDefault(t) + 1;
            if (!RoutingOk((JointType)t, pj, j, out string why)) routeMis.Add($"{j.Name}({t}):{why}");
        }
        L("  種別別 Joint 数: " + string.Join(", ", typeCount.OrderBy(k => k.Key).Select(k => $"type{k.Key}={k.Value}")));
        L($"  ルーティング不一致={routeMis.Count}" + (routeMis.Count > 0 ? " ★\n      " + string.Join("\n      ", routeMis.Take(10)) : " (全て仕様どおり)"));

        // 5. FrameInA/B: CSVから独立再計算して照合
        L("\n---------- 5. FrameInA / FrameInB (CSVから独立再計算) ----------");
        foreach (var j in builder.World.Joints)
        {
            var r = rowsJoint.First(x => Unq(x[1]) == j.Name);
            var wf = RigidTransform.FromEuler(new Vec3(F(r[6]), F(r[7]), F(r[8])), new Vec3((float)(F(r[9]) * D2R), (float)(F(r[10]) * D2R), (float)(F(r[11]) * D2R)));
            var ra = bodyCsv[Unq(r[3])]; var rbb = bodyCsv[Unq(r[4])];
            var wa = RigidTransform.FromEuler(new Vec3(F(ra[11]), F(ra[12]), F(ra[13])), new Vec3((float)(F(ra[14]) * D2R), (float)(F(ra[15]) * D2R), (float)(F(ra[16]) * D2R)));
            var wb = RigidTransform.FromEuler(new Vec3(F(rbb[11]), F(rbb[12]), F(rbb[13])), new Vec3((float)(F(rbb[14]) * D2R), (float)(F(rbb[15]) * D2R), (float)(F(rbb[16]) * D2R)));
            var expA = wa.InverseTimes(wf); var expB = wb.InverseTimes(wf);
            aFrameAO.Add(expA.Origin.x, j.FrameInA.Origin.x, TolFrame, j.Name); aFrameAO.Add(expA.Origin.y, j.FrameInA.Origin.y, TolFrame, j.Name); aFrameAO.Add(expA.Origin.z, j.FrameInA.Origin.z, TolFrame, j.Name);
            aFrameBO.Add(expB.Origin.x, j.FrameInB.Origin.x, TolFrame, j.Name); aFrameBO.Add(expB.Origin.y, j.FrameInB.Origin.y, TolFrame, j.Name); aFrameBO.Add(expB.Origin.z, j.FrameInB.Origin.z, TolFrame, j.Name);
            double qaA = QuatAngle(expA.Rotation, j.FrameInA.Rotation), qaB = QuatAngle(expB.Rotation, j.FrameInB.Rotation);
            if (qaA > TolQuatAngle) { frameARotMis++; if (frameARotMis <= 6) L($"    [FrameInA.Rot] {j.Name}: Jrot=({r[9]},{r[10]},{r[11]})deg quatΔ={qaA:G3}rad"); }
            if (qaB > TolQuatAngle) { frameBRotMis++; if (frameBRotMis <= 6) L($"    [FrameInB.Rot] {j.Name}: Jrot=({r[9]},{r[10]},{r[11]})deg quatΔ={qaB:G3}rad"); }
        }
        L(aFrameAO.Line()); L(aFrameBO.Line());
        L($"  FrameInA.Rotation 不一致={frameARotMis}  FrameInB.Rotation 不一致={frameBRotMis}");
    }

    // FromPmx の種別ごとの限界書き換えが仕様どおりかを、raw PMX 値から検算。
    static bool RoutingOk(JointType t, PmxJoint pj, Joint j, out string why)
    {
        why = "";
        bool Eq(Vec3 a, Vec3 b) => (a - b).Length < 1e-4f;
        switch (t)
        {
            case JointType.Spring6Dof: // raw そのまま
                if (!Eq(j.AngularUpperLimit, pj.AngularUpperLimit)) { why = "angHi改変"; return false; }
                if (!Eq(j.LinearUpperLimit, pj.LinearUpperLimit)) { why = "linHi改変"; return false; }
                return true;
            case JointType.Generic6Dof: // バネのみ0化
                if (j.SpringLinear.Length > 1e-6f || j.SpringAngular.Length > 1e-6f) { why = "バネ非0"; return false; }
                return true;
            case JointType.Point2Point: // 角度フリー(lo>hi), 並進0固定, バネ0
                if (!(j.AngularLowerLimit.x > j.AngularUpperLimit.x)) { why = "角度非フリー"; return false; }
                if (!Eq(j.LinearLowerLimit, Vec3.Zero) || !Eq(j.LinearUpperLimit, Vec3.Zero)) { why = "並進非固定"; return false; }
                return true;
            case JointType.Hinge: // 並進0固定, 角度はX成分のみ
                if (!Eq(j.LinearLowerLimit, Vec3.Zero) || !Eq(j.LinearUpperLimit, Vec3.Zero)) { why = "並進非固定"; return false; }
                if (Math.Abs(j.AngularUpperLimit.y) > 1e-6f || Math.Abs(j.AngularUpperLimit.z) > 1e-6f) { why = "角度YZ非0"; return false; }
                return true;
            case JointType.Slider:
                if (Math.Abs(j.LinearUpperLimit.y) > 1e-6f || Math.Abs(j.LinearUpperLimit.z) > 1e-6f) { why = "並進YZ非0"; return false; }
                return true;
            case JointType.ConeTwist:
                if (!Eq(j.LinearLowerLimit, Vec3.Zero) || !Eq(j.LinearUpperLimit, Vec3.Zero)) { why = "並進非固定"; return false; }
                return true;
        }
        return true;
    }

    // ---------- CSV パース ----------
    static void ParseCsv(string path)
    {
        foreach (var line in File.ReadLines(path, Encoding.UTF8))
        {
            if (line.Length == 0 || line[0] == ';') continue;
            if (line.StartsWith("PmxBody,")) rowsBody.Add(Split(line));
            else if (line.StartsWith("PmxJoint,")) rowsJoint.Add(Split(line));
            else if (line.StartsWith("PmxBone,")) rowsBone.Add(Split(line));
        }
    }

    // 引用符対応の CSV 分割。
    static string[] Split(string line)
    {
        var outp = new List<string>(); var sb = new StringBuilder(); bool q = false;
        for (int i = 0; i < line.Length; i++)
        {
            char c = line[i];
            if (q) { if (c == '"') { if (i + 1 < line.Length && line[i + 1] == '"') { sb.Append('"'); i++; } else q = false; } else sb.Append(c); }
            else { if (c == '"') q = true; else if (c == ',') { outp.Add(sb.ToString()); sb.Clear(); } else sb.Append(c); }
        }
        outp.Add(sb.ToString());
        return outp.ToArray();
    }

    static string Unq(string s) => s;
    static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);
    static HashSet<int> ParseGroups(string s)
    {
        var set = new HashSet<int>();
        foreach (var t in s.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries))
            if (int.TryParse(t, out int g)) set.Add(g);
        return set;
    }
    static double QuatAngle(Quat a, Quat b)
    {
        float d = Math.Abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        return 2.0 * Math.Acos(Math.Min(1.0, d));
    }
}
