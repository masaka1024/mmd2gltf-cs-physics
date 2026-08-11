// 検証(タスク2): PmxReader(PMX直読み) と GlbPhysicsReader(GLB extras.mmd経由) で構築した
// PmxPhysicsModel を全項目突き合わせ、さらに両経路で PhysicsWorld を300ステップ回して統計一致を見る。
//   許容誤差の根拠: extras.mmd の rigidBodies/joints は PMX raw を Python で JSON 出力したもの。
//     元は float32、json は double(float32値)で往復可逆に出力されるため、当方が double→float32 で
//     読み戻すと raw スカラ項目は基本 EXACT 一致するはず。ボーン位置だけは glTF ローカル並進が
//     unitScale(0.08)倍で格納され、当方が /0.08 で戻すため double の丸め(~1e-16)が入り、
//     float32 化で 0〜数 ULP の差が出うる。位置は最大~18 なので 1e-3 を意味差の閾値とする。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;

static class GlbVerify
{
    const float TolScalar = 1e-6f;  // raw スカラ(サイズ/質量/減衰/限界/バネ等): float32往復でほぼEXACT
    const float TolPos = 1e-3f;     // ボーン位置: unitScale往復の丸め(下記コメント)
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    class Agg
    {
        public string F; public int n, fail; public double maxDiff; public string ex = "";
        public Agg(string f) { F = f; }
        public void Add(double exp, double act, double tol, string who)
        { n++; double d = Math.Abs(exp - act); if (d > maxDiff) maxDiff = d; if (d > tol) { fail++; if (fail <= 6) ex += $"\n      {who}: PMX={exp:G7} GLB={act:G7} 差={d:G4}"; } }
        public string Line() => $"  {F,-24} 照合={n,4} 不一致={fail,3} 最大差={maxDiff:G4}" + (fail > 0 ? "  ★" + ex : "  一致");
    }

    static void V3(Agg a, Vec3 p, Vec3 q, float tol, string who) { a.Add(p.x, q.x, tol, who + ".x"); a.Add(p.y, q.y, tol, who + ".y"); a.Add(p.z, q.z, tol, who + ".z"); }

    static int Main()
    {
        string pmxPath = TestData.PmxPath();
        string glbPath = TestData.GlbPath();
        if (pmxPath == null || glbPath == null) { Console.WriteLine($"[SKIP] pmx={(pmxPath != null)} glb={(glbPath != null)} (testdata/modelA.pmx と modelA.glb か環境変数を指定)"); return 0; }

        var pm = PmxReader.LoadFile(pmxPath);
        var gm = GlbPhysicsReader.LoadFile(glbPath);

        L("==================== PMX直読み vs GLB(extras.mmd) 一致検証 ====================");
        L($"PMX: {pmxPath}");
        L($"GLB: {glbPath}");
        L($"\n[件数] 剛体 PMX={pm.RigidBodies.Count}/GLB={gm.RigidBodies.Count}  Joint PMX={pm.Joints.Count}/GLB={gm.Joints.Count}  ボーン PMX={pm.BoneNames.Count}/GLB={gm.BoneNames.Count}");
        bool countOk = pm.RigidBodies.Count == gm.RigidBodies.Count && pm.Joints.Count == gm.Joints.Count && pm.BoneNames.Count == gm.BoneNames.Count;
        L($"  => {(countOk ? "件数一致 (剛体117/Joint165/ボーン179 想定)" : "件数不一致(!)")}");
        if (!countOk) { Console.Write(O.ToString()); return 1; }

        // ---- 剛体 全項目 ----
        L("\n[剛体] 全項目");
        int nameMis = 0, boneMis = 0, grpMis = 0, ncMis = 0, shapeMis = 0, modeMis = 0;
        var aSize = new Agg("サイズ"); var aPos = new Agg("位置"); var aRot = new Agg("回転");
        var aMass = new Agg("質量"); var aLd = new Agg("移動減衰"); var aAd = new Agg("回転減衰"); var aRe = new Agg("反発"); var aFr = new Agg("摩擦");
        for (int i = 0; i < pm.RigidBodies.Count; i++)
        {
            var p = pm.RigidBodies[i]; var g = gm.RigidBodies[i];
            if (p.Name != g.Name) nameMis++;
            if (p.BoneIndex != g.BoneIndex) boneMis++;
            if (p.Group != g.Group) grpMis++;
            if (p.NonCollisionGroup != g.NonCollisionGroup) ncMis++;
            if (p.ShapeType != g.ShapeType) shapeMis++;
            if (p.PhysicsMode != g.PhysicsMode) modeMis++;
            V3(aSize, p.Size, g.Size, TolScalar, p.Name); V3(aPos, p.Position, g.Position, TolScalar, p.Name); V3(aRot, p.Rotation, g.Rotation, TolScalar, p.Name);
            aMass.Add(p.Mass, g.Mass, TolScalar, p.Name); aLd.Add(p.LinearDamping, g.LinearDamping, TolScalar, p.Name); aAd.Add(p.AngularDamping, g.AngularDamping, TolScalar, p.Name);
            aRe.Add(p.Restitution, g.Restitution, TolScalar, p.Name); aFr.Add(p.Friction, g.Friction, TolScalar, p.Name);
        }
        L($"  名前不一致={nameMis} 関連ボーン不一致={boneMis} グループ不一致={grpMis} 非衝突マスク不一致={ncMis} 形状不一致={shapeMis} タイプ不一致={modeMis}");
        foreach (var a in new[] { aSize, aPos, aRot, aMass, aLd, aAd, aRe, aFr }) L(a.Line());

        // ---- Joint 全項目 ----
        L("\n[Joint] 全項目");
        int jnMis = 0, jtMis = 0, abMis = 0;
        var jPos = new Agg("位置"); var jRot = new Agg("回転"); var jLl = new Agg("移動下限"); var jLu = new Agg("移動上限");
        var jAl = new Agg("回転下限"); var jAu = new Agg("回転上限"); var jSl = new Agg("バネ移動"); var jSa = new Agg("バネ回転");
        for (int i = 0; i < pm.Joints.Count; i++)
        {
            var p = pm.Joints[i]; var g = gm.Joints[i];
            if (p.Name != g.Name) jnMis++;
            if (p.JointType != g.JointType) jtMis++;
            if (p.RigidBodyAIndex != g.RigidBodyAIndex || p.RigidBodyBIndex != g.RigidBodyBIndex) abMis++;
            V3(jPos, p.Position, g.Position, TolScalar, p.Name); V3(jRot, p.Rotation, g.Rotation, TolScalar, p.Name);
            V3(jLl, p.LinearLowerLimit, g.LinearLowerLimit, TolScalar, p.Name); V3(jLu, p.LinearUpperLimit, g.LinearUpperLimit, TolScalar, p.Name);
            V3(jAl, p.AngularLowerLimit, g.AngularLowerLimit, TolScalar, p.Name); V3(jAu, p.AngularUpperLimit, g.AngularUpperLimit, TolScalar, p.Name);
            V3(jSl, p.SpringLinear, g.SpringLinear, TolScalar, p.Name); V3(jSa, p.SpringAngular, g.SpringAngular, TolScalar, p.Name);
        }
        L($"  名前不一致={jnMis} 種別不一致={jtMis} 接続剛体AB不一致={abMis}");
        foreach (var a in new[] { jPos, jRot, jLl, jLu, jAl, jAu, jSl, jSa }) L(a.Line());

        // ---- ボーン 名前/親/変形階層/位置 ----
        L("\n[ボーン] 名前/親/変形階層/位置");
        int bnMis = 0, bpMis = 0, blMis = 0;
        var bPos = new Agg("位置(階層法)");
        for (int i = 0; i < pm.BoneNames.Count; i++)
        {
            if (pm.BoneNames[i] != gm.BoneNames[i]) bnMis++;
            if (pm.BoneParents[i] != gm.BoneParents[i]) bpMis++;
            if (pm.BoneDeformLayers[i] != gm.BoneDeformLayers[i]) blMis++;
            V3(bPos, pm.BonePositions[i], gm.BonePositions[i], TolPos, pm.BoneNames[i]);
        }
        L($"  名前不一致={bnMis} 親不一致={bpMis} 変形階層不一致={blMis}");
        L(bPos.Line());

        // ---- ボーン位置の2経路検証 (階層法 vs ibm法) ----
        var ibm = GlbPhysicsReader.BonePositionsFromIbm(glbPath);
        var bIbm = new Agg("位置: 階層法 vs ibm法");
        for (int i = 0; i < gm.BonePositions.Count && i < ibm.Length; i++) V3(bIbm, gm.BonePositions[i], ibm[i], TolPos, gm.BoneNames[i]);
        var bIbmPmx = new Agg("位置: ibm法 vs PMX");
        for (int i = 0; i < pm.BonePositions.Count && i < ibm.Length; i++) V3(bIbmPmx, pm.BonePositions[i], ibm[i], TolPos, pm.BoneNames[i]);
        L("\n[ボーン位置 2経路検証]");
        L(bIbm.Line()); L(bIbmPmx.Line());

        // ---- 300ステップ 統計一致 ----
        L("\n[300ステップ 物理統計の一致] (重力-98既定, 実効1/60, PMX経路 vs GLB経路)");
        // 両経路で構築し300ステップ回して同一 index の動的剛体の最終姿勢を突き合わせ
        var bp = PmxPhysicsBuilder.Build(pm); var bg = PmxPhysicsBuilder.Build(gm);
        for (int s = 0; s < 300; s++) { bp.World.StepSimulation(bp.World.FixedTimeStep); bg.World.StepSimulation(bg.World.FixedTimeStep); }
        double maxPosDiff = 0, maxRotAcos = 0, maxQComp = 0; int cmp = 0;
        for (int i = 0; i < bp.Bodies.Count && i < bg.Bodies.Count; i++)
        {
            if (bp.Bodies[i].IsStaticOrKinematic) continue;
            var pp = bp.Bodies[i].WorldTransform; var gg = bg.Bodies[i].WorldTransform;
            maxPosDiff = Math.Max(maxPosDiff, (pp.Origin - gg.Origin).Length);
            float dot = Math.Abs(pp.Rotation.x * gg.Rotation.x + pp.Rotation.y * gg.Rotation.y + pp.Rotation.z * gg.Rotation.z + pp.Rotation.w * gg.Rotation.w);
            maxRotAcos = Math.Max(maxRotAcos, 2.0 * Math.Acos(Math.Min(1.0, dot)));
            // クォータニオン成分の直接差 (acos の下限ノイズと実差を区別する)
            maxQComp = Math.Max(maxQComp, Math.Max(Math.Abs(pp.Rotation.x - gg.Rotation.x), Math.Max(Math.Abs(pp.Rotation.y - gg.Rotation.y), Math.Max(Math.Abs(pp.Rotation.z - gg.Rotation.z), Math.Abs(pp.Rotation.w - gg.Rotation.w)))));
            cmp++;
        }
        L($"  動的剛体 {cmp} 個の300ステップ後 最終姿勢: 位置最大差={maxPosDiff:G4} 回転(acos)最大差={maxRotAcos:G4}rad 回転(成分直接)最大差={maxQComp:G4}");
        bool bitIdentical = maxPosDiff == 0.0 && maxQComp == 0.0;
        L($"  => {(bitIdentical ? "★完全ビット一致 (位置・回転成分とも差0。acos差は単位クォータニオン内積のfloat32下限)" : (maxPosDiff < 1e-4 && maxQComp < 1e-4 ? "実質一致 (差はfloat32丸め由来・意味差なし)" : "差あり(要調査)"))}");

        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "glbverify_out.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
