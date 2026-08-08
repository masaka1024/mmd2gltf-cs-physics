// 診断(調整なし): カプセル慣性の Bullet 準拠化がカプセル剛体(髪)に効き、
// 箱剛体(スカート)に効かないことを確認する。
//   - CSV収録ボーンを列挙し、髪ボーンが本家参照に存在するか確認(=自前/本家比較の可否)。
//   - 髪の縦Joint(子=カプセル/dynamic, 名前に"横"を含まない)を抽出し選定を報告。
//   - スカート(箱)と髪(カプセル)の傾き統計(平時/ターン窓)を「同じ物差し」で測る。
//   本体は不変。変更前/後の2回走らせ、髪が変化・スカートが不変であることを比較する。
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class HairCheck
{
    static readonly string PmxPath = TestData.PmxPath();
    const float DT = 1f / 30f;

    static StringBuilder O = new(); static void L(string s = "") => O.AppendLine(s);

    // 髪等の縦Joint。子=カプセル/dynamic, 名前に"横"を含まないものを縦鎖とみなす。
    struct ChainJoint { public string Name, ParentBone, ChildBone; public int ParentRb, ChildRb; }

    static int Main()
    {
        string csvPath = BoneCsv.FindPath();
        if (csvPath == null || !File.Exists(PmxPath)) { Console.WriteLine("[SKIP] no csv/pmx"); return 0; }
        var csv = BoneCsv.Load(csvPath);
        var model = PmxReader.LoadFile(PmxPath);
        int F = csv.FrameCount;

        // ---- CSV収録ボーン (本家参照が持つボーン) ----
        var csvBones = csv.BoneNames.OrderBy(x => x).ToList();
        int hairBonesInCsv = csvBones.Count(b => b.Contains("髪"));
        L("========== CSV(本家参照)の収録ボーン ==========");
        L($"  総数={csvBones.Count}  うち\"髪\"を含む={hairBonesInCsv}");
        L("  一覧: " + string.Join(", ", csvBones));

        // ---- カプセル/dynamic 剛体と 髪縦Joint の抽出 ----
        string BN(int i) => (i >= 0 && i < model.BoneNames.Count) ? model.BoneNames[i] : $"(#{i})";
        int capDyn = 0, capAll = 0;
        for (int i = 0; i < model.RigidBodies.Count; i++)
        {
            if (model.RigidBodies[i].ShapeType == 2) { capAll++; if (model.RigidBodies[i].PhysicsMode != 0) capDyn++; }
        }

        var hair = new List<ChainJoint>();
        var excludedYoko = new List<string>();
        foreach (var j in model.Joints)
        {
            int a = j.RigidBodyAIndex, b = j.RigidBodyBIndex;
            if (a < 0 || b < 0 || a >= model.RigidBodies.Count || b >= model.RigidBodies.Count) continue;
            var rbB = model.RigidBodies[b];
            if (rbB.ShapeType != 2) continue;      // 子がカプセルのJointのみ(髪など)
            if (rbB.PhysicsMode == 0) continue;    // 子がdynamicのみ
            if (j.Name.StartsWith("スカート")) continue; // スカートは箱なので通常来ないが念のため
            if (j.Name.Contains("横")) { excludedYoko.Add(j.Name); continue; } // 横リンクは傾きに使わない
            hair.Add(new ChainJoint
            {
                Name = j.Name, ParentBone = BN(model.RigidBodies[a].BoneIndex), ChildBone = BN(rbB.BoneIndex),
                ParentRb = a, ChildRb = b,
            });
        }
        L("\n========== 剛体形状と髪縦Jointの選定 ==========");
        L($"  カプセル剛体: 全{capAll}個 / うちdynamic {capDyn}個 (この変更が効く対象)");
        L($"  箱スカート剛体: 36個 (この変更が効かない対象)");
        L($"  髪縦Joint(子=カプセル/dynamic, 名前に\"横\"を含まない): {hair.Count}本");
        L($"  除外した\"横\"Joint(子カプセル): {excludedYoko.Count}本  例: {string.Join(", ", excludedYoko.Take(6))}");
        L("  選定した髪縦Joint(先頭16): 名前 [親ボーン→子ボーン]");
        foreach (var h in hair.Take(16)) L($"    {h.Name}  [{h.ParentBone}→{h.ChildBone}]");
        bool hairChildInCsv = hair.Any(h => csv.HasBone(h.ChildBone));
        L($"  → 髪の子ボーンがCSVに存在するか: {(hairChildInCsv ? "あり" : "なし")} " +
          $"({(hairChildInCsv ? "本家参照と比較可能" : "本家参照が無いため自前(SELF)値のみ。変更前/後の比較で効果を見る")})");

        // ---- 物理を走らせて 傾き統計 (スカート/髪 両方) ----
        var skirt = SkirtMeasure.ExtractVerticalJoints(model);
        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;

        // スカート⇔髪 が衝突しうるか (=髪の慣性変更がスカートへ波及する経路か) を静的に判定。
        var skirtRb = skirt.Select(s => s.ChildRb).Distinct().ToList();
        var hairRb = hair.Select(h => h.ChildRb).Distinct().ToList();
        long shCollide = 0, shTotal = 0;
        foreach (var si in skirtRb) foreach (var hi in hairRb)
        { shTotal++; if (PhysicsWorld.ShouldCollide(builder.Bodies[si], builder.Bodies[hi])) shCollide++; }
        L($"\n  スカート⇔髪 の衝突フィルタ判定: {shCollide}/{shTotal} ペアが衝突可 " +
          $"({(shCollide > 0 ? "接触経由で髪→スカートへ波及しうる" : "直接接触は無し")})");

        var driven = new List<(BoneLink link, string bone)>();
        foreach (var link in builder.BoneLinks)
            if (link.Mode == PhysicsMode.BoneFollow && link.BoneIndex >= 0 && link.BoneIndex < model.BoneNames.Count && csv.HasBone(model.BoneNames[link.BoneIndex]))
                driven.Add((link, model.BoneNames[link.BoneIndex]));
        void Apply(int f) { foreach (var (l, bn) in driven) if (csv.TryGet(f, bn, out var bw)) l.Body.KinematicTarget = bw * l.BodyOffsetFromBone; }
        Quat BoneRot(int rb) { var l = builder.BoneLinks[rb]; return (l.Body.WorldTransform * l.BodyOffsetFromBone.Inverse()).Rotation; }

        // ターン窓
        var yaw = new float[F];
        for (int f = 1; f < F; f++) if (csv.TryGet(f - 1, "下半身", out var q0) && csv.TryGet(f, "下半身", out var q1)) yaw[f] = SkirtMeasure.YawRateDeg(q0.Rotation, q1.Rotation, DT);
        var wins = SkirtMeasure.DetectTurnWindows(yaw, 360f);
        var inWin = new bool[F];
        foreach (var w in wins) for (int f = w.StartFrame; f <= Math.Min(F - 1, w.EndFrame + 30); f++) inWin[f] = true;

        Apply(0); for (int s = 0; s < 60; s++) world.StepSimulation(DT);

        var skCalm = new List<float>(); var hrCalm = new List<float>();
        var skWin = new List<float>(); var hrWin = new List<float>();
        // 窓ごとの frame-max-tilt ピーク (スカート/髪)
        var skWinPeak = new float[wins.Count]; var hrWinPeak = new float[wins.Count];

        for (int f = 0; f < F; f++)
        {
            Apply(f);
            world.StepSimulation(DT);
            bool win = inWin[f];
            float skMax = 0, hrMax = 0;
            foreach (var sj in skirt)
            {
                float t = SkirtMeasure.TiltDeg(BoneRot(sj.ParentRb), BoneRot(sj.ChildRb));
                if (win) skWin.Add(t); else skCalm.Add(t);
                if (t > skMax) skMax = t;
            }
            foreach (var h in hair)
            {
                float t = SkirtMeasure.TiltDeg(BoneRot(h.ParentRb), BoneRot(h.ChildRb));
                if (win) hrWin.Add(t); else hrCalm.Add(t);
                if (t > hrMax) hrMax = t;
            }
            for (int w = 0; w < wins.Count; w++)
                if (f >= wins[w].StartFrame && f <= Math.Min(F - 1, wins[w].EndFrame + 30))
                { if (skMax > skWinPeak[w]) skWinPeak[w] = skMax; if (hrMax > hrWinPeak[w]) hrWinPeak[w] = hrMax; }
        }

        (float m, float p90, float mx) St(List<float> a) => SkirtMeasure.Stats(a);
        var sc = St(skCalm); var hc = St(hrCalm); var sw2 = St(skWin); var hw = St(hrWin);
        L("\n========== 傾き統計 (ボーン空間, 同一物差し) ==========");
        L($"  [スカート/箱] 平時 中央={sc.m:F2} p90={sc.p90:F2} max={sc.mx:F2} | 窓内 中央={sw2.m:F2} p90={sw2.p90:F2} max={sw2.mx:F2}");
        L($"  [髪/カプセル] 平時 中央={hc.m:F2} p90={hc.p90:F2} max={hc.mx:F2} | 窓内 中央={hw.m:F2} p90={hw.p90:F2} max={hw.mx:F2}");
        L("\n  窓ごとの frame-max-tilt ピーク (スカート | 髪):");
        for (int w = 0; w < wins.Count; w++)
            L($"    窓{w + 1} F{wins[w].StartFrame}-{wins[w].EndFrame} peakYaw={wins[w].PeakYaw:F0}: スカート={skWinPeak[w]:F2}  髪={hrWinPeak[w]:F2}");

        string tag = Environment.GetEnvironmentVariable("HAIR_TAG") ?? "run";
        File.WriteAllText(Path.Combine(AppContext.BaseDirectory, $"haircheck_{tag}.txt"), O.ToString(), new UTF8Encoding(false));
        Console.Write(O.ToString());
        return 0;
    }
}
