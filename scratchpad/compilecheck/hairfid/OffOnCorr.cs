// ===========================================================================
// 補正層のデータ考古学 (タスク1: 相関表)。
// OFF版(純Bullet=補正前)とON版(補正後)は同一入力のペア。フレーム×物理ボーンごとの
// OFF→ON 差分(位置/回転)を応答変数に、OFF配置で計算した説明変数
//   (a)角度リミット超過量 (b)体との貫入量 (c)アンカー誤差 (d)FK位置との距離
// との Pearson 相関を出す。差分が何と強く相関するかで補正の正体(射影の種類)を読む。
// 実行: hairfid で env CORR=1 MMD_TEST_HAIRCSV=<OFF108csv> CORR_ON_CSV=<ON108csv>
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletPhysics;
using BulletPhysics.Pmx;
using BoneCheck;

static class OffOnCorr
{
    // Pearson 累積器
    sealed class Corr
    {
        double sx, sy, sxx, syy, sxy; long n;
        public void Add(double x, double y) { sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; n++; }
        public double R
        {
            get
            {
                if (n < 2) return 0;
                double cov = sxy - sx * sy / n;
                double vx = sxx - sx * sx / n, vy = syy - sy * sy / n;
                return (vx <= 0 || vy <= 0) ? 0 : cov / Math.Sqrt(vx * vy);
            }
        }
        public long N => n;
    }

    static float RelAngleDeg(Quat a, Quat b)
    {
        float cx = -a.x, cy = -a.y, cz = -a.z, cw = a.w;
        float dx = b.w * cx + b.x * cw + b.y * cz - b.z * cy;
        float dy = b.w * cy - b.x * cz + b.y * cw + b.z * cx;
        float dz = b.w * cz + b.x * cy - b.y * cx + b.z * cw;
        float dw = b.w * cw - b.x * cx - b.y * cy - b.z * cz;
        float s = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return 2f * (float)Math.Atan2(s, Math.Abs(dw)) * 57.29578f;
    }

    static Vec3 EulerXYZ(Quat q)
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

    public static int Run(PmxPhysicsModel model, PmxPhysicsBuilder builder,
        List<(BoneLink link, string bone, RigidTransform bindBone)> physLinks,
        List<(BoneLink link, string bone)> colliderLinks,
        List<(BoneLink link, string bone)> driven,
        BoneCsv off, BoneCsv on)
    {
        var world = builder.World;
        int F = Math.Min(off.FrameCount, on.FrameCount);
        // ボーン→隣接ジョイント (BodyA/B のどちらかがそのボーンの剛体)
        var bodyToJoints = new Dictionary<RigidBody, List<Joint>>();
        foreach (var j in world.Joints)
        {
            if (j.BodyA != null) { if (!bodyToJoints.TryGetValue(j.BodyA, out var l)) bodyToJoints[j.BodyA] = l = new(); l.Add(j); }
            if (j.BodyB != null) { if (!bodyToJoints.TryGetValue(j.BodyB, out var l)) bodyToJoints[j.BodyB] = l = new(); l.Add(j); }
        }
        // 相関: 応答 dPos / dRot × 説明 {overshoot, pen, anchorErr, fkDist} × 区分 {全体, skirt, hair}
        string[] varName = { "超過量(deg)", "貫入量", "アンカー誤差", "FK距離" };
        var cPos = new Corr[3, 4]; var cRot = new Corr[3, 4];
        for (int g = 0; g < 3; g++) for (int v = 0; v < 4; v++) { cPos[g, v] = new(); cRot[g, v] = new(); }
        var dPosAll = new List<float>(); var dRotAll = new List<float>();
        var buf = new List<ContactPoint>();
        bool IsSkirt(string n) => n != null && n.Contains("スカート");

        var fkPos = new Dictionary<int, Vec3>(); // boneIndex -> FK bone pos (このフレーム)
        for (int f = 0; f < F; f++)
        {
            // 1) FK-rest 配置 → FK ボーン位置をスナップショット (駆動は OFF=ON 同一)
            builder.ResetBodiesToBonePoseFk(i =>
                (i >= 0 && i < model.BoneNames.Count && off.TryGet(f, model.BoneNames[i], out var bw)) ? (RigidTransform?)bw : null);
            fkPos.Clear();
            foreach (var (link, bone, _) in physLinks)
                fkPos[link.BoneIndex] = (link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse()).Origin;

            // 2) OFF 姿勢へ全剛体を配置 (補正前の実配置)
            foreach (var (link, bone) in colliderLinks) if (off.TryGet(f, bone, out var bw)) { link.Body.WorldTransform = bw * link.BodyOffsetFromBone; link.Body.UpdateInertiaWorld(); }
            foreach (var (link, bone, _) in physLinks) if (off.TryGet(f, bone, out var bw)) { link.Body.WorldTransform = bw * link.BodyOffsetFromBone; link.Body.UpdateInertiaWorld(); }

            // 3) 各物理ボーン: 説明変数(OFF配置) + 応答(OFF→ON差分)
            foreach (var (link, bone, _) in physLinks)
            {
                if (!off.TryGet(f, bone, out var ofw) || !on.TryGet(f, bone, out var onw)) continue;
                float dPos = (onw.Origin - ofw.Origin).Length;
                float dRot = RelAngleDeg(ofw.Rotation, onw.Rotation);

                // (a) 隣接ジョイントの角度超過量 max (deg)
                float overshoot = 0f; float anchorErr = 0f;
                if (bodyToJoints.TryGetValue(link.Body, out var js))
                    foreach (var j in js)
                    {
                        var wA = j.BodyA.WorldTransform * j.FrameInA;
                        var wB = j.BodyB.WorldTransform * j.FrameInB;
                        anchorErr = Math.Max(anchorErr, (wA.Origin - wB.Origin).Length);
                        var eu = EulerXYZ((wA.Rotation.Conjugated() * wB.Rotation).Normalized);
                        for (int d = 0; d < 3; d++)
                        {
                            float lo = j.AngularLowerLimit[d], hi = j.AngularUpperLimit[d];
                            if (lo > hi) continue;
                            float ov = eu[d] < lo ? lo - eu[d] : eu[d] > hi ? eu[d] - hi : 0f;
                            overshoot = Math.Max(overshoot, ov * 57.29578f);
                        }
                    }
                // (b) 体コライダーとの貫入 max (ShouldCollide ゲート)
                float pen = 0f;
                foreach (var (cl, _) in colliderLinks)
                {
                    if (!PhysicsWorld.ShouldCollide(link.Body, cl.Body)) continue;
                    var aa = link.Body.ComputeAabb(); var bb = cl.Body.ComputeAabb();
                    if (!aa.Intersects(ref bb)) continue;
                    buf.Clear(); GjkEpa.Detect(link.Body, cl.Body, buf);
                    foreach (var cp in buf) pen = Math.Max(pen, -cp.Distance);
                }
                // (d) FK距離
                float fkD = fkPos.TryGetValue(link.BoneIndex, out var fp) ? (ofw.Origin - fp).Length : 0f;

                int grp = IsSkirt(bone) ? 1 : 2; // 1=skirt 2=hair
                float[] xs = { overshoot, pen, anchorErr, fkD };
                for (int v = 0; v < 4; v++)
                {
                    cPos[0, v].Add(xs[v], dPos); cRot[0, v].Add(xs[v], dRot);
                    cPos[grp, v].Add(xs[v], dPos); cRot[grp, v].Add(xs[v], dRot);
                }
                dPosAll.Add(dPos); dRotAll.Add(dRot);
            }
        }

        var O = new StringBuilder();
        O.AppendLine($"[補正層 相関分析] OFF(補正前)→ON(補正後) 差分 vs OFF配置の説明変数  F={F} サンプル={cPos[0, 0].N}");
        dPosAll.Sort(); dRotAll.Sort();
        O.AppendLine($"[補正量の分布] 位置|ON-OFF| 中央={dPosAll[dPosAll.Count / 2]:F3} p90={dPosAll[(int)(dPosAll.Count * 0.9)]:F3} 最大={dPosAll[^1]:F3}   回転 中央={dRotAll[dRotAll.Count / 2]:F1}° p90={dRotAll[(int)(dRotAll.Count * 0.9)]:F1}° 最大={dRotAll[^1]:F1}°");
        string[] gName = { "全体", "skirt", "hair" };
        for (int g = 0; g < 3; g++)
        {
            O.Append($"   [{gName[g],-5}] 位置差との r: ");
            for (int v = 0; v < 4; v++) O.Append($"{varName[v]}={cPos[g, v].R:F3}  ");
            O.AppendLine();
            O.Append($"   [{gName[g],-5}] 回転差との r: ");
            for (int v = 0; v < 4; v++) O.Append($"{varName[v]}={cRot[g, v].R:F3}  ");
            O.AppendLine();
        }
        Console.Write(O.ToString());
        return 0;
    }
}
