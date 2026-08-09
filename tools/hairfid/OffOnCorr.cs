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

        // ===== 直接検証(タスク2) 用の収集器 =====
        // 1) |ON位置-FK位置| 2) cos((ON-OFF),(FK-OFF)) 4) 回転: raw |ON-OFF| / keep(ON親+OFF相対) / clamp(ON親+clamp相対)
        var onFkPos = new List<float>[3] { new(), new(), new() };   // 0=全体 1=skirt 2=hair
        var reconErr = new List<float>[3] { new(), new(), new() };  // 1b) ON親チェーン再構成誤差
        var reconErrOff = new List<float>[3] { new(), new(), new() }; // 対照: OFF側の同誤差
        var dirCos = new List<float>[3] { new(), new(), new() };
        var rotRaw = new List<float>[3] { new(), new(), new() };
        var rotKeep = new List<float>[3] { new(), new(), new() };
        var rotClamp = new List<float>[3] { new(), new(), new() };
        // 剛体 -> (ボーン名, オフセット) 対応 (親のON姿勢の再構成に使用)
        var bodyInfo = new Dictionary<RigidBody, (string bone, RigidTransform off)>();
        foreach (var (link, bone, _) in physLinks) bodyInfo[link.Body] = (bone, link.BodyOffsetFromBone);
        foreach (var (link, bone) in colliderLinks) if (!bodyInfo.ContainsKey(link.Body)) bodyInfo[link.Body] = (bone, link.BodyOffsetFromBone);
        // 各物理剛体の「親側ジョイント」(BodyB=自分, BodyA=親でボーン既知)
        var parentJoint = new Dictionary<RigidBody, Joint>();
        foreach (var j in world.Joints)
            if (j.BodyB != null && j.BodyA != null && bodyInfo.ContainsKey(j.BodyA) && !parentJoint.ContainsKey(j.BodyB))
                parentJoint[j.BodyB] = j;
        Quat QuatAxis(int axis, float a)
        {
            float s = (float)Math.Sin(a * 0.5f), c = (float)Math.Cos(a * 0.5f);
            return axis == 0 ? new Quat(s, 0, 0, c) : axis == 1 ? new Quat(0, s, 0, c) : new Quat(0, 0, s, c);
        }

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

                // ===== 直接検証 =====
                // 1b) 親チェーン再構成: ON子位置 ≈ ON親位置 + qON親·(bind子-bind親) か
                //     (readme「移動分を捨てて回転のみフィードバック」の厳密解釈。ONデータのみで判定, 軌道分岐の交絡なし)
                {
                    int pi = link.BoneIndex < model.BoneParents.Count ? model.BoneParents[link.BoneIndex] : -1;
                    if (pi >= 0 && on.TryGet(f, model.BoneNames[pi], out var pOnW))
                    {
                        var bindRel = model.BonePositions[link.BoneIndex] - model.BonePositions[pi];
                        var recon = pOnW.Origin + pOnW.Rotation.Rotate(bindRel);
                        float dRecon = (onw.Origin - recon).Length;
                        reconErr[0].Add(dRecon); reconErr[grp].Add(dRecon);
                        // 対照: OFF側は満たさないはず
                        if (off.TryGet(f, model.BoneNames[pi], out var pOffW))
                        {
                            var reconOff = pOffW.Origin + pOffW.Rotation.Rotate(bindRel);
                            reconErrOff[0].Add((ofw.Origin - reconOff).Length); reconErrOff[grp].Add((ofw.Origin - reconOff).Length);
                        }
                    }
                }
                // 1) |ON位置 - FK位置|
                if (fkPos.TryGetValue(link.BoneIndex, out var fp2))
                {
                    float dOnFk = (onw.Origin - fp2).Length;
                    onFkPos[0].Add(dOnFk); onFkPos[grp].Add(dOnFk);
                    // 2) 方向余弦 cos((ON-OFF),(FK-OFF))
                    var u = onw.Origin - ofw.Origin; var v2 = fp2 - ofw.Origin;
                    float du = u.Length, dv = v2.Length;
                    if (du > 0.02f && dv > 0.02f)
                    {
                        float cs = u.Dot(v2) / (du * dv);
                        dirCos[0].Add(cs); dirCos[grp].Add(cs);
                    }
                }
                // 4) 回転: raw / keep(ON親+OFF相対) / clamp(ON親+clamp相対)
                rotRaw[0].Add(dRot); rotRaw[grp].Add(dRot);
                if (parentJoint.TryGetValue(link.Body, out var pj))
                {
                    var (pBone, pOff) = bodyInfo[pj.BodyA];
                    if (on.TryGet(f, pBone, out var pOn))
                    {
                        // OFF配置での相対euler (bodyはOFF姿勢に配置済み)
                        var wAoff = pj.BodyA.WorldTransform * pj.FrameInA;
                        var wBoff = pj.BodyB.WorldTransform * pj.FrameInB;
                        var eu2 = EulerXYZ((wAoff.Rotation.Conjugated() * wBoff.Rotation).Normalized);
                        // clamp into limits (free dof はそのまま)
                        var ec = eu2;
                        for (int d = 0; d < 3; d++)
                        {
                            float lo = pj.AngularLowerLimit[d], hi = pj.AngularUpperLimit[d];
                            if (lo > hi) continue;
                            if (ec[d] < lo) ec[d] = lo; else if (ec[d] > hi) ec[d] = hi;
                        }
                        // ON親のworldAフレーム回転
                        var qAon = ((pOn * pOff).Rotation * pj.FrameInA.Rotation).Normalized;
                        Quat Recon(Vec3 e)
                        {
                            var qRel2 = QuatAxis(0, e.x) * QuatAxis(1, e.y) * QuatAxis(2, e.z);
                            var bodyRot = (qAon * qRel2) * pj.FrameInB.Rotation.Conjugated();
                            return bodyRot * link.BodyOffsetFromBone.Rotation.Conjugated(); // bone rot
                        }
                        float keepErr = RelAngleDeg(Recon(eu2), onw.Rotation);
                        float clampErr = RelAngleDeg(Recon(ec), onw.Rotation);
                        rotKeep[0].Add(keepErr); rotKeep[grp].Add(keepErr);
                        rotClamp[0].Add(clampErr); rotClamp[grp].Add(clampErr);
                    }
                }
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
        // ===== 直接検証の集計 =====
        (float m, float p, float x) St(List<float> v) { if (v.Count == 0) return (0, 0, 0); v.Sort(); return (v[v.Count / 2], v[(int)(v.Count * 0.9)], v[^1]); }
        O.AppendLine("\n[直接検証1] |ON位置 - FK位置| (仮説「移動分を捨てる」なら ≈0):");
        for (int g = 0; g < 3; g++) { var (m, p, x) = St(onFkPos[g]); O.AppendLine($"   {gName[g],-5}: 中央={m:F4} p90={p:F4} 最大={x:F4} (n={onFkPos[g].Count})"); }
        O.AppendLine("[直接検証1b] 親チェーン再構成誤差 |ON子 - (ON親 + qON親·bindRel)| (回転のみフィードバック説なら ON≈0, OFF>0):");
        for (int g = 0; g < 3; g++)
        {
            var (m, p, x) = St(reconErr[g]); var (mo, po, _) = St(reconErrOff[g]);
            O.AppendLine($"   {gName[g],-5}: ON 中央={m:F4} p90={p:F4} 最大={x:F4}   OFF(対照) 中央={mo:F4} p90={po:F4}");
        }
        O.AppendLine("[直接検証2] cos((ON-OFF),(FK-OFF)) (FK方向への射影なら ≈1):");
        for (int g = 0; g < 3; g++) { var (m, p, x) = St(dirCos[g]); O.AppendLine($"   {gName[g],-5}: 中央={m:F4} p10={(dirCos[g].Count > 0 ? dirCos[g][(int)(dirCos[g].Count * 0.1)] : 0):F4} (n={dirCos[g].Count})"); }
        O.AppendLine("[直接検証4] 回転の再構成誤差(deg) raw=|ON-OFF| / keep=ON親+OFF相対 / clamp=ON親+clamp相対:");
        for (int g = 0; g < 3; g++)
        {
            var (rm, rp, _) = St(rotRaw[g]); var (km, kp, _) = St(rotKeep[g]); var (cm, cp2, _) = St(rotClamp[g]);
            O.AppendLine($"   {gName[g],-5}: raw 中央={rm:F1}/p90={rp:F1}   keep 中央={km:F1}/p90={kp:F1}   clamp 中央={cm:F1}/p90={cp2:F1}");
        }
        Console.Write(O.ToString());
        return 0;
    }
}
