// ===========================================================================
// スカート傾き/ターンの計測 (Python側と同じ物差しを狙う)。
// 傾き=スイングのみ: rel=parent^-1*child, tilt = rel*up と up(+Y)のなす角。
//   ヨー(up軸まわりの捻り)は rel*up を変えないので自動的に除外される。
// MMD参照 (CSV) も自前物理も「同じ関数」でここを通す。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace BoneCheck
{
    // 1本の縦スカートJoint (取付ジョイント)。親=体側/上段, 子=下段。
    public struct SkirtJoint
    {
        public int Ring;            // 子ボーン スカート_R_C の R
        public int Col;             // C
        public string JointName;
        public string ParentBone, ChildBone;
        public int ParentRb, ChildRb;   // 剛体index (物理側で使用)
    }

    public static class SkirtMeasure
    {
        // PMX の 縦スカートJoint (スカート_R_C, 横は除く) を抽出。
        public static List<SkirtJoint> ExtractVerticalJoints(PmxPhysicsModel m)
        {
            var list = new List<SkirtJoint>();
            foreach (var j in m.Joints)
            {
                if (!j.Name.StartsWith("スカート")) continue;
                if (j.Name.Contains("横")) continue;       // 横リンクは傾きに使わない
                int a = j.RigidBodyAIndex, b = j.RigidBodyBIndex;
                if (a < 0 || b < 0 || a >= m.RigidBodies.Count || b >= m.RigidBodies.Count) continue;
                string childBone = BoneName(m, m.RigidBodies[b].BoneIndex);
                if (!TryParseRingCol(childBone, out int ring, out int col)) continue;
                list.Add(new SkirtJoint
                {
                    Ring = ring, Col = col, JointName = j.Name,
                    ParentBone = BoneName(m, m.RigidBodies[a].BoneIndex),
                    ChildBone = childBone,
                    ParentRb = a, ChildRb = b,
                });
            }
            return list;
        }

        private static bool TryParseRingCol(string bone, out int ring, out int col)
        {
            ring = col = -1;
            // "スカート_R_C"
            var parts = bone.Split('_');
            if (parts.Length != 3) return false;
            return int.TryParse(parts[1], out ring) && int.TryParse(parts[2], out col);
        }

        private static string BoneName(PmxPhysicsModel m, int i) =>
            (i >= 0 && i < m.BoneNames.Count) ? m.BoneNames[i] : $"(#{i})";

        // --- 傾き (スイングのみ), 度 ---
        public static float TiltDeg(Quat parentRot, Quat childRot)
        {
            var rel = parentRot.Conjugated() * childRot;       // parent^-1 * child
            var tiltedUp = (rel.Normalized) * Vec3.YAxis;      // 子の up を親フレームで見た向き
            float cy = Math.Clamp(tiltedUp.y, -1f, 1f);        // up(+Y)との内積 = cos
            return (float)(Math.Acos(cy) * 180.0 / Math.PI);
        }

        // swing-twist 分解: q = swing * twist (twist は axis まわりの回転)。twist角(度, 符号付き)を返す。
        // qv を axis へ射影して twist クォータニオンを作る標準手法。
        // 特異点: swing≈180° (qのwと軸射影が共に≈0) では twist が未定義 → 0 を返す。
        public static float TwistAngleDeg(Quat q, Vec3 axis)
        {
            q = q.Normalized;
            if (q.w < 0f) q = new Quat(-q.x, -q.y, -q.z, -q.w); // 正の半球へ正規化 (double cover)
            var qv = new Vec3(q.x, q.y, q.z);
            float d = qv.Dot(axis);        // sin(θ/2) 相当 (axis まわり成分)
            float tw = q.w;                // cos(θ/2) 相当
            if (Math.Sqrt(d * d + tw * tw) < 1e-6f) return 0f; // 180°swing 特異点
            float ang = 2f * (float)Math.Atan2(d, tw);         // tw>=0 なので [-π, π]
            return (float)(ang * 180.0 / Math.PI);
        }

        // 取付相対のヨー角(度): rel=parent^-1*child の up(+Y)まわり twist (共回転遅れ)。
        public static float YawOfRelDeg(Quat parentRot, Quat childRot)
        {
            var rel = (parentRot.Conjugated() * childRot).Normalized;
            return TwistAngleDeg(rel, Vec3.YAxis);
        }

        // --- ヨー角速度 (度/秒): 世界Y軸まわりの瞬間角速度 (フレーム間差分) ---
        public static float YawRateDeg(Quat prev, Quat cur, float dt)
        {
            var dq = (cur * prev.Conjugated()).Normalized;     // 世界相対回転
            if (dq.w < 0f) dq = new Quat(-dq.x, -dq.y, -dq.z, -dq.w); // 最短
            float angle = 2f * (float)Math.Acos(Math.Clamp(dq.w, -1f, 1f)); // [0,π]
            var axis = new Vec3(dq.x, dq.y, dq.z);
            float len = axis.Length;
            if (len < 1e-9f) return 0f;
            axis /= len;
            float omegaY = axis.y * (angle / dt);              // rad/s (世界Y成分)
            return (float)(omegaY * 180.0 / Math.PI);
        }

        // --- 統計 (numpy 既定と同じ線形補間パーセンタイル) ---
        public static float Percentile(float[] sortedAsc, double p)
        {
            int n = sortedAsc.Length;
            if (n == 0) return float.NaN;
            if (n == 1) return sortedAsc[0];
            double rank = (p / 100.0) * (n - 1);
            int lo = (int)Math.Floor(rank), hi = (int)Math.Ceiling(rank);
            double frac = rank - lo;
            return (float)(sortedAsc[lo] * (1 - frac) + sortedAsc[hi] * frac);
        }

        public static (float med, float p90, float max) Stats(IEnumerable<float> vals)
        {
            var a = vals.ToArray();
            if (a.Length == 0) return (float.NaN, float.NaN, float.NaN);
            Array.Sort(a);
            return (Percentile(a, 50), Percentile(a, 90), a[a.Length - 1]);
        }

        // ターン窓: |yawRate| > threshold の連続フレームを1窓とする。
        public struct TurnWindow { public int StartFrame, EndFrame; public float PeakYaw; }

        public static List<TurnWindow> DetectTurnWindows(float[] yawRate, float thresholdDegPerSec)
        {
            var wins = new List<TurnWindow>();
            int i = 0; int n = yawRate.Length;
            while (i < n)
            {
                if (Math.Abs(yawRate[i]) > thresholdDegPerSec)
                {
                    int s = i; float peak = 0f;
                    while (i < n && Math.Abs(yawRate[i]) > thresholdDegPerSec)
                    {
                        if (Math.Abs(yawRate[i]) > Math.Abs(peak)) peak = yawRate[i];
                        i++;
                    }
                    wins.Add(new TurnWindow { StartFrame = s, EndFrame = i - 1, PeakYaw = peak });
                }
                else i++;
            }
            return wins;
        }
    }
}
