// ===========================================================================
// Unity Bullet 互換物理エンジン – Narrowphase (GJK + EPA)
// 凸形状同士の接触判定・貫入量・法線・接触点を算出する。
// Bullet の btGjkPairDetector + btGjkEpaPenetrationDepthSolver 相当。
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    /// <summary>1 つの接触点。ワールド座標系。</summary>
    public struct ContactPoint
    {
        public Vec3 PositionWorldA;   // A 上の接触点
        public Vec3 PositionWorldB;   // B 上の接触点
        public Vec3 Normal;           // B から A へ向かう単位法線
        public float Distance;        // 負値 = 貫入量

        // ソルバ用の蓄積インパルス (ウォームスタート)。
        public float NormalImpulse;
        public float TangentImpulse1;
        public float TangentImpulse2;
    }

    /// <summary>剛体ペアの接触点集合 (最大 4 点)。フレーム間で保持する。</summary>
    public sealed class PersistentManifold
    {
        public RigidBody BodyA;
        public RigidBody BodyB;
        public readonly List<ContactPoint> Points = new(4);

        public PersistentManifold(RigidBody a, RigidBody b) { BodyA = a; BodyB = b; }

        public void Refresh()
        {
            // 姿勢更新後に古い接触点が離れていれば破棄。
            for (int i = Points.Count - 1; i >= 0; i--)
            {
                var cp = Points[i];
                var an = BodyA.WorldTransform.TransformPoint(
                    BodyA.WorldTransform.InverseTransformPoint(cp.PositionWorldA));
                // 単純に法線方向の距離と横ずれで判定。
                var d = (cp.PositionWorldA - cp.PositionWorldB).Dot(cp.Normal);
                var lateral = (cp.PositionWorldA - cp.PositionWorldB) - cp.Normal * d;
                if (d > 0.04f || lateral.LengthSquared > 0.04f * 0.04f)
                    Points.RemoveAt(i);
            }
        }

        public void AddPoint(ContactPoint cp)
        {
            // 近い既存点があればウォームスタート値を引き継いで置換。
            const float mergeDist2 = 0.02f * 0.02f;
            for (int i = 0; i < Points.Count; i++)
            {
                if ((Points[i].PositionWorldA - cp.PositionWorldA).LengthSquared < mergeDist2)
                {
                    cp.NormalImpulse = Points[i].NormalImpulse;
                    cp.TangentImpulse1 = Points[i].TangentImpulse1;
                    cp.TangentImpulse2 = Points[i].TangentImpulse2;
                    Points[i] = cp;
                    return;
                }
            }
            if (Points.Count < 4) Points.Add(cp);
            else Points[WorstPointIndex(cp)] = cp;
        }

        private int WorstPointIndex(ContactPoint candidate)
        {
            // 最も浅い点を置換候補にする簡易版。
            int worst = 0; float maxDist = Points[0].Distance;
            for (int i = 1; i < Points.Count; i++)
                if (Points[i].Distance > maxDist) { maxDist = Points[i].Distance; worst = i; }
            return worst;
        }
    }

    /// <summary>GJK による距離判定と EPA による貫入解決。</summary>
    public static class GjkEpa
    {
        private const int MaxIterations = 32;
        private const float Epsilon = 1e-7f;

        private struct SupportVert
        {
            public Vec3 V;   // Minkowski 差の点
            public Vec3 A;   // A 上の support
            public Vec3 B;   // B 上の support
        }

        private static Vec3 WorldSupport(RigidBody body, Vec3 dirWorld, out Vec3 witness)
        {
            var localDir = body.WorldTransform.InverseTransformDirection(dirWorld);
            var localP = body.Shape.LocalSupportWithMargin(localDir);
            witness = body.WorldTransform.TransformPoint(localP);
            return witness;
        }

        private static SupportVert Support(RigidBody a, RigidBody b, Vec3 dir)
        {
            var pa = WorldSupport(a, dir, out var wa);
            var pb = WorldSupport(b, -dir, out var wb);
            return new SupportVert { V = pa - pb, A = wa, B = wb };
        }

        /// <summary>
        /// A と B の接触を判定。接触があれば true と接触点を返す。
        /// </summary>
        public static bool Detect(RigidBody a, RigidBody b, out ContactPoint contact)
        {
            contact = default;

            // --- GJK: 原点が Minkowski 差に含まれるか ---
            var simplex = new List<SupportVert>(4);
            var dir = a.WorldTransform.Origin - b.WorldTransform.Origin;
            if (dir.LengthSquared < Epsilon) dir = Vec3.XAxis;

            simplex.Add(Support(a, b, dir));
            dir = -simplex[0].V;

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                if (dir.LengthSquared < Epsilon) break;
                var p = Support(a, b, dir);
                if (p.V.Dot(dir) < 0)
                    return false; // 原点を越えられない → 分離
                simplex.Add(p);
                if (DoSimplex(simplex, ref dir))
                {
                    // 原点包含 → EPA で貫入解決。
                    return Epa(a, b, simplex, out contact);
                }
            }
            return false;
        }

        // GJK simplex 更新。原点を含んだら true。
        private static bool DoSimplex(List<SupportVert> s, ref Vec3 dir)
        {
            if (s.Count == 2) return Line(s, ref dir);
            if (s.Count == 3) return Triangle(s, ref dir);
            return Tetrahedron(s, ref dir);
        }

        private static bool Line(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[1].V; var b = s[0].V;
            var ab = b - a; var ao = -a;
            if (ab.Dot(ao) > 0)
                dir = Vec3.Cross(Vec3.Cross(ab, ao), ab);
            else { s.RemoveAt(0); dir = ao; }
            return false;
        }

        private static bool Triangle(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[2].V; var b = s[1].V; var c = s[0].V;
            var ab = b - a; var ac = c - a; var ao = -a;
            var abc = Vec3.Cross(ab, ac);

            if (Vec3.Cross(abc, ac).Dot(ao) > 0)
            {
                if (ac.Dot(ao) > 0) { s.RemoveAt(1); dir = Vec3.Cross(Vec3.Cross(ac, ao), ac); }
                else return StarLine(s, ref dir);
            }
            else if (Vec3.Cross(ab, abc).Dot(ao) > 0)
            {
                return StarLine(s, ref dir);
            }
            else
            {
                if (abc.Dot(ao) > 0) { dir = abc; }
                else { (s[0], s[1]) = (s[1], s[0]); dir = -abc; }
                return false;
            }
            return false;
        }

        private static bool StarLine(List<SupportVert> s, ref Vec3 dir)
        {
            // 辺 AB を残す。
            s.RemoveAt(0);
            return Line(s, ref dir);
        }

        private static bool Tetrahedron(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[3].V; var b = s[2].V; var c = s[1].V; var d = s[0].V;
            var ao = -a;
            var abc = Vec3.Cross(b - a, c - a);
            var acd = Vec3.Cross(c - a, d - a);
            var adb = Vec3.Cross(d - a, b - a);

            if (abc.Dot(ao) > 0) { s.RemoveAt(0); dir = abc; return Triangle(s, ref dir); }
            if (acd.Dot(ao) > 0) { s.RemoveAt(2); dir = acd; return Triangle(s, ref dir); }
            if (adb.Dot(ao) > 0) { s.RemoveAt(1); dir = adb; return Triangle(s, ref dir); }
            return true; // 原点は四面体内部
        }

        // --- EPA: 貫入方向と深さ ---
        private struct Face { public int A, B, C; public Vec3 Normal; public float Dist; }

        private static bool Epa(RigidBody a, RigidBody b, List<SupportVert> simplex, out ContactPoint contact)
        {
            contact = default;
            if (simplex.Count < 4) { if (!ExpandToTetra(a, b, simplex)) return false; }

            var verts = new List<SupportVert>(simplex);
            var faces = new List<Face>
            {
                MakeFace(verts, 0, 1, 2),
                MakeFace(verts, 0, 2, 3),
                MakeFace(verts, 0, 3, 1),
                MakeFace(verts, 1, 3, 2),
            };

            Face closest = default;
            for (int iter = 0; iter < MaxIterations; iter++)
            {
                closest = FindClosestFace(faces);
                var p = Support(a, b, closest.Normal);
                var d = p.V.Dot(closest.Normal);
                if (d - closest.Dist < 1e-4f)
                    break;
                // 面を p から見える範囲で削除し再構築。
                var edges = new List<(int, int)>();
                for (int i = faces.Count - 1; i >= 0; i--)
                {
                    if (faces[i].Normal.Dot(p.V - verts[faces[i].A].V) > 0)
                    {
                        AddEdge(edges, faces[i].A, faces[i].B);
                        AddEdge(edges, faces[i].B, faces[i].C);
                        AddEdge(edges, faces[i].C, faces[i].A);
                        faces.RemoveAt(i);
                    }
                }
                int newIndex = verts.Count;
                verts.Add(p);
                foreach (var (e0, e1) in edges)
                    faces.Add(MakeFace(verts, e0, e1, newIndex));
                if (faces.Count == 0) return false;
            }

            // 接触点を重心座標で復元。
            var normal = closest.Normal;
            var depth = closest.Dist;
            BarycentricProject(verts, closest, out var baryA, out var baryB);

            contact.Normal = normal;
            contact.Distance = -depth; // 貫入は負
            contact.PositionWorldA = baryA;
            contact.PositionWorldB = baryB;
            return true;
        }

        private static bool ExpandToTetra(RigidBody a, RigidBody b, List<SupportVert> s)
        {
            // 退化 simplex を四面体まで補う (稀ケースの保険)。
            var dirs = new[] { Vec3.XAxis, Vec3.YAxis, Vec3.ZAxis, -Vec3.XAxis, -Vec3.YAxis, -Vec3.ZAxis };
            foreach (var d in dirs)
            {
                if (s.Count >= 4) break;
                var p = Support(a, b, d);
                bool dup = false;
                foreach (var e in s) if ((e.V - p.V).LengthSquared < 1e-8f) { dup = true; break; }
                if (!dup) s.Add(p);
            }
            return s.Count >= 4;
        }

        private static Face MakeFace(List<SupportVert> v, int a, int b, int c)
        {
            var n = Vec3.Cross(v[b].V - v[a].V, v[c].V - v[a].V);
            var len = n.Length;
            n = len > Epsilon ? n / len : Vec3.YAxis;
            var dist = n.Dot(v[a].V);
            if (dist < 0) { n = -n; dist = -dist; (b, c) = (c, b); }
            return new Face { A = a, B = b, C = c, Normal = n, Dist = dist };
        }

        private static Face FindClosestFace(List<Face> faces)
        {
            var best = faces[0];
            for (int i = 1; i < faces.Count; i++)
                if (faces[i].Dist < best.Dist) best = faces[i];
            return best;
        }

        private static void AddEdge(List<(int, int)> edges, int a, int b)
        {
            // 反対向きの辺があれば相殺 (穴の輪郭抽出)。
            for (int i = 0; i < edges.Count; i++)
                if (edges[i].Item1 == b && edges[i].Item2 == a) { edges.RemoveAt(i); return; }
            edges.Add((a, b));
        }

        private static void BarycentricProject(List<SupportVert> v, Face f, out Vec3 onA, out Vec3 onB)
        {
            // 原点を面へ射影し、重心座標で A/B 上の witness を補間。
            var pa = v[f.A]; var pb = v[f.B]; var pc = v[f.C];
            var proj = f.Normal * f.Dist;
            Barycentric(proj, pa.V, pb.V, pc.V, out var u, out var vv, out var w);
            onA = pa.A * u + pb.A * vv + pc.A * w;
            onB = pa.B * u + pb.B * vv + pc.B * w;
        }

        private static void Barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c,
            out float u, out float v, out float w)
        {
            var v0 = b - a; var v1 = c - a; var v2 = p - a;
            var d00 = v0.Dot(v0); var d01 = v0.Dot(v1); var d11 = v1.Dot(v1);
            var d20 = v2.Dot(v0); var d21 = v2.Dot(v1);
            var denom = d00 * d11 - d01 * d01;
            if (Math.Abs(denom) < Epsilon) { u = 1; v = 0; w = 0; return; }
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1f - v - w;
        }
    }
}
