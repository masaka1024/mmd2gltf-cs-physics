// ===========================================================================
// GLB (glTF binary) の extras.mmd から PmxPhysicsModel を構築するリーダ。
// PmxReader (PMX 直読み) と並ぶ入力経路。物理置換(統合フェーズ)の土台。
//   - 剛体 : extras.mmd.rigidBodies (PMX raw) をそのままマップ
//   - Joint: extras.mmd.joints (PMX raw) をそのままマップ
//   - ボーン: glTF nodes から 名前 / 親(children階層) / 変形階層(extras.mmd.layer) /
//            raw PMX world 位置(ローカル並進をunitScaleで戻して階層合成)
//   - unitScale は extras.mmd.unitScale から読む (既定を仮定しない)
//   - physicsGltf(変換済みビュュー)は使わず raw を使う
// 当面 GLB(バイナリ)のみ対応 (.gltf JSON+外部bin は未対応)。読み取り専用・本体物理に非関与。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.IO;

namespace BulletPhysics.Pmx
{
    public static class GlbPhysicsReader
    {
        // 座標: extras.mmd は raw PMX (MMD左手系, 未スケール)。glTF は cpos=(x,y,-z)*s。
        // よって glTF ローカル並進 lt から raw ローカルへ戻すのは (lt.x/s, lt.y/s, -lt.z/s)。
        // (バインドは回転恒等なので world = ローカルの単純合成でよい。)

        public static PmxPhysicsModel LoadFile(string path)
        {
            ParseGlb(File.ReadAllBytes(path), out var root, out _);
            return BuildModel(root);
        }

        // ibm(inverseBindMatrices)から算出した raw PMX ボーン world 位置 (検証の別経路)。
        public static Vec3[] BonePositionsFromIbm(string path)
        {
            ParseGlb(File.ReadAllBytes(path), out var root, out var bin);
            var mm = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(root, "extras")), "mmd"));
            float s = MiniJson.Flt(MiniJson.Get(mm, "unitScale"));
            var skin0 = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "skins"))[0]);
            int nb = MiniJson.Arr(MiniJson.Get(skin0, "joints")).Count;
            int ibmAcc = MiniJson.Int(MiniJson.Get(skin0, "inverseBindMatrices"));
            var mats = ReadFloatAccessor(root, bin, ibmAcc); // nb*16
            var pos = new Vec3[nb];
            for (int i = 0; i < nb; i++)
            {
                // 列優先 MAT4 の並進は要素 [12],[13],[14] = (-wp.x*s, -wp.y*s, wp.z*s)。
                float tx = mats[i * 16 + 12], ty = mats[i * 16 + 13], tz = mats[i * 16 + 14];
                pos[i] = new Vec3(-tx / s, -ty / s, tz / s);
            }
            return pos;
        }

        // ---- GLB コンテナ分解 (JSON チャンク + BIN チャンク) ----
        private static void ParseGlb(byte[] d, out Dictionary<string, object> root, out byte[] bin)
        {
            uint magic = BitConverter.ToUInt32(d, 0);
            if (magic != 0x46546C67) throw new InvalidDataException("GLB マジックが不正 (glTF ではない)");
            // uint version = BitConverter.ToUInt32(d, 4); uint length = BitConverter.ToUInt32(d, 8);
            int off = 12;
            string json = null; bin = null;
            while (off + 8 <= d.Length)
            {
                int clen = (int)BitConverter.ToUInt32(d, off);
                uint ctype = BitConverter.ToUInt32(d, off + 4);
                int cdata = off + 8;
                if (ctype == 0x4E4F534A)      // "JSON"
                    json = System.Text.Encoding.UTF8.GetString(d, cdata, clen);
                else if (ctype == 0x004E4942) // "BIN\0"
                {
                    bin = new byte[clen];
                    Array.Copy(d, cdata, bin, 0, clen);
                }
                off = cdata + clen;
                if ((clen & 3) != 0) off += 4 - (clen & 3); // 4バイト境界パディング
            }
            if (json == null) throw new InvalidDataException("GLB に JSON チャンクが無い");
            root = MiniJson.Obj(MiniJson.Parse(json));
        }

        // ---- extras.mmd → PmxPhysicsModel ----
        private static PmxPhysicsModel BuildModel(Dictionary<string, object> root)
        {
            var model = new PmxPhysicsModel();
            var mm = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(root, "extras")), "mmd"));
            if (mm == null) throw new InvalidDataException("extras.mmd が無い GLB (物理情報なし)");
            float scale = MiniJson.Flt(MiniJson.Get(mm, "unitScale"));

            // --- ボーン (nodes + skins[0].joints) ---
            var nodes = MiniJson.Arr(MiniJson.Get(root, "nodes"));
            var skin0 = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "skins"))[0]);
            var jointNodes = MiniJson.Arr(MiniJson.Get(skin0, "joints"));
            int nb = jointNodes.Count;

            var localRaw = new Vec3[nb];       // raw PMX ローカル並進
            var parent = new int[nb];
            for (int i = 0; i < nb; i++) parent[i] = -1;

            for (int i = 0; i < nb; i++)
            {
                var node = MiniJson.Obj(nodes[MiniJson.Int(jointNodes[i])]);
                model.BoneNames.Add(MiniJson.Str(MiniJson.Get(node, "name")) ?? ("bone_" + i));
                var lt = Vec3FromArr(MiniJson.Get(node, "translation"));
                localRaw[i] = new Vec3(lt.x / scale, lt.y / scale, -lt.z / scale); // glTF→raw
                var ex = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(node, "extras")), "mmd"));
                model.BoneDeformLayers.Add(ex != null ? MiniJson.Int(MiniJson.Get(ex, "layer")) : 0);
            }
            // 親: 各ノードの children から。子ノードindexが 0..nb-1 のボーンなら親を設定。
            //     親ノードが 0..nb-1 のボーンなら親=そのindex、そうでなければ(スケルトンroot等)=-1。
            for (int j = 0; j < nodes.Count; j++)
            {
                var ch = MiniJson.Arr(MiniJson.Get(MiniJson.Obj(nodes[j]), "children"));
                if (ch == null) continue;
                foreach (var c in ch)
                {
                    int ci = MiniJson.Int(c);
                    if (ci >= 0 && ci < nb) parent[ci] = (j < nb) ? j : -1;
                }
            }
            for (int i = 0; i < nb; i++) model.BoneParents.Add(parent[i]);

            // world 位置 = ローカル並進の階層合成 (バインド回転恒等)。親を先に確定。
            var world = new Vec3?[nb];
            Vec3 World(int i)
            {
                if (world[i].HasValue) return world[i].Value;
                int p = parent[i];
                Vec3 w = (p >= 0 && p < nb) ? localRaw[i] + World(p) : localRaw[i];
                world[i] = w; return w;
            }
            for (int i = 0; i < nb; i++) model.BonePositions.Add(World(i));

            // --- 剛体 (raw) ---
            var rbs = MiniJson.Arr(MiniJson.Get(mm, "rigidBodies"));
            if (rbs != null)
                foreach (var o in rbs)
                {
                    var r = MiniJson.Obj(o);
                    model.RigidBodies.Add(new PmxRigidBody
                    {
                        Name = MiniJson.Str(MiniJson.Get(r, "name")) ?? "",
                        NameEn = MiniJson.Str(MiniJson.Get(r, "name_en")) ?? "",
                        BoneIndex = MiniJson.Int(MiniJson.Get(r, "bone")),
                        Group = (byte)MiniJson.Int(MiniJson.Get(r, "group")),
                        NonCollisionGroup = (ushort)MiniJson.Int(MiniJson.Get(r, "no_collision_mask")),
                        ShapeType = (byte)MiniJson.Int(MiniJson.Get(r, "shape")),
                        Size = Vec3FromArr(MiniJson.Get(r, "size")),
                        Position = Vec3FromArr(MiniJson.Get(r, "pos")),
                        Rotation = Vec3FromArr(MiniJson.Get(r, "rot")),
                        Mass = MiniJson.Flt(MiniJson.Get(r, "mass")),
                        LinearDamping = MiniJson.Flt(MiniJson.Get(r, "linear_damping")),
                        AngularDamping = MiniJson.Flt(MiniJson.Get(r, "angular_damping")),
                        Restitution = MiniJson.Flt(MiniJson.Get(r, "restitution")),
                        Friction = MiniJson.Flt(MiniJson.Get(r, "friction")),
                        PhysicsMode = (byte)MiniJson.Int(MiniJson.Get(r, "mode")),
                    });
                }

            // --- Joint (raw) ---
            var jts = MiniJson.Arr(MiniJson.Get(mm, "joints"));
            if (jts != null)
                foreach (var o in jts)
                {
                    var j = MiniJson.Obj(o);
                    model.Joints.Add(new PmxJoint
                    {
                        Name = MiniJson.Str(MiniJson.Get(j, "name")) ?? "",
                        NameEn = MiniJson.Str(MiniJson.Get(j, "name_en")) ?? "",
                        JointType = (byte)MiniJson.Int(MiniJson.Get(j, "type")),
                        RigidBodyAIndex = MiniJson.Int(MiniJson.Get(j, "rigid_a")),
                        RigidBodyBIndex = MiniJson.Int(MiniJson.Get(j, "rigid_b")),
                        Position = Vec3FromArr(MiniJson.Get(j, "pos")),
                        Rotation = Vec3FromArr(MiniJson.Get(j, "rot")),
                        LinearLowerLimit = Vec3FromArr(MiniJson.Get(j, "pos_min")),
                        LinearUpperLimit = Vec3FromArr(MiniJson.Get(j, "pos_max")),
                        AngularLowerLimit = Vec3FromArr(MiniJson.Get(j, "rot_min")),
                        AngularUpperLimit = Vec3FromArr(MiniJson.Get(j, "rot_max")),
                        SpringLinear = Vec3FromArr(MiniJson.Get(j, "spring_pos")),
                        SpringAngular = Vec3FromArr(MiniJson.Get(j, "spring_rot")),
                    });
                }

            return model;
        }

        private static Vec3 Vec3FromArr(object o)
        {
            var a = MiniJson.Arr(o);
            if (a == null || a.Count < 3) return Vec3.Zero;
            return new Vec3(MiniJson.Flt(a[0]), MiniJson.Flt(a[1]), MiniJson.Flt(a[2]));
        }

        // accessor(FLOAT)を BIN から読み出す (bufferView.byteOffset + accessor.byteOffset)。
        private static float[] ReadFloatAccessor(Dictionary<string, object> root, byte[] bin, int accIdx)
        {
            var acc = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "accessors"))[accIdx]);
            int count = MiniJson.Int(MiniJson.Get(acc, "count"));
            string type = MiniJson.Str(MiniJson.Get(acc, "type"));
            int comps = type == "MAT4" ? 16 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : type == "SCALAR" ? 1 : 16;
            int accOff = acc.ContainsKey("byteOffset") ? MiniJson.Int(MiniJson.Get(acc, "byteOffset")) : 0;
            var bv = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "bufferViews"))[MiniJson.Int(MiniJson.Get(acc, "bufferView"))]);
            int bvOff = bv.ContainsKey("byteOffset") ? MiniJson.Int(MiniJson.Get(bv, "byteOffset")) : 0;
            int baseOff = bvOff + accOff;
            var outp = new float[count * comps];
            for (int i = 0; i < outp.Length; i++) outp[i] = BitConverter.ToSingle(bin, baseOff + i * 4);
            return outp;
        }
    }
}
