// タスク1(堅牢性・実装前確認): GlbPhysicsReader が想定外入力で例外死せず、
// 警告を出して読み進めるかを合成GLB(JSONのみのGLBコンテナ)で確認する。本体物理は不変。
// ケース: SoftBody混在 / extras無し / rigidBodies空 / 不正なrigid参照Joint / ボーンindex範囲外 /
//        壊れたGLB(非glTF)。各ケースで「例外なし」を確認し、警告と読取件数を報告。
using System;
using System.Text;
using System.Collections.Generic;
using BulletPhysics.Pmx;

static class GlbRobust
{
    static StringBuilder O = new StringBuilder(); static void L(string s = "") { O.Append(s); O.Append('\n'); }

    // JSON文字列から最小GLBコンテナ(JSONチャンクのみ)を作る。
    static byte[] MakeGlb(string json)
    {
        var jb = Encoding.UTF8.GetBytes(json);
        int pad = (4 - (jb.Length & 3)) & 3;
        int clen = jb.Length + pad;
        var ms = new System.IO.MemoryStream();
        var w = new System.IO.BinaryWriter(ms);
        w.Write(0x46546C67u);           // magic "glTF"
        w.Write(2u);                    // version
        w.Write((uint)(12 + 8 + clen)); // total length
        w.Write((uint)clen);            // chunk length
        w.Write(0x4E4F534Au);           // "JSON"
        w.Write(jb);
        for (int i = 0; i < pad; i++) w.Write((byte)0x20);
        return ms.ToArray();
    }

    static void Case(string name, string json)
    {
        L($"\n[ケース] {name}");
        try
        {
            var m = GlbPhysicsReader.LoadBytes(MakeGlb(json), out float scale, out List<string> warns);
            L($"  例外なし ✓  剛体={m.RigidBodies.Count} Joint={m.Joints.Count} ボーン={m.BoneNames.Count} unitScale={scale}");
            // 構築まで通るかも確認(範囲外参照が構築で落ちないこと)
            var b = PmxPhysicsBuilder.Build(m);
            L($"  構築OK ✓  剛体={b.Bodies.Count} Joint={b.World.Joints.Count}");
            if (warns.Count > 0) foreach (var wn in warns) L($"    警告: {wn}");
            else L("    警告なし");
        }
        catch (Exception e) { L($"  ★例外! {e.GetType().Name}: {e.Message}"); }
    }

    static int Main()
    {
        L("==================== GlbPhysicsReader 堅牢性テスト (合成入力) ====================");

        // 共通の最小ボーン(nodes+skin): ボーン2個
        string bones = "\"nodes\":[{\"name\":\"root\",\"translation\":[0,0,0],\"children\":[1]},{\"name\":\"child\",\"translation\":[0,1,0]}]," +
                       "\"skins\":[{\"joints\":[0,1]}]";

        // 1) SoftBody 混在 (extras.mmd に softBodies キー。アダプタは無視して rigid/joint を読む)
        Case("SoftBody混在 (softBodies キーを無視できるか)",
            "{" + bones + ",\"extras\":{\"mmd\":{\"unitScale\":0.08," +
            "\"softBodies\":[{\"name\":\"sb\",\"shape\":0,\"anchors\":[1,2,3]}]," +
            "\"rigidBodies\":[{\"name\":\"rb0\",\"bone\":0,\"group\":0,\"no_collision_mask\":0,\"shape\":1,\"size\":[1,1,1],\"pos\":[0,0,0],\"rot\":[0,0,0],\"mass\":1,\"linear_damping\":0.5,\"angular_damping\":0.5,\"restitution\":0,\"friction\":0.5,\"mode\":1}]," +
            "\"joints\":[]}}}");

        // 2) extras.mmd 無し
        Case("extras.mmd 無し (物理情報なし)", "{" + bones + "}");

        // 3) rigidBodies 空
        Case("rigidBodies 空", "{" + bones + ",\"extras\":{\"mmd\":{\"unitScale\":0.08,\"rigidBodies\":[],\"joints\":[]}}}");

        // 4) 存在しない剛体を参照する Joint
        Case("Joint が存在しない剛体を参照 (rigid_b=5, 剛体1個)",
            "{" + bones + ",\"extras\":{\"mmd\":{\"unitScale\":0.08," +
            "\"rigidBodies\":[{\"name\":\"rb0\",\"bone\":0,\"shape\":1,\"size\":[1,1,1],\"pos\":[0,0,0],\"rot\":[0,0,0],\"mass\":1,\"mode\":1}]," +
            "\"joints\":[{\"name\":\"j0\",\"type\":0,\"rigid_a\":0,\"rigid_b\":5,\"pos\":[0,0,0],\"rot\":[0,0,0],\"pos_min\":[0,0,0],\"pos_max\":[0,0,0],\"rot_min\":[0,0,0],\"rot_max\":[0,0,0],\"spring_pos\":[0,0,0],\"spring_rot\":[0,0,0]}]}}}");

        // 5) 剛体の関連ボーンindex 範囲外
        Case("剛体の関連ボーンindex 範囲外 (bone=99, ボーン2個)",
            "{" + bones + ",\"extras\":{\"mmd\":{\"unitScale\":0.08," +
            "\"rigidBodies\":[{\"name\":\"rb0\",\"bone\":99,\"shape\":0,\"size\":[1,0,0],\"pos\":[0,0,0],\"rot\":[0,0,0],\"mass\":1,\"mode\":1}]," +
            "\"joints\":[]}}}");

        // 6) unitScale=0 (ゼロ除算回避)
        Case("unitScale=0 (ゼロ除算回避)",
            "{" + bones + ",\"extras\":{\"mmd\":{\"unitScale\":0,\"rigidBodies\":[],\"joints\":[]}}}");

        // 7) skins/nodes 無し
        Case("skins/nodes 無し (ボーン0)", "{\"extras\":{\"mmd\":{\"unitScale\":0.08,\"rigidBodies\":[],\"joints\":[]}}}");

        // 8) 壊れたGLB (非glTF) → LoadBytes は例外を投げるのが妥当(コンテナ不正)。呼び出し側でtry/catchする想定。
        L("\n[ケース] 壊れたGLB (非glTF, マジック不正)");
        try { GlbPhysicsReader.LoadBytes(new byte[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }, out _, out _); L("  例外なし(想定外)"); }
        catch (Exception e) { L($"  例外(想定内・呼出側でcatchする): {e.GetType().Name}"); }

        Console.Write(O.ToString());
        return 0;
    }
}
