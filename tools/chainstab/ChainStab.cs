// ===========================================================================
// chainstab: 揺れ物の鎖が伸び縮みしないかを測る。
//
// 指標: 物理剛体が付いたボーンと、その親ボーンとの距離を、バインド姿勢での長さに対する
//       比で見る。ジョイントが並進ロック(pos_min=pos_max=0)の鎖では 1.00 のまま保たれるのが正しい。
//
// 書き戻しは MmdPhysicsBehaviour.PullPhysicsToBones と同一ロジックで再現する:
//   mode1 (Dynamic)          -> 剛体の生の姿勢
//   mode2 (DynamicBoneMerge) -> ComputeAlignedBonePoses の再構成姿勢
//   それ以外(非物理)          -> 駆動姿勢、無ければ親からの FK
//
// 駆動 CSV (MMD_TEST_BONECSV) を渡すとアニメ再生になる。無ければバインド姿勢で静止。
// env: FRAMES / FPS / WARMUP / ITERS / SUBSTEPS / ALIGNALL / ALPHA / TOP
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using BulletPhysics;
using BulletPhysics.Pmx;

static class ChainStab
{
    // --- 駆動 CSV (frame,boneName,posX..quatW) ----------------------------------
    sealed class DriveCsv
    {
        readonly Dictionary<string, RigidTransform[]> _col = new Dictionary<string, RigidTransform[]>();
        public int FrameCount;
        public int BoneCount { get { return _col.Count; } }

        public static DriveCsv Load(string path)
        {
            var rows = new Dictionary<string, List<KeyValuePair<int, RigidTransform>>>();
            int maxF = -1;
            using (var sr = new StreamReader(path))
            {
                string line = sr.ReadLine(); // ヘッダ
                while ((line = sr.ReadLine()) != null)
                {
                    if (line.Length == 0) continue;
                    var t = line.Split(',');
                    if (t.Length < 9) continue;
                    int f = int.Parse(t[0], CultureInfo.InvariantCulture);
                    string bone = t[1];
                    var xf = new RigidTransform(
                        new Quat(P(t, 5), P(t, 6), P(t, 7), P(t, 8)).Normalized,
                        new Vec3(P(t, 2), P(t, 3), P(t, 4)));
                    List<KeyValuePair<int, RigidTransform>> l;
                    if (!rows.TryGetValue(bone, out l)) { l = new List<KeyValuePair<int, RigidTransform>>(); rows[bone] = l; }
                    l.Add(new KeyValuePair<int, RigidTransform>(f, xf));
                    if (f > maxF) maxF = f;
                }
            }
            var csv = new DriveCsv();
            csv.FrameCount = maxF + 1;
            foreach (var kv in rows)
            {
                var arr = new RigidTransform[csv.FrameCount];
                foreach (var e in kv.Value) arr[e.Key] = e.Value;
                csv._col[kv.Key] = arr;
            }
            return csv;
        }

        static float P(string[] t, int i) { return float.Parse(t[i], CultureInfo.InvariantCulture); }

        public bool TryGet(int frame, string bone, out RigidTransform xf)
        {
            xf = default(RigidTransform);
            RigidTransform[] arr;
            if (!_col.TryGetValue(bone, out arr)) return false;
            if (frame < 0 || frame >= arr.Length) return false;
            xf = arr[frame];
            return true;
        }
    }

    // --- 1ボーンぶんの集計 -------------------------------------------------------
    sealed class Stat
    {
        public string Name, Parent, Mode;
        public float Bind;
        public float Min = float.MaxValue, Max = float.MinValue;
        public int MinFrame, MaxFrame;
        public void Add(float r, int f)
        {
            if (r < Min) { Min = r; MinFrame = f; }
            if (r > Max) { Max = r; MaxFrame = f; }
        }
    }

    static int EnvI(string k, int def) { int v; return int.TryParse(Environment.GetEnvironmentVariable(k), out v) ? v : def; }
    static float EnvF(string k, float def)
    {
        float v;
        return float.TryParse(Environment.GetEnvironmentVariable(k), NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : def;
    }

    static PmxPhysicsModel _model;
    static DriveCsv _csv;
    static int _n;
    static RigidTransform?[] _disp;

    static RigidTransform? DrivenAt(int f, int i)
    {
        if (_csv == null || i < 0 || i >= _n) return null;
        RigidTransform x;
        return _csv.TryGet(f % _csv.FrameCount, _model.BoneNames[i], out x) ? (RigidTransform?)x : null;
    }

    static RigidTransform Fk(int i, int depth, Func<int, RigidTransform?> driven)
    {
        if (_disp[i].HasValue) return _disp[i].Value;
        if (depth > 512) { _disp[i] = new RigidTransform(Quat.Identity, _model.BonePositions[i]); return _disp[i].Value; }
        RigidTransform res;
        var d = driven(i);
        int p = (i < _model.BoneParents.Count) ? _model.BoneParents[i] : -1;
        if (d.HasValue) res = d.Value;
        else if (p < 0 || p >= _n) res = new RigidTransform(Quat.Identity, _model.BonePositions[i]);
        else
        {
            var pw = Fk(p, depth + 1, driven);
            res = new RigidTransform(pw.Rotation, pw.Rotation * (_model.BonePositions[i] - _model.BonePositions[p]) + pw.Origin);
        }
        _disp[i] = res;
        return res;
    }

    static int Main()
    {
        string pmxPath = TestData.PmxPath();
        if (pmxPath == null) { Console.WriteLine("[SKIP] PMX が無い (MMD_TEST_PMX か testdata/modelA.pmx)"); return 0; }
        _model = PmxReader.LoadFile(pmxPath);
        var model = _model;
        Console.WriteLine("[chainstab] PMX=" + Path.GetFileName(pmxPath) + " ボーン" + model.BoneNames.Count +
                          " 剛体" + model.RigidBodies.Count + " ジョイント" + model.Joints.Count);

        string csvPath = Environment.GetEnvironmentVariable("MMD_TEST_BONECSV");
        _csv = (!string.IsNullOrEmpty(csvPath) && File.Exists(csvPath)) ? DriveCsv.Load(csvPath) : null;
        if (_csv != null) Console.WriteLine("[chainstab] 駆動CSV=" + Path.GetFileName(csvPath) + " " + _csv.BoneCount + "ボーン × " + _csv.FrameCount + "フレーム");
        else Console.WriteLine("[chainstab] 駆動CSV無し = バインド姿勢で静止 (体は動かさない)");

        var builder = PmxPhysicsBuilder.Build(model);
        var world = builder.World;
        // Unity 側 (MmdPhysicsBehaviour) の既定に合わせる。
        world.FixedTimeStep = 1f / 60f;
        world.SubSteps = EnvI("SUBSTEPS", 2);
        world.SolverIterations = EnvI("ITERS", 10);
        world.JointVelocityIterations = EnvI("JOINTITERS", 0);  // 0=従来
        world.JointMaxCorrectionVel = EnvF("JOINTMAXCORR", 0f); // 0=従来(10)
        bool alignAll = Environment.GetEnvironmentVariable("ALIGNALL") == "1"; // = AlignBonePositions
        float alpha = EnvF("ALPHA", 0.5f);                                     // = AlignRotClampAlpha
        int fps = EnvI("FPS", 60);
        // LOOPS>1 で駆動CSVを繰り返す (アニメのループ境界を跨がせる)。
        int loops = EnvI("LOOPS", 1);
        int frames = EnvI("FRAMES", _csv != null ? _csv.FrameCount * loops : 600);
        if (_csv != null && frames > _csv.FrameCount * loops) frames = _csv.FrameCount * loops;
        // 駆動ボーンが1フレームでこの距離以上飛んだら起動時と同じ再整合をやり直す
        // (MmdPhysicsBehaviour.TeleportResetThreshold と同じ規則。0=無効=従来)。
        float teleTh = EnvF("TELEPORT", 0f);
        int resetDelay = EnvI("RESETDELAY", 2); // = PoseResetDelayFrames
        float teleFrac = EnvF("TELEPORTFRAC", 0.25f); // 同時に飛んだ駆動ボーンの割合しきい値
        int resetCountdown = 0;
        int teleHits = 0;
        int warmup = EnvI("WARMUP", 0);
        float dt = 1f / fps;

        _n = model.BoneNames.Count;
        int n = _n;

        // ボーン -> リンク (同じボーンに複数剛体が付く場合は ComputeAlignedBonePoses の physRot と
        // 同じ「最後が残る」規則にする)。
        var linkOf = new BoneLink?[n];
        foreach (var link in builder.BoneLinks)
            if (link.BoneIndex >= 0 && link.BoneIndex < n && link.Mode != PhysicsMode.BoneFollow)
                linkOf[link.BoneIndex] = link;

        // 起動時の再整合 (MmdPhysicsBehaviour.Start 相当)。
        Func<int, RigidTransform?> driven0 = i => DrivenAt(0, i);
        builder.ApplyKinematicTargets(driven0);
        builder.ResetBodiesToBonePoseFk(driven0);
        for (int s = 0; s < warmup; s++) world.StepSimulation(dt);

        var stats = new Dictionary<int, Stat>();
        _disp = new RigidTransform?[n];

        // 単調に溜まる(ラチェット)のか、一過性のスパイクなのかを見分けるための窓別最大。
        int win = EnvI("WIN", 1000);
        int nwin = (frames + win - 1) / win;
        var winMax = new float[nwin];
        var winWho = new string[nwin];

        for (int f = 0; f < frames; f++)
        {
            int ff = f;
            Func<int, RigidTransform?> driven = i => DrivenAt(ff, i);

            // テレポート検出 (MmdPhysicsBehaviour と同じ規則: 目標を更新する前の
            // KinematicTarget と、これから与える目標との距離で見る)。
            if (resetCountdown <= 0 && teleTh > 0f)
            {
                float th2 = teleTh * teleTh;
                int over = 0, total = 0; string worstBone = null; float worstD = 0f;
                foreach (var link in builder.BoneLinks)
                {
                    if (link.Mode != PhysicsMode.BoneFollow || link.BoneIndex < 0) continue;
                    var bw = driven(link.BoneIndex);
                    if (!bw.HasValue) continue;
                    total++;
                    var d = (bw.Value * link.BodyOffsetFromBone).Origin - link.Body.KinematicTarget.Origin;
                    if (d.LengthSquared <= th2) continue;
                    over++;
                    if (d.Length > worstD) { worstD = d.Length; worstBone = model.BoneNames[link.BoneIndex]; }
                }
                // ★1本だけ飛ぶのは速い腕/手首。骨格ごと飛んだ(=ループ境界)ときだけ再整合したいので、
                //   駆動ボーンのうち一定割合が同時に飛んだことを条件にする。
                if (over > 0 && over >= System.Math.Max(1, (int)System.Math.Ceiling(total * teleFrac)))
                {
                    resetCountdown = resetDelay > 0 ? resetDelay : 1;
                    teleHits++;
                    Console.WriteLine("           テレポート検出 F" + f + ": 駆動ボーン " + over + "/" + total +
                                      " 本が飛んだ (最大 " + worstBone + " " + (worstD * 8f).ToString("F1") + "cm)");
                }
                else if (over > 0)
                {
                    Console.WriteLine("           (見送り) F" + f + ": " + over + "/" + total +
                                      " 本のみ (最大 " + worstBone + " " + (worstD * 8f).ToString("F1") + "cm)");
                }
            }

            builder.ApplyKinematicTargets(driven);
            if (resetCountdown > 0) { builder.ResetBodiesToBonePoseFk(driven); resetCountdown--; }
            world.StepSimulation(dt);

            // --- 表示される骨格を組む (PullPhysicsToBones と同一) ---
            Array.Clear(_disp, 0, n);
            bool needAligned = alignAll || builder.HasBoneMergeBodies;
            var aligned = needAligned ? builder.ComputeAlignedBonePoses(driven, alpha, alignAll) : null;
            foreach (var link in builder.BoneLinks)
            {
                if (link.Mode == PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || link.BoneIndex >= n) continue;
                bool useAligned = aligned != null && aligned[link.BoneIndex].HasValue &&
                                  (alignAll || link.Mode == PhysicsMode.DynamicBoneMerge);
                _disp[link.BoneIndex] = useAligned
                    ? aligned[link.BoneIndex].Value
                    : link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();
            }
            for (int i = 0; i < n; i++) Fk(i, 0, driven);

            // --- 親との距離を比で測る ---
            for (int i = 0; i < n; i++)
            {
                if (!linkOf[i].HasValue) continue;
                int p = (i < model.BoneParents.Count) ? model.BoneParents[i] : -1;
                if (p < 0 || p >= n || !_disp[p].HasValue || !_disp[i].HasValue) continue;
                float bind = (model.BonePositions[i] - model.BonePositions[p]).Length;
                if (bind < 1e-3f) continue;
                float cur = (_disp[i].Value.Origin - _disp[p].Value.Origin).Length;
                if (float.IsNaN(cur)) continue;
                Stat st;
                if (!stats.TryGetValue(i, out st))
                {
                    st = new Stat();
                    st.Name = model.BoneNames[i];
                    st.Parent = model.BoneNames[p];
                    st.Mode = linkOf[i].Value.Mode == PhysicsMode.DynamicBoneMerge ? "mode2" : "mode1";
                    st.Bind = bind;
                    stats[i] = st;
                }
                float ratio = cur / bind;
                st.Add(ratio, f);
                int w = f / win;
                if (ratio > winMax[w]) { winMax[w] = ratio; winWho[w] = st.Name; }
            }
        }

        // --- 報告 ---
        int top = EnvI("TOP", 12);
        var list = new List<Stat>(stats.Values);
        Console.WriteLine("[chainstab] " + frames + "フレーム @" + fps + "Hz sub" + world.SubSteps +
                          " iters" + world.SolverIterations +
                          " jointiters" + (world.JointVelocityIterations > 0 ? world.JointVelocityIterations.ToString() : "-") +
                          " jointmaxcorr" + (world.JointMaxCorrectionVel > 0f ? world.JointMaxCorrectionVel.ToString("F0") : "-") +
                          " align=" + (alignAll ? "全再構成" : "mode2のみ") +
                          " 対象" + list.Count + "ボーン");
        if (teleTh > 0f) Console.WriteLine("[chainstab] テレポート再整合: しきい値 " + teleTh + " PMX単位 → 発火 " + teleHits + " 回");
        float worstMax = 1f, worstMin = 1f;
        foreach (var s in list) { if (s.Max > worstMax) worstMax = s.Max; if (s.Min < worstMin) worstMin = s.Min; }
        Console.WriteLine("[chainstab] 全体: 最大 " + worstMax.ToString("F3") + " 倍 / 最小 " + worstMin.ToString("F3") + " 倍");

        if (nwin > 1)
        {
            var sb = new System.Text.StringBuilder("[chainstab] " + win + "フレーム窓ごとの最大比: ");
            for (int w = 0; w < nwin; w++) sb.Append(winMax[w].ToString("F2")).Append(w + 1 < nwin ? " → " : "");
            Console.WriteLine(sb.ToString());
            Console.WriteLine("            (窓ごとの主犯: " + string.Join(" / ", winWho) + ")");
            Console.WriteLine("            単調に増えるなら誤差が溜まるラチェット、上下するなら一過性のたわみ。");
        }

        list.Sort((a, b) => Math.Max(b.Max - 1f, 1f - b.Min).CompareTo(Math.Max(a.Max - 1f, 1f - a.Min)));
        Console.WriteLine("  " + Pad("ボーン", 18) + Pad("親", 18) + "mode  bind(cm)  min比   max比   (min@F, max@F)");
        for (int k = 0; k < list.Count && k < top; k++)
        {
            var s = list[k];
            Console.WriteLine("  " + Pad(s.Name, 18) + Pad(s.Parent, 18) + s.Mode + "  " +
                              (s.Bind * 8f).ToString("F2").PadLeft(7) + "  " +
                              s.Min.ToString("F3").PadLeft(6) + "  " + s.Max.ToString("F3").PadLeft(6) +
                              "   (" + s.MinFrame + ", " + s.MaxFrame + ")");
        }
        return 0;
    }

    // 日本語(全角)を含む名前を等幅で揃える。
    static string Pad(string s, int width)
    {
        int w = 0;
        foreach (var c in s) w += c < 0x80 ? 1 : 2;
        return s + new string(' ', Math.Max(1, width - w));
    }
}
