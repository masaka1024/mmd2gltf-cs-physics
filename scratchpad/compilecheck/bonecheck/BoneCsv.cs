// ===========================================================================
// ボーン姿勢CSVローダ (Unity 非依存・計測専用。本体は変更しない)。
// 本家ベイク済みVMDから書き出した「ワールド絶対姿勢」CSVを読む。
//   列: frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW (ヘッダ行あり, UTF-8)
//   座標系・単位は PMX ネイティブ (エンジン内部と同一。変換なし)。
//
// 方式: 全メモリ保持 (dense: frame 0..maxFrame × ボーン)。
//   RigidTransform は 28byte (Quat16 + Vec3 12)。43ボーン×7001フレーム ≒ 8.4MB と小さいため、
//   逐次読みより単純でランダムアクセスも O(1) の全保持を選択。
//   クォータニオンは読み込み時に正規化する (VMD由来float32で |q| が 1±1e-7 ずれ、
//   後段 acos で ~0.05° の見かけ誤差になるため、正規化を揃えて回帰判定の土台にする)。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using BulletPhysics;

namespace BoneCheck
{
    public sealed class BoneCsv
    {
        private readonly Dictionary<string, int> _col = new();   // boneName -> 列index
        private RigidTransform[][] _pose;                        // [frame][col]
        private bool[][] _present;                               // [frame][col]

        public int FrameCount { get; private set; }
        public int MaxFrame => FrameCount - 1;
        public IReadOnlyCollection<string> BoneNames => _col.Keys;
        public int BoneCount => _col.Count;
        public long ApproxBytes { get; private set; }

        /// <summary>環境変数 MMD_TEST_BONECSV か testdata/ から探す。無ければ null。(TestData に委譲)</summary>
        public static string FindPath() => TestData.BoneCsvPath();

        public static BoneCsv Load(string path)
        {
            var csv = new BoneCsv();
            csv.ReadAll(path);
            return csv;
        }

        private void ReadAll(string path)
        {
            // 1パス目: ボーン名の列割り当てと最大フレームを確定。
            int maxFrame = -1;
            var rows = new List<(int frame, string bone, RigidTransform xf)>(310000);
            bool header = true;
            foreach (var line in File.ReadLines(path))
            {
                if (header) { header = false; continue; }
                if (line.Length == 0) continue;
                var s = line.Split(',');
                if (s.Length < 9) continue;
                int frame = int.Parse(s[0], CultureInfo.InvariantCulture);
                string bone = s[1];
                var pos = new Vec3(F(s[2]), F(s[3]), F(s[4]));
                // クォータニオン正規化 (|q| の float32 誤差を吸収)。
                var q = new Quat(F(s[5]), F(s[6]), F(s[7]), F(s[8])).Normalized;
                if (!_col.ContainsKey(bone)) _col[bone] = _col.Count;
                if (frame > maxFrame) maxFrame = frame;
                rows.Add((frame, bone, new RigidTransform(q, pos)));
            }
            FrameCount = maxFrame + 1;
            int cols = _col.Count;

            _pose = new RigidTransform[FrameCount][];
            _present = new bool[FrameCount][];
            for (int f = 0; f < FrameCount; f++)
            {
                _pose[f] = new RigidTransform[cols];
                _present[f] = new bool[cols];
            }
            foreach (var (frame, bone, xf) in rows)
            {
                int c = _col[bone];
                _pose[frame][c] = xf;
                _present[frame][c] = true;
            }
            // RigidTransform 28byte + present 1byte 概算。
            ApproxBytes = (long)FrameCount * cols * (28 + 1);
        }

        private static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);

        public bool HasBone(string bone) => _col.ContainsKey(bone);

        /// <summary>指定フレーム・ボーンのワールド姿勢。無ければ false。</summary>
        public bool TryGet(int frame, string bone, out RigidTransform xf)
        {
            xf = RigidTransform.Identity;
            if (frame < 0 || frame >= FrameCount) return false;
            if (!_col.TryGetValue(bone, out int c)) return false;
            if (!_present[frame][c]) return false;
            xf = _pose[frame][c];
            return true;
        }
    }
}
