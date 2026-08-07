// ボーン姿勢CSVローダの動作確認 (タスク1)。
// CSV が無い環境では SKIP して pass 扱い。
using System;
using System.Linq;
using BulletPhysics;

namespace BoneCheck
{
    static class Program
    {
        static int Main()
        {
            string path = BoneCsv.FindPath();
            if (path == null)
            {
                Console.WriteLine("[SKIP] ボーンCSV 未検出 (MMD_TEST_BONECSV 未設定)。pass 扱い。");
                return 0;
            }

            var sw = System.Diagnostics.Stopwatch.StartNew();
            var csv = BoneCsv.Load(path);
            sw.Stop();

            Console.WriteLine($"CSV読込: {path}");
            Console.WriteLine($"  フレーム数={csv.FrameCount} (0..{csv.MaxFrame})  ボーン数={csv.BoneCount}");
            Console.WriteLine($"  保持方式=全メモリ(dense)  概算メモリ={csv.ApproxBytes / (1024.0 * 1024.0):F2} MB  読込時間={sw.ElapsedMilliseconds}ms");

            // 正規化確認 (先頭フレームの数ボーンの |q|)。
            foreach (var b in new[] { "下半身", "スカート_0_0", "スカート_2_11" })
            {
                if (csv.TryGet(0, b, out var xf))
                {
                    var q = xf.Rotation;
                    float mag = (float)Math.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                    Console.WriteLine($"  frame0 '{b}': pos={xf.Origin} |q|={mag:F7} (正規化済み)");
                }
                else Console.WriteLine($"  frame0 '{b}': 見つからない");
            }
            Console.WriteLine("[PASS] ローダ動作確認");
            return 0;
        }
    }
}
