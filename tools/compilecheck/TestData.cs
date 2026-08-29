// ===========================================================================
// テストデータ(モデル/CSV)のパス解決。公開リポジトリに絶対パスを残さないための共通ヘルパ。
// 解決順 (最初に見つかったものを使う):
//   1) 環境変数 (MMD_TEST_PMX / MMD_TEST_BONECSV / MMD_TEST_PMXCSV)
//   2) リポジトリ相対 testdata/<既定ファイル名> (実行バイナリから上へ辿って探す)
//   3) どちらも無ければ null → 呼び出し側は SKIP して pass 扱い (既存ハーネス方針)
// モデル/CSVは各自用意し testdata/ に置くか環境変数で指定する (README 参照)。再配布はしない。
// ===========================================================================
using System;
using System.IO;

internal static class TestData
{
    public const string DefaultPmx = "modelA.pmx";
    // ★2026-08-29 昇格: 既定参照を「真OFF (統制ペア・補正層OFF = 純ソルバ)」へ切替。
    //   旧既定 modelA_bone_world_pose.csv は実体が【補正OFF + Fix前の整え込み】で、
    //   整え作業ぶんの開始状態が交絡していた (忠実度指標は 1e-4 しか動かないが、
    //   交絡の無い方を北極星にする / NORTH_STAR §5-a)。旧参照も検証は通るので併用可。
    public const string DefaultBoneCsv = "modelA_bone_world_pose_detoff.csv";
    public const string LegacyBoneCsv  = "modelA_bone_world_pose.csv";   // 旧既定 (撤回せず残す)
    public const string DefaultStructureCsv = "ia.csv";
    public const string DefaultGlb = "modelA.glb";

    public static string PmxPath() => Resolve("MMD_TEST_PMX", DefaultPmx);
    public static string BoneCsvPath() => Resolve("MMD_TEST_BONECSV", DefaultBoneCsv);
    public static string StructureCsvPath() => Resolve("MMD_TEST_PMXCSV", DefaultStructureCsv);
    public static string GlbPath() => Resolve("MMD_TEST_GLB", DefaultGlb);

    public static string Resolve(string envVar, string defaultFile)
    {
        var env = Environment.GetEnvironmentVariable(envVar);
        if (!string.IsNullOrEmpty(env) && File.Exists(env)) return env;
        // 実行バイナリ (bin/<cfg>/<tfm>/) からリポジトリルートまで上へ辿り testdata/<file> を探す。
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        for (int i = 0; i < 12 && dir != null; i++, dir = dir.Parent)
        {
            var cand = Path.Combine(dir.FullName, "testdata", defaultFile);
            if (File.Exists(cand)) return cand;
        }
        return null;
    }
}
