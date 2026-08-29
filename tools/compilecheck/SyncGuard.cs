// ===========================================================================
// SyncGuard : ハーネス起動時の自動検査 2種。**不一致なら即 FAIL して実行しない。**
//
//   (A) 3複製の同期一致
//       エンジンは 3箇所に同じ実体がある (正本 / importer repo / Unityプロジェクト)。
//       ★同期漏れは 2026-08-26 で **2回目**。しかも 2026-08-24 のタスク78 (腕長ゲート) と
//         タスク79/81 (摩擦セット) が Unityプロジェクトへ入っておらず、
//         **実機の観測が正本と別エンジンで取られていた**。
//         「台帳に書く」方式は効いていないので機械化する。
//       比較は **コメント・空白・BOM を除去したハッシュ**。コメントの差 (名前スクラブ) は許す。
//
//   (B) A/B フラグの配線
//       env で ON/OFF を切り替えたのに出力ハッシュが同一なら「そのフラグは配線されていない」。
//       計測バグ #11/#14/#15/#16 はすべてこの同型で、毎回「出力が完全一致」で気づいている。
//       検査そのものはハーネス側の責務なので、ここでは判定関数だけ提供する。
//
//   使い方: 各ハーネスの Main 冒頭で `SyncGuard.RequireInSync();` を呼ぶ。
//     env `SYNCGUARD=0` で無効化 (複製が手元に無い環境用)。
//     複製が見つからない場合は **警告のみ** で通す (CI 等で片方しか無いことがある)。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;

public static class SyncGuard
{
    /// <summary>正本から見た複製の場所。存在しないものは黙って飛ばす。
    /// ★複製は各自のローカル配置なので **リポジトリに絶対パスを書かない**。次の順で探す:
    ///     1) 環境変数 `MMD_ENGINE_COPIES` (パスを `;` 区切り)
    ///     2) リポジトリ直下の `syncguard.local.txt` (1行1パス・`#` はコメント・.gitignore 済み)
    ///   どちらも無ければ複製 0 個 = 警告のみで通す (従来どおり)。</summary>
    private static string[] LoadCopyRoots()
    {
        var list = new List<string>();
        var env = Environment.GetEnvironmentVariable("MMD_ENGINE_COPIES");
        if (!string.IsNullOrEmpty(env))
            foreach (var t in env.Split(';'))
                if (t.Trim().Length > 0) list.Add(t.Trim());
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        for (int i = 0; i < 12 && dir != null; i++, dir = dir.Parent)
        {
            var cand = Path.Combine(dir.FullName, "syncguard.local.txt");
            if (!File.Exists(cand)) continue;
            foreach (var raw in File.ReadAllLines(cand))
            {
                var t = raw.Trim();
                if (t.Length > 0 && t[0] != '#') list.Add(t);
            }
            break;
        }
        return list.ToArray();
    }

    private static readonly string[] CopyRoots = LoadCopyRoots();

    /// <summary>正本 (このリポジトリの Assets/MmdPhysics) を、実行ファイルの位置から遡って探す。</summary>
    private static string FindMaster()
    {
        var dir = AppContext.BaseDirectory;
        for (int i = 0; i < 10 && dir != null; i++)
        {
            string cand = Path.Combine(dir, "Assets", "MmdPhysics");
            if (Directory.Exists(cand)) return cand;
            dir = Path.GetDirectoryName(dir.TrimEnd(Path.DirectorySeparatorChar));
        }
        return null;
    }

    /// <summary>コメント・空白・BOM を落とした正規形のハッシュ。名前スクラブ差は無視される。</summary>
    private static string NormalizedHash(string path)
    {
        var sb = new StringBuilder();
        foreach (var raw in File.ReadAllLines(path, Encoding.UTF8))
        {
            var s = raw;
            int c = s.IndexOf("//", StringComparison.Ordinal);
            if (c >= 0) s = s.Substring(0, c);      // 行コメントを落とす (文字列内の // は稀なので許容)
            s = s.Trim();
            if (s.Length == 0) continue;
            sb.Append(s).Append('\n');
        }
        using var md5 = MD5.Create();
        var h = md5.ComputeHash(Encoding.UTF8.GetBytes(sb.ToString()));
        return Convert.ToHexString(h);
    }

    /// <summary>3複製が同期しているか。ズレていたら例外で落とす。</summary>
    public static void RequireInSync()
    {
        if (Environment.GetEnvironmentVariable("SYNCGUARD") == "0") return;
        string master = FindMaster();
        if (master == null) { Console.Error.WriteLine("[SyncGuard] 正本が見つからない。検査を飛ばす。"); return; }

        var problems = new List<string>();
        int compared = 0;
        foreach (var copy in CopyRoots)
        {
            if (!Directory.Exists(copy)) continue;
            foreach (var f in Directory.GetFiles(master, "*.cs", SearchOption.AllDirectories))
            {
                string rel = Path.GetRelativePath(master, f);
                string cand = Path.Combine(copy, rel);
                // 配置差: 正本の DevTools/ は複製では Unity/ にある。
                if (!File.Exists(cand))
                    cand = Path.Combine(copy, "Unity", Path.GetFileName(f));
                if (!File.Exists(cand)) { problems.Add($"{rel} が {copy} に無い"); continue; }
                compared++;
                if (NormalizedHash(f) != NormalizedHash(cand))
                    problems.Add($"{rel} が {copy} と一致しない");
            }
        }
        if (compared == 0) { Console.Error.WriteLine("[SyncGuard] 複製が手元に無い。検査を飛ばす。"); return; }
        if (problems.Count > 0)
        {
            var msg = new StringBuilder();
            msg.AppendLine("[SyncGuard] ★エンジン3複製が同期していない。実行を中止する。");
            msg.AppendLine("  同期漏れの状態で測ると、**正本と別のエンジンの数字**を正本の結果として記録してしまう");
            msg.AppendLine("  (2026-08-26 に実際に起きた: 実機だけ腕長ゲートと摩擦セットが入っていなかった)。");
            foreach (var p in problems) msg.AppendLine("  - " + p);
            msg.AppendLine("  対処: 正本から実コードを移植する (丸コピー禁止。複製側の名前スクラブ差は残す)。");
            msg.Append("  どうしても回避したいときだけ SYNCGUARD=0。");
            throw new InvalidOperationException(msg.ToString());
        }
        Console.Error.WriteLine($"[SyncGuard] 3複製の同期 OK ({compared} ファイル比較)");
    }

    // ─── (B) フラグ配線の検査 ──────────────────────────────────────────────
    /// <summary>A/B の出力ハッシュが同一なら「そのフラグは配線されていない」。
    /// 期待が「変わるはず」なのに変わらなかった場合に落とす。
    /// ★無反応が正常と確定しているフラグ (例 CMARGIN) には使わないこと。</summary>
    public static void RequireFlagWired(string flagName, ulong hashOff, ulong hashOn)
    {
        if (hashOff != hashOn) return;
        throw new InvalidOperationException(
            $"[SyncGuard] ★フラグ '{flagName}' の ON/OFF で出力が完全一致した = 配線されていない。\n" +
            "  計測バグ #11/#14/#15/#16 と同型。A/B を取ったのに出力がビット一致したら、\n" +
            "  まず『そのフラグがハーネスから実際にエンジンへ渡っているか』を疑うこと。\n" +
            "  無反応が正常と確定しているフラグには、この検査を掛けないこと。");
    }
}
