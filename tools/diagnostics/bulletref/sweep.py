# -*- coding: utf-8 -*-
"""
sweep.py -- タスク20 の詰め: 「当エンジンのどの差が減衰を殺しているか」を、
            本物の Bullet 2.75 を正解に置いて1つずつ潰す。

同じ最小網 (Joint 5本 / 剛体6個 / 接触ゼロ) を、
  - 当エンジン (restosc NETDUMP=1) を各フラグ設定で
  - 本物の Bullet 2.75 (bulletref.exe) を各 ERP で
既定のままだと髪剛体が体側の静的剛体 (下半身/太もも/ひざ) と接触し、net.txt には
その静的剛体が入っていないので比較が成立しない。両側で接触を消してから比べる。
走らせ、後半1/3の |w| 中央と per-frame |Δp| 中央を並べる。

★両側とも接触ゼロで回す (engine: NOCONTACT=1 / bullet: --nocontact)。
  したがってここで出る差は **ジョイントのソルバ構造だけ** に由来する。

使い方:
    python sweep.py                 # 既定の一式を回す
    python sweep.py --only base,L1  # 変種を絞る
    python sweep.py --frames 600
"""

import argparse
import csv
import math
import os
import shutil
import statistics as st
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESTOSC = os.path.join(HERE, "..", "restosc")
# ★モデルの実ファイルはローカルにしか無いので、パスは env から取る。
# 例: MMD_TEST_PMX=<最小網の PMX> python sweep.py
PMX = os.environ.get("MMD_TEST_PMX") or sys.exit("MMD_TEST_PMX を指定してください (最小網の PMX)")
R2D = 57.29577951308232

# name -> extra env for restosc
ENGINE_VARIANTS = [
    ("base",        {}),                                              # 現行既定
    # ─ タスク22 出力ゲート第1段: Bullet 規約を累積で足す ─
    ("ANGCONV",     {"ANGCONV": "1"}),                                 # 角度抽出を Bullet 実挙動へ
    ("ANGCONV_AXES", {"ANGCONV": "1", "AXES": "1"}),                   # + 角度軸を混合軸へ
    ("ANGCONV_AXES_L1", {"ANGCONV": "1", "AXES": "1", "LEVER": "1"}),  # + 線形レバーを 2.75 式へ
    # ─ 参考: タスク20 で測った旧3点 (角度が転置のままの構成) ─
    ("AXES_only",   {"AXES": "1"}),
    ("L1_only",     {"LEVER": "1"}),
]

# name -> extra args for bulletref
BULLET_VARIANTS = [
    ("bullet",       []),                    # Bullet 2.75 既定 (linear/angular とも ERP 0.5)
    ("bullet_erp02", ["--erp", "0.2"]),      # 係数だけ当エンジンに合わせた Bullet
]


def load(p):
    with open(p, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def metrics(state_csv):
    rows = load(state_csv)
    by = {}
    for r in rows:
        by.setdefault(r["name"], []).append(r)
    ws, dps = [], []
    for n, v in by.items():
        w = [math.sqrt(float(r["wx"]) ** 2 + float(r["wy"]) ** 2 + float(r["wz"]) ** 2) * R2D for r in v]
        if max(w) == 0:
            continue          # kinematic anchor
        dp = [math.dist((float(a["px"]), float(a["py"]), float(a["pz"])),
                        (float(b["px"]), float(b["py"]), float(b["pz"])))
              for a, b in zip(v, v[1:])]
        lo = len(w) - len(w) // 3
        ws.append(st.median(w[lo:]))
        dps.append(st.median(dp[len(dp) - len(dp) // 3:]))
    return st.median(ws), st.median(dps)


def err_median(rows_csv):
    rows = load(rows_csv)
    v = [abs(float(r["err"])) for r in rows if r["angular"] == "0" and int(r["frame"]) >= 400]
    return st.median(v) if v else float("nan")


def run_engine(name, env_extra, frames, substeps, iters):
    out = os.path.join(HERE, "sweep", name)
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ)
    for k in ("LEVER", "AXES", "ANGCONV", "JBETA", "JSPLIT", "JWARM", "ANGBETA", "ERRDB", "BIASDB",
              "XSPLIT", "FREEZEAX", "JOINTORDER", "INITSTATE", "ROWTRACE"):
        env.pop(k, None)
    env.update({
        "MMD_TEST_PMX": PMX, "NETDUMP": "1",
        "FRAMES": str(frames), "SUBSTEPS": str(substeps), "ITERS": str(iters),
        "ROWFRAMES": "20", "OUTDIR": out, "NOCONTACT": "1",
    })
    env.update(env_extra)
    r = subprocess.run(["dotnet", "run", "-c", "Release"], cwd=RESTOSC, env=env,
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("restosc failed for " + name)
    echo = [l for l in r.stdout.splitlines() if l.startswith("[実効]")]
    return out, echo


def run_bullet(name, args, frames, substeps, iters, netpath):
    out = os.path.join(HERE, "sweep", name)
    os.makedirs(out, exist_ok=True)
    shutil.copyfile(netpath, os.path.join(out, "net.txt"))
    cmd = [os.path.join(HERE, "bulletref.exe"), "--net", os.path.join(out, "net.txt"),
           "--frames", str(frames), "--substeps", str(substeps), "--iters", str(iters),
           "--rowframes", "20", "--out", out, "--nocontact"] + args
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("bulletref failed for " + name)
    man = [l for l in r.stdout.splitlines() if "manifolds" in l]
    return out, man


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--substeps", type=int, default=2)
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--only", default=None)
    a = ap.parse_args()

    want = set(a.only.split(",")) if a.only else None
    results = []

    print("=" * 104)
    print("タスク20 スイープ: 同一最小網 (Joint5 / 剛体6 / 接触ゼロ) を当エンジンと Bullet 2.75 で")
    print("  %d フレーム (SubSteps=%d, Iters=%d)。判定は後半1/3の中央値。" % (a.frames, a.substeps, a.iters))
    print("=" * 104)

    netpath = None
    for name, env in ENGINE_VARIANTS:
        if want and name not in want:
            continue
        out, echo = run_engine(name, env, a.frames, a.substeps, a.iters)
        if netpath is None or name == "base":
            netpath = os.path.join(out, "net.txt")
        w, dp = metrics(os.path.join(out, "net_engine_state.csv"))
        e = err_median(os.path.join(out, "net_engine_rows.csv"))
        results.append(("自前 " + name, w, dp, e))
        print("  [engine %-11s] %s" % (name, " ".join(sorted(env.items())[0]) if env else "(既定)"))
        for l in echo:
            print("    " + l)

    if netpath is None:
        netpath = os.path.join(HERE, "net.txt")

    for name, args in BULLET_VARIANTS:
        if want and name not in want:
            continue
        out, man = run_bullet(name, args, a.frames, a.substeps, a.iters, netpath)
        w, dp = metrics(os.path.join(out, "net_bullet_state.csv"))
        e = err_median(os.path.join(out, "net_bullet_rows.csv"))
        results.append(("Bullet " + name.replace("bullet", "").lstrip("_") or "Bullet 既定", w, dp, e))
        print("  [bullet %-11s] %s" % (name, " ".join(args) if args else "(2.75 既定 ERP 0.5)"))
        for l in man:
            print("    " + l)

    print()
    print("  %-22s %14s %16s %16s" % ("条件", "|w| 中央 deg/s", "|Δp| 中央/F", "並進|err| 中央"))
    print("  " + "-" * 72)
    for label, w, dp, e in results:
        print("  %-22s %14.4f %16.6g %16.6g" % (label, w, dp, e))
    print("=" * 104)
    print("  ※ 参考: 参照データの静区間フロア |Δp| = 0.0028 〜 0.0128 (髪65本のボーン別中央)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
