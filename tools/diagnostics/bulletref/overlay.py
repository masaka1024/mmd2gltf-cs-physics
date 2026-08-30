# -*- coding: utf-8 -*-
"""
overlay.py -- 減衰カーブを3者で重ねる。

  1. PMXエディタのベイク (純Bullet)         … static600_baked_OFF.vmd のボーン位置
  2. bulletref (Bullet 2.75 実ビルド)       … net_bullet_state.csv の剛体位置
  3. 当エンジン                              … net_engine_state.csv の剛体位置

指標は per-frame |Δp| (PMX単位/フレーム **@30fps**)。
★ハーネス側の CSV は 1/60 秒フレームなので **2フレームおきに間引いて** 30fps に揃える。
  これをやらないと 2倍ずれる。

★ハーネスは剛体位置、PMXe はボーン位置。両者は BodyOffsetFromBone だけ離れた剛な関係なので
  |Δp| は厳密には一致しないが、桁と減衰形状の比較には足りる。
  (既存のアンカー 0.04091 は剛体位置、参照フロア 0.0028〜0.0128 はボーン位置。元から混在している)

使い方:
    python overlay.py [--dir final] [--vmd <baked.vmd>] [--csv out.csv]
"""

import argparse
import csv
import importlib.util
import os
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# ★モデルの実ファイルはローカルにしか無いので、パスは env から取る。
PMX = os.environ.get("MMD_TEST_PMX") or sys.exit("MMD_TEST_PMX を指定してください (最小網の PMX)")
# ★ベイク VMD もローカルにしか無いので env から取る。
VMD = os.environ.get("MMD_TEST_VMD") or sys.exit("MMD_TEST_VMD を指定してください (ベイク済み VMD)")
COMPOSE = os.path.join(HERE, "..", "..", "..", "reference", "vmd_pose_dump", "compose.py")
TARGET_BONES = ["髪BR1", "髪BR2", "髪BR3", "髪BR4", "髪BCR3"]


def curve_from_state(p, step=2):
    """1/60秒フレームの state CSV から 30fps 相当の per-frame |Δp| を作る。"""
    rows = list(csv.DictReader(open(p, encoding="utf-8")))
    by = {}
    for r in rows:
        by.setdefault(r["name"], []).append(r)
    out = {}
    for n, v in by.items():
        P = np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in v])
        if np.abs(np.diff(P, axis=0)).max() == 0:
            continue                      # kinematic なアンカーは除く
        out[n] = np.linalg.norm(np.diff(P[::step], axis=0), axis=1)
    return out


def curve_from_vmd(vmd, pmx, bones_want):
    spec = importlib.util.spec_from_file_location("vpd_compose", COMPOSE)
    C = importlib.util.module_from_spec(spec)
    sys.modules["vpd_compose"] = C
    spec.loader.exec_module(C)
    bones, _ = C.parse_pmx(pmx)
    order, _ = C.build_order(bones)
    nidx = {b["name"]: i for i, b in enumerate(bones)}
    keys = C.parse_vmd(vmd)
    nf = max(max(f for f, _, _ in v) for v in keys.values()) + 1
    Wp, _ = C.compose_all(bones, order, keys, nf)
    return {b: np.linalg.norm(np.diff(np.array(Wp[nidx[b]]), axis=0), axis=1)
            for b in bones_want if b in nidx}


def agg(d, s, e):
    v = [x for n in d for x in d[n][s:e]]
    return st.median(v) if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(HERE, "final"))
    ap.add_argument("--vmd", default=VMD)
    ap.add_argument("--pmx", default=PMX)
    ap.add_argument("--win", type=int, default=15, help="窓幅 [30fpsフレーム] (既定15=0.5秒)")
    ap.add_argument("--csv", default=None)
    a = ap.parse_args()

    P = curve_from_vmd(a.vmd, a.pmx, TARGET_BONES)
    B = curve_from_state(os.path.join(a.dir, "net_bullet_state.csv"))
    E = curve_from_state(os.path.join(a.dir, "net_engine_state.csv"))

    n = max(max(len(v) for v in d.values()) for d in (P, B, E))
    print("=" * 96)
    print("減衰カーブ 3者重ね  per-frame |Δp| (PMX単位/フレーム @30fps)  窓 %d F (%.2f 秒)"
          % (a.win, a.win / 30.0))
    print("  %4s %7s %16s %16s %14s %11s"
          % ("窓", "秒", "PMXe(純Bullet)", "bulletref2.75", "自前", "自前/PMXe"))
    print("=" * 96)
    for w in range(n // a.win):
        s, e = w * a.win, (w + 1) * a.win
        x, y, z = agg(P, s, e), agg(B, s, e), agg(E, s, e)
        print("  %4d %7.2f %16.5g %16.5g %14.5g %11s"
              % (w, w * a.win / 30.0, x, y, z,
                 "%.1f" % (z / x) if x == x and z == z and x > 0 else "-"))

    print()
    print("後半窓 (F200 以降 @30fps) のボーン/剛体別中央の中央値:")
    for tag, d in (("PMXe (補正OFF)", P), ("bulletref (Bullet 2.75)", B), ("自前エンジン", E)):
        per = [st.median(v[200:]) for v in d.values() if len(v) > 200]
        if not per:
            per = [st.median(v[len(v) * 2 // 3:]) for v in d.values()]
        print("  %-26s %.5g   (個別: %s)"
              % (tag, st.median(per), " / ".join("%.4g" % x for x in sorted(per))))
    print("=" * 96)

    if a.csv:
        with open(a.csv, "w", encoding="utf-8", newline="") as f:
            f.write("frame30,sec,pmxe,bulletref,engine\n")
            for i in range(n):
                def at(d):
                    v = [dd[i] for dd in d.values() if i < len(dd)]
                    return st.median(v) if v else ""
                f.write("%d,%.4f,%s,%s,%s\n" % (i + 1, (i + 1) / 30.0, at(P), at(B), at(E)))
        print("書き出した: %s" % a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
