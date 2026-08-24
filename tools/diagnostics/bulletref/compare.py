# -*- coding: utf-8 -*-
"""
compare.py -- タスク20 の3段階突き合わせ。当エンジン (restosc NETDUMP=1) と
              本物の Bullet 2.75 (bulletref.exe) を、同じ最小網で並べる。

  段階1 初期一致          … frame 0 の剛体状態と、ジョイントのアンカー
  段階2 1サブステップ目の行 … 行ごとの err / 目標速度 / 構築時の相対速度
  段階3 定常 err の分岐点   … |w| と |err| の時系列がどこで離れるか

使い方:
    python compare.py [--dir .] [--stage 0|1|2|3]

前提 (同じディレクトリに揃っていること):
    net_engine_state.csv / net_engine_rows.csv   restosc NETDUMP=1 の出力
    net_bullet_state.csv / net_bullet_rows.csv   bulletref.exe の出力
"""

import argparse
import csv
import math
import os
import statistics as st
import sys


def load(p):
    with open(p, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    return float(r[k])


def vec(r, *keys):
    return tuple(float(r[k]) for k in keys)


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def med(v):
    return st.median(v) if v else float("nan")


def p90(v):
    return sorted(v)[int(len(v) * 0.9)] if v else float("nan")


R2D = 57.29577951308232


# ---------------------------------------------------------------------------

def stage1(eng, bul):
    print("=" * 100)
    print("段階1: 初期一致  (frame 0 のフレーム末状態 -- 1フレーム進んだ後なので厳密一致はしない。")
    print("       ここで見るのは『同じ網が同じ初期姿勢で立ち上がっているか』)")
    print("=" * 100)
    e0 = [r for r in eng if r["frame"] == "0"]
    b0 = [r for r in bul if r["frame"] == "0"]
    print("  %-10s %14s %14s %14s %14s" % ("剛体", "位置差", "|w| 自前", "|w| Bullet", "速度差"))
    worst = 0.0
    for a, b in zip(e0, b0):
        if a["name"] != b["name"]:
            print("  ★NG 剛体の並びが違う: %s vs %s" % (a["name"], b["name"]))
            return
        dp = dist(vec(a, "px", "py", "pz"), vec(b, "px", "py", "pz"))
        wa = math.sqrt(sum(fnum(a, k) ** 2 for k in ("wx", "wy", "wz"))) * R2D
        wb = math.sqrt(sum(fnum(b, k) ** 2 for k in ("wx", "wy", "wz"))) * R2D
        dv = dist(vec(a, "vx", "vy", "vz"), vec(b, "vx", "vy", "vz"))
        worst = max(worst, dp)
        print("  %-10s %14.6g %14.4f %14.4f %14.6g" % (a["name"], dp, wa, wb, dv))
    print("  -> 最大位置差 %.6g  %s" % (worst, "OK (実質一致)" if worst < 1e-3 else "★差あり"))
    print()


def stage2(er, br):
    print("=" * 100)
    print("段階2: 1サブステップ目の行一致  (frame=0, sub=0)")
    print("  err は当エンジンの符号に揃えてある (bulletref が変換済み)。")
    print("  targetVel は係数を含む: 自前 Beta=0.2 / Bullet 既定 ERP=0.5。")
    print("  係数差を抜いて比べたい場合は  bulletref.exe --erp 0.2  で焼き直すこと。")
    print("=" * 100)
    ee = {(r["joint"], r["dof"], r["angular"]): r for r in er if r["frame"] == "0" and r["sub"] == "0"}
    bb = {(r["joint"], r["dof"], r["angular"]): r for r in br if r["frame"] == "0" and r["sub"] == "0"}
    keys = sorted(set(ee) | set(bb), key=lambda k: (k[0], k[2], k[1]))
    print("  %-14s %4s %4s %13s %13s %11s %11s %11s %11s"
          % ("joint", "dof", "ang", "err 自前", "err Bullet", "tv 自前", "tv Bullet", "rv 自前", "rv Bullet"))
    onlyE, onlyB = [], []
    for k in keys:
        a, b = ee.get(k), bb.get(k)
        if a is None:
            onlyB.append(k)
        elif b is None:
            onlyE.append(k)
        print("  %-14s %4s %4s %13s %13s %11s %11s %11s %11s"
              % (k[0], k[1], "回" if k[2] == "1" else "並",
                 ("%.6g" % fnum(a, "err")) if a else "-",
                 ("%.6g" % fnum(b, "err")) if b else "-",
                 ("%.5g" % fnum(a, "targetVel")) if a else "-",
                 ("%.5g" % fnum(b, "targetVel")) if b else "-",
                 ("%.5g" % fnum(a, "relVel")) if a else "-",
                 ("%.5g" % fnum(b, "relVel")) if b else "-"))
    print("  行の集合: 自前 %d / Bullet %d   自前だけ %s   Bulletだけ %s"
          % (len(ee), len(bb), onlyE if onlyE else "なし", onlyB if onlyB else "なし"))
    both = [k for k in keys if k in ee and k in bb]
    if both:
        de = [abs(fnum(ee[k], "err") - fnum(bb[k], "err")) for k in both]
        print("  err の差: 最大 %.6g / 中央 %.6g" % (max(de), med(de)))
    print()


def stage3(es, bs, er, br, win):
    print("=" * 100)
    print("段階3: 定常 err の分岐点")
    print("=" * 100)

    def series(rows):
        by = {}
        for r in rows:
            by.setdefault(r["name"], []).append(r)
        names = [n for n in by if any(
            abs(fnum(r, "wx")) + abs(fnum(r, "wy")) + abs(fnum(r, "wz")) > 0 for r in by[n])]
        out = {}
        for n in names:
            v = by[n]
            out[n] = {
                "w": [math.sqrt(sum(fnum(r, k) ** 2 for k in ("wx", "wy", "wz"))) * R2D for r in v],
                "dp": [dist(vec(a, "px", "py", "pz"), vec(b, "px", "py", "pz"))
                       for a, b in zip(v, v[1:])],
            }
        return out

    E, B = series(es), series(bs)
    common = [n for n in E if n in B]
    nf = min(len(E[n]["w"]) for n in common)

    print("[|w| 中央 (deg/s) の推移]  窓 %dF" % win)
    print("  %6s %8s %12s %12s %10s" % ("窓", "秒(60fps)", "自前", "Bullet", "比"))
    nwin = nf // win
    for k in range(nwin):
        s = slice(k * win, (k + 1) * win)
        we = med([x for n in common for x in E[n]["w"][s]])
        wb = med([x for n in common for x in B[n]["w"][s]])
        print("  %6d %8.2f %12.4f %12.4f %10s"
              % (k, k * win / 60.0, we, wb, "%.1f" % (we / wb) if wb > 1e-9 else "-"))
    print()

    print("[後半1/3 の水準]")
    lo = nf - nf // 3
    print("  %-10s %12s %12s %10s | %13s %13s %10s"
          % ("剛体", "|w| 自前", "|w| Bullet", "比", "|dp| 自前", "|dp| Bullet", "比"))
    for n in common:
        we, wb = med(E[n]["w"][lo:]), med(B[n]["w"][lo:])
        pe, pb = med(E[n]["dp"][lo:]), med(B[n]["dp"][lo:])
        print("  %-10s %12.4f %12.4f %10s | %13.6g %13.6g %10s"
              % (n, we, wb, "%.1f" % (we / wb) if wb > 1e-9 else "-",
                 pe, pb, "%.1f" % (pe / pb) if pb > 1e-12 else "-"))
    allE = [x for n in common for x in E[n]["w"][lo:]]
    allB = [x for n in common for x in B[n]["w"][lo:]]
    print("  " + "-" * 84)
    print("  %-10s %12.4f %12.4f %10s"
          % ("全体中央", med(allE), med(allB), "%.1f" % (med(allE) / med(allB)) if med(allB) > 1e-9 else "-"))
    print()

    print("[並進ロック行の |err| 中央 (拘束されている並進行のみ)]")
    def errseries(rows):
        by = {}
        for r in rows:
            if r["angular"] == "1":
                continue
            by.setdefault(int(r["frame"]), []).append(abs(fnum(r, "err")))
        return by
    Ee, Be = errseries(er), errseries(br)
    fr = sorted(set(Ee) & set(Be))
    early = [f for f in fr if f < 20]
    late = [f for f in fr if f >= max(fr) - 200] if fr else []
    for label, sel in (("先頭20F", early), ("末尾窓", late)):
        if not sel:
            continue
        ve = [x for f in sel for x in Ee[f]]
        vb = [x for f in sel for x in Be[f]]
        print("  %-8s 自前 中央 %.6g / p90 %.6g    Bullet 中央 %.6g / p90 %.6g   比 %s"
              % (label, med(ve), p90(ve), med(vb), p90(vb),
                 "%.1f" % (med(ve) / med(vb)) if med(vb) > 1e-12 else "-"))
    print()

    print("[分岐点] フレームごとの |w| 中央の比 (自前/Bullet) が各倍率を超えた最初のフレーム")
    we = [med([x for n in common for x in [E[n]["w"][f]]]) for f in range(nf)]
    wb = [med([x for n in common for x in [B[n]["w"][f]]]) for f in range(nf)]
    for thr in (1.5, 2, 5, 10, 50):
        hit = None
        for f in range(nf):
            if wb[f] > 1e-9 and we[f] / wb[f] >= thr:
                hit = f
                break
        print("  %5.1f 倍 : %s" % (thr, ("F%d (%.3f 秒)" % (hit, hit / 60.0)) if hit is not None else "到達せず"))
    print("=" * 100)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--stage", type=int, default=0, help="0=すべて")
    ap.add_argument("--win", type=int, default=60)
    a = ap.parse_args()

    def P(n):
        return os.path.join(a.dir, n)

    for n in ("net_engine_state.csv", "net_engine_rows.csv",
              "net_bullet_state.csv", "net_bullet_rows.csv"):
        if not os.path.isfile(P(n)):
            print("★NG %s が無い" % P(n))
            return 1

    es, er = load(P("net_engine_state.csv")), load(P("net_engine_rows.csv"))
    bs, br = load(P("net_bullet_state.csv")), load(P("net_bullet_rows.csv"))

    if a.stage in (0, 1):
        stage1(es, bs)
    if a.stage in (0, 2):
        stage2(er, br)
    if a.stage in (0, 3):
        stage3(es, bs, er, br, a.win)
    return 0


if __name__ == "__main__":
    sys.exit(main())
