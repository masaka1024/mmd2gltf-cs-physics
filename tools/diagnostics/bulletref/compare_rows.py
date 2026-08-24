# -*- coding: utf-8 -*-
"""
compare_rows.py -- タスク21: 行レベル突き合わせ。最初に食い違った段で止める。

  段(a) 構築の一致   : 第1フレーム第1サブステップの **行集合**
                       (本数 / どの軸に行が立つか / err / bias / 実効質量)
  段(b) 求解の一致   : 同サブステップ内の **反復ごとのインパルス系列とクランプ**
  段(c) 分岐点の特定 : (a)(b) が一致するなら時間方向へ二分探索

入力 (同一ディレクトリ、共通スキーマ):
  rowtrace_engine.csv   restosc NETDUMP=1 ROWTRACE=1 SUBSTEPS=1
  rowtrace_bullet.csv   bulletref.exe --rowtrace --substeps 1

共通スキーマ:
  frame,substep,iter,joint,dof,angular,axisX,axisY,axisZ,err,bias,targetVel,
  lower,upper,effMass,appliedImpulse,dImpulse,clamped,relVelBefore,relVelAfter
  (Bullet 側はさらに生値 errBt,posErrBt,relVelSetupBt,appliedBt,lowerBt,upperBt を持つ)

★符号について: Bullet の行は当エンジンの鏡像 (相対速度を Bullet は A基準、当方は B基準で測る)
  ので err / 目標 / 相対速度 / インパルスが**まとめて反転**する。bulletref はその変換を
  かけた列を出しつつ、**生値も残している**。角度行についてはこの変換が正しいか自明でないので、
  本スクリプトは両符号での一致を数え、データから判定する。

使い方:
    python compare_rows.py [--dir t21] [--frame 0] [--tol 1e-6]
"""

import argparse
import csv
import math
import os
import sys


def load(path):
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def fl(r, k):
    v = r.get(k, "")
    if v is None or v == "":
        return float("nan")
    try:
        return float(v)
    except ValueError:
        return float("nan")


def key(r):
    return (r["joint"], int(r["dof"]), int(r["angular"]))


def kstr(k):
    return "%s dof%d %s" % (k[0], k[1], "回転" if k[2] else "並進")


def close(a, b, tol):
    if math.isnan(a) and math.isnan(b):
        return True
    if math.isnan(a) or math.isnan(b):
        return False
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


# ---------------------------------------------------------------------------

def stage_a(E, B, frame, tol, out):
    e = {key(r): r for r in E if int(r["frame"]) == frame and int(r["iter"]) == -1}
    b = {key(r): r for r in B if int(r["frame"]) == frame and int(r["iter"]) == -1}
    out("=" * 110)
    out("段(a) 構築の一致  frame=%d substep=0 iter=-1 (行を作った直後)" % frame)
    out("=" * 110)
    out("  行数: 自前 %d / Bullet %d" % (len(e), len(b)))

    onlyE = sorted(set(e) - set(b))
    onlyB = sorted(set(b) - set(e))
    both = sorted(set(e) & set(b))

    if onlyE or onlyB:
        out("  ★行集合が違う")
        if onlyE:
            out("    自前だけに立つ行 (%d):" % len(onlyE))
            for k in onlyE:
                r = e[k]
                out("      %-22s err=%-14.6g bias=%-14.6g effMass=%-12.6g lower=%-11.4g upper=%-11.4g"
                    % (kstr(k), fl(r, "err"), fl(r, "bias"), fl(r, "effMass"),
                       fl(r, "lower"), fl(r, "upper")))
        if onlyB:
            out("    Bulletだけに立つ行 (%d):" % len(onlyB))
            for k in onlyB:
                r = b[k]
                out("      %-22s err=%-14.6g bias=%-14.6g effMass=%-12.6g"
                    % (kstr(k), fl(r, "err"), fl(r, "bias"), fl(r, "effMass")))
    else:
        out("  行集合は一致 (%d 行)" % len(both))

    out("")
    out("  共通行の量の比較 (tol=%g):" % tol)
    out("  %-22s %14s %14s %10s | %13s %13s %8s | %12s %12s %8s"
        % ("row", "err 自前", "err Bullet", "一致", "effMass 自前", "effMass Bullet", "一致",
           "bias 自前", "bias Bullet", "一致"))
    nbad = 0
    for k in both:
        re_, rb = e[k], b[k]
        ee, eb = fl(re_, "err"), fl(rb, "err")
        me, mb = fl(re_, "effMass"), fl(rb, "effMass")
        be, bb = fl(re_, "bias"), fl(rb, "bias")
        oke, okm, okb = close(ee, eb, tol), close(me, mb, tol), close(be, bb, tol)
        if not (oke and okm and okb):
            nbad += 1
        out("  %-22s %14.6g %14.6g %10s | %13.6g %13.6g %8s | %12.6g %12.6g %8s"
            % (kstr(k), ee, eb, "OK" if oke else "★NG",
               me, mb, "OK" if okm else "★NG", be, bb, "OK" if okb else "★NG"))

    # 軸の向き (符号込み) も見る
    out("")
    out("  行の軸 (正規化なし):")
    for k in both:
        ae = [fl(e[k], "axis" + c) for c in "XYZ"]
        ab = [fl(b[k], "axis" + c) for c in "XYZ"]
        dot = sum(x * y for x, y in zip(ae, ab))
        na = math.sqrt(sum(x * x for x in ae)) or 1.0
        nb = math.sqrt(sum(x * x for x in ab)) or 1.0
        out("      %-22s 自前(%8.5f %8.5f %8.5f) |a|=%.5f  Bullet(%8.5f %8.5f %8.5f) |b|=%.5f  cos=%+.6f"
            % (kstr(k), ae[0], ae[1], ae[2], na, ab[0], ab[1], ab[2], nb, dot / (na * nb)))

    ok = (not onlyE) and (not onlyB) and nbad == 0
    out("")
    out("  段(a) 判定: %s" % ("一致 → 段(b) へ" if ok else "★不一致。ここで止める"))
    return ok, both


def stage_b(E, B, frame, both, tol, out):
    out("")
    out("=" * 110)
    out("段(b) 求解の一致  frame=%d substep=0 反復ごとの累積インパルスとクランプ" % frame)
    out("=" * 110)
    firstbad = None
    for k in both:
        se = sorted([r for r in E if key(r) == k and int(r["frame"]) == frame and int(r["iter"]) >= 0],
                    key=lambda r: int(r["iter"]))
        sb = sorted([r for r in B if key(r) == k and int(r["frame"]) == frame and int(r["iter"]) >= 0],
                    key=lambda r: int(r["iter"]))
        out("  --- %s ---" % kstr(k))
        out("    %4s %14s %14s %10s | %8s %8s"
            % ("iter", "acc 自前", "acc Bullet", "一致", "clmp自前", "clmpBul"))
        n = min(len(se), len(sb))
        for i in range(n):
            ae, ab = fl(se[i], "appliedImpulse"), fl(sb[i], "appliedImpulse")
            ce, cb = se[i]["clamped"], sb[i]["clamped"]
            okv = close(ae, ab, tol) and ce == cb
            if not okv and firstbad is None:
                firstbad = (k, i, ae, ab)
            out("    %4d %14.7g %14.7g %10s | %8s %8s"
                % (i, ae, ab, "OK" if okv else "★NG", ce, cb))
    out("")
    out("  段(b) 判定: %s" % ("一致 → 段(c) へ" if firstbad is None
                            else "★不一致。最初の食い違い: %s iter=%d 自前 %.7g / Bullet %.7g"
                                 % (kstr(firstbad[0]), firstbad[1], firstbad[2], firstbad[3])))
    return firstbad is None


def stage_c(E, B, tol, out):
    out("")
    out("=" * 110)
    out("段(c) 分岐点の特定  err の軌跡が分かれ始める最初のフレーム")
    out("=" * 110)
    frames = sorted(set(int(r["frame"]) for r in E) & set(int(r["frame"]) for r in B))

    def snap(rows, f):
        return {key(r): fl(r, "err") for r in rows if int(r["frame"]) == f and int(r["iter"]) == -1}

    def agree(f):
        a, b = snap(E, f), snap(B, f)
        if set(a) != set(b):
            return False
        return all(close(a[k], b[k], tol) for k in a)

    first = None
    for f in frames:
        if not agree(f):
            first = f
            break
    if first is None:
        out("  取得した全フレーム (%d 個) で一致。分岐点は観測窓の外。" % len(frames))
        return
    out("  最初に食い違うフレーム: F%d" % first)
    a, b = snap(E, first), snap(B, first)
    onlyE, onlyB = sorted(set(a) - set(b)), sorted(set(b) - set(a))
    if onlyE:
        out("    自前だけの行  : %s" % ", ".join(kstr(k) for k in onlyE))
    if onlyB:
        out("    Bulletだけの行: %s" % ", ".join(kstr(k) for k in onlyB))
    for k in sorted(set(a) & set(b)):
        if not close(a[k], b[k], tol):
            out("    %-22s err 自前 %.7g / Bullet %.7g  (差 %.3g)"
                % (kstr(k), a[k], b[k], a[k] - b[k]))


def stage_a0(dirpath, frame, out):
    """段(a) の前段: ジョイント相対角そのものを突き合わせる。
       行が立つかどうかは角度で決まるので、角度が違えば行集合の差はその結果でしかない。
       angles_*.csv は「行が立たない (リミット内) 軸も含めて」3軸全部を持っている。"""
    import csv as _csv
    ep = os.path.join(dirpath, "angles_engine.csv")
    bp = os.path.join(dirpath, "angles_bullet.csv")
    if not (os.path.isfile(ep) and os.path.isfile(bp)):
        return
    E = {(r["joint"], int(r["dof"])): r
         for r in _csv.DictReader(open(ep, encoding="utf-8")) if int(r["frame"]) == frame}
    B = {(r["joint"], int(r["dof"])): r
         for r in _csv.DictReader(open(bp, encoding="utf-8")) if int(r["frame"]) == frame}
    out("=" * 110)
    out("段(a0) ジョイント相対角の一致  frame=%d  (単位 rad)" % frame)
    out("  state: 自前 0=free 1=範囲内(行なし) 2=locked 3=下限外 4=上限外")
    out("         Bullet m_currentLimit 0=範囲内(行なし) 1=下限外 2=上限外 3=locked")
    out("=" * 110)
    out("  %-14s %4s | %13s %13s %10s | %6s %6s | %13s %13s"
        % ("joint", "dof", "cur 自前", "cur Bullet", "差(deg)", "st自前", "stBul", "err 自前", "err Bullet"))
    nbad = 0
    for k in sorted(set(E) & set(B)):
        e, b = E[k], B[k]
        ce, cb = float(e["cur"]), float(b["cur"])
        if abs(ce - cb) > 1e-6:
            nbad += 1
        out("  %-14s %4d | %13.7g %13.7g %10.4f | %6s %6s | %13.6g %13.6g"
            % (k[0], k[1], ce, cb, math.degrees(ce - cb), e["state"], b["state"],
               float(e["err"]), float(b["err"])))
    out("")
    out("  角度が食い違う DOF: %d / %d" % (nbad, len(set(E) & set(B))))
    out("")

def scan(E, B, out):
    """全トレースフレームで「行集合が一致するか」だけを走査する。
       段(a) の不一致がバインド姿勢だけの縮退なのか、構造的に続くのかを見分けるため。"""
    out("=" * 110)
    out("走査: 行集合が一致するフレーム / しないフレーム")
    out("=" * 110)

    def rowset(rows):
        d = {}
        for r in rows:
            if int(r["iter"]) != -1:
                continue
            d.setdefault(int(r["frame"]), set()).add(key(r))
        return d

    de, db = rowset(E), rowset(B)
    frames = sorted(set(de) & set(db))
    same, diff = [], []
    for f in frames:
        (same if de[f] == db[f] else diff).append(f)
    out("  トレース済フレーム %d 個 / 行集合一致 %d 個 / 不一致 %d 個"
        % (len(frames), len(same), len(diff)))
    if same:
        out("  一致するフレーム (先頭20): %s" % same[:20])
    out("")
    out("  行数の推移 (先頭12フレームと末尾窓の先頭6フレーム):")
    show = frames[:12] + [f for f in frames if f >= 800][:6]
    for f in show:
        oe, ob = sorted(de[f] - db[f]), sorted(db[f] - de[f])
        out("    F%-5d 自前 %2d 行 / Bullet %2d 行  %s"
            % (f, len(de[f]), len(db[f]),
               "一致" if not oe and not ob else
               ("自前だけ[" + ", ".join(kstr(k) for k in oe) + "] " if oe else "")
               + ("Bulletだけ[" + ", ".join(kstr(k) for k in ob) + "]" if ob else "")))
    # 行ごとの「立つ率」
    out("")
    out("  行が立つ率 (トレース済フレーム全体):")
    allk = sorted(set().union(*de.values()) | set().union(*db.values()))
    out("    %-24s %10s %10s" % ("row", "自前", "Bullet"))
    for k in allk:
        ce = sum(1 for f in frames if k in de[f])
        cb = sum(1 for f in frames if k in db[f])
        out("    %-24s %9.1f%% %9.1f%%" % (kstr(k), 100.0 * ce / len(frames), 100.0 * cb / len(frames)))
    return same


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "t21"))
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--tol", type=float, default=1e-6)
    ap.add_argument("--out", default=None)
    ap.add_argument("--scan", action="store_true", help="全フレームの行集合一致を走査する")
    a = ap.parse_args()

    E = load(os.path.join(a.dir, "rowtrace_engine.csv"))
    B = load(os.path.join(a.dir, "rowtrace_bullet.csv"))

    buf = []

    def out(s=""):
        print(s)
        buf.append(s)

    if a.scan:
        scan(E, B, out)
        out("")

    stage_a0(a.dir, a.frame, out)
    ok, both = stage_a(E, B, a.frame, a.tol, out)
    if ok:
        ok = stage_b(E, B, a.frame, both, a.tol, out)
    if ok:
        stage_c(E, B, a.tol, out)

    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write("\n".join(buf) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
