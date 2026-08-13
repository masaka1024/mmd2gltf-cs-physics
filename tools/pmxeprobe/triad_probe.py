# -*- coding: utf-8 -*-
# 測定用四面体から、子ボーンに実際に適用された変換を復元する。
#
# PMX のボーンレコードは位置しか持たないので、[現在の形状で保存] してもボーンの向きは残らない。
# そこで子ボーンに BDEF1 で剛体ウェイト付けした4頂点(原点 O と X/Y/Z へ伸ばした3点)を仕込む。
# 保存された頂点はスキニング済みの座標なので、
#     R の各列 = (v_X - v_O, v_Y - v_O, v_Z - v_O)
#     平行移動  = v_O - bind の O
# として、メッシュに実際に効いた変換行列がそのまま読める。
#
# 「ボーンレコードの位置」ではなく「見た目に効いた変換」が取れるのが要点。
# [ボーン位置合わせ再計算] が ON のとき両者が食い違うかどうかを、これで初めて判定できる。
#
# 使い方:
#   python triad_probe.py models/E7_triad_mode1.pmx out/E7_*.pmx

import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_probe import parse  # noqa: E402


TRIAD_BONE = 1  # 子ボーン


def triad_of(m):
    """子ボーンにウェイト付けされた4頂点を、生成順(O, X, Y, Z)で取り出す。"""
    vs = [p for (p, b) in m["verts"] if b == TRIAD_BONE]
    return vs if len(vs) == 4 else None


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def analyze(m):
    t = triad_of(m)
    if t is None:
        return None
    o, x, y, z = t
    cols = (sub(x, o), sub(y, o), sub(z, o))
    lens = tuple(norm(c) for c in cols)

    # 直交性(列同士の内積)。0 でなければ剪断/スケールが混じっている
    def dot(a, b):
        return sum(p * q for p, q in zip(a, b))
    ortho = (dot(cols[0], cols[1]), dot(cols[1], cols[2]), dot(cols[2], cols[0]))

    # X 軸まわりの回転角。このプローブは X のみ可動なので、Z 列の傾きで読む
    cz = cols[2]
    ang_x = math.degrees(math.atan2(-cz[1], cz[2]))

    return dict(origin=o, cols=cols, lens=lens, ortho=ortho, ang_x=ang_x)


def main(argv):
    if len(argv) < 3:
        print("usage: triad_probe.py <baseline.pmx> <result.pmx>...")
        return 1

    paths = []
    for a in argv[1:]:
        g = sorted(glob.glob(a))
        paths.extend(g if g else [a])

    base = parse(paths[0])
    b = analyze(base)
    if b is None:
        print(f"{paths[0]} に測定用四面体がありません(子ボーン重みの頂点が4つ必要)")
        return 1
    print(f"baseline : {os.path.basename(paths[0])}")
    print(f"  O={tuple(round(v, 6) for v in b['origin'])}  軸長={tuple(round(v, 6) for v in b['lens'])}")
    print()

    hdr = (f"{'file':26s} {'回転X°':>10s} {'O移動量':>10s} "
           f"{'軸長(X,Y,Z)':>28s} {'直交性(最大)':>12s}")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for p in paths[1:]:
        m = parse(p)
        r = analyze(m)
        if r is None:
            print(f"{os.path.basename(p):26s} 四面体なし")
            continue
        d_o = norm(sub(r["origin"], b["origin"]))
        lens = "(" + ", ".join(f"{v:.5f}" for v in r["lens"]) + ")"
        print(f"{os.path.basename(p):26s} {r['ang_x']:+10.4f} {d_o:10.6f} "
              f"{lens:>28s} {max(abs(v) for v in r['ortho']):12.2e}")
        rows.append((os.path.basename(p), r))

    if len(rows) >= 2:
        print()
        print("条件間の差 (先頭を基準):")
        ref = rows[0]
        for name, r in rows[1:]:
            d_ang = r["ang_x"] - ref[1]["ang_x"]
            d_o = norm(sub(r["origin"], ref[1]["origin"]))
            same = abs(d_ang) < 1e-4 and d_o < 1e-6
            print(f"  {ref[0]} -> {name:26s} d回転={d_ang:+.4f}° dO={d_o:.6f}"
                  + ("  == 完全一致" if same else ""))

    print()
    print("読み方: 軸長が1.0から外れる/直交性が0でない = 回転以外(スケール・剪断)が混じっている。")
    print("        回転X°が剛体の -5.0000° と一致すれば、メッシュは剛体の向きに従っている。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
