# -*- coding: utf-8 -*-
# PmxEditor の [Fixモーションの非物理化保存] で吐いた VMD を2本比べる。
#
# なぜ VMD 比較なのか:
#   静的平衡のサンプリングでは「過渡にだけ効く補正」が原理的に見えない。
#   Fix ベイクなら 1F 毎に全キーが配置される(readme.txt:4043)ので、軌跡ごと比較できる。
#   収束待ちも不要になり、GUI 操作は条件あたり1回で済む。
#
# 使い方:
#   python compare_vmd.py out/E9_fix_A1B1.vmd out/E9_fix_A1B0.vmd

import math
import os
import struct
import sys


def parse_vmd(path):
    d = open(path, "rb").read()
    assert d[:25] == b"Vocaloid Motion Data 0002", "VMD ヘッダが違う"
    o = 50
    n = struct.unpack("<I", d[o:o + 4])[0]
    o += 4

    bones = {}
    for _ in range(n):
        name = d[o:o + 15].split(b"\x00")[0].decode("shift_jis", "replace")
        frame = struct.unpack("<I", d[o + 15:o + 19])[0]
        pos = struct.unpack("<3f", d[o + 19:o + 31])
        quat = struct.unpack("<4f", d[o + 31:o + 47])
        o += 111
        bones.setdefault(name, {})[frame] = (pos, quat)
    return bones


def quat_angle_deg(a, b):
    """2つの四元数の間の角度(度)。符号の任意性を吸収する。

    2*acos(|dot|) は使わない。ほぼ一致する四元数では条件数が最悪で、
    float32 の丸め(1-dot ~ 1e-7)だけで sqrt(2e-7) = 0.026° の偽の差を出す。
    実際それに騙されて「差がある」と誤読した(2026-08-13)。
    相対四元数の atan2 で測れば 0 近傍でも素直に 0 になる。
    """
    if sum(x * y for x, y in zip(a, b)) < 0.0:
        b = tuple(-v for v in b)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    # r = b * conj(a)
    rw = bw * aw + bx * ax + by * ay + bz * az
    rx = bx * aw - bw * ax - by * az + bz * ay
    ry = by * aw - bw * ay - bz * ax + bx * az
    rz = bz * aw - bw * az - bx * ay + by * ax
    v = math.sqrt(rx * rx + ry * ry + rz * rz)
    return math.degrees(2.0 * math.atan2(v, abs(rw)))


def main(argv):
    if len(argv) < 3:
        print("usage: compare_vmd.py <a.vmd> <b.vmd>")
        return 1

    A = parse_vmd(argv[1])
    B = parse_vmd(argv[2])
    na, nb = os.path.basename(argv[1]), os.path.basename(argv[2])

    only_a = sorted(set(A) - set(B))
    only_b = sorted(set(B) - set(A))
    if only_a or only_b:
        print(f"※片方にしかないボーン: A={only_a} B={only_b}")

    common = sorted(set(A) & set(B))
    print(f"{na}  ボーン{len(A)}本")
    print(f"{nb}  ボーン{len(B)}本")
    print(f"共通 {len(common)} 本を比較")
    print()

    w = max((len(b) for b in common), default=10) + 1
    hdr = f"{'bone':{w}s} {'共通F':>7s} {'最大d位置':>11s} {'@F':>6s} {'最大d回転°':>12s} {'@F':>6s}"
    print(hdr)
    print("-" * len(hdr))

    worst = []
    for b in common:
        fa, fb = A[b], B[b]
        frames = sorted(set(fa) & set(fb))
        if not frames:
            print(f"{b:{w}s} 共通フレームなし")
            continue
        mp, mpf, mr, mrf = 0.0, -1, 0.0, -1
        for f in frames:
            pa, qa = fa[f]
            pb, qb = fb[f]
            dp = math.sqrt(sum((x - y) ** 2 for x, y in zip(pa, pb)))
            dr = quat_angle_deg(qa, qb)
            if dp > mp: mp, mpf = dp, f
            if dr > mr: mr, mrf = dr, f
        print(f"{b:{w}s} {len(frames):7d} {mp:11.6f} {mpf:6d} {mr:12.4f} {mrf:6d}")
        worst.append((max(mp, mr / 100.0), b, mp, mr))

    print()
    if not worst:
        print("比較できるデータがありません")
        return 1
    worst.sort(reverse=True)
    top = worst[0]
    if top[2] < 1e-5 and top[3] < 1e-3:
        print("★全ボーン・全フレームで一致。2条件の軌跡は同一。")
    else:
        print(f"★最大の差: {top[1]}  位置 {top[2]:.6f} / 回転 {top[3]:.4f}°")
        print("  過渡だけに出るのか、最後まで残るのかは @F 列を見る")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
