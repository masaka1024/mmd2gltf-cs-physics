# -*- coding: utf-8 -*-
# PmxEditor の [ファイル]-[現在の形状で保存] で吐いた PMX を読み、
# 補正トグル4条件の静的平衡を比較する。
#
# なぜ VPD でなく PMX か:
#   VPD は「変形分のみ」(readme.txt:3769) で、物理で動いたぶんは記録されない
#   (実測: 4条件とも bone count 0 の空ファイル)。
#   [現在の形状で保存] は「剛体／Jointも形状に合わせて変形され」るので、
#   剛体の位置・回転がそのままレコードへ書かれる = 測りたい量が直接取れる。
#
# 使い方:
#   python compare_probe.py models/E2_lim5_m1.pmx out/E2_m1_A*.pmx

import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_probe import parse  # noqa: E402


L0_LOCAL = 2.0  # 子ローカルでのジョイント位置(初期の腕長)。build_pendulum の LINK と一致


def deg(r):
    return r * 180.0 / math.pi


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vlen(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def measure(m):
    """親剛体をアンカーとした腕ベクトルから、静的平衡の各指標を出す。
    モデルは build_pendulum 前提: 剛体0=親(ボーン追従) 剛体1=子(動的)、腕は初期 -Z 水平。"""
    bodies = m["bodies"]
    bones = m["bones"]
    if len(bodies) < 2:
        return None

    anchor = bodies[0]["pos"]
    child = bodies[1]["pos"]
    arm = vsub(child, anchor)
    L = vlen(arm)

    # ★移動ロックの真の違反量。
    # 腕長の変化だけを見ると半径方向成分しか拾えず過小評価になる(実測 0.0085 vs 真値 0.186)。
    # 子ローカルで見たジョイント位置 r_local を剛体の回転で回し、A側アンカーとの距離を取る。
    th = bodies[1]["rot"][0]  # X 軸まわり(このプローブは X のみ可動)
    r_local = (0.0, 0.0, L0_LOCAL)
    c, s = math.cos(th), math.sin(th)
    w = (r_local[0], r_local[1] * c - r_local[2] * s, r_local[1] * s + r_local[2] * c)
    joint_b = (child[0] + w[0], child[1] + w[1], child[2] + w[2])
    anchor_err = vlen(vsub(joint_b, anchor))

    # 水平(-Z)からどれだけ下へ倒れたか。腕が -Z 初期なので、
    # 傾き = atan2(-dy, -dz) : 真下向きが +90°
    tilt = deg(math.atan2(-arm[1], -arm[2]))

    # ボーン側(スキニングに効くのはこちら)
    bone_arm = vsub(bones[1][1], bones[0][1]) if len(bones) >= 2 else (0, 0, 0)
    bone_tilt = deg(math.atan2(-bone_arm[1], -bone_arm[2]))
    bone_L = vlen(bone_arm)

    return dict(
        anchor_err=anchor_err,  # ★移動ロックの違反量(真値)
        arm_len=L,            # 半径方向成分のみ。過小評価するので参考値
        tilt=tilt,            # 剛体の傾き
        body_rot=tuple(deg(v) for v in bodies[1]["rot"]),  # 剛体レコードの回転
        bone_tilt=bone_tilt,  # ボーンの傾き
        bone_len=bone_L,
        body_pos=child,
        bone_pos=bones[1][1] if len(bones) >= 2 else None,
    )


def joint_limit_deg(m):
    if not m["joints"]:
        return None
    j = m["joints"][0]
    return (deg(j["ang"][0][0]), deg(j["ang"][1][0]))


def main(argv):
    if len(argv) < 2:
        print(__doc__ or "usage: compare_probe.py <baseline.pmx> <result*.pmx>")
        return 1

    paths = []
    for a in argv[1:]:
        g = sorted(glob.glob(a))
        paths.extend(g if g else [a])

    base = parse(paths[0])
    lim = joint_limit_deg(base)
    b0 = measure(base)
    if b0 is None:
        print("baseline に剛体が2つありません"); return 1

    print(f"baseline : {os.path.basename(paths[0])}")
    print(f"  回転リミット X = {lim[0]:+.3f}° .. {lim[1]:+.3f}°" if lim else "  (Joint なし)")
    print(f"  初期  腕長={b0['arm_len']:.6f}  傾き={b0['tilt']:+.4f}°")
    print()

    hdr = (f"{'file':22s} {'傾き°':>10s} {'超過°':>9s} {'ロック違反':>11s} "
           f"{'腕長':>9s} {'ボーン傾き°':>12s} {'剛体回転X°':>11s}")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for p in paths[1:]:
        try:
            m = parse(p)
        except Exception as e:
            print(f"{os.path.basename(p):22s} PARSE FAIL: {e}")
            continue
        r = measure(m)
        if r is None:
            print(f"{os.path.basename(p):22s} 剛体不足")
            continue
        excess = r["tilt"] - lim[1] if lim else float("nan")
        lock_err = r["arm_len"] - b0["arm_len"]
        print(f"{os.path.basename(p):22s} {r['tilt']:+10.4f} {excess:+9.4f} "
              f"{r['anchor_err']:11.6f} {r['arm_len']:9.5f} {r['bone_tilt']:+12.4f} "
              f"{r['body_rot'][0]:+11.4f}")
        rows.append((os.path.basename(p), r))

    if len(rows) >= 2:
        print()
        print("条件間の差 (先頭を基準):")
        ref = rows[0]
        for name, r in rows[1:]:
            d_tilt = r["tilt"] - ref[1]["tilt"]
            d_len = r["anchor_err"] - ref[1]["anchor_err"]
            d_bone = r["bone_tilt"] - ref[1]["bone_tilt"]
            same = abs(d_tilt) < 1e-4 and abs(d_len) < 1e-6 and abs(d_bone) < 1e-4
            mark = "  == 完全一致" if same else ""
            print(f"  {ref[0]} -> {name:22s} "
                  f"d傾き={d_tilt:+.4f}° dロック違反={d_len:+.6f} dボーン傾き={d_bone:+.4f}°{mark}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
