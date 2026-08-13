# -*- coding: utf-8 -*-
# 複合プローブ(E9_multi)の結果を島ごとに集計する。
#
# 剛体マスク 0x0000 で島どうしは干渉しないので、1モデルに独立実験を並べられる。
# GUI 1回の保存で島の数だけデータ点が取れる = GUI 往復を劇的に減らすための仕組み。
#
# 各 Joint について「A側アンカーと B側アンカーのワールド距離」= 拘束違反量を出す。
# 初期姿勢では全剛体の回転が 0 なので、ジョイントのローカル位置は
# (joint_pos - body_pos) をそのまま使える。可動は X 軸まわりのみなのでオイラー順は影響しない。
#
# 使い方:
#   python multi_probe.py models/E9_multi.pmx out/E9_A1B1.pmx out/E9_A1B0.pmx

import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_probe import parse  # noqa: E402


# 差ありと判定する閾値。同条件を2回撮って全島がこれを下回れば収束している。
VIOL_TOL = 1e-4
ROT_TOL = 1e-2


def rx(t, v):
    c, s = math.cos(t), math.sin(t)
    return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def measure(base, res):
    """base(初期モデル)から各 Joint のローカルアンカーを決め、res での違反量を測る。"""
    out = []
    for k, j in enumerate(base["joints"]):
        a, b = j["a"], j["b"]
        r_a = sub(j["pos"], base["bodies"][a]["pos"])
        r_b = sub(j["pos"], base["bodies"][b]["pos"])

        ba, bb = res["bodies"][a], res["bodies"][b]
        wa = add(ba["pos"], rx(ba["rot"][0], r_a))
        wb = add(bb["pos"], rx(bb["rot"][0], r_b))

        lim = math.degrees(base["joints"][k]["ang"][1][0])
        rot_b = math.degrees(bb["rot"][0])
        out.append(dict(
            name=j["name"],
            viol=norm(sub(wa, wb)),
            rot=rot_b,
            excess=abs(rot_b) - lim,
            moved=norm(sub(bb["pos"], base["bodies"][b]["pos"])),
        ))
    return out


def main(argv):
    if len(argv) < 3:
        print("usage: multi_probe.py <baseline.pmx> <result.pmx>...")
        return 1

    base = parse(argv[1])
    paths = []
    for a in argv[2:]:
        g = sorted(glob.glob(a))
        paths.extend(g if g else [a])

    results = []
    for p in paths:
        results.append((os.path.basename(p), measure(base, parse(p))))

    names = [j["name"] for j in base["joints"]]
    w = max(len(n) for n in names) + 1

    print(f"baseline : {os.path.basename(argv[1])}   Joint数={len(names)}")
    print()
    head = f"{'joint':{w}s}"
    for fn, _ in results:
        head += f" | {fn[:20]:>20s}"
    print(head)
    print("-" * len(head))

    diffs = []
    for i, n in enumerate(names):
        line = f"{n:{w}s}"
        vals = []
        for _, rows in results:
            r = rows[i]
            line += f" | 違反{r['viol']:8.5f} 回転{r['rot']:+7.2f}"
            vals.append((r["viol"], r["rot"]))
        # 条件間の差。閾値は「同条件の再撮でも出るゆらぎ」を除くために置く。
        # 1e-6 のような機械精度では未収束の島を全部拾ってしまい判定にならない。
        dv = max(abs(v[0] - vals[0][0]) for v in vals[1:]) if len(vals) > 1 else 0.0
        dr = max(abs(v[1] - vals[0][1]) for v in vals[1:]) if len(vals) > 1 else 0.0
        if len(vals) > 1:
            line += f"  | d違反{dv:8.5f} d回転{dr:6.2f}"
        if dv > VIOL_TOL or dr > ROT_TOL:
            line += "  <<<"
            diffs.append(n)
        print(line)

    print()
    if len(results) < 2:
        print("(比較には結果ファイルが2つ以上必要)")
    elif diffs:
        print(f"★差が出た島 (違反>{VIOL_TOL} または 回転>{ROT_TOL}°): {', '.join(diffs)}")
        print("  同条件の再撮なら = 未収束。条件を変えた撮影なら = トグルの効果。")
    else:
        print(f"全島で差なし (違反<={VIOL_TOL}, 回転<={ROT_TOL}°)。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
