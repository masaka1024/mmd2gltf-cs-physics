# -*- coding: utf-8 -*-
"""fricprobe: MMD/PmxEditor の **摩擦合成方式** を実証で特定するための最小 PMX を作る。

原理:
  傾いた静的な箱の上に動的な箱を置く。滑るか否かは tanθ と実効摩擦 μ の大小だけで決まる。
  合成方式の候補 (積 / 幾何平均 / 算術平均 / min / max) は同じ (f0,f1) から違う μ を出すので、
  θ を候補値の**間**に置けば「滑る / 滑らない」の二値で方式が割れる。

  ★ジョイントを使わない。接触と重力だけ。カオスも鎖漏れも無い。
  ★箱は扁平にして重心を低くし、転倒でなく滑りだけが起きるようにする。

使い方:
    python make_slope.py <出力ディレクトリ>
"""
import math, os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'pmxeprobe'))
import make_probe_pmx as P

# 判別行列。(f0=坂, f1=箱)
PAIRS = [("A", 1.0, 0.25), ("B", 0.5, 0.5)]
TANS  = [0.35, 0.55, 0.8]

def combos(f0, f1):
    return {
        "積":     f0 * f1,
        "幾何平均": math.sqrt(f0 * f1),
        "算術平均": (f0 + f1) * 0.5,
        "min":    min(f0, f1),
        "max":    max(f0, f1),
    }

def build(tag, f0, f1, tan, out_dir):
    th = math.atan(tan)
    n = (-math.sin(th), math.cos(th), 0.0)      # 坂の法線 (Z 回りに th 傾けた +Y)
    SLOPE_HY, BOX_HY = 0.5, 0.25
    m = P.ProbeModel("fric_%s_t%03d" % (tag, int(round(tan * 100))))
    m.add_bone("全ての親", (0.0, 0.0, 0.0), -1)
    m.add_bone("box", (n[0] * (SLOPE_HY + BOX_HY),
                       n[1] * (SLOPE_HY + BOX_HY), 0.0), 0)
    # 坂: ボーン追従 (= キネマティック)。グループ 0。
    m.add_body("slope", 0, (0.0, 0.0, 0.0), 0.0, 0, shape=1,
               size=(8.0, SLOPE_HY, 4.0), rot=(0.0, 0.0, th),
               friction=f0, group=0, mask=0xFFFF)
    # 箱: 物理演算。グループ 1 (同一グループ同士の除外に引っかからないように分ける)。
    m.add_body("box", 1, (n[0] * (SLOPE_HY + BOX_HY),
                          n[1] * (SLOPE_HY + BOX_HY), 0.0), 1.0, 1, shape=1,
               size=(1.0, BOX_HY, 1.0), rot=(0.0, 0.0, th),
               friction=f1, group=1, mask=0xFFFF)
    name = "fric_%s_tan%03d.pmx" % (tag, int(round(tan * 100)))
    p = os.path.join(out_dir, name)
    m.save(p, "friction combine probe: slope f=%.3g / box f=%.3g / tan=%.2f" % (f0, f1, tan))
    return name, th

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)
    methods = ["積", "幾何平均", "算術平均", "min", "max"]
    print("=" * 92)
    print("摩擦合成方式の判別行列  (滑る = tanθ > μ)")
    print("=" * 92)
    print("  %-6s %-14s %-7s %-7s | %s" % ("対", "(坂f0, 箱f1)", "tanθ", "θ[度]",
                                           "  ".join("%-9s" % k for k in methods)))
    made = []
    for tag, f0, f1 in PAIRS:
        cb = combos(f0, f1)
        for tan in TANS:
            name, th = build(tag, f0, f1, tan, out_dir)
            pred = ["滑る" if tan > cb[k] else "止まる" for k in methods]
            print("  %-6s (%.2f, %.2f)   %-7.2f %-7.2f | %s"
                  % (tag, f0, f1, tan, math.degrees(th),
                     "  ".join("%-9s" % x for x in pred)))
            made.append(name)
    print()
    print("  μ の値: " + " / ".join("%s=(%s)" % (t, ", ".join(
        "%s %.4g" % (k, combos(f0, f1)[k]) for k in methods)) for t, f0, f1 in PAIRS))
    print()
    print("  ★判別の要点")
    print("    B@0.35 … **積だけが滑る**。FRICMUL(積) か否かを一発で分ける決定セル。")
    print("    A@0.35 … {積, min} が滑る / {幾何, 算術, max} が止まる")
    print("    A@0.55 … {積, 幾何, min} が滑る / {算術, max} が止まる")
    print("    A@0.80 … max だけが止まる")
    print()
    print("  生成 %d 個 -> %s" % (len(made), os.path.abspath(out_dir)))

main()
