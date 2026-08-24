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

# 第2ラウンド (裏取り用の階段)。
#   第1ラウンドが「全部滑る」だと、候補のうち残るのは 積 (μ=0.25) だけになる。
#   ただし **摩擦がまるで効いていない (μ≈0)** ときも同じ「全部滑る」になるので、
#   それを切り分けられない。浅い角度を並べて **止まり始める角度から μ を直に読む**。
#   対Bのみ (f0=f1=0.5) を使う。積なら μ=0.25 なので 0.30 で滑り 0.22 で止まるはず。
LADDER = [0.30, 0.22, 0.15, 0.08]

# 第4ラウンド (ベイクで「どの角でも止まる」と分かったあとの詰め)。
#   S: 対A (1.0, 0.25) を tanθ=1.5 で。**max(=1.0) でも滑らねばならない**角度。
#   Z: 摩擦を両方 0.05 に落として tanθ=0.30。**どの合成方式でも μ≤0.05** なので必ず滑る。
#   どちらでも止まるなら、止めているのは μ ではない。
EXTREME = [("S", 1.0, 0.25, 1.5), ("Z", 0.05, 0.05, 0.30)]

# 第5ラウンド (回転ロック)。(tag, f0, f1, tanθ, 坂の半長)
#   tanθ=1.50 は どの候補 μ (最大 1.0) でも必ず滑る角度。飛距離が μ で決まる。
#   L は第4ラウンドと同じ条件で、回転ロックが効いたかを見る対照。
# ★急角度 (tanθ=1.50) は使わない。較正で当エンジン自身が理論値から外れた
#   (回転ロックが接触に余分な抵抗を生む / ロック無しでも理論値を超える)。
#   **浅い角度では理論値とよく合う**ので、そこに絞って μ を挟む。
#   摩擦対は B (0.5, 0.5) 固定。積 なら μ=0.25、それ以外は全部 μ=0.5 になる。
LOCKED = [("B", 0.5, 0.5, 0.20, 8.0),
          ("B", 0.5, 0.5, 0.30, 12.0),
          ("B", 0.5, 0.5, 0.40, 40.0),
          ("B", 0.5, 0.5, 0.60, 90.0),
          # ★第6ラウンド: tan0.60 でも MMD が止まったので、μ>=0.6 を上から挟む。
          #   候補5方式は全部 <=0.5 なので、どれでもない。和 (f0+f1=1.0) が次の候補。
          ("B", 0.5, 0.5, 0.80, 120.0),
          ("B", 0.5, 0.5, 1.00, 140.0),
          ("B", 0.5, 0.5, 1.20, 160.0)]

# ★第7ラウンド: 第6ラウンドで MMD 側の箱が坂に **埋まって** いた。
#   沈み 0.326 / 0.438 に対し箱の半分の厚みは 0.25 — 自分の全高より深く潜っている。
#   当エンジンは同じ PMX で沈み 0.003 なので、これは MMD 側の接触の硬さの問題。
#   厚みを 8 倍にして、同じ 0.4 の沈みが効かないようにする。
THICK = [("B", 0.5, 0.5, 0.30, 12.0),
         ("B", 0.5, 0.5, 1.00, 140.0),
         ("B", 0.5, 0.5, 1.20, 160.0),
         # ★確認セル。(0.5,0.5) だけだと「MMD は摩擦値を無視して常に 0.25」でも
         #   同じ結果になってしまう。μ が対の値で動くことを別の対で確かめる。
         ("C", 1.0, 0.5,  1.00, 140.0),    # 積 -> 0.5    固定0.25 なら 0.25
         ("D", 0.25, 0.25, 0.30, 12.0)]    # 積 -> 0.0625 固定0.25 なら 0.25

def combos(f0, f1):
    return {
        "積":     f0 * f1,
        "幾何平均": math.sqrt(f0 * f1),
        "算術平均": (f0 + f1) * 0.5,
        "min":    min(f0, f1),
        "max":    max(f0, f1),
    }

def build(tag, f0, f1, tan, out_dir, lock_rot=False, slope_hx=8.0, prefix="fric",
          slope_hy=0.5, box_hy=0.25):
    """lock_rot=True で、箱と坂の間に **回転3軸ロック / 並進は自由** の 6DOF ジョイントを入れる。
    第4ラウンドで MMD 側の箱が転がってしまい μ の測定にならなかったため
    (回転 2〜56度・法線方向に 1.4〜3.0 浮いた)。転がりを殺せば飛距離が純粋に μ の関数になる。
    ★並進は3軸とも自由にする。法線方向を止めると接触が荷重を持たなくなり、
      μ·Pn が消えて摩擦そのものが効かなくなるため。"""
    th = math.atan(tan)
    n = (-math.sin(th), math.cos(th), 0.0)      # 坂の法線 (Z 回りに th 傾けた +Y)
    SLOPE_HY, BOX_HY = slope_hy, box_hy
    m = P.ProbeModel("%s_%s_t%03d" % (prefix, tag, int(round(tan * 100))))
    m.add_bone("全ての親", (0.0, 0.0, 0.0), -1)
    m.add_bone("box", (n[0] * (SLOPE_HY + BOX_HY),
                       n[1] * (SLOPE_HY + BOX_HY), 0.0), 0)
    # 坂: ボーン追従 (= キネマティック)。グループ 0。
    m.add_body("slope", 0, (0.0, 0.0, 0.0), 0.0, 0, shape=1,
               size=(slope_hx, SLOPE_HY, 4.0), rot=(0.0, 0.0, th),
               friction=f0, group=0, mask=0xFFFF)
    # 箱: 物理演算。グループ 1 (同一グループ同士の除外に引っかからないように分ける)。
    box_c = (n[0] * (SLOPE_HY + BOX_HY), n[1] * (SLOPE_HY + BOX_HY), 0.0)
    m.add_body("box", 1, box_c, 1.0, 1, shape=1,
               size=(1.0, BOX_HY, 1.0), rot=(0.0, 0.0, th),
               friction=f1, group=1, mask=0xFFFF)
    # ★見えるメッシュ。MMD は物理演算中のボーンマーカーを描かないので、
    #   形状が無いと「滑ったかどうか」が目視できない。
    #   坂と出発点マーカーは親ボーン(動かない)に、箱は box ボーン(物理で動く)に付ける。
    m.add_box_mesh(0, (0.0, 0.0, 0.0), (slope_hx, SLOPE_HY, 4.0), th,
                   "坂", (0.55, 0.58, 0.62, 1.0))
    # 出発点マーカー: 箱の初期位置に薄い赤い板を置く。箱がここから離れたら「滑った」。
    m.add_box_mesh(0, (box_c[0] - n[0] * BOX_HY * 0.9,
                       box_c[1] - n[1] * BOX_HY * 0.9, 0.0),
                   (1.15, 0.03, 1.15), th, "出発点", (0.90, 0.20, 0.20, 1.0))
    m.add_box_mesh(1, box_c, (1.0, BOX_HY, 1.0), th,
                   "箱", (0.20, 0.45, 0.90, 1.0))
    if lock_rot:
        # 坂 (静的) と 箱 の 6DOF ジョイント。フレームは坂と同じ傾き。
        #   並進: 3軸とも十分広く取って実質自由 (接触に荷重を持たせるため)
        #   回転: 3軸とも min=max=0 でロック
        m.add_joint("lockrot", 0, 1, box_c, rot=(0.0, 0.0, th),
                    lin_min=(-500.0, -500.0, -500.0), lin_max=(500.0, 500.0, 500.0),
                    ang_min=(0.0, 0.0, 0.0), ang_max=(0.0, 0.0, 0.0))
    name = "%s_%s_tan%03d.pmx" % (prefix, tag, int(round(tan * 100)))
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
    # --- 第2ラウンド: μ を挟む階段 (対B のみ) ---
    print()
    print("=" * 92)
    print("第2ラウンド: 「全部滑る」の裏取り。止まり始める角度から μ を直に読む (対B: f0=f1=0.5)")
    print("=" * 92)
    print("  %-22s %-7s %-7s | %s" % ("ファイル", "tanθ", "θ[度]",
                                      "  ".join("%-11s" % ("μ=%.3g なら" % m) for m in (0.25, 0.125, 0.0))))
    for tan in LADDER:
        name, th = build("L", 0.5, 0.5, tan, out_dir)
        preds = ["滑る" if tan > m else "止まる" for m in (0.25, 0.125, 0.0)]
        print("  %-22s %-7.2f %-7.2f | %s"
              % (name, tan, math.degrees(th), "  ".join("%-11s" % x for x in preds)))
        made.append(name)
    print()
    print("  読み方: 止まり始めた角の tanθ が μ の下限、その1つ上の角の tanθ が上限。")
    print("          どれも止まらなければ **摩擦がほぼ効いていない** ということになり、")
    print("          第1ラウンドの「積」という読みは取り下げになる。")
    # --- 第4ラウンド: 極端条件 ---
    print()
    print("=" * 92)
    print("第4ラウンド: 「μ ではない」を確かめる極端条件")
    print("=" * 92)
    for tag, f0, f1, tan in EXTREME:
        name, th = build(tag, f0, f1, tan, out_dir)
        cb = combos(f0, f1)
        worst = max(cb.values())
        print("  %-22s (%.2f, %.2f) tanθ=%.2f (%.1f度)  最大の μ = %.3g -> %s"
              % (name, f0, f1, tan, math.degrees(th), worst,
                 "どの方式でも滑る" if tan > worst else "★設計ミス"))
        made.append(name)
    print("  → ここで止まるなら、止めているのは摩擦係数ではない。")

    # --- 第5ラウンド: 回転ロック。転がりを消して μ を距離から直に読む ---
    print()
    print("=" * 92)
    print("第5ラウンド: **回転ロック**。転がりを消し、2秒の飛距離から μ を直に読む")
    print("=" * 92)
    print("  6DOF ジョイントで回転3軸をロック / 並進は自由。坂は長くして飛距離を受け止める。")
    print()
    print("  %-24s %-13s %-6s | %s" % ("ファイル", "(坂f0,箱f1)", "tanθ",
                                       "  ".join("%-11s" % k for k in methods)))
    g = 98.0
    for tag, f0, f1, tan, hx in THICK:
        build(tag, f0, f1, tan, out_dir, lock_rot=True, slope_hx=hx, prefix="fricT",
              slope_hy=2.0, box_hy=2.0)
    for tag, f0, f1, tan, hx in LOCKED:
        name, th = build(tag, f0, f1, tan, out_dir, lock_rot=True, slope_hx=hx, prefix="fricR")
        cb = combos(f0, f1)
        dists = []
        for k in methods:
            a = g * (math.sin(th) - cb[k] * math.cos(th))
            dists.append("%.1f" % (0.5 * a * 4) if a > 0 else "止まる")
        print("  %-24s (%.2f,%.2f)   %-6.2f | %s"
              % (name, f0, f1, tan, "  ".join("%-11s" % x for x in dists)))
        made.append(name)
    print()
    print("  ★2秒後の飛距離 [PMX単位] の予測。方式ごとに離れているので、")
    print("    ベイクした距離を読むだけで μ が決まる (二値判定が要らない)。")
    print()
    print("  生成 %d 個 -> %s" % (len(made), os.path.abspath(out_dir)))

main()
