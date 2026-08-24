# -*- coding: utf-8 -*-
"""タスク47: 接触行の反復単位の突き合わせ。

第一不一致点が「反復0の接触求解」と判ったので、その中身を見る。
検証する予測:
  反復0でインパルスが割れる接触行が「深さの違う3ペア」に集中するか。
    集中する → 種は接触生成の深さ差 (core 配置の逸脱)
    分散する → 求解内の別量

使い方:
  python compare_contactiter.py <engineDir> <bulletDir> [frame]
"""
import csv, io, sys, os


def load(path, frame):
    out = []
    with io.open(path, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            if int(r['frame']) != frame:
                continue
            out.append(r)
    return out


def pairkey(r):
    a, b = r['bodyA'], r['bodyB']
    return (a, b) if a <= b else (b, a)


def gen_depths(path, frame):
    """接触生成 CSV から ペア -> 最小 dist を作る。"""
    d = {}
    with io.open(path, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            if int(r['frame']) != frame:
                continue
            k = pairkey(r)
            v = float(r['dist'])
            if k not in d or v < d[k]:
                d[k] = v
    return d


def main():
    eng, bul = sys.argv[1], sys.argv[2]
    frame = int(sys.argv[3]) if len(sys.argv) > 3 else 0

    ge = gen_depths(os.path.join(eng, 'contacts_engine.csv'), frame)
    gb = gen_depths(os.path.join(bul, 'contacts_bullet.csv'), frame)
    deep = set(k for k in set(ge) & set(gb) if abs(ge[k] - gb[k]) > 1e-5)
    print("接触生成: 共通ペア %d / 深さが違うペア %d" % (len(set(ge) & set(gb)), len(deep)))
    for k in sorted(deep):
        print("   ★深さ差  %-18s %-18s  自前 %+.6f / Bullet %+.6f  差 %+.2e"
              % (k[0], k[1], ge[k], gb[k], ge[k] - gb[k]))
    print()

    E = load(os.path.join(eng, 'contactiter_engine.csv'), frame)
    B = load(os.path.join(bul, 'contactiter_bullet.csv'), frame)
    print("接触行の反復トレース: 自前 %d 行 / Bullet %d 行" % (len(E), len(B)))
    if not E or not B:
        print("  ★どちらかが空。計装が効いていない。")
        return

    # ペア単位で集計する (点 index は両者で一致する保証が無いため、ペアごとの合計で比べる)
    def agg(rows, it):
        d = {}
        for r in rows:
            if int(r['iter']) != it:
                continue
            k = pairkey(r)
            a = d.setdefault(k, [0.0, 0.0, 0])
            a[0] += float(r['ni'])
            a[1] += abs(float(r['t1'])) + abs(float(r['t2']))
            a[2] += 1
        return d

    iters = sorted(set(int(r['iter']) for r in E) & set(int(r['iter']) for r in B))
    scale = max([abs(float(r['ni'])) for r in B] or [1.0])
    print("  法線インパルスの尺度 (Bullet 全反復の最大) = %.4g" % scale)
    print()
    print("  %-5s %-9s %-9s %s" % ("iter", "割れる行", "うち深さ差", "最大差のペア"))
    for it in iters:
        de, db = agg(E, it), agg(B, it)
        common = sorted(set(de) & set(db))
        bad = [k for k in common if abs(de[k][0] - db[k][0]) > 1e-3 * scale]
        inboth = [k for k in bad if k in deep]
        mx, mk = 0.0, None
        for k in common:
            v = abs(de[k][0] - db[k][0])
            if v > mx:
                mx, mk = v, k
        print("  %-5d %-9s %-9s %s (%.4g)"
              % (it, "%d/%d" % (len(bad), len(common)),
                 "%d/%d" % (len(inboth), len(deep)),
                 ("%s↔%s" % mk) if mk else "-", mx))

    # 反復0の詳細
    it0e, it0b = agg(E, iters[0]), agg(B, iters[0])
    common = sorted(set(it0e) & set(it0b))
    rows = sorted(((abs(it0e[k][0] - it0b[k][0]), k) for k in common), reverse=True)
    print()
    print("  反復%d でのペア別 法線インパルス (差の大きい順・上位12)" % iters[0])
    print("  %-18s %-18s %12s %12s %12s %s" % ("bodyA", "bodyB", "自前", "Bullet", "差", "深さ差"))
    for v, k in rows[:12]:
        print("  %-18s %-18s %12.6g %12.6g %12.3e %s"
              % (k[0], k[1], it0e[k][0], it0b[k][0], it0e[k][0] - it0b[k][0],
                 "★あり" if k in deep else ""))


main()
