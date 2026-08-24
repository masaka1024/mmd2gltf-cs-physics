# -*- coding: utf-8 -*-
"""JOINTORDER の順序ファイルを rowtrace_bullet.csv から生成する。

なぜ要るか (タスク59 で2度目に踏んだ罠の恒久対策):
  Bullet は btDiscreteDynamicsWorld::solveConstraints で拘束配列を島ソートするので、
  ジョイントを解く順序が定義順から入れ替わる。Gauss-Seidel は順序依存なので、
  行レベルの突き合わせでは JOINTORDER でこれを揃える必要がある。
  ★順序は **そのランの網と初期条件** で決まる。別のランで作った順序ファイルを
    使い回すと行順序が食い違い、「行数が違う」「順序が不一致」に化ける。
    実際に2度踏んだ (§105 / §128)。突き合わせ相手の rowtrace から毎回作ること。

使い方:
    python make_jointorder.py <rowtrace_bullet.csv> [out.txt] [frame]
  out.txt を省略すると rowtrace と同じディレクトリの order_bullet.txt へ書く。
  frame を省略すると rowtrace に最初に現れるフレームを使う。
"""
import csv
import io
import os
import sys


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(src) or '.', 'order_bullet.txt')
    want = int(sys.argv[3]) if len(sys.argv) > 3 else None

    seen = []
    first = None
    with io.open(src, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            fr = int(r['frame'])
            if first is None:
                first = fr if want is None else want
            if fr < first:
                continue
            if fr > first:
                break
            if r['joint'] not in seen:
                seen.append(r['joint'])

    if not seen:
        print("★ジョイント行が見つからない: %s (frame=%s)" % (src, first))
        return 1

    io.open(out, 'w', encoding='utf-8', newline='\n').write('\n'.join(seen))
    print("frame %d の島ソート順 %d 本 -> %s" % (first, len(seen), out))
    print("  先頭5: %s" % ", ".join(seen[:5]))
    return 0


sys.exit(main())
