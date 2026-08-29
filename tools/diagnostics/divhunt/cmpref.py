# -*- coding: utf-8 -*-
# ===========================================================================
# cmpref v1.0  (2026-08-26 タスク80⑤ で正式化。以後の表は必ず "cmpref v1.0" を併記する)
#
#   divhunt の REFDUMP CSV と、参照 (PMXエディタ Fix ベイク由来) CSV を突き合わせる。
#   ★集計の定義をここに固定する。過去の表で f784 の絶対値が食い違ったのは、
#     集計が RMS だったか最大だったかを記録していなかったため (タスク80⑤で無効化)。
#
#   定義:
#     - 対象   : 両CSVに同じ (frame, boneName) がある行のみ
#     - 位置差 : そのフレームの全対象ボーンにわたる **RMS** (単位 = engine 単位)
#     - 角度差 : 同じく **RMS**。1ペアの角度は 2*acos(|dot(q1,q2)|) [deg] (符号違いを同一視)
#     - 区間指定 (FROM-TO) のときは各フレームの値を出し、その **最大** も併記する
#
#   ★★これは「2026-08-27 に実際に起きた参照の捏造をまさに検出する仕組み」である。削除禁止。
#   ★v1.1 (2026-08-27 タスク83②): **参照の健全性ゲートを内蔵**。突き合わせの前に必ず走り、
#     落ちたら数値を出さずに FAIL する。参照が壊れたまま数値を積み上げた事故が3回あったため
#     (MASSKIN / f784集計 / 参照捏造)。`--nohealth` で明示的に飛ばせるが、飛ばしたことは表示される。
#
#     ゲートA 親子距離の変動 = **警告のみ (FAIL にしない)**。
#              ★v1.1 では FAIL にしていたが**前提が誤り**だった: 物理ベイクはボーンに
#                **並進も焼き込む**ので、親子距離は不変ではない。実際、検証済みの
#                modelA 参照 (スカート) で 1.56 の変動が出て誤検知した。
#              名前からの親子推定も当てにならない (`スカート_0_5` と `_0_4` は
#                兄弟のことがある)。よって参考値として出すだけにする。
#     ゲートB 不自然にきれい: 鎖の各ボーンの毎フレーム回転角が、親のそれと
#              全フレームの 90% 超で一致 (<1e-3 deg) するなら「独立運動が無い」= FAIL
#              (2026-08-27 の捏造参照は 100% 一致だった)。
#
#   使い方:
#     python cmpref.py <ours.csv> <ref.csv> <frames> [--nohealth]
#       <frames> = "700,784"  (個別)  もしくは  "780-790" (区間)
# ===========================================================================
import sys, math, io

VERSION = "cmpref v1.2"

def load(p):
    d = {}
    with io.open(p, encoding='utf-8') as f:
        f.readline()
        for ln in f:
            v = ln.rstrip('\n').split(',')
            if len(v) < 9: continue
            d[(int(v[0]), v[1])] = tuple(float(x) for x in v[2:9])
    return d

def qang(a, b):
    dot = abs(sum(a[i] * b[i] for i in range(4)))
    return 2 * math.degrees(math.acos(max(-1.0, min(1.0, dot))))

def frame_rms(ours, ref, F):
    ks = [k for k in ours if k[0] == F and k in ref]
    if not ks: return None
    sp = sa = 0.0
    for k in ks:
        o, r = ours[k], ref[k]
        sp += sum((o[i] - r[i]) ** 2 for i in range(3))
        sa += qang(o[3:7], r[3:7]) ** 2
    n = len(ks)
    return (n, math.sqrt(sp / n), math.sqrt(sa / n))


# ─── 参照の健全性ゲート (v1.1) ───────────────────────────────────────────
def _chain_names(d):
    import re
    fam = {}
    for (F, nm) in d:
        m = re.match(r'^(.*?)_(\d+)_(\d+)$', nm)
        if m: fam.setdefault(m.group(1), set()).add(nm)
    if not fam: return []
    big = max(fam.values(), key=len)
    return sorted(big, key=lambda n: int(n.split('_')[-2]))

def health(path, d):
    """参照CSVの健全性。問題のリストを返す (空なら健全)。"""
    names = _chain_names(d)
    bad = []; warn = []
    if len(names) < 3:
        return (['鎖として認識できるボーンが %d 本しかない' % len(names)], [])
    frames = sorted({F for (F, _) in d})
    # ゲートA: 親子距離の不変性
    worst = 0.0; warg = None
    for i in range(len(names) - 1):
        a, b = names[i], names[i + 1]
        Ls = [math.dist(d[(F, a)][:3], d[(F, b)][:3]) for F in frames if (F, a) in d and (F, b) in d]
        if len(Ls) < 10: continue
        L0 = sum(Ls) / len(Ls)
        if L0 <= 0: continue
        rel = (max(Ls) - min(Ls)) / L0
        if rel > worst: worst, warg = rel, (a, b, min(Ls), max(Ls))
    if worst > 1e-3:
        warn.append('ゲートA(警告) 隣接名ボーンの距離が %.3e 変動。%s→%s が %.4f〜%.4f'
                    ' ※ベイクは並進も焼くので正常なこともある' % ((worst,) + warg))
    # ゲートB: 独立運動の有無
    same = tot = 0
    for i in range(1, len(names)):
        p, c = names[i - 1], names[i]
        for F in frames[1:]:
            ks = [(F - 1, p), (F, p), (F - 1, c), (F, c)]
            if any(k not in d for k in ks): continue
            wp = qang(d[ks[0]][3:7], d[ks[1]][3:7])
            wc = qang(d[ks[2]][3:7], d[ks[3]][3:7])
            tot += 1
            if abs(wp - wc) < 1e-3: same += 1
    if tot and same / tot > 0.90:
        bad.append('ゲートB 不自然にきれい: 親子の毎フレーム回転角が %.1f%% のサンプルで一致'
                   ' = 鎖に独立運動が無い (捏造参照の兆候)' % (100.0 * same / tot))
    return bad, warn

def main():
    argv = [a for a in sys.argv if a != '--nohealth']
    skip = len(argv) != len(sys.argv)
    ours, ref = load(argv[1]), load(argv[2])
    spec = argv[3]
    if skip:
        print('%s  ⚠ 健全性ゲートを --nohealth で飛ばした' % VERSION)
    else:
        bad, warn = health(argv[2], ref)
        for w in warn: print('%s  ⚠ %s' % (VERSION, w))
        if bad:
            print('%s  ★参照 %s は健全性ゲートに落ちた。数値は出さない。' % (VERSION, argv[2]))
            for b in bad: print('   - ' + b)
            sys.exit(2)
    if '-' in spec:
        a, b = (int(x) for x in spec.split('-'))
        rows = [(F, frame_rms(ours, ref, F)) for F in range(a, b + 1)]
        rows = [(F, r) for F, r in rows if r]
        if not rows: print('%s: no data' % VERSION); return
        mp = max(rows, key=lambda t: t[1][1]); ma = max(rows, key=lambda t: t[1][2])
        print('%s  f%d-%d  posRMS最大=%.3f (f%d)  angRMS最大=%.2f deg (f%d)'
              % (VERSION, a, b, mp[1][1], mp[0], ma[1][2], ma[0]))
    else:
        for F in (int(x) for x in spec.split(',')):
            r = frame_rms(ours, ref, F)
            print('%s  f%-5d %s' % (VERSION, F,
                  'no data' if not r else 'n=%-3d posRMS=%.3f  angRMS=%.2f deg' % r))

main()
