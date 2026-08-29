# -*- coding: utf-8 -*-
"""物理ベイク VMD から、鎖の物理ボーンのワールド姿勢 (engine 座標) を作る。
★駆動ボーンは VMD から作らない (MMD の IK/付与親の評価が要るため)。
  鎖のアンカーだけ glb アニメ由来の値を使う (Unity 実測と一致検証済み)。
  鎖の中は親子直列なので、VMD のローカル値だけで FK できる。"""
import struct, sys, math, io, os
sys.setrecursionlimit(10000)
from glbanim import Anim, mat_from_trs, mat_mul, mat_to_pos, mat_to_quat

# ★2026-08-29 タスク3: 真OFF ベイクを食わせるため、入力VMDを環境変数で差し替え可能にした
#   (既定は従来どおり。値の計算には一切触れていない)。
VMD = os.environ.get('REFGEN_VMD', 'bake.vmd')   # ★絶対パスは書かない。env で指す
# ★★2026-08-27 タスク82: **重大な修正**。
#   VMD のボーン名は 15バイト shift_jis 固定。このモデルの物理ボーンは中国語名なので
#   PMXエディタのベイク時に shift_jis に無い簡体字が -> ? と **潰れて保存される**。
#   旧版は glb 名で完全一致検索していたため 1本もヒットせず samp() が恒等を返し、
#   **鎖が「剛体の棒」として捏造されていた**。タスク79-81 の この鎖の参照値は全部これ。
#   対応: glb 名を同じ規則 (shift_jis へ replace エンコード) で潰してから引く。
def vmd_key(name):
    return name.encode('shift_jis', errors='replace').decode('shift_jis')

SC, SGN, OFF = 12.5, (1, 1, -1), (-11.1146, 0.0044, 25.1444)
PREFIX = sys.argv[1] if len(sys.argv) > 1 else '\u5de6\u9a6c\u5c3e'
NFRAME = int(sys.argv[2]) if len(sys.argv) > 2 else 600     # 60Hz フレーム数
OUT = sys.argv[3] if len(sys.argv) > 3 else 'ref_chain.csv'

A = Anim()
chain = [i for i in range(A.n) if A.name[i].startswith(PREFIX)]
chain.sort(key=lambda i: A.depth(i))
anchor = A.parent[chain[0]]
want = {vmd_key(A.name[i]) for i in chain}
sys.stderr.write('chain=%d anchor_depth=%d\n' % (len(chain), A.depth(anchor)))

# --- VMD から鎖のローカル値だけ抜く ---
d = open(VMD, 'rb').read()
n = struct.unpack_from('<I', d, 50)[0]
off = 54
maxvf = NFRAME // 2 + 2
tr = {}
for _ in range(n):
    nm = d[off:off + 15].split(b'\x00')[0].decode('shift_jis', errors='replace'); off += 15
    fr = struct.unpack_from('<I', d, off)[0]; off += 4
    v = struct.unpack_from('<7f', d, off); off += 28 + 64
    if nm in want and fr <= maxvf:
        tr.setdefault(nm, {})[fr] = v
sys.stderr.write('vmd tracks=%d\n' % len(tr))

# ★★これは「2026-08-27 に実際に起きた参照の捏造をまさに検出する仕組み」である。削除禁止。
# ★★2026-08-27 タスク83②: **黙って通さない**。
#   2026-08-27 の事故は「一致 0本」で何も言わずに進み、samp() が恒等を返して
#   鎖が「剛体の棒」の捏造参照になったこと。SyncGuard と同じ思想でここで止める。
#   期待値 = 鎖のボーン数 - 2 (末端の "先" ボーン等の欠落を許容)。REFGEN_MIN_TRACKS で上書き可。
_min_tracks = int(os.environ.get('REFGEN_MIN_TRACKS', max(1, len(chain) - 2)))
if len(tr) < _min_tracks:
    sys.stderr.write(
        chr(10) + '[refgen] ★VMD トラックの一致が %d 本しかない (期待 %d 本以上)。参照を作らずに中止する。' % (len(tr), _min_tracks) + chr(10) +
        '  原因: VMD のボーン名は 15バイト shift_jis 固定。中国語ボーン名は ? に潰れて保存される。' + chr(10) +
        '  vmd_key() で glb 名を同じ規則に潰してから引くこと。' + chr(10) +
        '  ここで止めないと samp() が恒等を返し、鎖が「剛体の棒」の捏造参照になる。' + chr(10))
    sys.exit(2)

def samp(nm, vf):
    t = tr.get(nm)
    if not t: return (0, 0, 0, 0, 0, 0, 1)
    if vf in t: return t[vf]
    ks = [k for k in t if k <= vf]
    return t[max(ks)] if ks else (0, 0, 0, 0, 0, 0, 1)

def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)

buf = io.StringIO()
buf.write('frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW\n')
for f in range(NFRAME):
    t = f / 60.0
    W = A.sample(t)                      # glb アニメ由来 (アンカーはここから)
    Wc = {anchor: W[anchor]}
    vf = int(round(f / 2.0))             # VMD は 30fps
    for i in chain:
        nm = A.name[i]
        px, py, pz, qx, qy, qz, qw = samp(vmd_key(nm), vf)
        # MMD -> glTF: 位置 (x,y,-z)*0.08 / 回転 (-x,-y,z,w)
        bt = A.base_t[i]
        lt = (bt[0] + px * 0.08, bt[1] + py * 0.08, bt[2] - pz * 0.08)
        lq = (-qx, -qy, qz, qw)
        local = mat_from_trs(lt, lq, A.base_s[i])
        Wc[i] = mat_mul(Wc[A.parent[i]], local)
    for i in chain:
        p = mat_to_pos(Wc[i]); q = mat_to_quat(Wc[i])
        buf.write('%d,%s,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n' % (
            f, A.name[i],
            SGN[0]*p[0]*SC+OFF[0], SGN[1]*p[1]*SC+OFF[1], SGN[2]*p[2]*SC+OFF[2],
            -q[0], -q[1], q[2], q[3]))
    if f % 200 == 0: sys.stderr.write('  f%d/%d\n' % (f, NFRAME))
open(OUT, 'w', encoding='utf-8', newline='').write(buf.getvalue())
sys.stderr.write('wrote %s\n' % OUT)
