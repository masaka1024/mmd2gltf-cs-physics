# -*- coding: utf-8 -*-
import sys, math, io
sys.setrecursionlimit(10000)
from glbanim import *

SC = 12.5
SGN = (1, 1, -1)
OFF = (-11.1146, 0.0044, 25.1444)
FPS = 60
T_END = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else 'dance.csv'

A = Anim()
ref = load_ref()
bones = sorted(ref[0].keys())
idx = {b: A.idx_of[b] for b in bones}

# 必要なノードだけに絞る (20ボーンとその祖先)
need = set()
for b in bones:
    i = idx[b]
    while i >= 0:
        need.add(i); i = A.parent[i]
order = sorted(need, key=lambda i: A.depth(i))
ch = [c for c in A.ch if c[0] in need]
sys.stderr.write('needed nodes=%d  channels=%d\n' % (len(need), len(ch)))

base_t, base_q, base_s = A.base_t, A.base_q, A.base_s
parent = A.parent

def sample_fast(t):
    T = dict((i, base_t[i]) for i in need)
    Q = dict((i, base_q[i]) for i in need)
    S = dict((i, base_s[i]) for i in need)
    for node, path, inp, out in ch:
        n = len(inp)
        if t <= inp[0]: i0 = i1 = 0; u = 0.0
        elif t >= inp[-1]: i0 = i1 = n - 1; u = 0.0
        else:
            lo, hi = 0, n - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if inp[mid] <= t: lo = mid
                else: hi = mid
            i0, i1 = lo, hi
            u = (t - inp[i0]) / (inp[i1] - inp[i0])
        if path == 'rotation':
            Q[node] = slerp(out[i0], out[i1], u) if i0 != i1 else tuple(out[i0])
        elif path == 'translation':
            a0, a1 = out[i0], out[i1]
            T[node] = tuple(a0[k] + u * (a1[k] - a0[k]) for k in range(3))
        elif path == 'scale':
            a0, a1 = out[i0], out[i1]
            S[node] = tuple(a0[k] + u * (a1[k] - a0[k]) for k in range(3))
    W = {}
    for i in order:
        local = mat_from_trs(T[i], Q[i], S[i])
        p = parent[i]
        W[i] = local if p < 0 or p not in W else mat_mul(W[p], local)
    return W

nf = int(T_END * FPS)
buf = io.StringIO()
buf.write('frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW\n')
for f in range(nf):
    W = sample_fast(f / float(FPS))
    for b in bones:
        m = W[idx[b]]
        p = mat_to_pos(m); q = mat_to_quat(m)
        px = SGN[0] * p[0] * SC + OFF[0]
        py = SGN[1] * p[1] * SC + OFF[1]
        pz = SGN[2] * p[2] * SC + OFF[2]
        qx, qy, qz, qw = -q[0], -q[1], q[2], q[3]
        buf.write('%d,%s,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n' % (f, b, px, py, pz, qx, qy, qz, qw))
    if f % 600 == 0: sys.stderr.write('  frame %d/%d\n' % (f, nf))
open(OUT, 'w', encoding='utf-8', newline='').write(buf.getvalue())
sys.stderr.write('wrote %s  frames=%d\n' % (OUT, nf))
