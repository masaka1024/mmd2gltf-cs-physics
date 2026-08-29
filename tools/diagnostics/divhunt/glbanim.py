# -*- coding: utf-8 -*-
"""glb のアニメーションからボーンのワールド姿勢を取り出す。
座標変換は Unity が出した bonepose_dump.csv を基準に校正する（推測しない）。"""
import os
import struct, json, math, sys, csv, io

GLB = os.environ.get('MMD_TEST_GLB', 'model.glb')          # ★絶対パスは書かない。env で指す
REF = os.environ.get('MMD_BONEPOSE_CSV', 'bonepose_dump.csv')

def load_glb(p):
    f = open(p, 'rb')
    struct.unpack('<III', f.read(12))
    clen, ct = struct.unpack('<II', f.read(8))
    j = json.loads(f.read(clen).decode('utf-8'))
    blen, bt = struct.unpack('<II', f.read(8))
    base = f.tell()
    f.seek(0)
    data = f.read()
    return j, data, base

CTYPE = {5120:('b',1),5121:('B',1),5122:('h',2),5123:('H',2),5125:('I',4),5126:('f',4)}
NCOMP = {'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}

def read_acc(j, data, base, idx):
    a = j['accessors'][idx]
    bv = j['bufferViews'][a['bufferView']]
    fmt, sz = CTYPE[a['componentType']]
    n = NCOMP[a['type']]
    off = base + bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = bv.get('byteStride') or (sz * n)
    out = []
    for i in range(a['count']):
        s = off + i * stride
        out.append(struct.unpack_from('<' + fmt * n, data, s))
    return out

# ---- 行列ユーティリティ (行優先 4x4, 列ベクトル規約) ----
def mat_from_trs(t, q, s):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    r = [[1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
         [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
         [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]]
    return [[r[i][k]*s[k] for k in range(3)] + [t[i]] for i in range(3)] + [[0,0,0,1]]

def mat_mul(a, b):
    return [[sum(a[i][k]*b[k][jj] for k in range(4)) for jj in range(4)] for i in range(4)]

def mat_to_pos(m):
    return (m[0][3], m[1][3], m[2][3])

def mat_to_quat(m):
    # スケール除去
    c = [[m[i][k] for i in range(3)] for k in range(3)]  # 列ベクトル
    ln = [math.sqrt(sum(v*v for v in col)) or 1.0 for col in c]
    r = [[m[i][k]/ln[k] for k in range(3)] for i in range(3)]
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s; x = (r[2][1]-r[1][2])/s; y = (r[0][2]-r[2][0])/s; z = (r[1][0]-r[0][1])/s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w = (r[2][1]-r[1][2])/s; x = 0.25*s; y = (r[0][1]+r[1][0])/s; z = (r[0][2]+r[2][0])/s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w = (r[0][2]-r[2][0])/s; x = (r[0][1]+r[1][0])/s; y = 0.25*s; z = (r[1][2]+r[2][1])/s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w = (r[1][0]-r[0][1])/s; x = (r[0][2]+r[2][0])/s; y = (r[1][2]+r[2][1])/s; z = 0.25*s
    n = math.sqrt(x*x+y*y+z*z+w*w) or 1.0
    return (x/n, y/n, z/n, w/n)

def slerp(a, b, t):
    d = sum(a[i]*b[i] for i in range(4))
    if d < 0: b = tuple(-v for v in b); d = -d
    if d > 0.9995:
        r = tuple(a[i] + t*(b[i]-a[i]) for i in range(4))
    else:
        th0 = math.acos(max(-1.0, min(1.0, d))); th = th0*t
        s0 = math.sin(th0); s1 = math.sin(th0-th)/s0; s2 = math.sin(th)/s0
        r = tuple(a[i]*s1 + b[i]*s2 for i in range(4))
    n = math.sqrt(sum(v*v for v in r)) or 1.0
    return tuple(v/n for v in r)

class Anim:
    def __init__(self, glb=GLB):
        self.j, self.data, self.base = load_glb(glb)
        j = self.j
        self.nodes = j['nodes']
        self.n = len(self.nodes)
        self.parent = [-1]*self.n
        for i, nd in enumerate(self.nodes):
            for c in nd.get('children', []):
                self.parent[c] = i
        self.name = [nd.get('name', '') for nd in self.nodes]
        self.idx_of = {}
        for i, nm in enumerate(self.name):
            self.idx_of.setdefault(nm, i)
        self.base_t = []; self.base_q = []; self.base_s = []
        for nd in self.nodes:
            if 'matrix' in nd:
                m = nd['matrix']  # 列優先
                mm = [[m[k*4+i] for k in range(4)] for i in range(4)]
                self.base_t.append(mat_to_pos(mm)); self.base_q.append(mat_to_quat(mm)); self.base_s.append((1,1,1))
            else:
                self.base_t.append(tuple(nd.get('translation', (0,0,0))))
                self.base_q.append(tuple(nd.get('rotation', (0,0,0,1))))
                self.base_s.append(tuple(nd.get('scale', (1,1,1))))
        # チャンネル
        a = j['animations'][0]
        self.ch = []
        for c in a['channels']:
            path = c['target']['path']
            if path == 'weights': continue
            s = a['samplers'][c['sampler']]
            inp = [v[0] for v in read_acc(j, self.data, self.base, s['input'])]
            out = read_acc(j, self.data, self.base, s['output'])
            self.ch.append((c['target']['node'], path, inp, out))
        self.duration = max(max(c[2]) for c in self.ch)

    def sample(self, t):
        T = list(self.base_t); Q = list(self.base_q); S = list(self.base_s)
        for node, path, inp, out in self.ch:
            n = len(inp)
            if t <= inp[0]: i0 = i1 = 0; u = 0.0
            elif t >= inp[-1]: i0 = i1 = n-1; u = 0.0
            else:
                lo, hi = 0, n-1
                while hi - lo > 1:
                    mid = (lo+hi)//2
                    if inp[mid] <= t: lo = mid
                    else: hi = mid
                i0, i1 = lo, hi
                u = (t - inp[i0]) / (inp[i1] - inp[i0])
            if path == 'rotation':
                Q[node] = slerp(out[i0], out[i1], u) if i0 != i1 else tuple(out[i0])
            elif path == 'translation':
                a0, a1 = out[i0], out[i1]
                T[node] = tuple(a0[k] + u*(a1[k]-a0[k]) for k in range(3))
            elif path == 'scale':
                a0, a1 = out[i0], out[i1]
                S[node] = tuple(a0[k] + u*(a1[k]-a0[k]) for k in range(3))
        world = [None]*self.n
        order = sorted(range(self.n), key=lambda i: self.depth(i))
        for i in order:
            local = mat_from_trs(T[i], Q[i], S[i])
            p = self.parent[i]
            world[i] = local if p < 0 else mat_mul(world[p], local)
        return world

    _d = None
    def depth(self, i):
        if self._d is None:
            self._d = [-1]*self.n
        if self._d[i] >= 0: return self._d[i]
        p = self.parent[i]
        self._d[i] = 0 if p < 0 else self.depth(p)+1
        return self._d[i]

def load_ref(path=REF):
    d = {}
    with open(path, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            d.setdefault(int(r['frame']), {})[r['boneName']] = (
                float(r['posX']), float(r['posY']), float(r['posZ']),
                float(r['quatX']), float(r['quatY']), float(r['quatZ']), float(r['quatW']))
    return d
