# -*- coding: utf-8 -*-
# 生成した probe PMX を独立に読み直して検証する。
# 目的は「書いたバイト列が仕様どおりに読めるか」なので、生成器のコードは一切共有しない
# (共有するとバグが相殺されて検証にならない)。全セクションを消費し、EOF ちょうどで
# 終わることを確認する = レイアウト誤りは必ず検出される。

import glob
import os
import struct
import sys


class R:
    def __init__(self, d):
        self.d = d
        self.o = 0

    def raw(self, n):
        v = self.d[self.o:self.o + n]
        assert len(v) == n, f"EOF at {self.o}"
        self.o += n
        return v

    def u8(self):  return self.raw(1)[0]
    def i8(self):  return struct.unpack("<b", self.raw(1))[0]
    def u16(self): return struct.unpack("<H", self.raw(2))[0]
    def i32(self): return struct.unpack("<i", self.raw(4))[0]
    def f32(self): return struct.unpack("<f", self.raw(4))[0]
    def f3(self):  return struct.unpack("<3f", self.raw(12))

    def text(self, enc):
        n = self.i32()
        return self.raw(n).decode(enc)


def parse(path):
    r = R(open(path, "rb").read())
    assert r.raw(4) == b"PMX ", "magic"
    ver = r.f32()
    gcount = r.u8()
    g = r.raw(gcount)
    enc = "utf-16-le" if g[0] == 0 else "utf-8"
    adduv, vsz, tsz, msz, bsz, mosz, rsz = g[1], g[2], g[3], g[4], g[5], g[6], g[7]

    def idx(size, signed=True):
        b = r.raw(size)
        if size == 1: return struct.unpack("<b" if signed else "<B", b)[0]
        if size == 2: return struct.unpack("<h" if signed else "<H", b)[0]
        return struct.unpack("<i", b)[0]

    info = [r.text(enc) for _ in range(4)]

    nv = r.i32()
    for _ in range(nv):
        r.f3(); r.f3(); r.raw(8)
        r.raw(16 * adduv)
        w = r.u8()
        if w == 0:
            idx(bsz)
        elif w == 1:
            idx(bsz); idx(bsz); r.f32()
        elif w == 2:
            for _ in range(4): idx(bsz)
            for _ in range(4): r.f32()
        elif w == 3:
            idx(bsz); idx(bsz); r.f32(); r.f3(); r.f3(); r.f3()
        else:
            raise AssertionError(f"weight type {w}")
        r.f32()

    nf = r.i32()
    for _ in range(nf):
        idx(vsz, signed=False)

    nt = r.i32()
    for _ in range(nt):
        r.text(enc)

    nm = r.i32()
    for _ in range(nm):
        r.text(enc); r.text(enc)
        r.raw(16); r.raw(12); r.f32(); r.raw(12)
        r.u8()
        r.raw(16); r.f32()
        idx(tsz); idx(tsz); r.u8()
        shared = r.u8()
        if shared == 0: idx(tsz)
        else: r.u8()
        r.text(enc)
        r.i32()

    bones = []
    nb = r.i32()
    for _ in range(nb):
        name = r.text(enc); r.text(enc)
        pos = r.f3()
        parent = idx(bsz)
        r.i32()
        fl = r.u16()
        if fl & 0x0001: idx(bsz)
        else: r.f3()
        if fl & (0x0100 | 0x0200): idx(bsz); r.f32()
        if fl & 0x0400: r.f3()
        if fl & 0x0800: r.f3(); r.f3()
        if fl & 0x2000: r.i32()
        if fl & 0x0020:
            idx(bsz); r.i32(); r.f32()
            for _ in range(r.i32()):
                idx(bsz)
                if r.u8(): r.f3(); r.f3()
        bones.append((name, pos, parent))

    nmo = r.i32()
    for _ in range(nmo):
        r.text(enc); r.text(enc); r.u8()
        kind = r.u8()
        cnt = r.i32()
        for _ in range(cnt):
            if kind == 0: idx(vsz, signed=False); r.f3()
            elif kind in (1, 2, 3, 4): idx(vsz, signed=False); r.raw(16)
            elif kind == 2: pass
            elif kind == 8: idx(msz); r.raw(1 + 16 + 12 + 4 + 12 + 16 + 4 + 16 + 16 + 16)
            elif kind == 9: idx(mosz); r.f32()
            else: raise AssertionError(f"morph kind {kind}")

    nd = r.i32()
    for _ in range(nd):
        r.text(enc); r.text(enc); r.u8()
        for _ in range(r.i32()):
            if r.u8() == 0: idx(bsz)
            else: idx(mosz)

    bodies = []
    nrb = r.i32()
    for _ in range(nrb):
        name = r.text(enc); r.text(enc)
        bone = idx(bsz)
        group = r.u8(); mask = r.u16()
        shape = r.u8(); size = r.f3()
        pos = r.f3(); rot = r.f3()
        mass = r.f32(); ld = r.f32(); ad = r.f32(); rest = r.f32(); fric = r.f32()
        mode = r.u8()
        bodies.append(dict(name=name, bone=bone, group=group, mask=mask, shape=shape,
                           size=size, pos=pos, rot=rot, mass=mass, mode=mode,
                           damp=(ld, ad), rest=rest, fric=fric))

    joints = []
    nj = r.i32()
    for _ in range(nj):
        name = r.text(enc); r.text(enc)
        kind = r.u8()
        assert kind == 0
        a = idx(rsz); b = idx(rsz)
        pos = r.f3(); rot = r.f3()
        lmin = r.f3(); lmax = r.f3(); amin = r.f3(); amax = r.f3()
        spos = r.f3(); srot = r.f3()
        joints.append(dict(name=name, a=a, b=b, pos=pos, rot=rot,
                           lin=(lmin, lmax), ang=(amin, amax), spring=(spos, srot)))

    assert r.o == len(r.d), f"trailing {len(r.d) - r.o} bytes (offset {r.o}/{len(r.d)})"
    return dict(ver=ver, name=info[0], bones=bones, bodies=bodies, joints=joints)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    paths = sorted(glob.glob(os.path.join(here, "models", "*.pmx")))
    if not paths:
        print("no models. run make_probe_pmx.py first"); return 1

    ok = True
    for p in paths:
        try:
            m = parse(p)
        except AssertionError as e:
            print(f"FAIL {os.path.basename(p)}: {e}"); ok = False; continue
        b = m["bodies"]; j = m["joints"]
        print(f"OK   {os.path.basename(p):24s} ver{m['ver']:.1f} "
              f"bones={len(m['bones'])} bodies={len(b)} joints={len(j)}")
        for x in b:
            print(f"       body {x['name']:6s} bone={x['bone']} mode={x['mode']} "
                  f"mass={x['mass']:g} mask=0x{x['mask']:04X} pos={x['pos']}")
        for x in j:
            import math
            amin = tuple(round(v * 180 / math.pi, 3) for v in x["ang"][0])
            amax = tuple(round(v * 180 / math.pi, 3) for v in x["ang"][1])
            print(f"       joint {x['name']} A={x['a']} B={x['b']} pos={x['pos']} "
                  f"lin={x['lin'][0]}..{x['lin'][1]} ang(deg)={amin}..{amax}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
