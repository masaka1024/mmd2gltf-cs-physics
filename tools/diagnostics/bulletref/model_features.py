# -*- coding: utf-8 -*-
"""
model_features.py -- タスク23: 悪化したモデルの「特徴」を PMX から直に読む。

出すもの (1モデル1行):
  動的剛体数 / Joint数 / 横渡し型・鎖型の内訳 / ばね定数が非ゼロな Joint の割合 /
  ばね(移動・回転)の中央値 / 剛体形状の内訳

★横渡し型・鎖型の判定は **名前ではなく移動制限の値** で行う (Joint.IsCrossTypeJoint と同一規則):
   並進3軸すべてが lo==hi (ロック) なら鎖型、1軸でも lo!=hi なら横渡し型。

使い方:
    python model_features.py <models.txt|pmx...> [--only "名前の一部,..."]
"""

import argparse
import os
import struct
import sys


def parse(path):
    d = open(path, "rb").read()
    if d[:4] != b"PMX ":
        raise ValueError("PMX ではない")
    n = d[8]
    g = d[9:9 + n]
    o = 9 + n
    enc, addv = g[0], g[1]
    sz = {"vert": g[2], "tex": g[3], "mat": g[4], "bone": g[5], "morph": g[6], "rb": g[7]}
    dec = "utf-16-le" if enc == 0 else "utf-8"

    def rt(o):
        ln = struct.unpack("<i", d[o:o + 4])[0]
        o += 4
        return d[o:o + ln].decode(dec, "replace"), o + ln

    def idx(o, w, signed=True):
        if w == 1:
            v = struct.unpack("<b" if signed else "<B", d[o:o + 1])[0]
        elif w == 2:
            v = struct.unpack("<h" if signed else "<H", d[o:o + 2])[0]
        else:
            v = struct.unpack("<i", d[o:o + 4])[0]
        return v, o + w

    _, o = rt(o); _, o = rt(o); _, o = rt(o); _, o = rt(o)          # model / comment
    # vertices
    vc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(vc):
        o += 12 + 12 + 8
        o += 4 * addv
        wt = d[o]; o += 1
        if wt == 0:   o += sz["bone"]
        elif wt == 1: o += sz["bone"] * 2 + 4
        elif wt == 2: o += sz["bone"] * 4 + 16
        elif wt == 3: o += sz["bone"] * 2 + 4 + 36
        elif wt == 4: o += sz["bone"] * 4 + 16
        o += 4
    # faces
    fc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    o += fc * sz["vert"]
    # textures
    tc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(tc):
        _, o = rt(o)
    # materials
    mc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(mc):
        _, o = rt(o); _, o = rt(o)
        o += 16 + 12 + 4 + 16 + 12 + 4 + 1
        o += sz["tex"] * 2 + 1
        sph = d[o]; o += 1
        if sph == 0:
            o += sz["tex"]
        else:
            o += 1
        _, o = rt(o)
        o += 4
    # bones
    bc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(bc):
        _, o = rt(o); _, o = rt(o)
        o += 12
        _, o = idx(o, sz["bone"])
        o += 4
        fl = struct.unpack("<H", d[o:o + 2])[0]; o += 2
        if fl & 0x0001:
            _, o = idx(o, sz["bone"])
        else:
            o += 12
        if fl & 0x0100 or fl & 0x0200:
            _, o = idx(o, sz["bone"]); o += 4
        if fl & 0x0400: o += 12
        if fl & 0x0800: o += 24
        if fl & 0x2000: o += 4
        if fl & 0x0020:
            _, o = idx(o, sz["bone"]); o += 4 + 4
            lc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
            for _ in range(lc):
                _, o = idx(o, sz["bone"])
                lim = d[o]; o += 1
                if lim: o += 24
    # morphs
    mo = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(mo):
        _, o = rt(o); _, o = rt(o)
        o += 1
        mt = d[o]; o += 1
        cnt = struct.unpack("<i", d[o:o + 4])[0]; o += 4
        per = {0: sz["morph"] + 4, 1: sz["vert"] + 12, 2: sz["bone"] + 28,
               3: sz["vert"] + 16, 4: sz["vert"] + 16, 5: sz["vert"] + 16,
               6: sz["vert"] + 16, 7: sz["vert"] + 16, 8: sz["mat"] + 1 + 16 + 12 + 4 + 16 + 12 + 4 + 16 + 16 + 16}
        o += cnt * per[mt]
    # display frames
    fr = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    for _ in range(fr):
        _, o = rt(o); _, o = rt(o)
        o += 1
        ec = struct.unpack("<i", d[o:o + 4])[0]; o += 4
        for _ in range(ec):
            t = d[o]; o += 1
            o += sz["bone"] if t == 0 else sz["morph"]
    # rigid bodies
    rc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    modes, shapes, rbnames = [], [], []
    for _ in range(rc):
        rbn, o = rt(o); _, o = rt(o)
        _, o = idx(o, sz["bone"])
        o += 1 + 2
        sh = d[o]; o += 1
        o += 12 + 12 + 12
        o += 20                      # 質量/移動減衰/回転減衰/反発/摩擦
        md = d[o]; o += 1
        modes.append(md); shapes.append(sh); rbnames.append(rbn)
    # joints
    jc = struct.unpack("<i", d[o:o + 4])[0]; o += 4
    cross = chain = 0
    spring_lin_nz = spring_ang_nz = 0
    for _ in range(jc):
        _, o = rt(o); _, o = rt(o)
        o += 1
        _, o = idx(o, sz["rb"]); _, o = idx(o, sz["rb"])
        o += 12 + 12
        llo = struct.unpack("<3f", d[o:o + 12]); o += 12
        lhi = struct.unpack("<3f", d[o:o + 12]); o += 12
        o += 12 + 12
        sl = struct.unpack("<3f", d[o:o + 12]); o += 12
        sa = struct.unpack("<3f", d[o:o + 12]); o += 12
        if all(a == b for a, b in zip(llo, lhi)):
            chain += 1
        else:
            cross += 1
        if any(v != 0 for v in sl): spring_lin_nz += 1
        if any(v != 0 for v in sa): spring_ang_nz += 1
    dyn = sum(1 for m in modes if m != 0)
    sh_name = {0: "球", 1: "箱", 2: "カプセル"}
    shcnt = {}
    for s in shapes: shcnt[sh_name.get(s, "?")] = shcnt.get(sh_name.get(s, "?"), 0) + 1
    dynnames = [n for n, m in zip(rbnames, modes) if m != 0]
    return dict(rb=rc, dyn=dyn, joints=jc, cross=cross, chain=chain,
                spLin=spring_lin_nz, spAng=spring_ang_nz, shapes=shcnt, dynnames=dynnames)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--only", default=None)
    ap.add_argument("--names", action="store_true", help="動的剛体の名前を接頭辞でまとめて出す")
    a = ap.parse_args()

    files = []
    for p in a.paths:
        if p.lower().endswith(".txt"):
            for l in open(p, encoding="utf-8"):
                t = l.strip()
                if t and os.path.isfile(t):
                    files.append(t)
        else:
            files.append(p)
    want = [w for w in a.only.split(",")] if a.only else None

    seen = set()
    print("  %-30s %5s %5s %6s %6s %6s %8s %8s  %s"
          % ("モデル", "剛体", "動的", "Joint", "横渡し", "鎖", "ばね移動", "ばね回転", "形状"))
    for p in files:
        nm = os.path.splitext(os.path.basename(p))[0]
        if nm in seen:
            continue
        seen.add(nm)
        if want and not any(w in nm for w in want):
            continue
        try:
            f = parse(p)
        except Exception as e:
            print("  %-30s [読めない] %s" % (nm[:30], e))
            continue
        print("  %-30s %5d %5d %6d %6d %6d %7d本 %7d本  %s"
              % (nm[:30], f["rb"], f["dyn"], f["joints"], f["cross"], f["chain"],
                 f["spLin"], f["spAng"],
                 " ".join("%s%d" % (k, v) for k, v in sorted(f["shapes"].items()))))
        if a.names:
            import collections, re as _re
            g = collections.Counter()
            for n2 in f["dynnames"]:
                g[_re.sub(r"[0-9０-９]+.*$", "", n2) or n2] += 1
            print("      動的剛体の系統: " + " / ".join("%s×%d" % (k, v) for k, v in g.most_common(14)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
